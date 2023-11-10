/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2023, Linaro Ltd
 *
 * simple.cpp - Simple SoftwareIsp Image Processing Algorithm module
 */
#include <libcamera/ipa/simple_ipa_interface.h>

#include <sys/mman.h>

#include <libcamera/base/file.h>
#include <libcamera/base/log.h>

#include <libcamera/control_ids.h>
#include <libcamera/ipa/ipa_interface.h>
#include <libcamera/ipa/ipa_module_info.h>

#include "libcamera/internal/camera_sensor.h"
#include "libcamera/internal/soft_isp/statistics.h"
#include <numeric>

#define EXPOSURE_SATISFACTORY_OFFSET 0.2
#define EXPOSURE_CHANGE_VALUE 30

namespace libcamera {

LOG_DEFINE_CATEGORY(IPASimple)

namespace ipa::simple {

class IPASimple : public ipa::simple::IPASimpleInterface
{
public:
	IPASimple() : ignore_updates_(0)
	{
	}
	~IPASimple()
	{
		if (stats_)
			munmap(stats_, sizeof(Statistics));
	}

	int init(const IPASettings &settings,
		 const SharedFD &fd,
		 const ControlInfoMap &sensorInfoMap) override;
	int configure(const ControlInfoMap &sensorInfoMap) override;

	int start() override;
	void stop() override;

	void processStats(const ControlList &sensorControls) override;

private:
	void update_exposure(double ev_adjustment);
	void update_exposure2(std::vector<int> histRed, std::vector<int> histGreenRed, std::vector<int> histGreenBlue, std::vector<int> histBlue, std::vector<int> histLuminance);

	SharedFD fd_;
	Statistics *stats_;
	int exposure_min_, exposure_max_;
	int again_min_, again_max_;
	int again_, exposure_;
	int ignore_updates_;
};

int IPASimple::init([[maybe_unused]] const IPASettings &settings,
		    const SharedFD &fd, const ControlInfoMap &sensorInfoMap)
{
	fd_ = std::move(fd);
	if (!fd_.isValid()) {
		LOG(IPASimple, Error) << "Invalid Statistics handle";
		return -ENODEV;
	}

	stats_ = static_cast<Statistics *>(mmap(nullptr, sizeof(Statistics),
						PROT_READ | PROT_WRITE, MAP_SHARED,
						fd_.get(), 0));
	if (!stats_) {
		LOG(IPASimple, Error) << "Unable to map Statistics";
		return -ENODEV;
	}

	if (sensorInfoMap.find(V4L2_CID_EXPOSURE) == sensorInfoMap.end()) {
		LOG(IPASimple, Error) << "Don't have exposure control";
		return -EINVAL;
	}

	if (sensorInfoMap.find(V4L2_CID_ANALOGUE_GAIN) == sensorInfoMap.end()) {
		LOG(IPASimple, Error) << "Don't have gain control";
		return -EINVAL;
	}

	return 0;
}

int IPASimple::configure(const ControlInfoMap &sensorInfoMap)
{
	const ControlInfo &exposure_info = sensorInfoMap.find(V4L2_CID_EXPOSURE)->second;
	const ControlInfo &gain_info = sensorInfoMap.find(V4L2_CID_ANALOGUE_GAIN)->second;

	exposure_min_ = exposure_info.min().get<int>();
	if (!exposure_min_) {
		LOG(IPASimple, Warning) << "Minimum exposure is zero, that can't be linear";
		exposure_min_ = 1;
	}
	exposure_max_ = exposure_info.max().get<int>();
	again_min_ = gain_info.min().get<int>();
	if (!again_min_) {
		LOG(IPASimple, Warning) << "Minimum gain is zero, that can't be linear";
		again_min_ = 100;
	}
	again_max_ = gain_info.max().get<int>();

	LOG(IPASimple, Info) << "Exposure " << exposure_min_ << "-" << exposure_max_
			     << ", gain " << again_min_ << "-" << again_max_;

	return 0;
}

void IPASimple::update_exposure(double ev_adjustment)
{
	double exp = (double)exposure_;
	double gain = (double)again_;
	double ev = ev_adjustment * exp * gain;

	/*
	 * Try to use the minimal possible analogue gain.
	 * The exposure can be any value from exposure_min_ to exposure_max_,
	 * and normally this should keep the frame rate intact.
	 */

	exp = ev / again_min_;
	if (exp > exposure_max_) exposure_ = exposure_max_;
	else if (exp < exposure_min_) exposure_ = exposure_min_;
	else exposure_ = (int)exp;

	gain = ev / exposure_;
	if (gain > again_max_) again_ = again_max_;
	else if (gain < again_min_) again_ = again_min_;
	else again_ = (int)gain;

	LOG(IPASimple, Debug) << "Desired EV = " << ev
			      << ", real EV = " << (double)again_ * exposure_;
}

void IPASimple::update_exposure2(std::vector<int> histRed, std::vector<int> histGreenRed, std::vector<int> histGreenBlue, std::vector<int> histBlue, std::vector<int> histLuminance) {

	(void)histRed;
	(void)histGreenRed;
	(void)histGreenBlue;
	(void)histBlue;

	std::size_t const offset = histLuminance.size()/5;

	unsigned Num = 0;

	// Totals up all luminance values.
	unsigned Denom = std::accumulate(histLuminance.begin(),histLuminance.end(),0);

	// Calculates Numerator
	for(int i = 0; i <= 4; i++){
		std::vector<int>::iterator beginIterator = histLuminance.begin() + offset * i;
		std::vector<int>::iterator endIterator = histLuminance.begin() + offset * (i+1);

		unsigned Xi = std::accumulate(beginIterator,endIterator,0);

		Num += Xi * (i + 1);
	}

	// Correctly exposed when val = 2.5
	float val = (float)Num / Denom;

	// Algorithm will be satisfied if val is less than EXPOSURE_SATISFACTORY_OFFSET away from 2.5 (which is optimal)
	// Algorithm will change exposure by EXPOSURE_CHANGE_VALUE if exposure is not optimal.

	if (val < 2.5 - EXPOSURE_SATISFACTORY_OFFSET){

		// Exposure needs to be higher.
		exposure_ += EXPOSURE_CHANGE_VALUE;

		if (exposure_ >= exposure_max_){
			// Increase gain.
			again_ += EXPOSURE_CHANGE_VALUE;
		}
	}
	
	if (val > 2.5 + EXPOSURE_SATISFACTORY_OFFSET){

		// If exposure is maximum, and gain is not minimum, decrease gain.
		if (exposure_ == exposure_max_ && again_ != again_min_){
			again_ -= EXPOSURE_CHANGE_VALUE;
		}else {
			exposure_ -= EXPOSURE_CHANGE_VALUE;
		}
		// Exposure needs to be lower.
		
	}

	// Clamp exposure value between max and min value it's allowed to be.
	if (exposure_ > exposure_max_) exposure_ = exposure_max_;
	else if (exposure_ < exposure_min_) exposure_ = exposure_min_;
	
	// Clamp gain value between max and min value it's allowed to be.
	if (again_ > again_max_) again_ = again_max_;
	else if (again_ < again_min_) again_ = again_min_;

	// Set optimal gain to some default value. We first need to make sure exposure is correctly set before fiddling with gain.
	//again_ = 1.0;

	LOG(IPASimple, Debug) << "update_exposure2 returned exposure: " << exposure_ << " and gain: " << again_;
}

void IPASimple::processStats(const ControlList &sensorControls)
{
	double ev_adjustment = 0.0;
	(void)ev_adjustment;
	ControlList ctrls(sensorControls);

	/*
	 * Use 2 frames delay to make sure that the exposure and the gain set
	 * have applied to the camera sensor
	 */
	if (ignore_updates_ > 0) {
		LOG(IPASimple, Debug) << "Skipping exposure update: "
				      << ignore_updates_;
		--ignore_updates_;
		return;
	}

#if 0
	if (stats_->bright_ratio < 0.01) ev_adjustment = 1.1;
	if (stats_->too_bright_ratio > 0.04) ev_adjustment = 0.9;

	if (ev_adjustment != 0.0) {
		/* sanity check */
		if (!sensorControls.contains(V4L2_CID_EXPOSURE) ||
		    !sensorControls.contains(V4L2_CID_ANALOGUE_GAIN)) {
			LOG(IPASimple, Error) << "Control(s) missing";
			return;
		}

		exposure_ = ctrls.get(V4L2_CID_EXPOSURE).get<int>();
		again_ = ctrls.get(V4L2_CID_ANALOGUE_GAIN).get<int>();

		update_exposure(ev_adjustment);

		ctrls.set(V4L2_CID_EXPOSURE, exposure_);
		ctrls.set(V4L2_CID_ANALOGUE_GAIN, again_);

		ignore_updates_ = 2;

		setSensorControls.emit(ctrls);
	}
#else
	exposure_ = ctrls.get(V4L2_CID_EXPOSURE).get<int>();
	again_ = ctrls.get(V4L2_CID_ANALOGUE_GAIN).get<int>();
	update_exposure2(stats_->histRed, stats_->histGreenRed, stats_->histGreenBlue, stats_->histBlue, stats_->histLuminance);
	ctrls.set(V4L2_CID_EXPOSURE, exposure_);
	ctrls.set(V4L2_CID_ANALOGUE_GAIN, again_);

	ignore_updates_ = 2;

	setSensorControls.emit(ctrls);
#endif
}

int IPASimple::start()
{
	return 0;
}

void IPASimple::stop()
{
}

} /* namespace ipa::simple */

/*
 * External IPA module interface
 */
extern "C" {
const struct IPAModuleInfo ipaModuleInfo = {
	IPA_MODULE_API_VERSION,
	0,
	"SimplePipelineHandler",
	"simple",
};

IPAInterface *ipaCreate()
{
	return new ipa::simple::IPASimple();
}
}

} /* namespace libcamera */
