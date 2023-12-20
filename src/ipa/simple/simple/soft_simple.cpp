/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2023, Linaro Ltd
 *
 * soft_simple.cpp - Simple Software Image Processing Algorithm module
 */

#include <sys/mman.h>

#include <libcamera/base/file.h>
#include <libcamera/base/log.h>

#include <libcamera/control_ids.h>

#include <libcamera/ipa/ipa_interface.h>
#include <libcamera/ipa/ipa_module_info.h>

#include "libcamera/internal/camera_sensor.h"
#include "libcamera/internal/software_isp/swisp_stats.h"

#include "common/soft_base.h"

#define EXPOSURE_SATISFACTORY_OFFSET 0.2

#define EXPOSURE_CHANGE_VALUE 130

#define OPTIMAL_EXPOSURE_VALUE 2.5

namespace libcamera {

LOG_DECLARE_CATEGORY(IPASoft)

namespace ipa::soft {

class IPASoftSimple final : public IPASoftBase
{
public:
	IPASoftSimple()
		: IPASoftBase(), ignore_updates_(0)
	{
	}

	~IPASoftSimple()
	{
		if (stats_)
			munmap(stats_, sizeof(SwIspStats));
	}

	int platformInit(const ControlInfoMap &sensorInfoMap) override;
	int platformConfigure(const ControlInfoMap &sensorInfoMap) override;
	int platformStart() override;
	void platformStop() override;
	void platformProcessStats(const ControlList &sensorControls) override;

private:
	void update_exposure(double ev_adjustment);
	void update_y_exposure(double exposuremsv);

	SwIspStats *stats_;
	int exposure_min_, exposure_max_;
	int again_min_, again_max_;
	int again_, exposure_;
	int ignore_updates_;
};

int IPASoftSimple::platformInit(const ControlInfoMap &sensorInfoMap)
{
	stats_ = static_cast<SwIspStats *>(mmap(nullptr, sizeof(SwIspStats),
						PROT_READ | PROT_WRITE, MAP_SHARED,
						fdStats_.get(), 0));
	if (!stats_) {
		LOG(IPASoft, Error) << "Unable to map Statistics";
		return -ENODEV;
	}

	if (sensorInfoMap.find(V4L2_CID_EXPOSURE) == sensorInfoMap.end()) {
		LOG(IPASoft, Error) << "Don't have exposure control";
		return -EINVAL;
	}

	if (sensorInfoMap.find(V4L2_CID_ANALOGUE_GAIN) == sensorInfoMap.end()) {
		LOG(IPASoft, Error) << "Don't have gain control";
		return -EINVAL;
	}

	return 0;
}

int IPASoftSimple::platformConfigure(const ControlInfoMap &sensorInfoMap)
{
	const ControlInfo &exposure_info = sensorInfoMap.find(V4L2_CID_EXPOSURE)->second;
	const ControlInfo &gain_info = sensorInfoMap.find(V4L2_CID_ANALOGUE_GAIN)->second;

	exposure_min_ = exposure_info.min().get<int>();
	if (!exposure_min_) {
		LOG(IPASoft, Warning) << "Minimum exposure is zero, that can't be linear";
		exposure_min_ = 1;
	}
	exposure_max_ = exposure_info.max().get<int>();
	again_min_ = gain_info.min().get<int>();
	if (!again_min_) {
		LOG(IPASoft, Warning) << "Minimum gain is zero, that can't be linear";
		again_min_ = 100;
	}
	again_max_ = gain_info.max().get<int>();

	LOG(IPASoft, Info) << "Exposure " << exposure_min_ << "-" << exposure_max_
			   << ", gain " << again_min_ << "-" << again_max_;

	return 0;
}

int IPASoftSimple::platformStart()
{
	return 0;
}

void IPASoftSimple::platformStop()
{
}

void IPASoftSimple::platformProcessStats(const ControlList &sensorControls)
{
	ControlList ctrls(sensorControls);

	/*
	 * Use 2 frames delay to make sure that the exposure and the gain set
	 * have applied to the camera sensor
	 */
	if (ignore_updates_ > 0) {
		LOG(IPASoft, Debug) << "Skipping exposure update: "
				    << ignore_updates_;
		--ignore_updates_;
		return;
	}

	unsigned int denom = 0;

	unsigned int num = 0;

	int y_histogramSmall[5] = { };
	for (int i = 0; i < 16; i++){
		y_histogramSmall[(i - i / 8) / 3] += stats_->y_histogram[i];
		LOG(IPASoft, Debug) << i << ": " <<stats_->y_histogram[i];
	}

	for(int i = 0; i < 5; i++){
		denom += y_histogramSmall[i];
		num += y_histogramSmall[i] * (i + 1);
	}
	


	float exposuremsv = (float)num/denom;
	LOG(IPASoft,Debug) << "\nBin 1: " << y_histogramSmall[0] <<"\n" <<
							"Bin 2: " << y_histogramSmall[1] <<"\n" <<
							"Bin 3: " << y_histogramSmall[2] <<"\n" <<
							"Bin 4: " << y_histogramSmall[3] <<"\n" <<
							"Bin 5: " << y_histogramSmall[4] <<"\n" <<
							"ExposureMSV: " << exposuremsv << "\n" << 
							"Denom: " << denom;


	exposure_ = ctrls.get(V4L2_CID_EXPOSURE).get<int>();
	again_ = ctrls.get(V4L2_CID_ANALOGUE_GAIN).get<int>();

	//update_exposure(ev_adjustment);
	update_y_exposure(exposuremsv);

	ctrls.set(V4L2_CID_EXPOSURE, exposure_);
	ctrls.set(V4L2_CID_ANALOGUE_GAIN, again_);

	ignore_updates_ = 2;

	setSensorControls.emit(ctrls);
}

void IPASoftSimple::update_exposure(double ev_adjustment)
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
	if (exp > exposure_max_)
		exposure_ = exposure_max_;
	else if (exp < exposure_min_)
		exposure_ = exposure_min_;
	else
		exposure_ = (int)exp;

	gain = ev / exposure_;
	if (gain > again_max_)
		again_ = again_max_;
	else if (gain < again_min_)
		again_ = again_min_;
	else
		again_ = (int)gain;

	LOG(IPASoft, Debug) << "Desired EV = " << ev
			    << ", real EV = " << (double)again_ * exposure_;
}

void IPASoftSimple::update_y_exposure(double exposuremsv)
{
if (exposuremsv < OPTIMAL_EXPOSURE_VALUE - EXPOSURE_SATISFACTORY_OFFSET){
		LOG(IPASoft, Debug) << "UNDEREXPOSED";
		// Exposure needs to be higher.
		exposure_ += EXPOSURE_CHANGE_VALUE;

		if (exposure_ >= exposure_max_){
			// Increase gain.
			again_ += EXPOSURE_CHANGE_VALUE;
		}
	}
	
	if (exposuremsv > OPTIMAL_EXPOSURE_VALUE + EXPOSURE_SATISFACTORY_OFFSET){

		LOG(IPASoft, Debug) << "OVEREXPOSED";
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

	LOG(IPASoft, Debug) << "update_y_exposure returned exposure: " << exposure_ << " and gain: " << again_;

}

} /* namespace ipa::soft */

/*
 * External IPA module interface
 */
extern "C" {
const struct IPAModuleInfo ipaModuleInfo = {
	IPA_MODULE_API_VERSION,
	0,
	"SimplePipelineHandler",
	"soft/simple",
};

IPAInterface *ipaCreate()
{
	return new ipa::soft::IPASoftSimple();
}

} /* extern "C" */

} /* namespace libcamera */
