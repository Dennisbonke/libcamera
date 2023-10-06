/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2023, Linaro Ltd
 *
 * converter_softw.h - interface of software converter (runs 100% on CPU)
 */

#pragma once

#include <functional>
#include <map>
#include <memory>
#include <string>
#include <tuple>
#include <vector>

#include <libcamera/base/log.h>
#include <libcamera/base/mutex.h>
#include <libcamera/base/signal.h>
#include <libcamera/base/thread.h>

#include <libcamera/pixel_format.h>

#include "libcamera/internal/converter.h"

namespace libcamera {

class FrameBuffer;
class MediaDevice;
class Size;
class SizeRange;
struct StreamConfiguration;

class SwConverter : public Converter
{
public:
	SwConverter(MediaDevice *media);

	int loadConfiguration([[maybe_unused]] const std::string &filename) { return 0; }
	bool isValid() const { return isp_ != nullptr; }

	std::vector<PixelFormat> formats(PixelFormat input);
	SizeRange sizes(const Size &input);

	std::tuple<unsigned int, unsigned int>
	strideAndFrameSize(const PixelFormat &pixelFormat, const Size &size);

	int configure(const StreamConfiguration &inputCfg,
		      const std::vector<std::reference_wrapper<StreamConfiguration>> &outputCfg);
	int exportBuffers(unsigned int ouput, unsigned int count,
			  std::vector<std::unique_ptr<FrameBuffer>> *buffers);

	void process(FrameBuffer *input, FrameBuffer *output);
	int start();
	void stop();

	int queueBuffers(FrameBuffer *input,
			 const std::map<unsigned int, FrameBuffer *> &outputs);

private:
	class Isp : public Object
	{
	public:
		Isp(SwConverter *converter);
		~Isp();

		int configure(const StreamConfiguration &inputCfg,
			      const StreamConfiguration &outputCfg);
		int exportBuffers(unsigned int count,
				  std::vector<std::unique_ptr<FrameBuffer>> *buffers);
		void process(FrameBuffer *input, FrameBuffer *output);
		int start();
		void stop();
		void waitForIdle();

	private:
		void debayer(uint8_t *dst, const uint8_t *src);

		SwConverter *converter_;

		Thread thread_;

		libcamera::Mutex idleMutex_;
		libcamera::ConditionVariable idleCV_;
		bool idle_ LIBCAMERA_TSA_GUARDED_BY(idleMutex_);

		unsigned int width_;
		unsigned int height_;
		unsigned int stride_;
		Point red_shift_;

		unsigned long rNumerat_, rDenomin_; /* red gain for AWB */
		unsigned long bNumerat_, bDenomin_; /* blue gain for AWB */
		unsigned long gNumerat_, gDenomin_; /* green gain for AWB */
	};

	std::unique_ptr<Isp> isp_;
};

} /* namespace libcamera */
