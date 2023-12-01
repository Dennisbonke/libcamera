/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2023, Linaro Ltd
 *
 * converter_softw.h - interface of software converter (runs 100% on CPU)
 */

#pragma once

#include <map>
#include <tuple>
#include <vector>

#include <libcamera/base/log.h>
#include <libcamera/base/signal.h>
#include <libcamera/base/thread.h>

#include <libcamera/pixel_format.h>

#include "libcamera/internal/bayer_format.h"
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
	SwConverter(MediaDevice *media)
		: Converter(media) {}

	int loadConfiguration([[maybe_unused]] const std::string &filename) { return 0; }
	bool isValid() const { return true; }

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

	Signal<float, float, std::vector<int>, std::vector<int>, std::vector<int>, std::vector<int>, std::vector<int>> agcDataReady;

private:
	class Isp : public Object
	{
	public:
		Isp(SwConverter *converter)
			: converter_(converter) {}

		int configure(const StreamConfiguration &inputCfg,
			      const StreamConfiguration &outputCfg);
		int exportBuffers(unsigned int count,
				  std::vector<std::unique_ptr<FrameBuffer>> *buffers);
		void process(FrameBuffer *input, FrameBuffer *output);
		int start();
		void stop();

	private:
		void debayerP(uint8_t *dst, const uint8_t *src);
		void debayerNP(uint8_t *dst, const uint8_t *src);
		void debayerInfrarood(uint8_t *dst, const uint8_t *src);
		
		uint16_t* readByteFromCamera(const uint8_t *pin_base);
		const uint8_t* addrcoord(const uint8_t *pin_base,int x, int offx, int offy);
		SwConverter *converter_;

		Thread thread_;

		float r_avg, g_avg, b_avg;
		float blue_change, red_change;
		unsigned int red_count, blue_count, green_count;

		unsigned int width_;
		unsigned int height_;
		unsigned int stride_;
		Point red_shift_;

		unsigned long rNumerat_, rDenomin_; /* red gain for AWB */
		unsigned long bNumerat_, bDenomin_; /* blue gain for AWB */
		unsigned long gNumerat_, gDenomin_; /* green gain for AWB */		

		BayerFormat::Packing Packed;

		float bright_ratio_;		/* 0.0 to 1.0 (1.0 == 100%) */
		float too_bright_ratio_;	/* 0.0 to 1.0 */

		std::vector<int> histRed_;
		std::vector<int> histGreenRed_;
		std::vector<int> histGreenBlue_;
		std::vector<int> histBlue_;
		std::vector<int> histLuminance_;
	};

	std::unique_ptr<Isp> isp_;
};

} /* namespace libcamera */
