/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2023, Linaro Ltd
 *
 * converter_softw.h - interface of software converter (runs 100% on CPU)
 */

#include "libcamera/internal/converter/converter_softw.h"

#include <sys/mman.h>
#include <sys/types.h>
#include <tuple>
#include <unistd.h>
#include <vector>

#include <libcamera/base/log.h>
#include <libcamera/base/signal.h>
#include <libcamera/base/thread.h>

#include <libcamera/formats.h>
#include <libcamera/geometry.h>
#include <libcamera/pixel_format.h>
#include <libcamera/stream.h>

#include "libcamera/internal/bayer_format.h"
#include "libcamera/internal/framebuffer.h"
#include "libcamera/internal/mapped_framebuffer.h"

#include <bitset>

namespace libcamera {

LOG_DECLARE_CATEGORY(Converter)

std::vector<PixelFormat> SwConverter::formats(PixelFormat input)
{
	std::vector<PixelFormat> pixelFormats;
	BayerFormat inputFormat = BayerFormat::fromPixelFormat(input);

	/* Only RAW10P is currently supported */
	if (inputFormat.bitDepth == 10 && (inputFormat.packing == BayerFormat::Packing::CSI2 || inputFormat.packing == BayerFormat::Packing::None))
		pixelFormats.push_back(formats::RGB888);

	if (pixelFormats.empty())
		LOG(Converter, Info)
			<< "Unsupported input format " << input.toString();

	return pixelFormats;
}

SizeRange SwConverter::sizes(const Size &input)
{
	if (input.width < 2 || input.height < 2) {
		LOG(Converter, Error)
			<< "Input format size too small: " << input.toString();
		return {};
	}

	return SizeRange(Size(input.width - 2, input.height - 2));
}

std::tuple<unsigned int, unsigned int>
SwConverter::strideAndFrameSize(const PixelFormat &pixelFormat,
				const Size &size)
{
	/* Only RGB888 output is currently supported */
	if (pixelFormat == formats::RGB888) {
		int stride = size.width * 3;
		return std::make_tuple(stride, stride * (size.height));
	}

	return std::make_tuple(0, 0);
}

int SwConverter::configure(const StreamConfiguration &inputCfg,
			   const std::vector<std::reference_wrapper<StreamConfiguration>> &outputCfgs)
{
	if (outputCfgs.size() != 1) {
		LOG(Converter, Error)
			<< "Unsupported number of output streams: "
			<< outputCfgs.size();
		return -EINVAL;
	}

	isp_ = std::make_unique<SwConverter::Isp>(this);

	return isp_->configure(inputCfg, outputCfgs[0]);
}

int SwConverter::Isp::configure(const StreamConfiguration &inputCfg,
				const StreamConfiguration &outputCfg)
{
	BayerFormat bayerFormat =
		BayerFormat::fromPixelFormat(inputCfg.pixelFormat);
	width_ = inputCfg.size.width;
	height_ = inputCfg.size.height;
	stride_ = inputCfg.stride;

	if (bayerFormat.bitDepth != 10 ||
	    (bayerFormat.packing != BayerFormat::Packing::CSI2 && bayerFormat.packing != BayerFormat::Packing::None) ||
	    width_ < 2 || height_ < 2) {
		LOG(Converter, Error) << "Input format "
				      << inputCfg.size << "-"
				      << inputCfg.pixelFormat
				      << "not supported";
		return -EINVAL;
	}

	Packed = bayerFormat.packing;

	switch (bayerFormat.order) {
	case BayerFormat::BGGR:
		red_shift_ = Point(0, 0);
		break;
	case BayerFormat::GBRG:
		red_shift_ = Point(1, 0);
		break;
	case BayerFormat::GRBG:
		red_shift_ = Point(0, 1);
		break;
	case BayerFormat::RGGB:
	default:
		red_shift_ = Point(1, 1);
		break;
	}

	if (outputCfg.size.width != width_ - 2 ||
	    outputCfg.size.height != height_ - 2 ||
	    outputCfg.stride != (width_ - 2) * 3 ||
	    outputCfg.pixelFormat != formats::RGB888) {
		LOG(Converter, Error)
			<< "Output format not supported";
		return -EINVAL;
	}

	LOG(Converter, Info) << "SwConverter configuration: "
			     << inputCfg.size << "-" << inputCfg.pixelFormat
			     << " -> "
			     << outputCfg.size << "-" << outputCfg.pixelFormat;

	/* set r/g/b gains to 1.0 until frame data collected */
	rNumerat_ = rDenomin_ = 1;
	bNumerat_ = bDenomin_ = 1;
	gNumerat_ = gDenomin_ = 1;

	r_avg = 1;
	g_avg = 1;
	b_avg = 1;

	blue_change = 1;
	red_change = 1;

	histRed_.resize(256, 0);
	histGreenRed_.resize(256, 0);
	histGreenBlue_.resize(256, 0);
	histBlue_.resize(256, 0);

	return 0;
}

int SwConverter::exportBuffers(unsigned int output, unsigned int count,
			       std::vector<std::unique_ptr<FrameBuffer>> *buffers)
{
	/* single output for now */
	if (output >= 1)
		return -EINVAL;

	return isp_->exportBuffers(count, buffers);
}

int SwConverter::Isp::exportBuffers(unsigned int count,
				    std::vector<std::unique_ptr<FrameBuffer>> *buffers)
{
	/* V4L2_PIX_FMT_BGR24 aka 'BGR3' for output: */
	unsigned int bufSize = (height_ - 2) * (width_ - 2) * 3;

	for (unsigned int i = 0; i < count; i++) {
		std::string name = "frame-" + std::to_string(i);

		const int ispFd = memfd_create(name.c_str(), 0);
		int ret = ftruncate(ispFd, bufSize);
		if (ret < 0) {
			LOG(Converter, Error) << "ftruncate() for memfd failed "
					      << strerror(-ret);
			return ret;
		}

		FrameBuffer::Plane outPlane;
		outPlane.fd = SharedFD(std::move(ispFd));
		outPlane.offset = 0;
		outPlane.length = bufSize;

		std::vector<FrameBuffer::Plane> planes{ outPlane };
		buffers->emplace_back(std::make_unique<FrameBuffer>(std::move(planes)));
	}

	return count;
}

int SwConverter::Isp::start()
{
	moveToThread(&thread_);
	thread_.start();
	return 0;
}

void SwConverter::Isp::stop()
{
	thread_.exit();
	thread_.wait();
}

int SwConverter::start()
{
	return isp_->start();
}

void SwConverter::stop()
{
	return isp_->stop();
}

int SwConverter::queueBuffers(FrameBuffer *input,
			      const std::map<unsigned int, FrameBuffer *> &outputs)
{
	unsigned int mask = 0;

	/*
	 * Validate the outputs as a sanity check: at least one output is
	 * required, all outputs must reference a valid stream and no two
	 * outputs can reference the same stream.
	 */
	if (outputs.empty())
		return -EINVAL;

	for (auto [index, buffer] : outputs) {
		if (!buffer)
			return -EINVAL;
		if (index >= 1) /* only single stream atm */
			return -EINVAL;
		if (mask & (1 << index))
			return -EINVAL;

		mask |= 1 << index;
	}

	process(input, outputs.at(0));

	return 0;
}

void SwConverter::process(FrameBuffer *input, FrameBuffer *output)
{
	isp_->invokeMethod(&SwConverter::Isp::process,
			   ConnectionTypeQueued, input, output);
}

void SwConverter::Isp::process(FrameBuffer *input, FrameBuffer *output)
{
	/* Copy metadata from the input buffer */
	FrameMetadata &metadata = output->_d()->metadata();
	metadata.status = input->metadata().status;
	metadata.sequence = input->metadata().sequence;
	metadata.timestamp = input->metadata().timestamp;

	MappedFrameBuffer in(input, MappedFrameBuffer::MapFlag::Read);
	MappedFrameBuffer out(output, MappedFrameBuffer::MapFlag::Write);
	if (!in.isValid() || !out.isValid()) {
		LOG(Converter, Error) << "mmap-ing buffer(s) failed";
		metadata.status = FrameMetadata::FrameError;
		converter_->outputBufferReady.emit(output);
		converter_->inputBufferReady.emit(input);
		return;
	}
	
	if(Packed == BayerFormat::Packing::CSI2){
		debayerP(out.planes()[0].data(), in.planes()[0].data());
	} 
	else if (Packed == BayerFormat::Packing::None)
	{
		debayerNP(out.planes()[0].data(), in.planes()[0].data());
		//debayerInfrarood(out.planes()[0].data(), in.planes()[0].data());
	} else {
		LOG(Converter, Error) << "Packing format not recognized.";
	}

	metadata.planes()[0].bytesused = out.planes()[0].size();

	converter_->agcDataReady.emit(bright_ratio_, too_bright_ratio_, histRed_, histGreenRed_, histGreenBlue_, histBlue_, histLuminance_);

	converter_->outputBufferReady.emit(output);
	converter_->inputBufferReady.emit(input);
}

#define BRIGHT_LVL	(200U << 8) /* for 0 to 255 range of values */
#define TOO_BRIGHT_LVL	(240U << 8) /* for 0 to 255 range of values */

#define RED_Y_MUL	77	/* 0.30 * 256 */
#define GREEN_Y_MUL	150	/* 0.59 * 256 */
#define BLUE_Y_MUL	29	/* 0.11 * 256 */

void SwConverter::Isp::debayerP(uint8_t *dst, const uint8_t *src)
{
	histRed_.clear();
	histGreenRed_.clear();
	histGreenBlue_.clear();
	histBlue_.clear();
	/* RAW10P input format is assumed */

	/* output buffer is in BGR24 format and is of (width-2)*(height-2) */

	int w_out = width_ - 2;
	int h_out = height_ - 2;

	unsigned long sumR = 0;
	unsigned long sumB = 0;
	unsigned long sumG = 0;

	unsigned long bright_sum = 0;
	unsigned long too_bright_sum = 0;

	std::vector<int> histRed(256, 0);
	std::vector<int> histGreenRed(256, 0);
	std::vector<int> histGreenBlue(256, 0);
	std::vector<int> histBlue(256, 0);
	std::vector<int> histLuminance(256, 0);

	for (int y = 0; y < h_out; y++) {
		const uint8_t *pin_base = src + (y + 1) * stride_;
		uint8_t *pout = dst + y * w_out * 3;
		int phase_y = (y + red_shift_.y) % 2;

		for (int x = 0; x < w_out; x++) {
			int phase_x = (x + red_shift_.x) % 2;
			int phase = 2 * phase_y + phase_x;

			/* x part of the offset in the input buffer: */
			int x_m1 = x + x / 4;		/* offset for (x-1) */
			int x_0 = x + 1 + (x + 1) / 4;	/* offset for x */
			int x_p1 = x + 2 + (x + 2) / 4;	/* offset for (x+1) */
			/* the colour component value to write to the output */
			unsigned val;
			/* Y value times 256 */
			unsigned y_val;

			switch (phase) {
			case 0: /* at R pixel */
				/* blue: ((-1,-1)+(1,-1)+(-1,1)+(1,1)) / 4 */
				val = ( *(pin_base + x_m1 - stride_)
					+ *(pin_base + x_p1 - stride_)
					+ *(pin_base + x_m1 + stride_)
					+ *(pin_base + x_p1 + stride_) ) >> 2;
				y_val = BLUE_Y_MUL * val;
				val = val * bNumerat_ / bDenomin_;
				*pout++ = (uint8_t)std::min(val, 0xffU);
				/* green: ((0,-1)+(-1,0)+(1,0)+(0,1)) / 4 */
				val = ( *(pin_base + x_0 - stride_)
					+ *(pin_base + x_p1)
					+ *(pin_base + x_m1)
					+ *(pin_base + x_0 + stride_) ) >> 2;
				val = val * gNumerat_ / gDenomin_;
				y_val += GREEN_Y_MUL * val;
				*pout++ = (uint8_t)std::min(val, 0xffU);
				/* red: (0,0) */
				val = *(pin_base + x_0);
				sumR += val;
				y_val += RED_Y_MUL * val;
				if (y_val > BRIGHT_LVL) ++bright_sum;
				if (y_val > TOO_BRIGHT_LVL) ++too_bright_sum;
				val = val * rNumerat_ / rDenomin_;
				histRed[std::min(val, 0xffU)] += 1;
				*pout++ = (uint8_t)std::min(val, 0xffU);
				break;
			case 1: /* at Gr pixel */
				/* blue: ((0,-1) + (0,1)) / 2 */
				val = ( *(pin_base + x_0 - stride_)
					+ *(pin_base + x_0 + stride_) ) >> 1;
				y_val = BLUE_Y_MUL * val;
				val = val * bNumerat_ / bDenomin_;
				*pout++ = (uint8_t)std::min(val, 0xffU);
				/* green: (0,0) */
				val = *(pin_base + x_0);
				sumG += val;
				y_val += GREEN_Y_MUL * val;
				val = val * gNumerat_ / gDenomin_;
				histGreenRed[std::min(val, 0xffU)] += 1;
				*pout++ = (uint8_t)std::min(val, 0xffU);
				/* red: ((-1,0) + (1,0)) / 2 */
				val = ( *(pin_base + x_m1)
					+ *(pin_base + x_p1) ) >> 1;
				y_val += RED_Y_MUL * val;
				if (y_val > BRIGHT_LVL) ++bright_sum;
				if (y_val > TOO_BRIGHT_LVL) ++too_bright_sum;
				val = val * rNumerat_ / rDenomin_;
				*pout++ = (uint8_t)std::min(val, 0xffU);
				break;
			case 2: /* at Gb pixel */
				/* blue: ((-1,0) + (1,0)) / 2 */
				val = ( *(pin_base + x_m1)
					+ *(pin_base + x_p1) ) >> 1;
				y_val = BLUE_Y_MUL * val;
				val = val * bNumerat_ / bDenomin_;
				*pout++ = (uint8_t)std::min(val, 0xffU);
				/* green: (0,0) */
				val = *(pin_base + x_0);
				sumG += val;
				y_val += GREEN_Y_MUL * val;
				val = val * gNumerat_ / gDenomin_;
				histGreenBlue[std::min(val, 0xffU)] += 1;
				*pout++ = (uint8_t)std::min(val, 0xffU);
				/* red: ((0,-1) + (0,1)) / 2 */
				val = ( *(pin_base + x_0 - stride_)
					+ *(pin_base + x_0 + stride_) ) >> 1;
				y_val += RED_Y_MUL * val;
				if (y_val > BRIGHT_LVL) ++bright_sum;
				if (y_val > TOO_BRIGHT_LVL) ++too_bright_sum;
				val = val * rNumerat_ / rDenomin_;
				*pout++ = (uint8_t)std::min(val, 0xffU);
				break;
			default: /* at B pixel */
				/* blue: (0,0) */
				val = *(pin_base + x_0);
				sumB += val;
				y_val = BLUE_Y_MUL * val;
				val = val * bNumerat_ / bDenomin_;
				histBlue[std::min(val, 0xffU)] += 1;
				*pout++ = (uint8_t)std::min(val, 0xffU);
				/* green: ((0,-1)+(-1,0)+(1,0)+(0,1)) / 4 */
				val = ( *(pin_base + x_0 - stride_)
					+ *(pin_base + x_p1)
					+ *(pin_base + x_m1)
					+ *(pin_base + x_0 + stride_) ) >> 2;
				y_val += GREEN_Y_MUL * val;
				val = val * gNumerat_ / gDenomin_;
				*pout++ = (uint8_t)std::min(val, 0xffU);
				/* red: ((-1,-1)+(1,-1)+(-1,1)+(1,1)) / 4 */
				val = ( *(pin_base + x_m1 - stride_)
					+ *(pin_base + x_p1 - stride_)
					+ *(pin_base + x_m1 + stride_)
					+ *(pin_base + x_p1 + stride_) ) >> 2;
				y_val += RED_Y_MUL * val;
				if (y_val > BRIGHT_LVL) ++bright_sum;
				if (y_val > TOO_BRIGHT_LVL) ++too_bright_sum;
				val = val * rNumerat_ / rDenomin_;
				*pout++ = (uint8_t)std::min(val, 0xffU);
			}
			// Add 1 to luminance value bin in luminance histogram
			histLuminance[std::min(y_val/256, 0xffU)] += 1;
		}
	}

	/* calculate the fractions of "bright" and "too bright" pixels */
	bright_ratio_ = (float)bright_sum / (h_out * w_out);
	too_bright_ratio_ = (float)too_bright_sum / (h_out * w_out);
	histRed_ = histRed;
	histGreenRed_ = histGreenRed;
	histGreenBlue_ = histGreenBlue;
	histBlue_ = histBlue;
	histLuminance_ = histLuminance;
{
	static int xxx = 75;
	if (--xxx == 0) {
	xxx = 75;
	LOG(Converter, Info) << "bright_ratio_ = " << bright_ratio_
			      << ", too_bright_ratio_ = " << too_bright_ratio_;
	}
}

	/* calculate red and blue gains for simple AWB */
	LOG(Converter, Debug) << "sumR = " << sumR
			      << ", sumB = " << sumB << ", sumG = " << sumG;

	sumG /= 2; /* the number of G pixels is twice as big vs R and B ones */

	/* normalize red, blue, and green sums to fit into 22-bit value */
	unsigned long fRed = sumR / 0x400000;
	unsigned long fBlue = sumB / 0x400000;
	unsigned long fGreen = sumG / 0x400000;
	unsigned long fNorm = std::max({ 1UL, fRed, fBlue, fGreen });
	sumR /= fNorm;
	sumB /= fNorm;
	sumG /= fNorm;

	LOG(Converter, Debug) << "fNorm = " << fNorm;
	LOG(Converter, Debug) << "Normalized: sumR = " << sumR
			      << ", sumB= " << sumB << ", sumG = " << sumG;

	/* make sure red/blue gains never exceed approximately 256 */
	unsigned long minDenom;
	rNumerat_ = (sumR + sumB + sumG) / 3;
	minDenom = rNumerat_ / 0x100;
	rDenomin_ = std::max(minDenom, sumR);
	bNumerat_ = rNumerat_;
	bDenomin_ = std::max(minDenom, sumB);
	gNumerat_ = rNumerat_;
	gDenomin_ = std::max(minDenom, sumG);

	LOG(Converter, Debug) << "rGain = [ "
			      << rNumerat_ << " / " << rDenomin_
			      << " ], bGain = [ " << bNumerat_ << " / " << bDenomin_
			      << " ], gGain = [ " << gNumerat_ << " / " << gDenomin_
			      << " (minDenom = " << minDenom << ")";
}

uint16_t* SwConverter::Isp::readByteFromCamera(const uint8_t *pin_base){
	return (uint16_t*)pin_base;
}

void SwConverter::Isp::debayerNP(uint8_t *dst, const uint8_t *src)
{
	histRed_.clear();
	histGreenRed_.clear();
	histGreenBlue_.clear();
	histBlue_.clear();
	// Debayer GRGB_IGIG_GBGR_IGIG

	/* output buffer is in BGR24 format and is of (width-2)*(height-2) */

	int w_out = width_ - 2;
	int h_out = height_ - 2;

	unsigned long sumR = 0;
	unsigned long sumB = 0;
	unsigned long sumG = 0;


	unsigned long red_count = 0;
	unsigned long green_count = 0;
	unsigned long blue_count = 0;
	

	unsigned long bright_sum = 0;
	unsigned long too_bright_sum = 0;

	std::vector<int> histRed(256, 0);
	std::vector<int> histGreen(256, 0);
	std::vector<int> histBlue(256, 0);
	std::vector<int> histLuminance(256, 0);

	for (int y = 0; y < h_out; y++){
		uint8_t *pout = dst + y * w_out * 3;
		const uint8_t *pin_base = src + (y + 1) * stride_;
		int phase_y = (y) % 4;
		for (int x = 0; x < w_out; x++) {
			int phase_x = (x) % 4;
	 		//int phase = 2 * phase_y + phase_x;
	 		int phase = 4 * phase_y + phase_x;
			(void) phase;

			int x_m2 = 2 * x 	  ;		/* offset for (x-2) */
			int x_m1 = 2 * (x + 1);	/* offset for (x-1) */
			int x_0  = 2 * (x + 2);	/* offset for x     */
			int x_p1 = 2 * (x + 3);	/* offset for (x+1) */
			int x_p2 = 2 * (x + 4);	/* offset for (x+2) */
			(void)x_m2;		
			(void)x_m1;
			(void)x_0;
			(void)x_p1;
			(void)x_p2;		
			(void)pin_base;

			/* the colour component value to write to the output */
	 		unsigned val;
			(void) val;
			unsigned red = 0,green = 0,blue = 0;
	 		/* Y value times 256 */
	 		unsigned y_val = 0;

			switch(phase){
				case 0:// First green Top Left (0,0)
					/* blue: (-1,0) */
					blue = *readByteFromCamera(pin_base + x_m1);
					green = *readByteFromCamera(pin_base +x_0);
					red = *readByteFromCamera(pin_base + x_p1);
					sumG += green;
					green_count++;
					break;
				case 1:// Red (1,0)
					/* blue: ((2,0) + (-2,0) + (0,2) + (0,-2))/4 */
					blue = (*readByteFromCamera(pin_base + x_p2) +
							 *readByteFromCamera(pin_base + x_m2)) >> 1;

					/* green: ((1,0) + (-1,0) + (0,1) + (0,-1))/4 */
					green = (*readByteFromCamera(pin_base + x_p1) +
							 *readByteFromCamera(pin_base + x_m1) +
							 *readByteFromCamera(pin_base + x_0 + stride_) +
							 *readByteFromCamera(pin_base + x_0 - stride_)) >> 2;

					red = *readByteFromCamera(pin_base + x_0); /* red: (1,0) */
					sumR += red;
					red_count++;
					break;
				case 2:// Green (2,0)
					blue = *readByteFromCamera(pin_base + x_p1);/* blue: (1,0) */
					green = *readByteFromCamera(pin_base + x_0);/* green: (0,0) */
					red = *readByteFromCamera(pin_base + x_m1);/* red: (-1,0) */
					sumG += green;
					green_count++;
					break;

				case 3:// Blue (3,0)
					/* blue: (1,0) */
					blue = *readByteFromCamera(pin_base + x_0);
					/* green: ((1,0) + (-1,0) + (0,1) + (0,-1))/4 */
					green = (*readByteFromCamera(pin_base + x_p1) +
							 *readByteFromCamera(pin_base + x_m1) +
							 *readByteFromCamera(pin_base + x_0 + stride_) +
							 *readByteFromCamera(pin_base + x_0 - stride_)) >> 2;
					/* red: (-1,0) */
					red = (*readByteFromCamera(pin_base + x_p2) +
						   *readByteFromCamera(pin_base + x_m2)) >> 1;
					sumB += blue;
					blue_count++;
					break;
				case 4:// Infrared
					/* blue[(1, 1), (-1, -1)] */
					blue = (*readByteFromCamera(pin_base + x_p1 + stride_) + *readByteFromCamera(pin_base + x_m1 - stride_)) >> 1;
					/* green[(1, 0), (-1, 0), (0, 1), (0, -1)] */
					green = (*readByteFromCamera(pin_base + x_p1) + *readByteFromCamera(pin_base + x_m1) + *readByteFromCamera(pin_base + x_0 + stride_) + *readByteFromCamera(pin_base + x_0 - stride_)) >> 2;
					/* red[(-1, 1), (1, -1)] */
					red = (*readByteFromCamera(pin_base + x_m1 + stride_) + *readByteFromCamera(pin_base + x_p1 - stride_)) >> 1;
					break;
					
				case 5:// Green
					/* blue[(0, 1)] */
					blue = (*readByteFromCamera(pin_base + x_0 + stride_));   
					/* green[(0, 0)] */
					green = (*readByteFromCamera(pin_base + x_0));
					/* red[(0, -1)] */
					red = (*readByteFromCamera(pin_base + x_0 - stride_));
					sumG += green;
					green_count++;
					break;
				case 6:// Infrared
					/* blue[(-1, 1), (1, -1)] */
					blue = (*readByteFromCamera(pin_base + x_m1 + stride_) + *readByteFromCamera(pin_base + x_p1 - stride_)) >> 1;
					/* green[(1, 0), (-1, 0), (0, -1), (0, 1)] */
					green = (*readByteFromCamera(pin_base + x_p1) + *readByteFromCamera(pin_base + x_m1) + *readByteFromCamera(pin_base + x_0 - stride_) + *readByteFromCamera(pin_base + x_0 + stride_)) >> 2;
					/* red[(1, 1), (-1, -1)] */
					red = (*readByteFromCamera(pin_base + x_p1 + stride_) + *readByteFromCamera(pin_base + x_m1 - stride_)) >> 1;
					break;
				case 7:// Green
					/* blue[(0, -1)] */
					blue = (*readByteFromCamera(pin_base + x_0 - stride_));   
					/* green[(0, 0)] */
					green = (*readByteFromCamera(pin_base + x_0));
					/* red[(0, 1)] */
					red = (*readByteFromCamera(pin_base + x_0 + stride_));
					sumG += green;
					green_count++;
					break;
				case 8:// Green
					/* blue[(1, 0)] */
					blue = (*readByteFromCamera(pin_base + x_p1));
					/* green[(0, 0)] */
					green = (*readByteFromCamera(pin_base + x_0));
					/* red[(-1, 0)] */
					red = (*readByteFromCamera(pin_base + x_m1));
					sumG += green;
					green_count++;
					break;
				case 9:// Blue
					/* blue[(0, 0)] */
					blue = (*readByteFromCamera(pin_base + x_0));
					/* green[(1, 0), (-1, 0), (0, -1), (0, 1)] */
					green = (*readByteFromCamera(pin_base + x_p1) + *readByteFromCamera(pin_base + x_m1) + *readByteFromCamera(pin_base + x_0 - stride_) + *readByteFromCamera(pin_base + x_0 + stride_)) >> 2;
					/* red[(-2, 0), (2, 0)] */
					red = (*readByteFromCamera(pin_base + x_m2) + *readByteFromCamera(pin_base + x_p2)) >> 1;
					sumB += blue;
					blue_count++;
					break;
				case 10:// Green
					/* blue[(-1, 0)] */
					blue = (*readByteFromCamera(pin_base + x_m1));
					/* green[(0, 0)] */
					green = (*readByteFromCamera(pin_base + x_0));
					/* red[(1, 0)] */
					red = (*readByteFromCamera(pin_base + x_p1));
					sumG += green;
					green_count++;
					break;
				case 11:// Red
					/* blue[(-2, 0), (2, 0)] */
					blue = (*readByteFromCamera(pin_base + x_m2) + *readByteFromCamera(pin_base + x_p2)) >> 1;
					/* green[(1, 0), (-1, 0), (0, -1), (0, 1)] */
					green = (*readByteFromCamera(pin_base + x_p1) + *readByteFromCamera(pin_base + x_m1) + *readByteFromCamera(pin_base + x_0 - stride_) + *readByteFromCamera(pin_base + x_0 + stride_)) >> 2;
					/* red[(0, 0)] */
					red = (*readByteFromCamera(pin_base + x_0));
					sumR += red;
					red_count++;
					break;
				case 12:// Infrared
					/* blue[(1, -1), (-1, 1)] */
					blue = (*readByteFromCamera(pin_base + x_p1 - stride_) + *readByteFromCamera(pin_base + x_m1 + stride_)) >> 1;
					/* green[(1, 0), (-1, 0), (0, -1), (0, 1)] */
					green = (*readByteFromCamera(pin_base + x_p1) + *readByteFromCamera(pin_base + x_m1) + *readByteFromCamera(pin_base + x_0 - stride_) + *readByteFromCamera(pin_base + x_0 + stride_)) >> 2;
					/* red[(1, 1), (-1, -1)] */
					red = (*readByteFromCamera(pin_base + x_p1 + stride_) + *readByteFromCamera(pin_base + x_m1 - stride_)) >> 1;
					break;
				case 13:// Green
					/* blue[(0, -1)] */
					blue = (*readByteFromCamera(pin_base + x_0 - stride_));   
					/* green[(0, 0)] */
					green = (*readByteFromCamera(pin_base + x_0));
					/* red[(0, 1)] */
					red = (*readByteFromCamera(pin_base + x_0 + stride_));
					sumG += green;
					green_count++;
					break;
				case 14:// Infrared
					/* blue[(1, 1), (-1, -1)] */
					blue = (*readByteFromCamera(pin_base + x_p1 + stride_) + *readByteFromCamera(pin_base + x_m1 - stride_)) >> 1;
					/* green[(1, 0), (-1, 0), (0, -1), (0, 1)] */
					green = (*readByteFromCamera(pin_base + x_p1) + *readByteFromCamera(pin_base + x_m1) + *readByteFromCamera(pin_base + x_0 - stride_) + *readByteFromCamera(pin_base + x_0 + stride_)) >> 2;
					/* red[(1, -1), (-1, 1)] */
					red = (*readByteFromCamera(pin_base + x_p1 - stride_) + *readByteFromCamera(pin_base + x_m1 + stride_)) >> 1;
					break;
				case 15:// Green
					/* blue[(0, 1)] */
					blue = (*readByteFromCamera(pin_base + x_0 + stride_));   
					/* green[(0, 0)] */
					green = (*readByteFromCamera(pin_base + x_0));
					/* red[(0, -1)] */
					red = (*readByteFromCamera(pin_base + x_0 - stride_));
					sumG += green;
					green_count++;
					break;
				default:
					red = 0;
					green = 0;
					blue = 0;
				break;

			}

			green = green/4;
			red = red/4;
			blue = blue/4;

			y_val = BLUE_Y_MUL * blue;
			y_val += GREEN_Y_MUL * green;
			y_val += RED_Y_MUL * red;

			//sumG;
			// sumR += red;
			// sumB += blue;

			// Gray World Theory AWB
			//green = green * g_avg / g_avg; useless calculation
			//Corrected Red = Red * R-avg / G-avg
			
			red = (int)(red * red_change);
			//Corrected Blue = Blue * B-avg / G-avg
			blue = (int)(blue * blue_change);

			//LOG(Converter,Debug) << "(r_avg / g_avg): " << ((float)r_avg / g_avg);

			// *pout++ = (uint8_t)std::min(blue, 0xffU);
			// *pout++ = (uint8_t)std::min(green, 0xffU);
			// *pout++ = (uint8_t)std::min(red, 0xffU);

			*pout++ = (uint8_t)blue;
			*pout++ = (uint8_t)green;
			*pout++ = (uint8_t)red;

			if (y_val > BRIGHT_LVL) ++bright_sum;
			if (y_val > TOO_BRIGHT_LVL) ++too_bright_sum;

			histRed[std::min(blue, 0xffU)] += 1;
			histGreen[std::min(green, 0xffU)] += 1;
			histBlue[std::min(red, 0xffU)] += 1;
			histLuminance[std::min(y_val/256, 0xffU)] += 1;
		}
	}
	
	/* calculate the fractions of "bright" and "too bright" pixels */
	bright_ratio_ = (float)bright_sum / (h_out * w_out);
	too_bright_ratio_ = (float)too_bright_sum / (h_out * w_out);
	histRed_ = histRed;
	histGreenRed_ = histGreen;
	histGreenBlue_ = histGreen;
	histBlue_ = histBlue;
	histLuminance_ = histLuminance;
{
	static int xxx = 75;
	if (--xxx == 0) {
	xxx = 75;
	LOG(Converter, Info) << "bright_ratio_ = " << bright_ratio_
			      << ", too_bright_ratio_ = " << too_bright_ratio_;
	}
}

	/* calculate red, blue and green avg for simple AWB*/
	g_avg = (float)sumG / green_count;
	r_avg = (float)sumR / red_count;
	b_avg = (float)sumB / blue_count;

	red_change = ((float)r_avg / g_avg);
	blue_change = ((float)b_avg / g_avg);


	LOG(Converter,Debug) << "(r_avg / g_avg): " << red_change;
	LOG(Converter,Debug) << "(b_avg / g_avg): " << blue_change;
	/* calculate red and blue gains for simple AWB */
	// LOG(Converter, Debug) << "sumR = " << sumR
	// 		      << ", sumB = " << sumB << ", sumG = " << sumG;


	// /* normalize red, blue, and green sums to fit into 22-bit value */
	// unsigned long fRed = sumR / 0x400000;
	// unsigned long fBlue = sumB / 0x400000;
	// unsigned long fGreen = sumG / 0x400000;
	// unsigned long fNorm = std::max({ 1UL, fRed, fBlue, fGreen });
	// sumR /= fNorm;
	// sumB /= fNorm;
	// sumG /= fNorm;

	// LOG(Converter, Debug) << "fNorm = " << fNorm;
	// LOG(Converter, Debug) << "Normalized: sumR = " << sumR
	// 		      << ", sumB= " << sumB << ", sumG = " << sumG;

	// /* make sure red/blue gains never exceed approximately 256 */
	// unsigned long minDenom;
	// rNumerat_ = (sumR + sumB + sumG) / 3;
	// minDenom = rNumerat_ / 0x100;
	// rDenomin_ = std::max(minDenom, sumR);
	// bNumerat_ = rNumerat_;
	// bDenomin_ = std::max(minDenom, sumB);
	// gNumerat_ = rNumerat_;
	// gDenomin_ = std::max(minDenom, sumG);

	// LOG(Converter, Debug) << "rGain = [ "
	// 		      << rNumerat_ << " / " << rDenomin_
	// 		      << " ], bGain = [ " << bNumerat_ << " / " << bDenomin_
	// 		      << " ], gGain = [ " << gNumerat_ << " / " << gDenomin_
	// 		      << " (minDenom = " << minDenom << ")";
}




void SwConverter::Isp::debayerInfrarood(uint8_t *dst, const uint8_t *src)
{
	histRed_.clear();
	histGreenRed_.clear();
	histGreenBlue_.clear();
	histBlue_.clear();
	// Debayer GRGB_IGIG_GBGR_IGIG

	/* output buffer is in BGR24 format and is of (width-2)*(height-2) */

	int w_out = width_ - 2;
	int h_out = height_ - 2;

	unsigned long sumR = 0;
	unsigned long sumB = 0;
	unsigned long sumG = 0;

	unsigned long bright_sum = 0;
	unsigned long too_bright_sum = 0;

	std::vector<int> histRed(256, 0);
	std::vector<int> histGreen(256, 0);
	std::vector<int> histBlue(256, 0);
	std::vector<int> histLuminance(256, 0);

	for (int y = 0; y < h_out; y++){
		uint8_t *pout = dst + y * w_out * 3;
		const uint8_t *pin_base = src + (y + 1) * stride_;
		int phase_y = (y) % 2;
		for (int x = 0; x < w_out; x++) {
			int phase_x = (x) % 2;
	 		//int phase = 2 * phase_y + phase_x;
	 		int phase = 2 * phase_y + phase_x;
			(void) phase;

			int x_m2 = 2 * x 	  ;		/* offset for (x-2) */
			int x_m1 = 2 * (x + 1);	/* offset for (x-1) */
			int x_0  = 2 * (x + 2);	/* offset for x     */
			int x_p1 = 2 * (x + 3);	/* offset for (x+1) */
			int x_p2 = 2 * (x + 4);	/* offset for (x+2) */
			(void)x_m2;		
			(void)x_m1;
			(void)x_0;
			(void)x_p1;
			(void)x_p2;		
			(void)pin_base;

			/* the colour component value to write to the output */
	 		unsigned val;
			(void) val;
			unsigned red = 0,green = 0,blue = 0;
	 		/* Y value times 256 */
	 		unsigned y_val = 0;

			switch(phase){
				case 0:// First green Top Left (0,0)
					/* blue: (-1,0) */
					blue = *readByteFromCamera(pin_base + stride_ + x_0);
					green = *readByteFromCamera(pin_base + stride_ + x_0);
					red = *readByteFromCamera(pin_base + stride_ + x_0);
					//sumG += green;
					break;
				case 1:// Red (1,0)
					blue = *readByteFromCamera(pin_base + stride_ + x_m1);
					green = *readByteFromCamera(pin_base + stride_ + x_m1);
					red = *readByteFromCamera(pin_base + stride_ + x_m1);
					break;
				case 2:// Green (2,0)
					blue = *readByteFromCamera(pin_base + x_0);
					green = *readByteFromCamera(pin_base + x_0);
					red = *readByteFromCamera(pin_base + x_0);
					break;

				case 3:// Blue (3,0)
					blue = *readByteFromCamera(pin_base + x_m1);
					green = *readByteFromCamera(pin_base + x_m1);
					red = *readByteFromCamera(pin_base + x_m1);
					break;
				default:
					red = 255;
					green = 0;
					blue = 0;
				break;

			}

			green = green/4;
			red = red/4;
			blue = blue/4;

			y_val = BLUE_Y_MUL * blue;
			y_val += GREEN_Y_MUL * green;
			y_val += RED_Y_MUL * red;

			sumG += green;
			sumR += red;
			sumB += blue;

			// White balance is nog niet erg lekker

			// *pout++ = (uint8_t)std::min(blue, 0xffU);
			// *pout++ = (uint8_t)std::min(green, 0xffU);
			// *pout++ = (uint8_t)std::min(red, 0xffU);

			*pout++ = (uint8_t)blue;
			*pout++ = (uint8_t)green;
			*pout++ = (uint8_t)red;

			if (y_val > BRIGHT_LVL) ++bright_sum;
			if (y_val > TOO_BRIGHT_LVL) ++too_bright_sum;

			histRed[std::min(blue, 0xffU)] += 1;
			histGreen[std::min(green, 0xffU)] += 1;
			histBlue[std::min(red, 0xffU)] += 1;
			histLuminance[std::min(y_val/256, 0xffU)] += 1;
		}
	}
	
	/* calculate the fractions of "bright" and "too bright" pixels */
	bright_ratio_ = (float)bright_sum / (h_out * w_out);
	too_bright_ratio_ = (float)too_bright_sum / (h_out * w_out);
	histRed_ = histRed;
	histGreenRed_ = histGreen;
	histGreenBlue_ = histGreen;
	histBlue_ = histBlue;
	histLuminance_ = histLuminance;
{
	static int xxx = 75;
	if (--xxx == 0) {
	xxx = 75;
	LOG(Converter, Info) << "bright_ratio_ = " << bright_ratio_
			      << ", too_bright_ratio_ = " << too_bright_ratio_;
	}
}

	/* calculate red and blue gains for simple AWB */
	LOG(Converter, Debug) << "sumR = " << sumR
			      << ", sumB = " << sumB << ", sumG = " << sumG;


	/* normalize red, blue, and green sums to fit into 22-bit value */
	unsigned long fRed = sumR / 0x400000;
	unsigned long fBlue = sumB / 0x400000;
	unsigned long fGreen = sumG / 0x400000;
	unsigned long fNorm = std::max({ 1UL, fRed, fBlue, fGreen });
	sumR /= fNorm;
	sumB /= fNorm;
	sumG /= fNorm;

	LOG(Converter, Debug) << "fNorm = " << fNorm;
	LOG(Converter, Debug) << "Normalized: sumR = " << sumR
			      << ", sumB= " << sumB << ", sumG = " << sumG;

	/* make sure red/blue gains never exceed approximately 256 */
	unsigned long minDenom;
	rNumerat_ = (sumR + sumB + sumG) / 3;
	minDenom = rNumerat_ / 0x100;
	rDenomin_ = std::max(minDenom, sumR);
	bNumerat_ = rNumerat_;
	bDenomin_ = std::max(minDenom, sumB);
	gNumerat_ = rNumerat_;
	gDenomin_ = std::max(minDenom, sumG);

	LOG(Converter, Debug) << "rGain = [ "
			      << rNumerat_ << " / " << rDenomin_
			      << " ], bGain = [ " << bNumerat_ << " / " << bDenomin_
			      << " ], gGain = [ " << gNumerat_ << " / " << gDenomin_
			      << " (minDenom = " << minDenom << ")";
}


} /* namespace libcamera */
