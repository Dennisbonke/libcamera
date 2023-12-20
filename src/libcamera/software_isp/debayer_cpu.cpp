/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2023, Linaro Ltd
 * Copyright (C) 2023, Red Hat Inc.
 *
 * Authors:
 * Hans de Goede <hdegoede@redhat.com> 
 *
 * debayer_cpu.cpp - CPU based debayering class
 */

#include "libcamera/internal/software_isp/debayer_cpu.h"

#include <math.h>

#include <libcamera/formats.h>

#include "libcamera/internal/bayer_format.h"
#include "libcamera/internal/framebuffer.h"
#include "libcamera/internal/mapped_framebuffer.h"

namespace libcamera {

DebayerCpu::DebayerCpu(std::unique_ptr<SwStatsCpu> stats)
	: stats_(std::move(stats)), gamma_correction_(1.0)
{
	/* Initialize gamma to 1.0 curve */
	for (int i = 0; i < 1024; i++)
		gamma_[i] = i / 4;
}

/**
 * \brief Struct that holds the context for debayering lines.
 */
struct ctxt_8bit_src {
	/**
	 * \brief Pointer to the previous line.
	 */
	const uint8_t *prev;
	/**
	 * \brief Pointer to the current line.
	 */
	const uint8_t *curr;
	/**
	 * \brief Pointer to the next line.
	 */
	const uint8_t *next;

	/**
	 * Pointer to the red color lookup table for awb + gamma.
	 */
	const uint8_t *red;
	/**
	 * Pointer to the green color lookup table for awb + gamma.
	 */
	const uint8_t *green;
	/**
	 * Pointer to the blue color lookup table for awb + gamma.
	 */
	const uint8_t *blue;
};

static inline void bggr8_bgr888(const struct ctxt_8bit_src &c, uint8_t *&dst, int x, int p, int n)
{
	*dst++ = c.blue[c.curr[x]];
	*dst++ = c.green[(c.prev[x] + c.curr[x - p] + c.curr[x + n] + c.next[x]) / 4];
	*dst++ = c.red[(c.prev[x - p] + c.prev[x + n] + c.next[x - p]  + c.next[x + n]) / 4];
}

static inline void grbg8_bgr888(const struct ctxt_8bit_src &c, uint8_t *&dst, int x, int p, int n)
{
	*dst++ = c.blue[(c.prev[x] + c.next[x]) / 2];
	*dst++ = c.green[c.curr[x]];
	*dst++ = c.red[(c.curr[x - p] + c.curr[x + n]) / 2];
}

static inline void gbrg8_bgr888(const struct ctxt_8bit_src &c, uint8_t *&dst, int x, int p, int n)
{
	*dst++ = c.blue[(c.curr[x - p] + c.curr[x + n]) / 2];
	*dst++ = c.green[c.curr[x]];
	*dst++ = c.red[(c.prev[x] + c.next[x]) / 2];
}

static inline void rggb8_bgr888(const struct ctxt_8bit_src &c, uint8_t *&dst, int x, int p, int n)
{
	*dst++ = c.blue[(c.prev[x - p] + c.prev[x + n] + c.next[x - p]  + c.next[x + n]) / 4];
	*dst++ = c.green[(c.prev[x] + c.curr[x - p] + c.curr[x + n] + c.next[x]) / 4];
	*dst++ = c.red[c.curr[x]];
}

void DebayerCpu::debayer10P_BGBG_BGR888(uint8_t *dst, const uint8_t *src)
{
	const int width_in_bytes = window_.width * 5 / 4;
	struct ctxt_8bit_src c = {
		src - inputConfig_.stride, src, src + inputConfig_.stride,
		red_, green_, blue_ };

	/*
	 * For the first pixel getting a pixel from the previous column uses
	 * x - 2 to skip the 5th byte with least-significant bits for 4 pixels.
	 * Same for last pixel (uses x + 2) and looking at the next column.
	 * x++ in the for-loop skips the 5th byte with 4 x 2 lsb-s for 10bit packed.
	 */
	for (int x = 0; x < width_in_bytes; x++) {
		/* Even pixel */
		bggr8_bgr888(c, dst, x++, 2, 1);
		/* Odd pixel BGGR -> GBRG */
		gbrg8_bgr888(c, dst, x++, 1, 1);
		/* Same thing for next 2 pixels */
		bggr8_bgr888(c, dst, x++, 1, 1);
		gbrg8_bgr888(c, dst, x++, 1, 2);
	}
}

void DebayerCpu::debayer10P_GRGR_BGR888(uint8_t *dst, const uint8_t *src)
{
	const int width_in_bytes = window_.width * 5 / 4;
	struct ctxt_8bit_src c = {
		src - inputConfig_.stride, src, src + inputConfig_.stride,
		red_, green_, blue_ };

	for (int x = 0; x < width_in_bytes; x++) {
		/* Even pixel */
		grbg8_bgr888(c, dst, x++, 2, 1);
		/* Odd pixel GRBG -> RGGB */
		rggb8_bgr888(c, dst, x++, 1, 1);
		/* Same thing for next 2 pixels */
		grbg8_bgr888(c, dst, x++, 1, 1);
		rggb8_bgr888(c, dst, x++, 1, 2);
	}
}

void DebayerCpu::debayer10P_GBGB_BGR888(uint8_t *dst, const uint8_t *src)
{
	const int width_in_bytes = window_.width * 5 / 4;
	struct ctxt_8bit_src c = {
		src - inputConfig_.stride, src, src + inputConfig_.stride,
		red_, green_, blue_ };

	for (int x = 0; x < width_in_bytes; x++) {
		/* Even pixel */
		gbrg8_bgr888(c, dst, x++, 2, 1);
		/* Odd pixel GBGR -> BGGR */
		bggr8_bgr888(c, dst, x++, 1, 1);
		/* Same thing for next 2 pixels */
		gbrg8_bgr888(c, dst, x++, 1, 1);
		bggr8_bgr888(c, dst, x++, 1, 2);
	}
}

void DebayerCpu::debayer10P_RGRG_BGR888(uint8_t *dst, const uint8_t *src)
{
	const int width_in_bytes = window_.width * 5 / 4;
	struct ctxt_8bit_src c = {
		src - inputConfig_.stride, src, src + inputConfig_.stride,
		red_, green_, blue_ };

	for (int x = 0; x < width_in_bytes; x++) {
		/* Even pixel */
		rggb8_bgr888(c, dst, x++, 2, 1);
		/* Odd pixel RGGB -> GRBG*/
		grbg8_bgr888(c, dst, x++, 1, 1);
		/* Same thing for next 2 pixels */
		rggb8_bgr888(c, dst, x++, 1, 1);
		grbg8_bgr888(c, dst, x++, 1, 2);
	}
}

void DebayerCpu::debayer10_BGBG_BGR888(uint8_t *dst, const uint8_t *src)
{
	const uint16_t *prev = (const uint16_t *)(src - inputConfig_.stride);
	const uint16_t *curr = (const uint16_t *)src;
	const uint16_t *next = (const uint16_t *)(src + inputConfig_.stride);

	for (int x = 0; x < (int)window_.width; x++) {
		/*
		 * BGBG line even pixel: RGR
		 *                       GBG
		 *                       RGR
		 */
		*dst++ = blue_[curr[x] / 4];
		*dst++ = green_[(prev[x] + curr[x - 1] + curr[x + 1] + next[x]) / 16];
		*dst++ = red_[(prev[x - 1] + prev[x + 1] + next[x - 1]  + next[x + 1]) / 16];
		x++;

		/*
		 * BGBG line odd pixel: GRG
		 *                      BGB
		 *                      GRG
		 */
		*dst++ = blue_[(curr[x - 1] + curr[x + 1]) / 8];
		*dst++ = green_[curr[x] / 4];
		*dst++ = red_[(prev[x] + next[x]) / 8];
	}
}

void DebayerCpu::debayer10_GRGR_BGR888(uint8_t *dst, const uint8_t *src)
{
	const uint16_t *prev = (const uint16_t *)(src - inputConfig_.stride);
	const uint16_t *curr = (const uint16_t *)src;
	const uint16_t *next = (const uint16_t *)(src + inputConfig_.stride);

	for (int x = 0; x < (int)window_.width; x ++) {
		/*
		 * GRGR line even pixel: GBG
		 *                       RGR
		 *                       GBG
		 */
		*dst++ = blue_[(prev[x] + next[x]) / 8];
		*dst++ = green_[curr[x] / 4];
		*dst++ = red_[(curr[x - 1] + curr[x + 1]) / 8];
		x++;

		/*
		 * GRGR line odd pixel: BGB
		 *                      GRG
		 *                      BGB
		 */
		*dst++ = blue_[(prev[x - 1] + prev[x + 1] + next[x - 1] + next[x + 1]) / 16];
		*dst++ = green_[(prev[x] + curr[x - 1] + curr[x + 1] + next[x]) / 16];
		*dst++ = red_[curr[x] / 4];
	}
}

void DebayerCpu::debayerIGIG10Line0(uint8_t *dst, const uint8_t *src)
{
	/* Pointers to previous, current and next lines */
	const uint16_t *prev = (const uint16_t *) (src - inputConfig_.stride);
	const uint16_t *curr = (const uint16_t *) (src);
	const uint16_t *next = (const uint16_t *) (src + inputConfig_.stride);
	(void) prev; (void) curr; (void) next;

	for (int x = 0; x < (int)window_.width;) {
		/*
		 * IGIG line even pixel: IGIGI
		 * 						 GBGRG
		 *                       IGIGI
		 *                       GRGBG
		 * 						 IGIGI
		 * Write BGR
		 */
		*dst++ = blue_[((prev[x - 1] + next[x + 1])/8)];
		*dst++ = green_[((curr[x - 1] + curr[x + 1] + prev[x] + next[x])/16) ];
		*dst++ = red_[((prev[x + 1] + next[x - 1])/8)];
		x++;

		/*
		 * IGIG line even pixel: GIGIG
		 * 						 BGRGB
		 *                       GIGIG
		 *                       RGBGR
		 * 						 GIGIG
		 * Write BGR
		 */
		*dst++ = blue_[next[x]/4];
		*dst++ = green_[curr[x]/4];
		*dst++ = red_[prev[x]/4];
		x++;

		/*
		 * IGIG line even pixel: IGIGI
		 * 						 GRGBG
		 *                       IGIGI
		 *                       GBGRG
		 * 						 IGIGI
		 * Write BGR
		 */
		*dst++ = blue_[((prev[x + 1] + next[x - 1])/8)];
		*dst++ = green_[((curr[x - 1] + curr[x + 1] + prev[x] + next[x])/16) ];
		*dst++ = red_[((prev[x - 1] + next[x + 1])/8)];
		x++;

		/*
		 * IGIG line even pixel: GIGIG
		 * 						 BGRGB
		 *                       GIGIG
		 *                       RGBGR
		 * 						 GIGIG
		 * Write BGR
		 */
		*dst++ = blue_[prev[x]/4];
		*dst++ = green_[curr[x]/4];
		*dst++ = red_[next[x]/4];
		x++;
	}
}

void DebayerCpu::debayerGBGR10Line1(uint8_t *dst, const uint8_t *src)
{
	/* Pointers to previous, current and next lines */
	const uint16_t *prev2 = (const uint16_t *)(src - inputConfig_.stride * 2);
	const uint16_t *prev = (const uint16_t *) (src - inputConfig_.stride);
	const uint16_t *curr = (const uint16_t *) (src);
	const uint16_t *next = (const uint16_t *) (src + inputConfig_.stride);
	const uint16_t *next2 = (const uint16_t *)(src + inputConfig_.stride * 2);
	(void) prev2;(void) prev; (void) curr; (void) next; (void) next2;

	for (int x = 0; x < (int)window_.width;) {
		/*
		 * BGBG line even pixel: GBGRG
		 * 						 IGIGI
		 *                       GRGBG
		 *                       IGIGI
		 * 						 GBGRG
		 * Write BGR
		 */
		*dst++ = blue_[curr[x + 1]/4];
		*dst++ = green_[curr[x]/4];
		*dst++ = red_[curr[x - 1]/4];
		x++;

		/*
		 * BGBG line even pixel: BGRGB
		 * 						 GIGIG
		 *                       RGBGR
		 *                       GIGIG
		 * 						 BGRGB
		 * Write BGR
		 */
		*dst++ = blue_[curr[x]/4];
		*dst++ = green_[((curr[x - 1] + curr[x + 1] + prev[x] + next[x])/16)];
		*dst++ = red_[((curr[x - 2] + curr[x + 2] + prev2[x] + next2[x])/16)];
		x++;

		/*
		 * BGBG line even pixel: GRGBG
		 * 						 IGIGI
		 *                       GBGRG
		 *                       IGIGI
		 * 						 GRGBG
		 * Write BGR
		 */
		*dst++ = blue_[curr[x - 1]/4];
		*dst++ = green_[curr[x]/4];
		*dst++ = red_[curr[x + 1]/4];
		x++;

		/*
		 * BGBG line even pixel: RGBGR
		 * 						 GIGIG
		 *                       BGRGB
		 *                       GIGIG
		 * 						 RGBGR
		 * Write BGR
		 */
		*dst++ = blue_[((curr[x - 2] + curr[x + 2] + prev2[x] + next2[x])/16)];
		*dst++ = green_[((curr[x - 1] + curr[x + 1] + prev[x] + next[x])/16)];
		*dst++ = red_[curr[x]/4];
		x++;	
	}
}

void DebayerCpu::debayerIGIG10Line2(uint8_t *dst, const uint8_t *src)
{
	/* Pointers to previous, current and next lines */
	const uint16_t *prev = (const uint16_t *)(src - inputConfig_.stride);
	const uint16_t *curr = (const uint16_t *)src;
	const uint16_t *next = (const uint16_t *)(src + inputConfig_.stride);
	(void) prev; (void) curr; (void) next;
	
	for (int x = 0; x < (int)window_.width;) {
		/*
		 * IGIG line even pixel: IGIGI
		 * 						 GRGBG
		 *                       IGIGI
		 *                       GBGRG
		 * 						 IGIGI
		 * Write BGR
		 */
		*dst++ = blue_[((prev[x + 1] + next[x - 1])/8)];
		*dst++ = green_[((curr[x - 1] + curr[x + 1] + prev[x] + next[x])/16)];
		*dst++ = red_[((prev[x - 1] + next[x + 1])/8)];
		x++;

		/*
		 * IGIG line even pixel: GIGIG
		 * 						 RGBGR
		 *                       GIGIG
		 *                       BGRGB
		 * 						 GIGIG
		 * Write BGR
		 */
		*dst++ = blue_[prev[x]/4];
		*dst++ = green_[curr[x]/4];
		*dst++ = red_[next[x]/4];
		x++;

		/*
		 * IGIG line even pixel: IGIGI
		 * 						 GBGRG
		 *                       IGIGI
		 *                       GRGBG
		 * 						 IGIGI
		 * Write BGR
		 */
		*dst++ = blue_[((prev[x - 1] + next[x + 1])/8)];
		*dst++ = green_[((curr[x - 1] + curr[x + 1] + prev[x] + next[x])/16)];
		*dst++ = red_[((prev[x + 1] + next[x - 1])/8)];
		x++;
		/*
		 * IGIG line even pixel: GIGIG
		 * 						 RGBGR
		 *                       GIGIG
		 *                       BGRGB
		 * 						 GIGIG
		 * Write BGR
		 */
		*dst++ = blue_[next[x]/4];
		*dst++ = green_[curr[x]/4];
		*dst++ = red_[prev[x]/4];
		x++;
	}
}

void DebayerCpu::debayerGRGB10Line3(uint8_t *dst, const uint8_t *src)
{
	/* Pointers to previous, current and next lines */
	const uint16_t *prev2 = (const uint16_t *)(src - inputConfig_.stride * 2);
	const uint16_t *prev = (const uint16_t *) (src - inputConfig_.stride);
	const uint16_t *curr = (const uint16_t *) (src);
	const uint16_t *next = (const uint16_t *) (src + inputConfig_.stride);
	const uint16_t *next2 = (const uint16_t *)(src + inputConfig_.stride * 2);
	
	(void) prev2;(void) prev;(void) curr;(void) next;(void) next2;

	for (int x = 0; x < (int)window_.width;) {
		/*
		 * BGBG line even pixel: GRGBG
		 * 						 IGIGI
		 *                       GBGRG
		 *                       IGIGI
		 * 						 GRGBG
		 * Write BGR
		 */
		*dst++ = blue_[curr[x - 1]/4];
		*dst++ = green_[curr[x]/4];
		*dst++ = red_[curr[x + 1]/4];
		x++;

		/*
		 * BGBG line even pixel: RGBGR
		 * 						 GIGIG
		 *                       BGRGB
		 *                       GIGIG
		 * 						 RGBGR
		 * Write BGR
		 */
		*dst++ = blue_[((curr[x - 2] + curr[x + 2] + prev2[x] + next2[x])/16)];
		*dst++ = green_[((curr[x - 1] + curr[x + 1] + prev[x] + next[x])/16)];
		*dst++ = red_[curr[x]/4];
		x++;

		/*
		 * BGBG line even pixel: GBGRG
		 * 						 IGIGI
		 *                       GRGBG
		 *                       IGIGI
		 * 						 GBGRG
		 * Write BGR
		 */
		*dst++ = blue_[curr[x + 1]/4];
		*dst++ = green_[curr[x]/4];
		*dst++ = red_[curr[x - 1]/4];
		x++;

		/*
		 * BGBG line even pixel: BGRGB
		 * 						 GIGIG
		 *                       RGBGR
		 *                       GIGIG
		 * 						 BGRGB
		 * Write BGR
		 */
		*dst++ = blue_[curr[x]/4];
		*dst++ = green_[((curr[x - 1] + curr[x + 1] + prev[x] + next[x])/16)];
		*dst++ = red_[((curr[x - 2] + curr[x + 2] + prev2[x] + next2[x])/16)];
		x++;
	}
}



extern bool is_ov01a1s;

int DebayerCpu::getInputConfig(PixelFormat inputFormat, DebayerInputConfig &config)
{
	BayerFormat bayerFormat =
		BayerFormat::fromPixelFormat(inputFormat);

	if (is_ov01a1s)
		bayerFormat.order = BayerFormat::GRGB_IGIG_GBGR_IGIG;

	if (bayerFormat.bitDepth == 10 &&
	    bayerFormat.packing == BayerFormat::Packing::CSI2) {
	    	config.bpp = 10;
		config.patternSize.height = 2;
		config.patternSize.width = 4; /* 5 bytes per *4* pixels */
		config.x_shift = 0;
		config.outputFormats = std::vector<PixelFormat>({ formats::RGB888, formats::BGR888 });

		switch (bayerFormat.order) {
		case BayerFormat::BGGR:
		case BayerFormat::GBRG:
		case BayerFormat::GRBG:
		case BayerFormat::RGGB:
			return 0;
		default:
			break;
		}
	} else if (bayerFormat.bitDepth == 10 &&
		   bayerFormat.packing == BayerFormat::Packing::None) {
		config.bpp = 16;
		config.patternSize.height = 2;
		config.patternSize.width = 2;
		config.outputFormats = std::vector<PixelFormat>({ formats::RGB888, formats::BGR888 });

		switch (bayerFormat.order) {
		case BayerFormat::BGGR:
		case BayerFormat::GRBG:
			config.x_shift = 0;
			return 0;
		case BayerFormat::GBRG:
		case BayerFormat::RGGB:
			config.x_shift = 1; /* BGGR -> GBRG */
			return 0;
		case BayerFormat::GRGB_IGIG_GBGR_IGIG:
			config.x_shift = 0;
			config.outputFormats = std::vector<PixelFormat>({ formats::RGB888});
			config.patternSize.height = 4;
			config.patternSize.width = 4;
			return 0;
		default:
			break;
		}
	}

	LOG(Debayer, Info)
		<< "Unsupported input format " << inputFormat.toString();
	return -EINVAL;
}

int DebayerCpu::getOutputConfig(PixelFormat outputFormat, DebayerOutputConfig &config)
{
	if (outputFormat == formats::RGB888 || outputFormat == formats::BGR888) {
		config.bpp = 24;
		return 0;
	}

	LOG(Debayer, Info)
		<< "Unsupported output format " << outputFormat.toString();
	return -EINVAL;
}

int DebayerCpu::setDebayerFunctions(PixelFormat inputFormat, PixelFormat outputFormat)
{
	BayerFormat bayerFormat =
		BayerFormat::fromPixelFormat(inputFormat);

	if (is_ov01a1s)
		bayerFormat.order = BayerFormat::GRGB_IGIG_GBGR_IGIG;

	switch (outputFormat) {
	case formats::RGB888:
		swapRedBlueGains_ = false;
		break;
	case formats::BGR888:
		swapRedBlueGains_ = true;

		/* Swap R and B in bayer order */
		if (bayerFormat.packing == BayerFormat::Packing::None)
			inputConfig_.x_shift = !inputConfig_.x_shift;

		switch (bayerFormat.order) {
		case BayerFormat::BGGR:
			bayerFormat.order = BayerFormat::RGGB;
			break;
		case BayerFormat::GBRG:
			bayerFormat.order = BayerFormat::GRBG;
			break;
		case BayerFormat::GRBG:
			bayerFormat.order = BayerFormat::GBRG;
			break;
		case BayerFormat::RGGB:
			bayerFormat.order = BayerFormat::BGGR;
			break;
		default:
			goto unsupported_fmt;
		}
		break;
	default:
		goto unsupported_fmt;
	}

	if (bayerFormat.bitDepth == 10 &&
	    bayerFormat.packing == BayerFormat::Packing::CSI2) {
		switch (bayerFormat.order) {
		case BayerFormat::BGGR:
			debayer0_ = &DebayerCpu::debayer10P_BGBG_BGR888;
			debayer1_ = &DebayerCpu::debayer10P_GRGR_BGR888;
			return 0;
		case BayerFormat::GBRG:
			debayer0_ = &DebayerCpu::debayer10P_GBGB_BGR888;
			debayer1_ = &DebayerCpu::debayer10P_RGRG_BGR888;
			return 0;
		case BayerFormat::GRBG:
			debayer0_ = &DebayerCpu::debayer10P_GRGR_BGR888;
			debayer1_ = &DebayerCpu::debayer10P_BGBG_BGR888;
			return 0;
		case BayerFormat::RGGB:
			debayer0_ = &DebayerCpu::debayer10P_RGRG_BGR888;
			debayer1_ = &DebayerCpu::debayer10P_GBGB_BGR888;
			return 0;
		default:
			break;
		}
	} else if (bayerFormat.bitDepth == 10 &&
		   bayerFormat.packing == BayerFormat::Packing::None) {
		switch (bayerFormat.order) {
		case BayerFormat::BGGR:
		case BayerFormat::GBRG: /* x_shift=1 BGGR -> GBRG */
			debayer0_ = &DebayerCpu::debayer10_BGBG_BGR888;
			debayer1_ = &DebayerCpu::debayer10_GRGR_BGR888;
			return 0;
		case BayerFormat::GRBG:
		case BayerFormat::RGGB: /* x_shift=1 GRBG -> RGGB */
			debayer0_ = &DebayerCpu::debayer10_GRGR_BGR888;
			debayer1_ = &DebayerCpu::debayer10_BGBG_BGR888;
			return 0;
		case BayerFormat::GRGB_IGIG_GBGR_IGIG:
			debayer0_ = &DebayerCpu::debayerIGIG10Line0;
			debayer1_ = &DebayerCpu::debayerGBGR10Line1;
			debayer2_ = &DebayerCpu::debayerIGIG10Line2;
			debayer3_ = &DebayerCpu::debayerGRGB10Line3;
			return 0;
		default:
			break;
		}
	}

unsupported_fmt:
	LOG(Debayer, Error) << "Unsupported input output format combination";
	return -EINVAL;
}

int DebayerCpu::configure(const StreamConfiguration &inputCfg,
			  const std::vector<std::reference_wrapper<StreamConfiguration>> &outputCfgs)
{
	if (getInputConfig(inputCfg.pixelFormat, inputConfig_) != 0)
		return -EINVAL;

	if (stats_->configure(inputCfg) != 0)
		return -EINVAL;

	const Size &stats_pattern_size = stats_->patternSize();
	if (inputConfig_.patternSize.width != stats_pattern_size.width ||
	    inputConfig_.patternSize.height != stats_pattern_size.height) {
		LOG(Debayer, Error)
			<< "mismatching stats and debayer pattern sizes for "
			<< inputCfg.pixelFormat.toString();
		return -EINVAL;
	}

	inputConfig_.stride = inputCfg.stride;

	if (outputCfgs.size() != 1) {
		LOG(Debayer, Error)
			<< "Unsupported number of output streams: "
			<< outputCfgs.size();
		return -EINVAL;
	}

	const StreamConfiguration &outputCfg = outputCfgs[0];
	SizeRange outSizeRange = sizes(inputCfg.pixelFormat, inputCfg.size);
	std::tie(outputConfig_.stride, outputConfig_.frameSize) =
		strideAndFrameSize(outputCfg.pixelFormat, outputCfg.size);

	if (!outSizeRange.contains(outputCfg.size) || outputConfig_.stride != outputCfg.stride) {
		LOG(Debayer, Error)
			<< "Invalid output size/stride: "
			<< "\n  " << outputCfg.size << " (" << outSizeRange << ")"
			<< "\n  " << outputCfg.stride << " (" << outputConfig_.stride << ")";
		return -EINVAL;
	}

	if (setDebayerFunctions(inputCfg.pixelFormat, outputCfg.pixelFormat) != 0)
		return -EINVAL;

	window_.x = ((inputCfg.size.width - outputCfg.size.width) / 2) &
		    ~(inputConfig_.patternSize.width - 1);
	window_.y = ((inputCfg.size.height - outputCfg.size.height) / 2) &
		    ~(inputConfig_.patternSize.height - 1);
	window_.width = outputCfg.size.width;
	window_.height = outputCfg.size.height;

	/* Don't pass x,y since process() already adjusts src before passing it */
	stats_->setWindow(Rectangle(window_.size()));

	return 0;
}

Size DebayerCpu::patternSize(PixelFormat inputFormat)
{
	DebayerCpu::DebayerInputConfig config;

	if (getInputConfig(inputFormat, config) != 0)
		return {};

	return config.patternSize;
}

std::vector<PixelFormat> DebayerCpu::formats(PixelFormat inputFormat)
{
	DebayerCpu::DebayerInputConfig config;

	if (getInputConfig(inputFormat, config) != 0)
		return std::vector<PixelFormat>();

	return config.outputFormats;
}

std::tuple<unsigned int, unsigned int>
DebayerCpu::strideAndFrameSize(const PixelFormat &outputFormat, const Size &size)
{
	DebayerCpu::DebayerOutputConfig config;

	if (getOutputConfig(outputFormat, config) != 0)
		return std::make_tuple(0, 0);

	/* round up to multiple of 8 for 64 bits alignment */
	unsigned int stride = (size.width * config.bpp / 8 + 7) & ~7;

	return std::make_tuple(stride, stride * size.height);
}

void DebayerCpu::process2(const uint8_t *src, uint8_t *dst)
{
	const unsigned int y_end = window_.y + window_.height;
	const unsigned int x_shift = inputConfig_.x_shift * inputConfig_.bpp / 8;

	/* Adjust src to top left corner of the window */
	src += window_.y * inputConfig_.stride + window_.x * inputConfig_.bpp / 8;

	stats_->startFrame();

	for (unsigned int y = window_.y; y < y_end; y+= 2) {
		stats_->processLine0(y, src, inputConfig_.stride);
		(this->*debayer0_)(dst, src + x_shift);
		src += inputConfig_.stride;
		dst += outputConfig_.stride;

		(this->*debayer1_)(dst, src + x_shift);
		src += inputConfig_.stride;
		dst += outputConfig_.stride;		
	}

	stats_->finishFrame();
}

void DebayerCpu::process4(const uint8_t *src, uint8_t *dst)
{
	const unsigned int y_end = window_.y + window_.height;
	const unsigned int x_shift = inputConfig_.x_shift * inputConfig_.bpp / 8;

	/* Adjust src to top left corner of the window */
	src += window_.y * inputConfig_.stride + window_.x * inputConfig_.bpp / 8;

	stats_->startFrame();

	for (unsigned int y = window_.y; y < y_end; y+= 4) {
		stats_->processLine0(y, src, inputConfig_.stride);
		(this->*debayer0_)(dst, src + x_shift);
		src += inputConfig_.stride;
		dst += outputConfig_.stride;

		(this->*debayer1_)(dst, src + x_shift);
		src += inputConfig_.stride;
		dst += outputConfig_.stride;		

		stats_->processLine2(y, src, inputConfig_.stride);
		(this->*debayer2_)(dst, src + x_shift);
		src += inputConfig_.stride;
		dst += outputConfig_.stride;

		(this->*debayer3_)(dst, src + x_shift);
		src += inputConfig_.stride;
		dst += outputConfig_.stride;		
	}

	stats_->finishFrame();
}

void DebayerCpu::process(FrameBuffer *input, FrameBuffer *output, DebayerParams params)
{
#if 0
	static const unsigned int RED_Y_MUL = 77;	/* 0.30 * 256 */
	static const unsigned int GREEN_Y_MUL = 150;	/* 0.59 * 256 */
	static const unsigned int BLUE_Y_MUL = 29;	/* 0.11 * 256 */

	/*
	 * HACK use stats from previous frame to calculate gains for simple AWB.
	 * This overrides the passed in gains since nothing is setting these yet.
	 * FIXME this needs to be moved to the IPA providing the params and
	 * our caller then passing these in.
	 */
	struct SwIspStats stats = stats_->getStats();

	/* Calculate the average sum of Y multiplied by 256 */
	unsigned long y256 = stats.sumR_ * RED_Y_MUL
		+ stats.sumG_ * GREEN_Y_MUL + stats.sumB_ * BLUE_Y_MUL;

	/* Clamp max gain at 4.0, this also avoids 0 division */
	if (stats.sumR_ <= y256 / 1024)
		params.gainR = 1024;
	else
		params.gainR  = y256 / stats.sumR_;

	if (stats.sumG_ <= y256 / 1024)
		params.gainG = 1024;
	else
		params.gainG = y256 / stats.sumG_;

	if (stats.sumB_ <= y256 / 1024)
		params.gainB = 1024;
	else
		params.gainB = y256 / stats.sumB_;

	LOG(Debayer, Debug)
		<< "AWB sums R " << stats.sumR_ << " G " << stats.sumG_ << " B " << stats.sumB_
		<< "\n    gain R " << params.gainR << " G " << params.gainG << " B " << params.gainB;

	/**** End of HACK / FIXME ****/

	/* Apply DebayerParams */
	if (params.gamma != gamma_correction_) {
		for (int i = 0; i < 1024; i++)
			gamma_[i] = 255 * powf(i / 1023.0, params.gamma);

		gamma_correction_ = params.gamma;
	}
#endif

	if (swapRedBlueGains_)
		std::swap(params.gainR, params.gainB);

	for (int i = 0; i < 256; i++) {
		int idx;

		/* Apply gamma after gain! */
		idx = std::min({ i * params.gainR / 64U, 1023U });
		red_[i] = gamma_[idx];

		idx = std::min({ i * params.gainG / 64U, 1023U });
		green_[i] = gamma_[idx];

		idx = std::min({ i * params.gainB / 64U, 1023U });
		blue_[i] = gamma_[idx];
	}

	/* Copy metadata from the input buffer */
	FrameMetadata &metadata = output->_d()->metadata();
	metadata.status = input->metadata().status;
	metadata.sequence = input->metadata().sequence;
	metadata.timestamp = input->metadata().timestamp;

	MappedFrameBuffer in(input, MappedFrameBuffer::MapFlag::Read);
	MappedFrameBuffer out(output, MappedFrameBuffer::MapFlag::Write);
	if (!in.isValid() || !out.isValid()) {
		LOG(Debayer, Error) << "mmap-ing buffer(s) failed";
		metadata.status = FrameMetadata::FrameError;
		return;
	}

	if (inputConfig_.patternSize.height == 2)
		process2(in.planes()[0].data(), out.planes()[0].data());
	else
		process4(in.planes()[0].data(), out.planes()[0].data());

	metadata.planes()[0].bytesused = out.planes()[0].size();

	outputBufferReady.emit(output);
	inputBufferReady.emit(input);
}

} /* namespace libcamera */
