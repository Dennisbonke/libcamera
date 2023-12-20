/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2023, Linaro Ltd
 * Copyright (C) 2023, Red Hat Inc.
 *
 * Authors:
 * Hans de Goede <hdegoede@redhat.com> 
 *
 * swstats_cpu.cpp - CPU based software statistics implementation
 */

#include "libcamera/internal/software_isp/swstats_cpu.h"

#include <libcamera/base/log.h>

#include <libcamera/stream.h>

#include "libcamera/internal/bayer_format.h"

#define SWISP_LINARO_START_LINE_STATS_IR()			\
	uint8_t r, g1, g2, g3, g4, b;				\
	unsigned int y_val;				\
							\
	unsigned long sumR = 0;				\
	unsigned long sumG = 0;				\
	unsigned long sumB = 0;				

#define SWISP_LINARO_ACCUMULATE_LINE_STATS_IR()		\
	sumR += r;					\
	sumG += g1 + g2 + g3 + g4;				\
	sumB += b;					\
								\
	red_count++;					\
	blue_count++;					\
	green_count += 4;				\
							\
	y_val = r * RED_Y_MUL;				\
	y_val += (g1 + g2 + g3 + g4) * GREEN_Y_MUL_IR;		\
	y_val += b * BLUE_Y_MUL;			\
	exposurebins[y_val/4096]++;

#define SWISP_LINARO_FINISH_LINE_STATS()		\
	sumR_ += sumR;					\
	sumG_ += sumG;					\
	sumB_ += sumB;	

namespace libcamera {


SwStatsCpu::SwStatsCpu()
	: SwStats()
{
	sharedStats_ = SharedMemObject<SwIspStats>("softIsp_stats");
	if (!sharedStats_.fd().isValid())
		LOG(SwStats, Error)
			<< "Failed to create shared memory for statistics";
}

/* for brightness values in the 0 to 255 range: */
static const unsigned int BRIGHT_LVL = 200U << 8;
static const unsigned int TOO_BRIGHT_LVL = 240U << 8;

static const unsigned int RED_Y_MUL = 77;		/* 0.30 * 256 */
static const unsigned int GREEN_Y_MUL = 150;		/* 0.59 * 256 */
static const unsigned int GREEN_Y_MUL_IR = 150 / 4;	/* 0.59 * 256 */
static const unsigned int BLUE_Y_MUL = 29;		/* 0.11 * 256 */

static inline __attribute__((always_inline)) void
statsBayer10P(const int width, const uint8_t *src0, const uint8_t *src1, bool bggr, unsigned int &bright_sum_, unsigned int &too_bright_sum_, SwIspStats &stats_)
{
	const int width_in_bytes = width * 5 / 4;
	uint8_t r, g, b, g2;
	unsigned int y_val;
	unsigned int sumR = 0;
	unsigned int sumG = 0;
	unsigned int sumB = 0;

	unsigned int bright_sum = 0;
	unsigned int too_bright_sum = 0;

	for (int x = 0; x < width_in_bytes; x += 5) {
		if (bggr) {
			/* BGGR */
			b  = src0[x];
			g  = src0[x + 1];
			g2 = src1[x];
			r  = src1[x + 1];
		} else {
			/* GBRG */
			g  = src0[x];
			b  = src0[x + 1];
			r  = src1[x];
			g2 = src1[x + 1];
		}
		g = (g + g2) / 2;

		sumR += r;
		sumG += g;
		sumB += b;

		y_val = r * RED_Y_MUL;
		y_val += g * GREEN_Y_MUL;
		y_val += b * BLUE_Y_MUL;
		if (y_val > BRIGHT_LVL) ++bright_sum;
		if (y_val > TOO_BRIGHT_LVL) ++too_bright_sum;
	}

	stats_.sumR_ += sumR;
	stats_.sumG_ += sumG;
	stats_.sumB_ += sumB;

	bright_sum_ += bright_sum;
	too_bright_sum_ += too_bright_sum;
}

void SwStatsCpu::statsBGGR10PLine0(const uint8_t *src, unsigned int stride)
{
	const uint8_t *src0 = src;
	const uint8_t *src1 = src + stride;

	if (swap_lines_)
		std::swap(src0, src1);

	statsBayer10P(window_.width, src0, src1, true, bright_sum_, too_bright_sum_, stats_);
}

void SwStatsCpu::statsGBRG10PLine0(const uint8_t *src, unsigned int stride)
{
	const uint8_t *src0 = src;
	const uint8_t *src1 = src + stride;

	if (swap_lines_)
		std::swap(src0, src1);

	statsBayer10P(window_.width, src0, src1, false, bright_sum_, too_bright_sum_, stats_);
}

void SwStatsCpu::statsBGGR10Line0(const uint8_t *src, unsigned int stride)
{
	const uint16_t *src0 = (const uint16_t *)src;
	const uint16_t *src1 = (const uint16_t *)(src + stride);
	uint16_t r, g, b, g2;
	unsigned int y_val;
	unsigned int sumR = 0;
	unsigned int sumG = 0;
	unsigned int sumB = 0;

	unsigned int bright_sum = 0;
	unsigned int too_bright_sum = 0;

	if (swap_lines_)
		std::swap(src0, src1);

	/* x += 4 sample every other 2x2 block */
	for (int x = 0; x < (int)window_.width; x += 4) {
		b  = src0[x];
		g  = src0[x + 1];
		g2 = src1[x];
		r  = src1[x + 1];

		g = (g + g2) / 2;

		sumR += r;
		sumG += g;
		sumB += b;

		y_val = r * RED_Y_MUL;
		y_val += g * GREEN_Y_MUL;
		y_val += b * BLUE_Y_MUL;
		/* Thresholds * 4 because 10 bit data */
		if (y_val > (BRIGHT_LVL * 4)) ++bright_sum;
		if (y_val > (TOO_BRIGHT_LVL * 4)) ++too_bright_sum;
	}

	stats_.sumR_ += sumR;
	stats_.sumG_ += sumG;
	stats_.sumB_ += sumB;

	bright_sum_ += bright_sum;
	too_bright_sum_ += too_bright_sum;
}

void SwStatsCpu::statsRGBIR10Line0(const uint8_t *src0, unsigned int stride)
{	

	const uint16_t *src0_16 = (const uint16_t *)src0 + (stride/2);
	const uint16_t *src1_16 = src0_16 + (stride/2);

	SWISP_LINARO_START_LINE_STATS_IR()

	for (int x = 0; x < (int)window_.width; x += 3) {
		
		/* IGIG*/
		//i = src0_16[x];
		g2  = src0_16[x + 1]/4;
		//i = src0_16[x + 2];
		g4  = src0_16[x + 3]/4;

		/* GRGB */
		g1  = src1_16[x]/4;
		r   = src1_16[x + 1]/4;
		g3  = src1_16[x + 2]/4;
		b   = src1_16[x + 3]/4;

		
		
		SWISP_LINARO_ACCUMULATE_LINE_STATS_IR()
	}
	stats_.sumR_ += sumR;
	stats_.sumG_ += sumG;
	stats_.sumB_ += sumB;
}

void SwStatsCpu::statsRGBIR10Line2(const uint8_t *src0, unsigned int stride)
{
	const uint16_t *src0_16 = (const uint16_t *)(src0 + stride);
	const uint16_t *src1_16 = src0_16 + (stride/2);

	SWISP_LINARO_START_LINE_STATS_IR()

	for (int x = 0; x < (int)window_.width; x += 3) {
		

		/* IGIG*/
		//i = src0_16[x];
		g2  = src0_16[x + 1]/4;
		//i = src0_16[x + 2];
		g4  = src0_16[x + 3]/4;

		/* GBGR */
		g1  = src1_16[x]/4;
		b   = src1_16[x + 1]/4;
		g3  = src1_16[x + 2]/4;
		r   = src1_16[x + 3]/4;
		
		SWISP_LINARO_ACCUMULATE_LINE_STATS_IR()
	}
	stats_.sumR_ += sumR;
	stats_.sumG_ += sumG;
	stats_.sumB_ += sumB;
}


void SwStatsCpu::resetStats(void)
{
	stats_.sumR_ = 0;
	stats_.sumB_ = 0;
	stats_.sumG_ = 0;

	bright_sum_ = 0;
	too_bright_sum_ = 0;
}

void SwStatsCpu::finishStats(void)
{
	/* calculate the fractions of "bright" and "too bright" pixels */
	stats_.bright_ratio = (float)bright_sum_ / (window_.height * window_.width / 16);
	stats_.too_bright_ratio = (float)too_bright_sum_ / (window_.height * window_.width / 16);

	*sharedStats_ = stats_;
	statsReady.emit(0);
}

extern bool is_ov01a1s;

int SwStatsCpu::configure(const StreamConfiguration &inputCfg)
{
	BayerFormat bayerFormat =
		BayerFormat::fromPixelFormat(inputCfg.pixelFormat);

	if (is_ov01a1s)
		bayerFormat.order = BayerFormat::GRGB_IGIG_GBGR_IGIG;

	startFrame_ = (SwStats::statsVoidFn)&SwStatsCpu::resetStats;
	finishFrame_ = (SwStats::statsVoidFn)&SwStatsCpu::finishStats;

	if (bayerFormat.bitDepth == 10 &&
	    bayerFormat.packing == BayerFormat::Packing::CSI2) {
		bpp_ = 10;
		patternSize_.height = 2;
		patternSize_.width = 4; /* 5 bytes per *4* pixels */
		y_skip_mask_ = 0x02; /* Skip every 3th and 4th line */
		x_shift_ = 0;

		switch (bayerFormat.order) {
		case BayerFormat::BGGR:
		case BayerFormat::GRBG:
			stats0_ = (SwStats::statsProcessFn)&SwStatsCpu::statsBGGR10PLine0;
			swap_lines_ = bayerFormat.order == BayerFormat::GRBG;
			return 0;
		case BayerFormat::GBRG:
		case BayerFormat::RGGB:
			stats0_ = (SwStats::statsProcessFn)&SwStatsCpu::statsGBRG10PLine0;
			swap_lines_ = bayerFormat.order == BayerFormat::RGGB;
			return 0;
		default:
			break;
		}
	} else if (bayerFormat.bitDepth == 10 &&
		   bayerFormat.packing == BayerFormat::Packing::None) {
		bpp_ = 16;
		patternSize_.height = 2;
		patternSize_.width = 2;
		y_skip_mask_ = 0x02; /* Skip every 3th and 4th line */
		stats0_ = (SwStats::statsProcessFn)&SwStatsCpu::statsBGGR10Line0;

		switch (bayerFormat.order) {
		case BayerFormat::BGGR:
			x_shift_ = 0;
			swap_lines_ = false;
			return 0;
		case BayerFormat::GBRG:
			x_shift_ = 1; /* BGGR -> GBRG */
			swap_lines_ = false;
			return 0;
		case BayerFormat::GRBG:
			x_shift_ = 0; 
			swap_lines_ = true; /* BGGR -> GRBG */
			return 0;
		case BayerFormat::RGGB:
			x_shift_ = 1; /* BGGR -> GBRG */
			swap_lines_ = true; /* GBRG -> RGGB */
			return 0;
		case BayerFormat::GRGB_IGIG_GBGR_IGIG:
			stats0_ = (SwStats::statsProcessFn)&SwStatsCpu::statsRGBIR10Line0;
			stats2_ = (SwStats::statsProcessFn)&SwStatsCpu::statsRGBIR10Line2;
			patternSize_.height = 4;
			patternSize_.width = 4;
			x_shift_ = 0;
			swap_lines_ = false;
			return 0;
		default:
			break;
		}
	}

	LOG(SwStats, Info)
		<< "Unsupported input format " << inputCfg.pixelFormat.toString();
	return -EINVAL;
}

} /* namespace libcamera */
