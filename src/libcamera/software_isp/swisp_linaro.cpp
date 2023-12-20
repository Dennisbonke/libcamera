/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2023, Linaro Ltd
 *
 * swisp_linaro.cpp - software ISP implementation by Linaro
 */

#include "libcamera/internal/software_isp/swisp_linaro.h"

#include <sys/mman.h>
#include <sys/types.h>
#include <unistd.h>
 
#include <bitset>

#include <libcamera/formats.h>
#include <libcamera/stream.h>

#include "libcamera/internal/bayer_format.h"
#include "libcamera/internal/framebuffer.h"
#include "libcamera/internal/mapped_framebuffer.h"

namespace libcamera {

LOG_DECLARE_CATEGORY(SoftwareIsp)

SwIspLinaro::SwIspLinaro(const std::string &name)
	: SoftwareIsp(name)
{
	ispWorker_ = std::make_unique<SwIspLinaro::IspWorker>(this);
	if (!ispWorker_) {
		LOG(SoftwareIsp, Error)
			<< "Failed to create ISP worker";
	} else {
		sharedStats_ = SharedMemObject<SwIspStats>("softIsp_stats");
		if (!sharedStats_.fd().isValid()) {
			LOG(SoftwareIsp, Error)
				<< "Failed to create shared memory for statistics";
			ispWorker_.reset();
		} else {
			ispWorker_->moveToThread(&ispWorkerThread_);
		}
	}
}

bool SwIspLinaro::isValid() const
{
	/* TODO: proper implementation */
	return !!ispWorker_;
}

/* for brightness values in the 0 to 255 range: */
static const unsigned int BRIGHT_LVL = 200U << 8;
static const unsigned int TOO_BRIGHT_LVL = 240U << 8;

static const unsigned int RED_Y_MUL = 77;		/* 0.30 * 256 */
static const unsigned int GREEN_Y_MUL = 150 / 2;	/* 0.59 * 256 */
static const unsigned int GREEN_Y_MUL_IR = 150 / 4;	/* 0.59 * 256 */

static const unsigned int BLUE_Y_MUL = 29;		/* 0.11 * 256 */

/*
 * These need to be macros because it accesses a whole bunch of local
 * variables (and copy and pasting this x times is undesirable)
 */
#define SWISP_LINARO_START_LINE_STATS()			\
	uint8_t r, g1, g2, b;				\
	unsigned int y_val;				\
							\
	unsigned long sumR = 0;				\
	unsigned long sumG = 0;				\
	unsigned long sumB = 0;				

#define SWISP_LINARO_START_LINE_STATS_IR()			\
	uint8_t r, g1, g2, g3, g4, b;				\
	unsigned int y_val;				\
							\
	unsigned long sumR = 0;				\
	unsigned long sumG = 0;				\
	unsigned long sumB = 0;				

#define SWISP_LINARO_ACCUMULATE_LINE_STATS()		\
	sumR += r;					\
	sumG += g1 + g2;				\
	sumB += b;					\
								\
	red_count++;					\
	blue_count++;					\
	green_count += 2;				\
							\
	y_val = r * RED_Y_MUL;				\
	y_val += (g1 + g2) * GREEN_Y_MUL;		\
	y_val += b * BLUE_Y_MUL;			\
	exposurebins[y_val/13108]++;

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
	exposurebins[y_val/13108]++;


#define SWISP_LINARO_FINISH_LINE_STATS()		\
	sumR_ += sumR;					\
	sumG_ += sumG;					\
	sumB_ += sumB;					

void SwIspLinaro::IspWorker::statsBGGR10PLine0(const uint8_t *src0)
{
	const int width_in_bytes = width_ * 5 / 4;
	const uint8_t *src1 = src0 + stride_;

	SWISP_LINARO_START_LINE_STATS()

	for (int x = 0; x < width_in_bytes; x += 3) {
		/* BGGR */
		b  = src0[x];
		g1 = src0[x + 1];
		g2 = src1[x];
		r  = src1[x + 1];

		SWISP_LINARO_ACCUMULATE_LINE_STATS()

		x += 2;

		/* BGGR */
		b  = src0[x];
		g1 = src0[x + 1];
		g2 = src1[x];
		r  = src1[x + 1];

		SWISP_LINARO_ACCUMULATE_LINE_STATS()
	}
	SWISP_LINARO_FINISH_LINE_STATS()
}

void SwIspLinaro::IspWorker::statsGBRG10PLine0(const uint8_t *src0)
{
	const int width_in_bytes = width_ * 5 / 4;
	const uint8_t *src1 = src0 + stride_;

	SWISP_LINARO_START_LINE_STATS()

	for (int x = 0; x < width_in_bytes; x += 3) {
		/* GBRG */
		g1 = src0[x];
		b  = src0[x + 1];
		r  = src1[x];
		g2 = src1[x + 1];

		SWISP_LINARO_ACCUMULATE_LINE_STATS()

		x += 2;

		/* GBRG */
		g1 = src0[x];
		b  = src0[x + 1];
		r  = src1[x];
		g2 = src1[x + 1];

		SWISP_LINARO_ACCUMULATE_LINE_STATS()
	}
	SWISP_LINARO_FINISH_LINE_STATS()
}

void SwIspLinaro::IspWorker::statsGRBG10PLine0(const uint8_t *src0)
{
	const int width_in_bytes = width_ * 5 / 4;
	const uint8_t *src1 = src0 + stride_;

	SWISP_LINARO_START_LINE_STATS()

	for (int x = 0; x < width_in_bytes; x += 3) {
		/* GRBG */
		g1 = src0[x];
		r  = src0[x + 1];
		b  = src1[x];
		g2 = src1[x + 1];

		SWISP_LINARO_ACCUMULATE_LINE_STATS()

		x += 2;

		/* GRBG */
		g1 = src0[x];
		r  = src0[x + 1];
		b  = src1[x];
		g2 = src1[x + 1];

		SWISP_LINARO_ACCUMULATE_LINE_STATS()
	}
	SWISP_LINARO_FINISH_LINE_STATS()
}

void SwIspLinaro::IspWorker::statsRGGB10PLine0(const uint8_t *src0)
{
	const int width_in_bytes = width_ * 5 / 4;
	const uint8_t *src1 = src0 + stride_;

	SWISP_LINARO_START_LINE_STATS()

	for (int x = 0; x < width_in_bytes; x += 3) {
		/* RGGB */
		r  = src0[x];
		g1 = src0[x + 1];
		g2 = src1[x];
		b  = src1[x + 1];

		SWISP_LINARO_ACCUMULATE_LINE_STATS()

		x += 2;

		/* RGGB */
		r  = src0[x];
		g1 = src0[x + 1];
		g2 = src1[x];
		b  = src1[x + 1];

		SWISP_LINARO_ACCUMULATE_LINE_STATS()
	}
	SWISP_LINARO_FINISH_LINE_STATS()
}

void SwIspLinaro::IspWorker::debayerBGGR10PLine0(uint8_t *dst, const uint8_t *src)
{
	const int width_in_bytes = 5 + width_ * 5 / 4;
	/* Pointers to previous, current and next lines */
	const uint8_t *prev = src - stride_;
	const uint8_t *curr = src;
	const uint8_t *next = src - stride_;

	for (int x = 5; x < width_in_bytes; x += 2) {
		/*
		 * BGBG line even pixel: RGR
		 *                       GBG
		 *                       RGR
		 * Write BGR
		 */
		*dst++ = curr[x] * stats_.blue_awb_correction;
		*dst++ = (prev[x] + curr[x - 2] + curr[x + 1] + next[x]) ;
		*dst++ = (prev[x - 2] + prev[x + 1] + next[x - 2]  + next[x + 1]) * stats_.red_awb_correction;
		x++;

		/*
		 * BGBG line odd pixel: GRG
		 *                      BGB
		 *                      GRG
		 * Write BGR
		 */
		*dst++ = (curr[x - 1] + curr[x + 1]) * stats_.blue_awb_correction;
		*dst++ = curr[x] ;
		*dst++ = (prev[x] + next[x]) * stats_.red_awb_correction;
		x++;

		/* Same thing for next 2 pixels */
		*dst++ = curr[x] * stats_.blue_awb_correction;
		*dst++ = (prev[x] + curr[x - 1] + curr[x + 1] + next[x]);
		*dst++ = (prev[x - 1] + prev[x + 1] + next[x - 1]  + next[x + 1]) * stats_.red_awb_correction;
		x++;

		*dst++ = (curr[x - 1] + curr[x + 2]) * stats_.blue_awb_correction;
		*dst++ = curr[x] ;
		*dst++ = (prev[x] + next[x]) * stats_.red_awb_correction;
	}
}

void SwIspLinaro::IspWorker::debayerBGGR10PLine1(uint8_t *dst, const uint8_t *src)
{
	const int width_in_bytes = 5 + width_ * 5 / 4;
	/* Pointers to previous, current and next lines */
	const uint8_t *prev = src - stride_;
	const uint8_t *curr = src;
	const uint8_t *next = src - stride_;

	/*
	 * For the first pixel getting a pixel from the previous column uses
	 * x - 2 to skip the 5th byte with least-significant bits for 4 pixels.
	 * Same for last pixel and looking at the nexy column.
	 */

	for (int x = 5; x < width_in_bytes; x += 2) {
		/*
		 * GRGR line even pixel: GBG
		 *                       RGR
		 *                       GBG
		 * Write BGR
		 */
		*dst++ = (prev[x] + next[x]) *  stats_.blue_awb_correction;
		*dst++ = curr[x] ;
		*dst++ = (curr[x - 2] + curr[x + 1]) * stats_.red_awb_correction;
		x++;

		/*
		 * GRGR line odd pixel: BGB
		 *                      GRG
		 *                      BGB
		 * Write BGR
		 */
		*dst++ = (prev[x - 1] + prev[x + 1] + next[x - 1] + next[x + 1]) *  stats_.blue_awb_correction;
		*dst++ = (prev[x] + curr[x - 1] + curr[x + 1] + next[x]);
		*dst++ = curr[x] * stats_.red_awb_correction;
		x++;

		/* Same thing for next 2 pixels */
		*dst++ = (prev[x] + next[x]) *  stats_.blue_awb_correction;
		*dst++ = curr[x] ;
		*dst++ = (curr[x - 1] + curr[x + 1]) * stats_.red_awb_correction;
		x++;

		*dst++ = (prev[x - 1] + prev[x + 2] + next[x - 1] + next[x + 2]) * stats_.blue_awb_correction;
		*dst++ = (prev[x] + curr[x - 1] + curr[x + 2] + next[x]);
		*dst++ = curr[x] * stats_.red_awb_correction;
	}
}

void SwIspLinaro::IspWorker::debayerGBRG10PLine0(uint8_t *dst, const uint8_t *src)
{
	const int width_in_bytes = 5 + width_ * 5 / 4;
	/* Pointers to previous, current and next lines */
	const uint8_t *prev = src - stride_;
	const uint8_t *curr = src;
	const uint8_t *next = src - stride_;

	for (int x = 5; x < width_in_bytes; x += 2) {
		/*
		 * GBGB line even pixel: GRG
		 *                       BGB
		 *                       GRG
		 * Write BGR
		 */
		*dst++ = (curr[x - 2] + curr[x + 1]) * stats_.blue_awb_correction;
		*dst++ = curr[x] ;
		*dst++ = (prev[x] + next[x]) * stats_.red_awb_correction;
		x++;

		/*
		 * GBGB line odd pixel: RGR
		 *                      GBG
		 *                      RGR
		 * Write BGR
		 */
		*dst++ = curr[x] * stats_.blue_awb_correction;
		*dst++ = (prev[x] + curr[x - 1] + curr[x + 1] + next[x]);
		*dst++ = (prev[x - 1] + prev[x + 1] + next[x - 1]  + next[x + 1]) * stats_.red_awb_correction;
		x++;

		/* Same thing for next 2 pixels */
		*dst++ = (curr[x - 1] + curr[x + 1]) * stats_.blue_awb_correction;
		*dst++ = curr[x] ;
		*dst++ = (prev[x] + next[x]) *stats_.red_awb_correction;
		x++;

		*dst++ = curr[x] * stats_.blue_awb_correction;
		*dst++ = (prev[x] + curr[x - 1] + curr[x + 2] + next[x]);
		*dst++ = (prev[x - 1] + prev[x + 2] + next[x - 1]  + next[x + 2]) * stats_.red_awb_correction;
	}
}

void SwIspLinaro::IspWorker::debayerGBRG10PLine1(uint8_t *dst, const uint8_t *src)
{
	const int width_in_bytes = 5 + width_ * 5 / 4;
	/* Pointers to previous, current and next lines */
	const uint8_t *prev = src - stride_;
	const uint8_t *curr = src;
	const uint8_t *next = src - stride_;

	for (int x = 5; x < width_in_bytes; x += 2) {
		/*
		 * RGRG line even pixel: BGB
		 *                       GRG
		 *                       BGB
		 * Write BGR
		 */
		*dst++ = (prev[x - 2] + prev[x + 1] + next[x - 2] + next[x + 1]) * stats_.blue_awb_correction;
		*dst++ = (prev[x] + curr[x - 2] + curr[x + 1] + next[x]);
		*dst++ = curr[x] * stats_.red_awb_correction;
		x++;

		/*
		 * RGRG line odd pixel: GBG
		 *                      RGR
		 *                      GBG
		 * Write BGR
		 */
		*dst++ = (prev[x] + next[x]) * stats_.blue_awb_correction;
		*dst++ = curr[x] ;
		*dst++ = (curr[x - 1] + curr[x + 1]) * stats_.red_awb_correction;
		x++;

		/* Same thing for next 2 pixels */
		*dst++ = (prev[x - 1] + prev[x + 1] + next[x - 1] + next[x + 1]) * stats_.blue_awb_correction;
		*dst++ = (prev[x] + curr[x - 1] + curr[x + 1] + next[x]);
		*dst++ = curr[x] * stats_.red_awb_correction;
		x++;

		*dst++ = (prev[x] + next[x]) * stats_.blue_awb_correction;
		*dst++ = curr[x] ;
		*dst++ = (curr[x - 1] + curr[x + 2]) * stats_.red_awb_correction;
	}
}

void SwIspLinaro::IspWorker::finishRaw10PStats(void)
{
	/* calculate the fractions of "bright" and "too bright" pixels */

	/* add sum values and color count to statistics*/
	stats_.sumG_ = sumG_;
	stats_.sumB_ = sumB_;
	stats_.sumR_ = sumR_;

	stats_.green_count = green_count;
	stats_.blue_count = blue_count;
	stats_.red_count = red_count;

	/* calculate red and blue gains for simple AWB */
	sumG_ /= 2; /* the number of G pixels is twice as big vs R and B ones */

	unsigned long sumMin = std::min({ sumR_, sumG_, sumB_ });

	rNumerat_ = 256UL * sumMin / sumR_;
	gNumerat_ = 256UL * sumMin / sumG_;
	bNumerat_ = 256UL * sumMin / sumB_;

{
	static int xxx = 75;
	if (--xxx == 0) {
	xxx = 75;
	LOG(SoftwareIsp, Info)
		<< "exposure[0] = " << stats_.exposurebins[0]
		<< "exposure[1] = " << stats_.exposurebins[1]
		<< "exposure[2] = " << stats_.exposurebins[2]
		<< "exposure[3] = " << stats_.exposurebins[3]
		<< "exposure[4] = " << stats_.exposurebins[4];
	LOG(SoftwareIsp, Info)
		<< "sumR = " << sumR_ << ", sumB = " << sumB_ << ", sumG = " << sumG_;
	LOG(SoftwareIsp, Info)
		<< "rGain = [ " << rNumerat_ << " / 256 ], "
		<< "bGain = [ " << bNumerat_ << " / 256 ], "
		<< "gGain = [ " << gNumerat_ << " / 256 ], ";
	}
}
}

SizeRange SwIspLinaro::IspWorker::outSizesRaw10P(const Size &inSize)
{
	if (inSize.width < 2 || inSize.height < 2) {
		LOG(SoftwareIsp, Error)
			<< "Input format size too small: " << inSize.toString();
		return {};
	}

	/*
	 * Debayering is done on 4x4 blocks because:
	 * 1. Some RGBI patterns repeat on a 4x4 basis
	 * 2. 10 bit packed bayer data packs 4 pixels in every 5 bytes
	 *
	 * For the width 1 extra column is needed for interpolation on each side
	 * and to keep the debayer code simple on the left side an entire block
	 * is skipped reducing the available width by 5 pixels.
	 *
	 * For the height 2 extra rows are needed for RGBI interpolation
	 * and to keep the debayer code simple on the top an entire block is
         * skipped reducing the available height by 6 pixels.
         *
         * As debayering is done in 4x4 blocks both must be a multiple of 4.
         */
	return SizeRange(Size((inSize.width - 5) & ~3, (inSize.height - 6) & ~3));
}

unsigned int SwIspLinaro::IspWorker::outStrideRaw10P(const Size &outSize)
{
	return outSize.width * 3;
}

// Unpacked RGB-IR Debayering part etc.

void SwIspLinaro::IspWorker::statsRGBIR10Line0(const uint8_t *src0)
{	

	const int width_in_bytes = width_;
	const uint16_t *src0_16 = (const uint16_t *)src0 + (stride_/2);
	const uint16_t *src1_16 = src0_16 + (stride_/2);

	SWISP_LINARO_START_LINE_STATS_IR()

	for (int x = 0; x < width_in_bytes; x += 3) {
		
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
	SWISP_LINARO_FINISH_LINE_STATS()
}

void SwIspLinaro::IspWorker::statsRGBIR10Line2(const uint8_t *src0)
{
	const int width_in_bytes = width_;
	const uint16_t *src0_16 = (const uint16_t *)src0 + (stride_/2);
	const uint16_t *src1_16 = src0_16 + (stride_/2);

	SWISP_LINARO_START_LINE_STATS_IR()

	for (int x = 0; x < width_in_bytes; x += 3) {
		

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
	SWISP_LINARO_FINISH_LINE_STATS()
}

void SwIspLinaro::IspWorker::debayerIGIG10Line0(uint8_t *dst, const uint8_t *src)
{
	/* Pointers to previous, current and next lines */
	const uint16_t *prev = (const uint16_t *) (src - stride_);
	const uint16_t *curr = (const uint16_t *) (src);
	const uint16_t *next = (const uint16_t *) (src + stride_);
	(void) prev; (void) curr; (void) next;

	for (int x = 4; x < (int)width_;) {
		/*
		 * IGIG line even pixel: IGIGI
		 * 						 GBGRG
		 *                       IGIGI
		 *                       GRGBG
		 * 						 IGIGI
		 * Write BGR
		 */
		*dst++ = std::min((int)((prev[x - 1] + next[x + 1])/8),0xff);
		*dst++ = std::min((int)((curr[x - 1] + curr[x + 1] + prev[x] + next[x])/16) ,0xff);
		*dst++ = std::min((int)((prev[x + 1] + next[x - 1])/8),0xff);
		x++;

		/*
		 * IGIG line even pixel: GIGIG
		 * 						 BGRGB
		 *                       GIGIG
		 *                       RGBGR
		 * 						 GIGIG
		 * Write BGR
		 */
		*dst++ = std::min((int)next[x]/4,0xff);
		*dst++ = std::min((int)curr[x]/4,0xff);
		*dst++ = std::min((int)prev[x]/4,0xff);
		x++;

		/*
		 * IGIG line even pixel: IGIGI
		 * 						 GRGBG
		 *                       IGIGI
		 *                       GBGRG
		 * 						 IGIGI
		 * Write BGR
		 */
		*dst++ = std::min((int)((prev[x + 1] + next[x - 1])/8),0xff);
		*dst++ = std::min((int)((curr[x - 1] + curr[x + 1] + prev[x] + next[x])/16) ,0xff);
		*dst++ = std::min((int)((prev[x - 1] + next[x + 1])/8),0xff);
		x++;

		/*
		 * IGIG line even pixel: GIGIG
		 * 						 BGRGB
		 *                       GIGIG
		 *                       RGBGR
		 * 						 GIGIG
		 * Write BGR
		 */
		*dst++ = std::min((int)prev[x]/4,0xff);
		*dst++ = std::min((int)curr[x]/4,0xff);
		*dst++ = std::min((int)next[x]/4,0xff);
		x++;
	}
}

void SwIspLinaro::IspWorker::debayerGBGR10Line1(uint8_t *dst, const uint8_t *src)
{
	/* Pointers to previous, current and next lines */
	const uint16_t *prev2 = (const uint16_t *)(src - stride_ * 2);
	const uint16_t *prev = (const uint16_t *) (src - stride_);
	const uint16_t *curr = (const uint16_t *) (src);
	const uint16_t *next = (const uint16_t *) (src + stride_);
	const uint16_t *next2 = (const uint16_t *)(src + stride_ * 2);
	(void) prev2;(void) prev; (void) curr; (void) next; (void) next2;

	for (int x = 4; x < (int)width_;) {
		/*
		 * BGBG line even pixel: GBGRG
		 * 						 IGIGI
		 *                       GRGBG
		 *                       IGIGI
		 * 						 GBGRG
		 * Write BGR
		 */
		*dst++ = std::min((int)curr[x + 1]/4,0xff);
		*dst++ = std::min((int)curr[x]/4,0xff);
		*dst++ = std::min((int)curr[x - 1]/4,0xff);
		x++;

		/*
		 * BGBG line even pixel: BGRGB
		 * 						 GIGIG
		 *                       RGBGR
		 *                       GIGIG
		 * 						 BGRGB
		 * Write BGR
		 */
		*dst++ = std::min((int)curr[x]/4,0xff);
		*dst++ = std::min((int)((curr[x - 1] + curr[x + 1] + prev[x] + next[x])/16),0xff);
		*dst++ = std::min((int)((curr[x - 2] + curr[x + 2] + prev2[x] + next2[x])/16),0xff);
		x++;

		/*
		 * BGBG line even pixel: GRGBG
		 * 						 IGIGI
		 *                       GBGRG
		 *                       IGIGI
		 * 						 GRGBG
		 * Write BGR
		 */
		*dst++ = std::min((int)curr[x - 1]/4,0xff);
		*dst++ = std::min((int)curr[x]/4,0xff);
		*dst++ = std::min((int)curr[x + 1]/4,0xff);
		x++;

		/*
		 * BGBG line even pixel: RGBGR
		 * 						 GIGIG
		 *                       BGRGB
		 *                       GIGIG
		 * 						 RGBGR
		 * Write BGR
		 */
		*dst++ = std::min((int)((curr[x - 2] + curr[x + 2] + prev2[x] + next2[x])/16),0xff);
		*dst++ = std::min((int)((curr[x - 1] + curr[x + 1] + prev[x] + next[x])/16),0xff);
		*dst++ = std::min((int)curr[x]/4,0xff);
		x++;	
	}
}

void SwIspLinaro::IspWorker::debayerIGIG10Line2(uint8_t *dst, const uint8_t *src)
{
	/* Pointers to previous, current and next lines */
	const uint16_t *prev = (const uint16_t *)(src - stride_);
	const uint16_t *curr = (const uint16_t *)src;
	const uint16_t *next = (const uint16_t *)(src + stride_);
	(void) prev; (void) curr; (void) next;
	
	for (int x = 4; x < (int)width_;) {
		/*
		 * IGIG line even pixel: IGIGI
		 * 						 GRGBG
		 *                       IGIGI
		 *                       GBGRG
		 * 						 IGIGI
		 * Write BGR
		 */
		*dst++ = std::min((int)((prev[x + 1] + next[x - 1])/8),0xff);
		*dst++ = std::min((int)((curr[x - 1] + curr[x + 1] + prev[x] + next[x])/16),0xff);
		*dst++ = std::min((int)((prev[x - 1] + next[x + 1])/8),0xff);
		x++;

		/*
		 * IGIG line even pixel: GIGIG
		 * 						 RGBGR
		 *                       GIGIG
		 *                       BGRGB
		 * 						 GIGIG
		 * Write BGR
		 */
		*dst++ = std::min((int)prev[x]/4,0xff);
		*dst++ = std::min((int)curr[x]/4,0xff);
		*dst++ = std::min((int)next[x]/4,0xff);
		x++;

		/*
		 * IGIG line even pixel: IGIGI
		 * 						 GBGRG
		 *                       IGIGI
		 *                       GRGBG
		 * 						 IGIGI
		 * Write BGR
		 */
		*dst++ = std::min((int)((prev[x - 1] + next[x + 1])/8),0xff);
		*dst++ = std::min((int)((curr[x - 1] + curr[x + 1] + prev[x] + next[x])/16),0xff);
		*dst++ = std::min((int)((prev[x + 1] + next[x - 1])/8),0xff);
		x++;
		/*
		 * IGIG line even pixel: GIGIG
		 * 						 RGBGR
		 *                       GIGIG
		 *                       BGRGB
		 * 						 GIGIG
		 * Write BGR
		 */
		*dst++ = std::min((int)next[x]/4,0xff);
		*dst++ = std::min((int)curr[x]/4,0xff);
		*dst++ = std::min((int)prev[x]/4,0xff);
		x++;
	}
}

void SwIspLinaro::IspWorker::debayerGRGB10Line3(uint8_t *dst, const uint8_t *src)
{
	/* Pointers to previous, current and next lines */
	const uint16_t *prev2 = (const uint16_t *)(src - stride_ * 2);
	const uint16_t *prev = (const uint16_t *) (src - stride_);
	const uint16_t *curr = (const uint16_t *) (src);
	const uint16_t *next = (const uint16_t *) (src + stride_);
	const uint16_t *next2 = (const uint16_t *)(src + stride_ * 2);
	
	(void) prev2;(void) prev;(void) curr;(void) next;(void) next2;

	for (int x = 4; x < (int)width_;) {
		/*
		 * BGBG line even pixel: GRGBG
		 * 						 IGIGI
		 *                       GBGRG
		 *                       IGIGI
		 * 						 GRGBG
		 * Write BGR
		 */
		*dst++ = std::min((int)curr[x - 1]/4,0xff);
		*dst++ = std::min((int)curr[x]/4,0xff);
		*dst++ = std::min((int)curr[x + 1]/4,0xff);
		x++;

		/*
		 * BGBG line even pixel: RGBGR
		 * 						 GIGIG
		 *                       BGRGB
		 *                       GIGIG
		 * 						 RGBGR
		 * Write BGR
		 */
		*dst++ = std::min((int)((curr[x - 2] + curr[x + 2] + prev2[x] + next2[x])/16),0xff);
		*dst++ = std::min((int)((curr[x - 1] + curr[x + 1] + prev[x] + next[x])/16),0xff);
		*dst++ = std::min((int)curr[x]/4,0xff);
		x++;

		/*
		 * BGBG line even pixel: GBGRG
		 * 						 IGIGI
		 *                       GRGBG
		 *                       IGIGI
		 * 						 GBGRG
		 * Write BGR
		 */
		*dst++ = std::min((int)curr[x + 1]/4,0xff);
		*dst++ = std::min((int)curr[x]/4,0xff);
		*dst++ = std::min((int)curr[x - 1]/4,0xff);
		x++;

		/*
		 * BGBG line even pixel: BGRGB
		 * 						 GIGIG
		 *                       RGBGR
		 *                       GIGIG
		 * 						 BGRGB
		 * Write BGR
		 */
		*dst++ = std::min((int)curr[x]/4,0xff);
		*dst++ = std::min((int)((curr[x - 1] + curr[x + 1] + prev[x] + next[x])/16),0xff);
		*dst++ = std::min((int)((curr[x - 2] + curr[x + 2] + prev2[x] + next2[x])/16),0xff);
		x++;
	}
}

void SwIspLinaro::IspWorker::finishRaw10Stats(void)
{
	/* Copy exposurebins to stats_ struct */
	std::copy(exposurebins, exposurebins+5,stats_.exposurebins);

	/* add sum values and color count to statistics*/
	stats_.sumG_ = sumG_;
	stats_.sumB_ = sumB_;
	stats_.sumR_ = sumR_;

	stats_.green_count = green_count;
	stats_.blue_count = blue_count;
	stats_.red_count = red_count;

	/* calculate red and blue gains for simple AWB */
	sumG_ /= 4; /* the number of G pixels is four times as big vs R and B ones */

	unsigned long sumMin = std::min({ sumR_, sumG_, sumB_ });

	rNumerat_ = 256UL * sumMin / sumR_;
	gNumerat_ = 256UL * sumMin / sumG_;
	bNumerat_ = 256UL * sumMin / sumB_;

{
	static int xxx = 75;
	if (--xxx == 0) {
	xxx = 75;
	LOG(SoftwareIsp, Info)
		<< "exposure[0] = " << stats_.exposurebins[0]
		<< "\nexposure[1] = " << stats_.exposurebins[1]
		<< "\nexposure[2] = " << stats_.exposurebins[2]
		<< "\nexposure[3] = " << stats_.exposurebins[3]
		<< "\nexposure[4] = " << stats_.exposurebins[4];
	LOG(SoftwareIsp, Info)
		<< "sumR = " << sumR_ << ", sumB = " << sumB_ << ", sumG = " << sumG_;
	LOG(SoftwareIsp, Info)
		<< "rGain = [ " << rNumerat_ << " / 256 ], "
		<< "bGain = [ " << bNumerat_ << " / 256 ], "
		<< "gGain = [ " << gNumerat_ << " / 256 ], ";
	}
}
}

SizeRange SwIspLinaro::IspWorker::outSizesRaw10(const Size &inSize)
{
	if (inSize.width < 2 || inSize.height < 2) {
		LOG(SoftwareIsp, Error)
			<< "Input format size too small: " << inSize.toString();
		return {};
	}

	/*
	 * Debayering is done on 4x4 blocks because:
	 * 1. Some RGBI patterns repeat on a 4x4 basis
	 * 2. 10 bit packed bayer data packs 4 pixels in every 5 bytes
	 *
	 * For the width 2 extra columns are needed for RGBI interpolation on each side
	 * and to keep the debayer code simple on the left side an entire block
	 * is skipped reducing the available width by 6 pixels.
	 *
	 * For the height 2 extra rows are needed for RGBI interpolation
	 * and to keep the debayer code simple on the top an entire block is
         * skipped reducing the available height by 6 pixels.
         *
         * As debayering is done in 4x4 blocks both must be a multiple of 4.
         */
	return SizeRange(Size((inSize.width - 8) & ~3, (inSize.height - 8) & ~3));
}

unsigned int SwIspLinaro::IspWorker::outStrideRaw10(const Size &outSize)
{
	return outSize.width * 3;
}


SwIspLinaro::IspWorker::IspWorker(SwIspLinaro *swIsp)
	: swIsp_(swIsp)
{
	// Martti Laptop DELL
	debayerInfos_[formats::SGRBG10] = { formats::RGB888,
		&SwIspLinaro::IspWorker::debayerIGIG10Line0,
		&SwIspLinaro::IspWorker::debayerGBGR10Line1,
		&SwIspLinaro::IspWorker::debayerIGIG10Line2,
		&SwIspLinaro::IspWorker::debayerGRGB10Line3,
		&SwIspLinaro::IspWorker::statsRGBIR10Line0,
		&SwIspLinaro::IspWorker::statsRGBIR10Line2,
		&SwIspLinaro::IspWorker::finishRaw10Stats,
		&SwIspLinaro::IspWorker::outSizesRaw10,
		&SwIspLinaro::IspWorker::outStrideRaw10 };

	debayerInfos_[formats::SBGGR10_CSI2P] = { formats::RGB888,
		&SwIspLinaro::IspWorker::debayerBGGR10PLine0,
		&SwIspLinaro::IspWorker::debayerBGGR10PLine1,
		NULL,
		NULL,
		&SwIspLinaro::IspWorker::statsBGGR10PLine0,
		NULL,
		&SwIspLinaro::IspWorker::finishRaw10PStats,
		&SwIspLinaro::IspWorker::outSizesRaw10P,
		&SwIspLinaro::IspWorker::outStrideRaw10P };
	debayerInfos_[formats::SGBRG10_CSI2P] = { formats::RGB888,
		&SwIspLinaro::IspWorker::debayerGBRG10PLine0,
		&SwIspLinaro::IspWorker::debayerGBRG10PLine1,
		NULL,
		NULL,
		&SwIspLinaro::IspWorker::statsGBRG10PLine0,
		NULL,
		&SwIspLinaro::IspWorker::finishRaw10PStats,
		&SwIspLinaro::IspWorker::outSizesRaw10P,
		&SwIspLinaro::IspWorker::outStrideRaw10P };
	debayerInfos_[formats::SGRBG10_CSI2P] = { formats::RGB888,
		/* GRBG is BGGR with the lines swapped */
		&SwIspLinaro::IspWorker::debayerBGGR10PLine1,
		&SwIspLinaro::IspWorker::debayerBGGR10PLine0,
		NULL,
		NULL,
		&SwIspLinaro::IspWorker::statsGRBG10PLine0,
		NULL,
		&SwIspLinaro::IspWorker::finishRaw10PStats,
		&SwIspLinaro::IspWorker::outSizesRaw10P,
		&SwIspLinaro::IspWorker::outStrideRaw10P };
	debayerInfos_[formats::SRGGB10_CSI2P] = { formats::RGB888,
		/* RGGB is GBRG with the lines swapped */
		&SwIspLinaro::IspWorker::debayerGBRG10PLine1,
		&SwIspLinaro::IspWorker::debayerGBRG10PLine0,
		NULL,
		NULL,
		&SwIspLinaro::IspWorker::statsRGGB10PLine0,
		NULL,
		&SwIspLinaro::IspWorker::finishRaw10PStats,
		&SwIspLinaro::IspWorker::outSizesRaw10P,
		&SwIspLinaro::IspWorker::outStrideRaw10P };

		LOG(SoftwareIsp, Debug) << "red correction value: " << stats_.red_awb_correction;
		LOG(SoftwareIsp, Debug) << "blue correction value: " << stats_.red_awb_correction;
}

int SwIspLinaro::IspWorker::setDebayerInfo(PixelFormat format)
{
	const auto it = debayerInfos_.find(format);
	if (it == debayerInfos_.end())
		return -1;

	debayerInfo_ = &it->second;
	return 0;
}

std::vector<PixelFormat> SwIspLinaro::IspWorker::formats(PixelFormat input)
{
	std::vector<PixelFormat> pixelFormats;

	const auto it = debayerInfos_.find(input);
	if (it == debayerInfos_.end())
		LOG(SoftwareIsp, Info)
			<< "Unsupported input format " << input.toString();
	else
		pixelFormats.push_back(it->second.outPixelFmt);

	return pixelFormats;
}

SizeRange SwIspLinaro::IspWorker::sizes(PixelFormat inputFormat,
					const Size &inputSize)
{
	const auto it = debayerInfos_.find(inputFormat);
	if (it == debayerInfos_.end()) {
		LOG(SoftwareIsp, Info)
			<< "Unsupported input format " << inputFormat.toString();
		return {};
	}

	return (*it->second.getOutSizes)(inputSize);
}

unsigned int SwIspLinaro::IspWorker::outStride(const PixelFormat &outputFormat,
					       const Size &outSize)
{
	/*
	 * Assuming that the output stride depends only on the outputFormat,
	 * we use the first debayerInfos_ entry with the matching output format
	 */
	for (auto it = debayerInfos_.begin(); it != debayerInfos_.end(); it++) {
		if (it->second.outPixelFmt == outputFormat)
			return (*it->second.getOutStride)(outSize);
	}

	return 0;
}

int SwIspLinaro::IspWorker::configure(const StreamConfiguration &inputCfg,
				      const StreamConfiguration &outputCfg)
{
	if (setDebayerInfo(inputCfg.pixelFormat) != 0) {
		LOG(SoftwareIsp, Error)
			<< "Input format " << inputCfg.pixelFormat
			<< "not supported";
		return -EINVAL;
	}

	/* check that:
	 * - output format is valid
	 * - output size matches the input size and is valid */
	SizeRange outSizeRange = (*debayerInfo_->getOutSizes)(inputCfg.size);
	if (debayerInfo_->outPixelFmt != outputCfg.pixelFormat ||
	    outputCfg.size.isNull() || !outSizeRange.contains(outputCfg.size) ||
	    (*debayerInfo_->getOutStride)(outputCfg.size) != outputCfg.stride) {
		LOG(SoftwareIsp, Error)
			<< "Invalid output format/size/stride: "
			<< "\n  " << outputCfg.pixelFormat << " ("
			<< debayerInfo_->outPixelFmt << ")"
			<< "\n  " << outputCfg.size << " ("
			<< outSizeRange << ")"
			<< "\n  " << outputCfg.stride << " ("
			<< (*debayerInfo_->getOutStride)(outputCfg.size) << ")";
		return -EINVAL;
	}

	width_ = inputCfg.size.width;
	height_ = inputCfg.size.height;
	stride_ = inputCfg.stride;

	BayerFormat bayerFormat =
		BayerFormat::fromPixelFormat(inputCfg.pixelFormat);
	switch (bayerFormat.order) {
	case BayerFormat::BGGR:
		redShift_ = Point(0, 0);
		break;
	case BayerFormat::GBRG:
		redShift_ = Point(1, 0);
		break;
	case BayerFormat::GRBG:
		redShift_ = Point(0, 1);
		break;
	case BayerFormat::RGGB:
	default:
		redShift_ = Point(1, 1);
		break;
	}

	outStride_ = outputCfg.stride;
	outWidth_  = outputCfg.size.width;
	outHeight_ = outputCfg.size.height;

	LOG(SoftwareIsp, Info) << "SoftwareISP configuration: "
		<< inputCfg.size << "-" << inputCfg.pixelFormat << " -> "
		<< outputCfg.size << "-" << outputCfg.pixelFormat;

	/* set r/g/b gains to 1.0 until frame data collected */
	rNumerat_ = 256;
	bNumerat_ = 256;
	gNumerat_ = 256;

	/* set awb correction values to 1 until frame data collected */
	stats_.blue_awb_correction = 1;
	stats_.red_awb_correction = 1;

	return 0;
}

/* May not be called before SwIspLinaro::IspWorker::configure() */
unsigned int SwIspLinaro::IspWorker::outBufferSize()
{
	return outHeight_ * outStride_;
}

std::vector<PixelFormat> SwIspLinaro::formats(PixelFormat inputFormat)
{
	ASSERT(ispWorker_ != nullptr);

	return ispWorker_->formats(inputFormat);
}

SizeRange SwIspLinaro::sizes(PixelFormat inputFormat, const Size &inputSize)
{
	ASSERT(ispWorker_ != nullptr);

	return ispWorker_->sizes(inputFormat, inputSize);
}

std::tuple<unsigned int, unsigned int>
SwIspLinaro::strideAndFrameSize(const PixelFormat &outputFormat, const Size &size)
{
	ASSERT(ispWorker_ != nullptr);

	unsigned int stride = ispWorker_->outStride(outputFormat, size);

	return std::make_tuple(stride, stride * size.height);
}

int SwIspLinaro::configure(const StreamConfiguration &inputCfg,
			   const std::vector<std::reference_wrapper<StreamConfiguration>> &outputCfgs)
{
	ASSERT(ispWorker_ != nullptr);

	if (outputCfgs.size() != 1) {
		LOG(SoftwareIsp, Error)
			<< "Unsupported number of output streams: "
			<< outputCfgs.size();
		return -EINVAL;
	}

	return ispWorker_->configure(inputCfg, outputCfgs[0]);
}

int SwIspLinaro::exportBuffers(unsigned int output, unsigned int count,
			       std::vector<std::unique_ptr<FrameBuffer>> *buffers)
{
	ASSERT(ispWorker_ != nullptr);

	/* single output for now */
	if (output >= 1)
		return -EINVAL;

	unsigned int bufSize = ispWorker_->outBufferSize();

	/* TODO: allocate from dma_heap; memfd buffs aren't allowed in FrameBuffer */
	for (unsigned int i = 0; i < count; i++) {
		std::string name = "frame-" + std::to_string(i);

		const int ispFd = memfd_create(name.c_str(), 0);
		int ret = ftruncate(ispFd, bufSize);
		if (ret < 0) {
			LOG(SoftwareIsp, Error)
				<< "ftruncate() for memfd failed "
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

int SwIspLinaro::queueBuffers(FrameBuffer *input,
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

int SwIspLinaro::start()
{
	ispWorkerThread_.start();
	return 0;
}

void SwIspLinaro::stop()
{
	ispWorkerThread_.exit();
	ispWorkerThread_.wait();
}

void SwIspLinaro::IspWorker::process(FrameBuffer *input, FrameBuffer *output)
{	
	/* Copy metadata from the input buffer */
	FrameMetadata &metadata = output->_d()->metadata();
	metadata.status = input->metadata().status;
	metadata.sequence = input->metadata().sequence;
	metadata.timestamp = input->metadata().timestamp;

	MappedFrameBuffer in(input, MappedFrameBuffer::MapFlag::Read);
	MappedFrameBuffer out(output, MappedFrameBuffer::MapFlag::Write);
	if (!in.isValid() || !out.isValid()) {
		LOG(SoftwareIsp, Error) << "mmap-ing buffer(s) failed";
		metadata.status = FrameMetadata::FrameError;
		swIsp_->outputBufferReady.emit(output);
		swIsp_->inputBufferReady.emit(input);
		return;
	}

	sumR_ = 0;
	sumB_ = 0;
	sumG_ = 0;

	exposurebins[0] = 0;
	exposurebins[1] = 0;
	exposurebins[2] = 0;
	exposurebins[3] = 0;
	exposurebins[4] = 0;


	const uint8_t *src = in.planes()[0].data();
	uint8_t *dst = out.planes()[0].data();

	if (debayerInfo_->debayer2) {
		/* Skip first 4 lines for debayer interpolation purposes */
		src += stride_ * 4;
		int lines = outHeight_ / 4;
		while (lines--) {
			(this->*debayerInfo_->stats0)(src);
			(this->*debayerInfo_->debayer0)(dst, src);
			src += stride_;
			dst += outStride_;

			(this->*debayerInfo_->debayer1)(dst, src);
			src += stride_;
			dst += outStride_;

			(this->*debayerInfo_->stats2)(src);
			(this->*debayerInfo_->debayer2)(dst, src);
			src += stride_;
			dst += outStride_;

			(this->*debayerInfo_->debayer3)(dst, src);
			src += stride_;
			dst += outStride_;
		}
	} else {
		/* Skip first 2 lines for debayer interpolation purposes */
		src += stride_ * 2;
		int lines = outHeight_ / 2;
		while (lines--) {
			(this->*debayerInfo_->stats0)(src);
			(this->*debayerInfo_->debayer0)(dst, src);
			src += stride_;
			dst += outStride_;

			(this->*debayerInfo_->debayer1)(dst, src);
			src += stride_;
			dst += outStride_;
		}
	}

	metadata.planes()[0].bytesused = out.planes()[0].size();

	(this->*debayerInfo_->finishStats)();
	*swIsp_->sharedStats_ = stats_;
	swIsp_->ispStatsReady.emit(0);

	swIsp_->outputBufferReady.emit(output);
	swIsp_->inputBufferReady.emit(input);
}

void SwIspLinaro::process(FrameBuffer *input, FrameBuffer *output)
{
	ispWorker_->invokeMethod(&SwIspLinaro::IspWorker::process,
				 ConnectionTypeQueued, input, output);
}

} /* namespace libcamera */
