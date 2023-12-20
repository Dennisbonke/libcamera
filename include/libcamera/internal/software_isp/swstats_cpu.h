/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2023, Linaro Ltd
 * Copyright (C) 2023, Red Hat Inc.
 *
 * Authors:
 * Hans de Goede <hdegoede@redhat.com> 
 *
 * swstats_cpu.h - CPU based software statistics implementation
 */

#pragma once

#include "libcamera/internal/shared_mem_object.h"
#include "libcamera/internal/software_isp/swisp_stats.h"
#include "libcamera/internal/software_isp/swstats.h"

namespace libcamera {

class SwStatsCpu : public SwStats
{
public:
	SwStatsCpu();
	~SwStatsCpu() { }

	bool isValid() const { return sharedStats_.fd().isValid(); }
	const SharedFD &getStatsFD() { return sharedStats_.fd(); }
	int configure(const StreamConfiguration &inputCfg);
	/* FIXME this should be dropped once AWB has moved to the IPA */
	SwIspStats getStats() { return *sharedStats_; }
private:
	/* Bayer 10bpp packed */
	void statsBGGR10PLine0(const uint8_t *src, unsigned int stride);
	void statsGBRG10PLine0(const uint8_t *src, unsigned int stride);
	/* Bayer 10 bpp unpacked */
	void statsBGGR10Line0(const uint8_t *src, unsigned int stride);
	void resetStats(void);
	void finishStats(void);

	void statsRGBIR10Line0(const uint8_t *src, unsigned int stride);
	void statsRGBIR10Line2(const uint8_t *src, unsigned int stride);
	void finishRaw10Stats(void);
	static SizeRange outSizesRaw10(const Size &inSize);
	static unsigned int outStrideRaw10(const Size &outSize);


	SharedMemObject<SwIspStats> sharedStats_;
	SwIspStats stats_;

	unsigned int bright_sum_;
	unsigned int too_bright_sum_;

	unsigned int red_count;
	unsigned int blue_count;
	unsigned int green_count;

	bool swap_lines_;
};

} /* namespace libcamera */
