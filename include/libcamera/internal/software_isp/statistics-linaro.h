/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2023, Linaro Ltd
 *
 * statistics.h - Statistics data format used by the software ISP
 */

#pragma once

namespace libcamera {

struct SwIspStats {
	float bright_ratio;
	float too_bright_ratio;
	unsigned long sumR_;
	unsigned long sumB_;
	unsigned long sumG_;

	unsigned int red_count;
	unsigned int blue_count;
	unsigned int green_count;

	unsigned int red_awb_correction;
	unsigned int blue_awb_correction;

};

}
