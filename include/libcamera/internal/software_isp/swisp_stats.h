/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2023, Linaro Ltd
 *
 * swisp_stats.h - Statistics data format used by the software ISP and software IPA
 */

#pragma once

namespace libcamera {

/**
 * \brief Struct that holds the statistics for the Software ISP.
 */
struct SwIspStats {
	/**
	 * \brief Holds the sum of all the red pixels.
	 */
	unsigned long sumR_;
	/**
	 * \brief Holds the sum of all the blue pixels.
	 */
	unsigned long sumB_;
	/**
	 * \brief Holds the sum of all the green pixels.
	 */
	unsigned long sumG_;

	/**
	 * \brief Holds the ratio of bright pixels.
	 */
	float bright_ratio;
	/**
	 * \brief Holds the ratio of too bright pixels.
	 */
	float too_bright_ratio;

	/**
	 * \brief A histogram of luminance values.
	 */
	int y_histogram[16];
};

} /* namespace libcamera */
