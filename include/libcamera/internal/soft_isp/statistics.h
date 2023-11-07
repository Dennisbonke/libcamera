/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2023, Linaro Ltd
 *
 * statistics.h - Statistics data format used by the software ISP
 */

#pragma once

#include <vector>

namespace libcamera {

/* TODO: move to ipa::simple namespace? */
struct Statistics {
	float bright_ratio;
	float too_bright_ratio;
	std::vector<int> histRed;
	std::vector<int> histGreenRed;
	std::vector<int> histGreenBlue;
	std::vector<int> histBlue;
	std::vector<int> histLuminance;
};

}; /* namespace libcamera */