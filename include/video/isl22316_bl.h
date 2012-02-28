/*
 * Copyright 2011 Creative Product Design
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

enum isl22316_bl_enable_types {
	ISL22316_BL_ENABLE_NONE,
	ISL22316_BL_ENABLE_GPIO_LOW,
	ISL22316_BL_ENABLE_GPIO_HIGH
};

struct isl22316_bl_platform_data {
	int inverted;

	enum isl22316_bl_enable_types enable_type;
	unsigned int enable_gpio;
};
