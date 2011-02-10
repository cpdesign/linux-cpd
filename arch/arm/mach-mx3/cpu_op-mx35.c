/*
 * Copyright (C) 2010 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/types.h>
#include <mach/hardware.h>
#include <linux/kernel.h>

static struct cpu_op mx35_cpu_op[] = {
	{
	.cpu_rate = 133000000,},
	{
	.cpu_rate = 532000000,},
};

struct cpu_op *mx35_get_cpu_op(int *op)
{
	*op = ARRAY_SIZE(mx35_cpu_op);
	return mx35_cpu_op;
}
