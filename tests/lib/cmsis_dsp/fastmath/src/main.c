/*
 * Copyright (c) 2021 Stephanos Ioannidis <root@stephanos.io>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <ztest.h>
#include <zephyr.h>

extern void test_fastmath_q15(void);
extern void test_fastmath_q31(void);
extern void test_fastmath_f16(void);
extern void test_fastmath_f32(void);

void test_main(void)
{
	test_fastmath_q15();
	test_fastmath_q31();
#ifdef CONFIG_CMSIS_DSP_FLOAT16
	test_fastmath_f16();
#endif
	test_fastmath_f32();
}
