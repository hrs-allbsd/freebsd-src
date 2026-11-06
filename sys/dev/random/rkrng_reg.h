/*-
 * SPDX-License-Identifier: BSD-2-Clause
 * Copyright (c) 2026 Hiroki Sato <hrs@FreeBSD.org>
 */
#ifndef _DEV_RANDOM_RKRNG_REG_H_
#define _DEV_RANDOM_RKRNG_REG_H_

/* RK3562 TRM V1.0 Part2, section 31.4.3, pp.1655-1656, 1659-1660. */
#define RKRNG_CTRL                      0x0010
#define RKRNG_CTRL_DRNG_REQ              (1U << 3)
#define RKRNG_CTRL_DRNG_WE               (RKRNG_CTRL_DRNG_REQ << 16)
#define RKRNG_STATE                     0x0014
#define RKRNG_STATE_DRNG_ACK             (1U << 3)
#define RKRNG_STATE_DRNG_TEST_FAIL       (1U << 5)
#define RKRNG_STATE_AVAR_OVERFLOW        (1U << 6)
#define RKRNG_STATE_WORK_CLK_ABNORMAL    (1U << 7)
#define RKRNG_STATE_ERRORS               (RKRNG_STATE_DRNG_TEST_FAIL | \
    RKRNG_STATE_AVAR_OVERFLOW | RKRNG_STATE_WORK_CLK_ABNORMAL)
#define RKRNG_DRNG_DATA0                 0x0070
#define RKRNG_BYTES                     32

#endif
