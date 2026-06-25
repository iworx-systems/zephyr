/*
 * Copyright (c) 2026 iWorx Systems Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/toolchain.h>
#include <zephyr/linker/sections.h>

/*
 * M480 register addresses (see modules/hal/nuvoton/m48x/Devices/M480/Include).
 *   SYS->REGLCTL  protected-register unlock port (write 0x59,0x16,0x88)
 *   CLK->AHBCLK   SPIMCKEN = bit 14
 *   SPIM->CTL1    CACHEOFF = bit 1, CCMEN = bit 2
 */
#define SYS_REGLCTL  0x40000100
#define CLK_AHBCLK   0x40000204
#define SPIM_CTL1    0x40007004

_ASM_FILE_PROLOGUE

GTEXT(soc_early_reset_hook)

SECTION_FUNC(TEXT, soc_early_reset_hook)

	/*
	 * Expose the M480 SPIM cache RAM as system SRAM -- the upper 32 KB
	 * (0x20020000-0x20028000, a.k.a. SRAM "Bank1").  It is dead at power-on
	 * and only appears once SPIMCKEN + CACHEOFF + CCMEN are set.
	 *
	 * This MUST run here, before any stack or RAM is touched: vector[0]
	 * (the initial MSP) and Zephyr's stacks/bss can live in Bank1, so a C
	 * hook's prologue would push to a stack that is itself in this
	 * not-yet-enabled bank and fault before it could enable it.  Implemented
	 * in assembly using only scratch registers r0-r2 (no stack access).
	 */

	/* Unlock protected registers: REGLCTL <= 0x59, 0x16, 0x88 */
	movw	r0, #(SYS_REGLCTL & 0xffff)
	movt	r0, #(SYS_REGLCTL >> 16)
	movs	r1, #0x59
	str	r1, [r0]
	movs	r1, #0x16
	str	r1, [r0]
	movs	r1, #0x88
	str	r1, [r0]

	/* CLK->AHBCLK |= SPIMCKEN (bit 14) */
	movw	r0, #(CLK_AHBCLK & 0xffff)
	movt	r0, #(CLK_AHBCLK >> 16)
	ldr	r1, [r0]
	movw	r2, #0x4000
	orr	r1, r1, r2
	str	r1, [r0]

	/* SPIM->CTL1 |= CACHEOFF (bit 1) | CCMEN (bit 2) */
	movw	r0, #(SPIM_CTL1 & 0xffff)
	movt	r0, #(SPIM_CTL1 >> 16)
	ldr	r1, [r0]
	orr	r1, r1, #0x6
	str	r1, [r0]

	/* Wait until cache-RAM mode (CCMEN) is active before returning */
.L_wait_ccmen:
	ldr	r1, [r0]
	tst	r1, #0x4
	beq	.L_wait_ccmen

	bx	lr
