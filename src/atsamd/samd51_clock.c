// Code to setup peripheral clocks on the SAMD51
//
// Copyright (C) 2018-2019  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "compiler.h" // DIV_ROUND_CLOSEST
#include "internal.h" // enable_pclock

// The "generic clock generators" that are configured
#define CLKGEN_MAIN 0
#define CLKGEN_48M 3
#define CLKGEN_2M 4

#define FREQ_MAIN 120000000
#define FREQ_32K 32768
#define FREQ_48M 48000000
#define FREQ_2M 2000000

// Configure a clock generator using a given source as input
static inline void
gen_clock(uint32_t clkgen_id, uint32_t flags)
{
    GCLK->GENCTRL[clkgen_id].reg = flags | GCLK_GENCTRL_GENEN;
    while (GCLK->SYNCBUSY.reg & GCLK_SYNCBUSY_GENCTRL(1 << clkgen_id))
        ;
}

// Route a peripheral clock to a given clkgen
static inline void
route_pclock(uint32_t pclk_id, uint32_t clkgen_id)
{
    uint32_t val = GCLK_PCHCTRL_GEN(clkgen_id) | GCLK_PCHCTRL_CHEN;
    GCLK->PCHCTRL[pclk_id].reg = val;
    while (GCLK->PCHCTRL[pclk_id].reg != val)
        ;
}

// Enable a peripheral clock and power to that peripheral
void
enable_pclock(uint32_t pclk_id, uint32_t pm_id)
{
    route_pclock(pclk_id, CLKGEN_48M);
    uint32_t pm_port = pm_id / 32, pm_bit = 1 << (pm_id % 32);
    (&MCLK->APBAMASK.reg)[pm_port] |= pm_bit;
}

// Return the frequency of the given peripheral clock
uint32_t
get_pclock_frequency(uint32_t pclk_id)
{
    return FREQ_48M;
}

// Configure a dpll to a given clock multiplier
static void
config_dpll(uint32_t pll, uint32_t mul, uint32_t ctrlb)
{
    OSCCTRL->Dpll[pll].DPLLCTRLA.reg = 0;
    while (OSCCTRL->Dpll[0].DPLLSYNCBUSY.reg & OSCCTRL_DPLLSYNCBUSY_ENABLE)
        ;
    OSCCTRL->Dpll[pll].DPLLRATIO.reg = OSCCTRL_DPLLRATIO_LDR(mul - 1);
    while (OSCCTRL->Dpll[0].DPLLSYNCBUSY.reg & OSCCTRL_DPLLSYNCBUSY_DPLLRATIO)
        ;
    OSCCTRL->Dpll[pll].DPLLCTRLB.reg = ctrlb | OSCCTRL_DPLLCTRLB_LBYPASS;
    OSCCTRL->Dpll[pll].DPLLCTRLA.reg = OSCCTRL_DPLLCTRLA_ENABLE;
    uint32_t mask = OSCCTRL_DPLLSTATUS_CLKRDY | OSCCTRL_DPLLSTATUS_LOCK;
    while ((OSCCTRL->Dpll[pll].DPLLSTATUS.reg & mask) != mask)
        ;
}

// Configure the dfll
static void
config_dfll(uint32_t dfllmul, uint32_t ctrlb)
{
    // Disable the dfllmul and reenable in this order due to chip errata
    OSCCTRL->DFLLCTRLA.reg = 0;
    while (OSCCTRL->DFLLSYNC.reg & OSCCTRL_DFLLSYNC_ENABLE)
        ;
    OSCCTRL->DFLLMUL.reg = dfllmul;
    while (OSCCTRL->DFLLSYNC.reg & OSCCTRL_DFLLSYNC_DFLLMUL)
        ;
    OSCCTRL->DFLLCTRLB.reg = 0;
    while (OSCCTRL->DFLLSYNC.reg & OSCCTRL_DFLLSYNC_DFLLCTRLB)
        ;
    OSCCTRL->DFLLCTRLA.reg = OSCCTRL_DFLLCTRLA_ENABLE;
    while (OSCCTRL->DFLLSYNC.reg & OSCCTRL_DFLLSYNC_ENABLE)
        ;
    OSCCTRL->DFLLVAL.reg = OSCCTRL->DFLLVAL.reg;
    while(OSCCTRL->DFLLSYNC.reg & OSCCTRL_DFLLSYNC_DFLLVAL)
        ;
    OSCCTRL->DFLLCTRLB.reg = ctrlb;
    while (OSCCTRL->DFLLSYNC.reg & OSCCTRL_DFLLSYNC_DFLLCTRLB)
        ;
}

// Initialize the clocks using an external 32K crystal
static void
clock_init_32k(void)
{
    // Enable external 32Khz crystal
    uint32_t val = (OSC32KCTRL_XOSC32K_ENABLE | OSC32KCTRL_XOSC32K_EN32K
                    | OSC32KCTRL_XOSC32K_CGM_XT | OSC32KCTRL_XOSC32K_XTALEN);
    OSC32KCTRL->XOSC32K.reg = val;
    while (!(OSC32KCTRL->STATUS.reg & OSC32KCTRL_STATUS_XOSC32KRDY))
        ;

    // Generate 120Mhz clock on PLL0 (with XOSC32 as reference)
    uint32_t mul = DIV_ROUND_CLOSEST(FREQ_MAIN, FREQ_32K);
    config_dpll(0, mul, OSCCTRL_DPLLCTRLB_REFCLK_XOSC32);

    // Switch main clock to 120Mhz PLL0
    gen_clock(CLKGEN_MAIN, GCLK_GENCTRL_SRC_DPLL0);

    // Generate 48Mhz clock on PLL1 (with XOSC32 as reference)
    mul = DIV_ROUND_CLOSEST(FREQ_48M, FREQ_32K);
    config_dpll(1, mul, OSCCTRL_DPLLCTRLB_REFCLK_XOSC32);
    gen_clock(CLKGEN_48M, GCLK_GENCTRL_SRC_DPLL1);
}

// Initialize clocks from factory calibrated internal clock
static void
clock_init_internal(void)
{
    // Enable USB clock recovery mode if applicable
    if (CONFIG_USBSERIAL) {
        // Temporarily switch main clock to internal 32K clock
        gen_clock(CLKGEN_MAIN, GCLK_GENCTRL_SRC_OSCULP32K);

        // Configure DFLL48M clock (with USB 1Khz SOF as reference)
        uint32_t mul = DIV_ROUND_CLOSEST(FREQ_48M, 1000);
        uint32_t dfllmul = OSCCTRL_DFLLMUL_FSTEP(10) | OSCCTRL_DFLLMUL_MUL(mul);
        uint32_t ctrlb = (OSCCTRL_DFLLCTRLB_MODE | OSCCTRL_DFLLCTRLB_USBCRM
                          | OSCCTRL_DFLLCTRLB_CCDIS);
        config_dfll(dfllmul, ctrlb);
    }

    // Route factory calibrated DFLL48M to CLKGEN_48M
    gen_clock(CLKGEN_48M, GCLK_GENCTRL_SRC_DFLL);

    // Generate CLKGEN_2M (with CLKGEN_48M as reference)
    uint32_t div = DIV_ROUND_CLOSEST(FREQ_48M, FREQ_2M);
    gen_clock(CLKGEN_2M, GCLK_GENCTRL_SRC_DFLL | GCLK_GENCTRL_DIV(div));

    // Generate 120Mhz clock on PLL0 (with CLKGEN_2M as reference)
    route_pclock(OSCCTRL_GCLK_ID_FDPLL0, CLKGEN_2M);
    uint32_t mul = DIV_ROUND_CLOSEST(FREQ_MAIN, FREQ_2M);
    config_dpll(0, mul, OSCCTRL_DPLLCTRLB_REFCLK_GCLK);

    // Switch main clock to 120Mhz PLL0
    gen_clock(CLKGEN_MAIN, GCLK_GENCTRL_SRC_DPLL0);
}

void
SystemInit(void)
{
    // Reset GCLK
    GCLK->CTRLA.reg = GCLK_CTRLA_SWRST;
    while (GCLK->SYNCBUSY.reg & GCLK_SYNCBUSY_SWRST)
        ;

    // Init clocks
    if (CONFIG_CLOCK_REF_X32K)
        clock_init_32k();
    else
        clock_init_internal();

    // Enable SAMD51 cache
    CMCC->CTRL.reg = 1;
}