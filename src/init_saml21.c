#include "uf2.h"

volatile bool g_interrupt_enabled = true;

// SAMD21 starts at 1MHz by default.
uint32_t current_cpu_frequency_MHz = 1;

static void gclk_sync(void) {
    while (GCLK->SYNCBUSY.reg & GCLK_SYNCBUSY_MASK)
        ;
}
// static void dfll_sync(void) {
//     while ((OSCCTRL->STATUS.reg & OSCCTRL_STATUS_DFLLRDY) == 0)
//         ;
// }
static void pll_sync(void) {
  while ( OSCCTRL->DPLLSYNCBUSY.reg & OSCCTRL_DPLLSYNCBUSY_MASK );
}

#define NVM_SW_CALIB_DFLL48M_COARSE_VAL   26

void system_init(void) {
  
  // Don't use cristalless (aka use ext. crystal)
  
  NVMCTRL->CTRLB.bit.MANW = 1; // ERRATA
  
  NVMCTRL->CTRLB.reg |= NVMCTRL_CTRLB_RWS_DUAL ;  
  
  OSC32KCTRL->XOSC32K.reg =
    (OSC32KCTRL_XOSC32K_STARTUP(0x4u) | OSC32KCTRL_XOSC32K_XTALEN | OSC32KCTRL_XOSC32K_EN32K | OSC32KCTRL_XOSC32K_EN1K);
  OSC32KCTRL->XOSC32K.bit.ENABLE = 1;
  while ( (OSC32KCTRL->STATUS.reg & OSC32KCTRL_STATUS_XOSC32KRDY) == 0 );

  OSCCTRL->DPLLRATIO.reg = ( OSCCTRL_DPLLRATIO_LDR(2929u) | OSCCTRL_DPLLRATIO_LDRFRAC(0u) );
  pll_sync();

  OSCCTRL->DPLLCTRLB.reg = OSCCTRL_DPLLCTRLB_REFCLK(0);
  OSCCTRL->DPLLPRESC.reg = 0;
  pll_sync();

  OSCCTRL->DPLLCTRLA.reg = OSCCTRL_DPLLCTRLA_ENABLE;
  pll_sync();

  while ( (OSCCTRL->DPLLSTATUS.reg & OSCCTRL_DPLLSTATUS_CLKRDY) != OSCCTRL_DPLLSTATUS_CLKRDY );

  GCLK->GENCTRL[0].reg =
    ( GCLK_GENCTRL_DIV(2) | GCLK_GENCTRL_SRC_DPLL96M | GCLK_GENCTRL_IDC | GCLK_GENCTRL_GENEN );
  gclk_sync();

  current_cpu_frequency_MHz = 48;

}

void SysTick_Handler(void) { LED_TICK(); }
