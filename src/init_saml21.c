#include "uf2.h"

#if !defined(SAML21)
  #define   SYSCTRL_FUSES_OSC32K_CAL_ADDR  (NVMCTRL_OTP4 + 4)
  #define   SYSCTRL_FUSES_OSC32K_CAL_Pos   6
  #define 	SYSCTRL_FUSES_OSC32K_ADDR      SYSCTRL_FUSES_OSC32K_CAL_ADDR
  #define 	SYSCTRL_FUSES_OSC32K_Pos       SYSCTRL_FUSES_OSC32K_CAL_Pos
  #define 	SYSCTRL_FUSES_OSC32K_Msk       (0x7Fu << SYSCTRL_FUSES_OSC32K_Pos)
#endif

volatile bool g_interrupt_enabled = true;

// SAMD21 starts at 1MHz by default.
uint32_t current_cpu_frequency_MHz = 1;

#if defined(SAML21)
  static void gclk_sync(void) {
      while (GCLK->SYNCBUSY.reg & GCLK_SYNCBUSY_MASK)
          ;
  }
  static void dfll_sync(void) {
      while ((OSCCTRL->STATUS.reg & OSCCTRL_STATUS_DFLLRDY) == 0)
          ;
  }
  static void pll_sync(void) {
    while ( OSCCTRL->DPLLSYNCBUSY.reg & OSCCTRL_DPLLSYNCBUSY_MASK );
  }
  #define NVM_SW_CALIB_DFLL48M_COARSE_VAL   26
#elif defined(SAMD21)
  static void gclk_sync(void) {
      while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY)
          ;
  }
  static void dfll_sync(void) {
      while ((SYSCTRL->PCLKSR.reg & SYSCTRL_PCLKSR_DFLLRDY) == 0)
          ;
  }
  #define NVM_SW_CALIB_DFLL48M_COARSE_VAL   58
  #define NVM_SW_CALIB_DFLL48M_FINE_VAL     64
#endif


void system_init(void) {
  
  // (Don't use cristalless)
  
  NVMCTRL->CTRLB.bit.MANW = 1; // ERRATA

  #if defined(SAML21)
  
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
    
    GCLK->GENCTRL[GENERIC_CLOCK_GENERATOR_MAIN].reg =
      ( GCLK_GENCTRL_DIV(2) | GCLK_GENCTRL_SRC_DPLL96M | GCLK_GENCTRL_IDC | GCLK_GENCTRL_GENEN );
    gclk_sync();
  
  #elif defined(SAMD21)
  
    NVMCTRL->CTRLB.bit.RWS = 1;
  
    SYSCTRL->XOSC32K.reg =
        SYSCTRL_XOSC32K_STARTUP(6) | SYSCTRL_XOSC32K_XTALEN | SYSCTRL_XOSC32K_EN32K;
    SYSCTRL->XOSC32K.bit.ENABLE = 1;
    while ((SYSCTRL->PCLKSR.reg & SYSCTRL_PCLKSR_XOSC32KRDY) == 0)
        ;

    GCLK->GENDIV.reg = GCLK_GENDIV_ID(1);
    gclk_sync();

    GCLK->GENCTRL.reg = GCLK_GENCTRL_ID(1) | GCLK_GENCTRL_SRC_XOSC32K | GCLK_GENCTRL_GENEN;
    gclk_sync();

    GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(0) | GCLK_CLKCTRL_GEN_GCLK1 | GCLK_CLKCTRL_CLKEN;
    gclk_sync();

    SYSCTRL->DFLLCTRL.bit.ONDEMAND = 0;
    dfll_sync();

    SYSCTRL->DFLLMUL.reg = SYSCTRL_DFLLMUL_CSTEP(31) | SYSCTRL_DFLLMUL_FSTEP(511) |
                           SYSCTRL_DFLLMUL_MUL((CPU_FREQUENCY / (32 * 1024)));
    dfll_sync();

    SYSCTRL->DFLLCTRL.reg |=
        SYSCTRL_DFLLCTRL_MODE | SYSCTRL_DFLLCTRL_WAITLOCK | SYSCTRL_DFLLCTRL_QLDIS;
    dfll_sync();

    SYSCTRL->DFLLCTRL.reg |= SYSCTRL_DFLLCTRL_ENABLE;

    while ((SYSCTRL->PCLKSR.reg & SYSCTRL_PCLKSR_DFLLLCKC) == 0 ||
           (SYSCTRL->PCLKSR.reg & SYSCTRL_PCLKSR_DFLLLCKF) == 0)
        ;
    dfll_sync();

    // Configure DFLL48M as source for GCLK_GEN 0
    GCLK->GENDIV.reg = GCLK_GENDIV_ID(0);
    gclk_sync();

    // Add GCLK_GENCTRL_OE below to output GCLK0 on the SWCLK pin.
    GCLK->GENCTRL.reg =
        GCLK_GENCTRL_ID(0) | GCLK_GENCTRL_SRC_DFLL48M | GCLK_GENCTRL_IDC | GCLK_GENCTRL_GENEN;
    gclk_sync();

    SysTick_Config(1000);
  
  #endif

    // Uncomment these two lines to output GCLK0 on the SWCLK pin.
    // PORT->Group[0].PINCFG[30].bit.PMUXEN = 1;
    // Set the port mux mask for odd processor pin numbers, PA30 = 30 is even number, PMUXE = PMUX Even
    // PORT->Group[0].PMUX[30 / 2].reg |= PORT_PMUX_PMUXE_H;

    current_cpu_frequency_MHz = 48;

}

void SysTick_Handler(void) { LED_TICK(); }
