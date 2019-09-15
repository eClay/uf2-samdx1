#include "uf2.h"
#define SYSCTRL_FUSES_OSC32K_CAL_ADDR   (NVMCTRL_OTP4 + 4)
#define SYSCTRL_FUSES_OSC32K_CAL_Pos   6
#define 	SYSCTRL_FUSES_OSC32K_ADDR   SYSCTRL_FUSES_OSC32K_CAL_ADDR
#define 	SYSCTRL_FUSES_OSC32K_Pos   SYSCTRL_FUSES_OSC32K_CAL_Pos
#define 	SYSCTRL_FUSES_OSC32K_Msk   (0x7Fu << SYSCTRL_FUSES_OSC32K_Pos)

volatile bool g_interrupt_enabled = true;

static void dfll_sync(void) {
    while (OSCCTRL->STATUS.bit.DFLLRDY == 0)
        ;
}

#define NVM_SW_CALIB_DFLL48M_COARSE_VAL   58
#define NVM_SW_CALIB_DFLL48M_FINE_VAL     64


void system_init(void) {

  /* Set 3 Flash Wait State for 48MHz */
  NVMCTRL->CTRLB.reg |= NVMCTRL_CTRLB_RWS(3);

#if defined(CRYSTALLESS)
  /* Configure OSC16M as source for GCLK_GEN 2 */
  GCLK->GENCTRL[2].reg = GCLK_GENCTRL_SRC_OSC16M_Val | GCLK_GENCTRL_GENEN;
  while (GCLK->SYNCBUSY.bit.GENCTRL2) {}

  // Turn on DFLL with USB correction and sync to internal 8 mhz oscillator
  OSCCTRL->DFLLCTRL.reg = OSCCTRL_DFLLCTRL_ENABLE;
  dfll_sync();

  OSCCTRL_DFLLVAL_Type dfllval_conf = {0};
  uint32_t coarse =( *((uint32_t *)(NVMCTRL_OTP5)
		       + (NVM_SW_CALIB_DFLL48M_COARSE_VAL / 32))
		     >> (NVM_SW_CALIB_DFLL48M_COARSE_VAL % 32))
    & ((1 << 6) - 1);
  if (coarse == 0x3f) {
    coarse = 0x1f;
  }
  dfllval_conf.bit.COARSE  = coarse;
  // TODO(tannewt): Load this from a well known flash location so that it can be
  // calibrated during testing.
  dfllval_conf.bit.FINE    = 0x1ff;

  OSCCTRL->DFLLMUL.reg = OSCCTRL_DFLLMUL_CSTEP( 0x1f / 4 ) | // Coarse step is 31, half of the max value
                         OSCCTRL_DFLLMUL_FSTEP( 10 ) |
                         48000;
  OSCCTRL->DFLLVAL.reg = dfllval_conf.reg;
  OSCCTRL->DFLLCTRL.reg = 0;
  dfll_sync();
  OSCCTRL->DFLLCTRL.reg = OSCCTRL_DFLLCTRL_MODE |
                          OSCCTRL_DFLLCTRL_CCDIS |
                          OSCCTRL_DFLLCTRL_USBCRM | /* USB correction */
                          OSCCTRL_DFLLCTRL_BPLCKC;
  dfll_sync();
  OSCCTRL->DFLLCTRL.reg |= OSCCTRL_DFLLCTRL_ENABLE ;
  dfll_sync();

//   GCLK_CLKCTRL_Type clkctrl={0};
//   uint16_t temp;
//   GCLK->CLKCTRL.bit.ID = 2; // GCLK_ID - DFLL48M Reference
//   temp = GCLK->CLKCTRL.reg;
//   clkctrl.bit.CLKEN = 1;
//   clkctrl.bit.WRTLOCK = 0;
//   clkctrl.bit.GEN = GCLK_CLKCTRL_GEN_GCLK0_Val;
//   GCLK->CLKCTRL[2].reg |= (clkctrl.reg | temp);

#else

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

#endif

    // Add GCLK_GENCTRL_OE below to output GCLK0 on the SWCLK pin.
    GCLK->GENCTRL[0].reg = GCLK_GENCTRL_SRC_DFLL48M | GCLK_GENCTRL_IDC | GCLK_GENCTRL_GENEN;
    while (GCLK->SYNCBUSY.bit.GENCTRL0) {}

    SysTick_Config(1000);

    // Uncomment these two lines to output GCLK0 on the SWCLK pin.
    // PORT->Group[0].PINCFG[30].bit.PMUXEN = 1;
    // Set the port mux mask for odd processor pin numbers, PA30 = 30 is even number, PMUXE = PMUX Even
    // PORT->Group[0].PMUX[30 / 2].reg |= PORT_PMUX_PMUXE_H;
}

void SysTick_Handler(void) { LED_TICK(); }
