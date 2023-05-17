#include <Arduino.h>
#include <Adafruit_ZeroDMA.h>

#define DATA_BUF_SIZE (256)
#define SAMPLE_RATE (256000) // max 350'000 samples/s
// Frequency of output signal: f = SAMPLE_RATE / DATA_BUF_SIZE (in this case 1 khz)

#define DAC_10BIT_BITMASK (0x03ff);

Adafruit_ZeroDMA DAC_DMA;
DmacDescriptor *dac_dmac_desc;

uint16_t dac_buf[DATA_BUF_SIZE];

void init_PORT() { // Datasheet: 7.1 Multiplexed Signals, 23 PORT
  PORT->Group[PORTA].PMUX[2 >> 1].bit.PMUXE = 0x01; // DAC (VOUT), PIN (PA02), Arduino (A0)
  PORT->Group[PORTA].PINCFG[2].reg = PORT_PINCFG_PMUXEN ;
}

void init_GCLK() { // Datasheet: 15 GCLK (Generic Clock Controller)
  while(GCLK->STATUS.bit.SYNCBUSY);
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID_TCC2_TC3;
  while(GCLK->STATUS.bit.SYNCBUSY);
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID_EVSYS_0;
  while(GCLK->STATUS.bit.SYNCBUSY);
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID_DAC;
  while(GCLK->STATUS.bit.SYNCBUSY);
}

void init_PM() { // Datasheet: 16 PM (Power Manager)
  PM->AHBMASK.reg |= PM_AHBMASK_DMAC;
  PM->APBBMASK.reg |= PM_APBBMASK_DMAC;
  PM->APBCMASK.reg |= PM_APBCMASK_EVSYS | PM_APBCMASK_TC3 | PM_APBCMASK_DAC;
}

void init_EVSYS() { // Datasheet: 24 EVSYS (Event System)
  EVSYS->USER.reg = EVSYS_USER_CHANNEL(0x1U) | EVSYS_USER_USER(EVSYS_ID_USER_DAC_START); // enable event listener for DAC Start Conversion on event channel 0
  // user channel = n - 1
  EVSYS->CHANNEL.reg = EVSYS_CHANNEL_CHANNEL(0x0U) | EVSYS_CHANNEL_EVGEN(EVSYS_ID_GEN_TC3_MCX_0) | EVSYS_CHANNEL_PATH_ASYNCHRONOUS | EVSYS_CHANNEL_EDGSEL_NO_EVT_OUTPUT; // enable event generator Timer 3 Match on event channel 0
  // DAC START Conversion event user needs asynchronous path (see Datasheet Table 24.2)
  // operational overview:
  // TC3 generates MATCH/CAPTURE event -> | Event Channel 0 | -> DAC receives START CONVERSION event
}

void init_TC() { // Datasheet: 30 TC (Timer/Counter)
  TC3->COUNT16.CTRLA.reg = TC_CTRLA_SWRST;
  while(TC3->COUNT16.STATUS.bit.SYNCBUSY);
  TC3->COUNT16.CTRLA.reg = TC_CTRLA_ENABLE | TC_CTRLA_MODE_COUNT16 | TC_CTRLA_PRESCALER_DIV1 | TC_CTRLA_WAVEGEN_MFRQ;
  TC3->COUNT16.CC[0].reg = SystemCoreClock / SAMPLE_RATE - 1; // SystemCoreClock is usually 48MHz (GCLK0)
  TC3->COUNT16.EVCTRL.reg |= TC_EVCTRL_MCEO0; // enable timer event on match
  while(TC3->COUNT16.STATUS.bit.SYNCBUSY);
}

void init_DAC() { // Datasheet: 35 DAC (Digital-to-Analog Converter)
  DAC->CTRLA.bit.ENABLE = 0;
  while(DAC->STATUS.bit.SYNCBUSY);
  DAC->CTRLB.reg = DAC_CTRLB_REFSEL_AVCC | DAC_CTRLB_EOEN;
  DAC->EVCTRL.reg = DAC_EVCTRL_STARTEI | DAC_EVCTRL_EMPTYEO; // enable DAC START CONVERSION event & DAC DATA BUFFER EMPTY event
  DAC->CTRLA.bit.ENABLE = 1;
  while(DAC->STATUS.bit.SYNCBUSY);
  // operational overview:
  // DAC receives DAC START CONVERSION event -> DAC copies value from DATABUF to DATA register & outputs analog value -> DAC generates DAC DATA BUFFER EMPTY event
}

void init_DMAC() { // Datasheet: 20 DMAC (Direct Memory Access Controller)
  DAC_DMA.allocate();
  DAC_DMA.setTrigger(DAC_DMAC_ID_EMPTY); // Trigger on DAC DATA BUFFER EMPTY event (does not need to be connected via EVSYS)
  DAC_DMA.setAction(DMA_TRIGGER_ACTON_BEAT); // Transfer one beat per trigger event (in this case HALFWORD: 16 bits)
  dac_dmac_desc = DAC_DMA.addDescriptor(
    (void *) &dac_buf[0], // src address (Buffer in memory)
    (void *) &DAC->DATABUF.reg, // dst address (DATABUF register of DAC)
    DATA_BUF_SIZE, // Number of beats
    DMA_BEAT_SIZE_HWORD, // HWORD: 16 bits
    true, // increase src address
    false); // don't increase dst address
  DAC_DMA.loop(true);
  DAC_DMA.startJob();
}

void setup() {
  for (int i = 0; i < DATA_BUF_SIZE; ++i) {
    dac_buf[i] = (uint16_t)((double)0x03ff * 0.5 * (1.0 + sin(PI * 2 * (double)i / (double)DATA_BUF_SIZE))) & DAC_10BIT_BITMASK;
  }

  init_PORT();
  init_GCLK();
  init_PM();
  init_EVSYS();
  init_TC();
  init_DAC();
  init_DMAC();
}

void loop() {

}