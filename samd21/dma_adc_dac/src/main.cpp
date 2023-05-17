#include <Arduino.h>

#define DATA_BUF_SIZE (32)
#define SAMPLE_RATE (25000) // max 350'000 samples/s (in this case 25'000 samples/s)

// DMAC Descriptors need to be 64 bit (8 Byte) aligned (See Datasheet: 20.8.15 Descriptor Memory Section Base Address, 20.8.16 Write-Back Memory Section Base Address)
DmacDescriptor dmac_section_desc[2] __attribute__ ((aligned (8))); // Dmac Descriptor Section (here only 2 from a max of 12 are used)
DmacDescriptor dmac_section_desc_wrb[2] __attribute__ ((aligned (8))); // Dmac Write-Back Section (here only 2 from a max of 12 are used)
DmacDescriptor dmac_desc[2] __attribute__ ((aligned (8))); // any further descriptors to link multiple transfers, also 64 bit aligned (See Datasheet: 20.10.5 Next Descriptor Address)

// for convenience I assigned the descriptors by usecase to the following arrays
DmacDescriptor *adc_dmac_desc[2] = { &dmac_section_desc[0], &dmac_desc[0] };
DmacDescriptor *dac_dmac_desc[2] = { &dmac_section_desc[1], &dmac_desc[1] };

uint8_t adc_buf[DATA_BUF_SIZE << 1]; // adc buffer (twice as big, so we can use one half in the program while the other one is being filled by the adc dma transfer)
volatile uint8_t *adc_buf_active = &adc_buf[DATA_BUF_SIZE]; // currently active adc buffer (that we can use in our program, other half is used by dma)
volatile bool adc_buf_used = false; // indicates if we have already used the current buffer

uint8_t dac_buf[DATA_BUF_SIZE << 1]; // dac buffer (twice as big, so we can use one half in the program while the other one is being read by the dac dma transfer)
volatile uint8_t *dac_buf_active = &dac_buf[DATA_BUF_SIZE]; // currently active dac buffer (that we can use in our program, other half is used by dma)
volatile bool dac_buf_used = false; // indicates if we have already used the current buffer

void DMAC_Handler(void) { // DMAC Interrupt Request Handler
  __disable_irq(); // temporarely disable interrupt requests so that the channel id won't be changed while we read the corresponding dma channel's interrupt flag
  DMAC->CHID.reg = DMAC_CHID_ID(0x00); // ADC (dma channel 0)
  if(DMAC->CHINTFLAG.bit.TCMPL) { // transfer complete
    DMAC->CHINTFLAG.bit.TCMPL = 1; // reset interrupt flag
    if (adc_buf_active == adc_buf) { // switch buffer
      adc_buf_active = &adc_buf[DATA_BUF_SIZE];
    } else {
      adc_buf_active = &adc_buf[0];
    }
    adc_buf_used = false;
  }
  DMAC->CHID.reg = DMAC_CHID_ID(0x01); // DAC (dma channel 1)
  if(DMAC->CHINTFLAG.bit.TCMPL) { // tranfer complete
    DMAC->CHINTFLAG.bit.TCMPL = 1; // reset interrupt flag
    if (dac_buf_active == dac_buf) { // switch buffer
      dac_buf_active = &dac_buf[DATA_BUF_SIZE];
    } else {
      dac_buf_active = &dac_buf[0];
    }
    dac_buf_used = false;
  }
  __enable_irq(); // reenable interrupt requests
}

void init_PORT() { // Datasheet: 7.1 Multiplexed Signals, 23 PORT
  //ADC (AIN2), PIN (PB08), Arduino (A1)
  PORT->Group[PORTB].PMUX[8 >> 1].bit.PMUXE = 0x01; // PMUX Port Function B (Analog)
  PORT->Group[PORTB].PINCFG[8].reg = PORT_PINCFG_PMUXEN; // Enable PMUX
  
  // DAC (VOUT), PIN (PA02), Arduino (A0)
  PORT->Group[PORTA].PMUX[2 >> 1].bit.PMUXE = 0x01; // PMUX Port Function B (Analog)
  PORT->Group[PORTA].PINCFG[2].reg = PORT_PINCFG_PMUXEN ; // Enable PMUX
}

void init_GCLK() { // Datasheet: 15 GCLK (Generic Clock Controller)
  // For max allowed frequency of each subsystem see Datasheet 34.9 Maximum Clock Frequencies
  while(GCLK->STATUS.bit.SYNCBUSY);
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID_TCC2_TC3; // timer TC3
  while(GCLK->STATUS.bit.SYNCBUSY);
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID_EVSYS_0; // event channel 0
  while(GCLK->STATUS.bit.SYNCBUSY);
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID_DAC;
  while(GCLK->STATUS.bit.SYNCBUSY);
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID_ADC;
  while(GCLK->STATUS.bit.SYNCBUSY);
}

void init_PM() { // Datasheet: 16 PM (Power Manager)
  PM->AHBMASK.reg |= PM_AHBMASK_DMAC;
  PM->APBBMASK.reg |= PM_APBBMASK_DMAC;
  PM->APBCMASK.reg |= PM_APBCMASK_EVSYS | PM_APBCMASK_TC3 | PM_APBCMASK_DAC | PM_APBCMASK_ADC;
}

void init_EVSYS() { // Datasheet: 24 EVSYS (Event System)
  // connect Timer 3 Match/Caputre event generator on event channel 0 to ADC Start Conversion event user and DAC Start Conversion event user
  EVSYS->USER.reg = EVSYS_USER_CHANNEL(0x1U) | EVSYS_USER_USER(EVSYS_ID_USER_ADC_START); // EVSYS_USER_CHANNEL(1) means channel 0 (See Datasheet: 24.8.3 User Multiplexer)
  EVSYS->USER.reg = EVSYS_USER_CHANNEL(0x1U) | EVSYS_USER_USER(EVSYS_ID_USER_DAC_START);
  EVSYS->CHANNEL.reg = EVSYS_CHANNEL_CHANNEL(0x0U) | EVSYS_CHANNEL_EVGEN(EVSYS_ID_GEN_TC3_MCX_0) | EVSYS_CHANNEL_PATH_ASYNCHRONOUS | EVSYS_CHANNEL_EDGSEL_NO_EVT_OUTPUT;
  // ADC and DAC need asynchronous path (Datasheet: 24.8.3, Table 24-2 User Multiplexer Selection)
}

void init_TC() { // Datasheet: 30 TC (Timer/Counter)
  TC3->COUNT16.CTRLA.reg = TC_CTRLA_SWRST;
  while(TC3->COUNT16.STATUS.bit.SYNCBUSY);
  TC3->COUNT16.CTRLA.reg = TC_CTRLA_ENABLE | TC_CTRLA_MODE_COUNT16 | TC_CTRLA_PRESCALER_DIV1 | TC_CTRLA_WAVEGEN_MFRQ;
  TC3->COUNT16.CC[0].reg = SystemCoreClock / SAMPLE_RATE - 1;
  TC3->COUNT16.EVCTRL.reg |= TC_EVCTRL_MCEO0; // enable timer event on match
  while(TC3->COUNT16.STATUS.bit.SYNCBUSY);
}

void init_ADC() { // Datasheet: 33 ADC (Analog-to-Digital Converter)
  ADC->CTRLA.bit.ENABLE = 0;
  while (ADC->STATUS.bit.SYNCBUSY);

  // read calibration values from NVM, I copied this from somewhere, I need to further investigate
  uint32_t bias = (*((uint32_t *) ADC_FUSES_BIASCAL_ADDR) & ADC_FUSES_BIASCAL_Msk) >> ADC_FUSES_BIASCAL_Pos;
  uint32_t linearity = (*((uint32_t *) ADC_FUSES_LINEARITY_0_ADDR) & ADC_FUSES_LINEARITY_0_Msk) >> ADC_FUSES_LINEARITY_0_Pos;
  linearity |= ((*((uint32_t *) ADC_FUSES_LINEARITY_1_ADDR) & ADC_FUSES_LINEARITY_1_Msk) >> ADC_FUSES_LINEARITY_1_Pos) << 5;
  while (ADC->STATUS.bit.SYNCBUSY);
  ADC->CALIB.reg = ADC_CALIB_BIAS_CAL(bias) | ADC_CALIB_LINEARITY_CAL(linearity);
  
  ADC->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_INTVCC1; // 1/2 VDDANA (on adafruit feather m0 VDANA is 3V3)
  ADC->CTRLB.reg = ADC_CTRLB_PRESCALER_DIV32 | ADC_CTRLB_RESSEL_8BIT; // prescale clock by 32 => 1.5 MHz, 8bit adc sampling resolution (result will be in the lower 8 bits of the RESULT register)
  ADC->INPUTCTRL.reg = ADC_INPUTCTRL_GAIN_DIV2 | ADC_INPUTCTRL_MUXPOS_PIN2 | ADC_INPUTCTRL_MUXNEG_GND; // Divide input by 2 => allows for input voltage range from 0V to 3V3
  ADC->EVCTRL.reg = ADC_EVCTRL_STARTEI | ADC_EVCTRL_RESRDYEO; // enable start conversion and result ready events
  ADC->CTRLA.bit.ENABLE = 1;
  while (ADC->STATUS.bit.SYNCBUSY);
}

void init_DAC() { // Datasheet: 35 DAC (Digital-to-Analog Converter)
  // 10 bit DAC, but here we only use it as a 8 bit DAC
  DAC->CTRLA.bit.ENABLE = 0;
  while(DAC->STATUS.bit.SYNCBUSY);
  DAC->CTRLB.reg = DAC_CTRLB_REFSEL_AVCC | DAC_CTRLB_EOEN | DAC_CTRLB_LEFTADJ; // VDDANA (3V3), left adjust value so that we can write the data to the upper 8 bits of the DATABUF register
  DAC->EVCTRL.reg = DAC_EVCTRL_STARTEI | DAC_EVCTRL_EMPTYEO; // enable start conversion and buffer empty events
  DAC->CTRLA.bit.ENABLE = 1;
  while(DAC->STATUS.bit.SYNCBUSY);
}

void init_DMAC() { // Datasheet: 20 DMAC (Direct Memory Access Controller)
  DMAC->CTRL.bit.DMAENABLE = 0;
	DMAC->BASEADDR.reg = (uint32_t) dmac_section_desc; // dmac descriptor section base address
	DMAC->WRBADDR.reg = (uint32_t) dmac_section_desc_wrb; // dmac write-back section base address
	DMAC->CTRL.reg = DMAC_CTRL_DMAENABLE | DMAC_CTRL_LVLEN(0xf); // enable levels for channel arbitrator

	DMAC->CHID.reg = DMAC_CHID_ID(0x00); // DMA channel 0
	DMAC->CHCTRLA.reg &= ~DMAC_CHCTRLA_ENABLE;
	DMAC->CHCTRLA.reg = DMAC_CHCTRLA_SWRST;
  DMAC->CHCTRLB.reg = DMAC_CHCTRLB_LVL(0) | DMAC_CHCTRLB_TRIGACT_BEAT | DMAC_CHCTRLB_TRIGSRC(ADC_DMAC_ID_RESRDY); // pri-level 0, one beat transfer per trigger, listen for ADC Result Ready event
  DMAC->CHINTENSET.reg = DMAC_CHINTENSET_TCMPL; // enable interrupt when transfer complete

  // first descriptor, write to first half of buffer
  adc_dmac_desc[0]->SRCADDR.reg = (uint32_t)  &ADC->RESULT.reg; // src address
  adc_dmac_desc[0]->DSTADDR.reg = ((uint32_t) &adc_buf[0]) + DATA_BUF_SIZE; // dst address, NOTE! when increment is used, then the end address must be used (see Datasheet: 20.10.4 Block Transfer Destination Address)
  adc_dmac_desc[0]->DESCADDR.reg = (uint32_t) adc_dmac_desc[1]; // address of next descriptor
  adc_dmac_desc[0]->BTCNT.reg = DATA_BUF_SIZE;
  adc_dmac_desc[0]->BTCTRL.reg = DMAC_BTCTRL_DSTINC | DMAC_BTCTRL_BEATSIZE_BYTE | DMAC_BTCTRL_BLOCKACT_INT | DMAC_BTCTRL_VALID;

  // second descriptor, write to second half of buffer
  adc_dmac_desc[1]->SRCADDR.reg = (uint32_t) &ADC->RESULT.reg; // src address
  adc_dmac_desc[1]->DSTADDR.reg = ((uint32_t) &adc_buf[DATA_BUF_SIZE]) + DATA_BUF_SIZE; // dst address, NOTE! when increment is used, then the end address must be used (see Datasheet: 20.10.4 Block Transfer Destination Address)
  adc_dmac_desc[1]->DESCADDR.reg = (uint32_t) adc_dmac_desc[0]; // address of next descriptor
  adc_dmac_desc[1]->BTCNT.reg = DATA_BUF_SIZE; // number of beats, in this case 8 bits per beat
  adc_dmac_desc[1]->BTCTRL.reg = DMAC_BTCTRL_DSTINC | DMAC_BTCTRL_BEATSIZE_BYTE | DMAC_BTCTRL_BLOCKACT_INT | DMAC_BTCTRL_VALID; // 8 bit beat size, enable interrupt after transfer is done

  DMAC->CHCTRLA.bit.ENABLE = 1;

  DMAC->CHID.reg = DMAC_CHID_ID(0x01);  // DMA channel 1
	DMAC->CHCTRLA.reg &= ~DMAC_CHCTRLA_ENABLE;
	DMAC->CHCTRLA.reg = DMAC_CHCTRLA_SWRST;
  DMAC->CHCTRLB.reg = DMAC_CHCTRLB_LVL(0) | DMAC_CHCTRLB_TRIGACT_BEAT | DMAC_CHCTRLB_TRIGSRC(DAC_DMAC_ID_EMPTY); // pri-level 0, one beat transfer per trigger, listen for DAC Data Buffer Empty event
  DMAC->CHINTENSET.reg = DMAC_CHINTENSET_TCMPL; // enable interrupt when transfer complete

  // first descriptor, read first half from buffer
  dac_dmac_desc[0]->SRCADDR.reg = ((uint32_t) &dac_buf[0]) + DATA_BUF_SIZE; // src address, NOTE! when increment is used, then the end address must be used (see Datasheet: 20.10.3 Block Transfer Source Address)
  dac_dmac_desc[0]->DSTADDR.reg = ((uint32_t) &DAC->DATABUF.reg) + 1; // dst address, upper 8 bits of DAC's DATABUF register to use it as an 8 bit DAC (set left adj accordingly)
  dac_dmac_desc[0]->DESCADDR.reg = (uint32_t) dac_dmac_desc[1]; // address of next descriptor
  dac_dmac_desc[0]->BTCNT.reg = DATA_BUF_SIZE; // number of beats, in this case 8 bits per beat
  dac_dmac_desc[0]->BTCTRL.reg = DMAC_BTCTRL_SRCINC | DMAC_BTCTRL_BEATSIZE_BYTE | DMAC_BTCTRL_BLOCKACT_INT | DMAC_BTCTRL_VALID; // 8 bit beat size, enable interrupt after transfer is done

  // second descriptor, read first half from buffer
  dac_dmac_desc[1]->SRCADDR.reg = ((uint32_t) &dac_buf[DATA_BUF_SIZE]) + DATA_BUF_SIZE; // src address, NOTE! when increment is used, then the end address must be used (see Datasheet: 20.10.3 Block Transfer Source Address)
  dac_dmac_desc[1]->DSTADDR.reg = ((uint32_t) &DAC->DATABUF.reg) + 1; // dst address, upper 8 bits of DAC's DATABUF register to use it as an 8 bit DAC (set left adj accordingly)
  dac_dmac_desc[1]->DESCADDR.reg = (uint32_t) dac_dmac_desc[0]; // address of next descriptor
  dac_dmac_desc[1]->BTCNT.reg = DATA_BUF_SIZE;
  dac_dmac_desc[1]->BTCTRL.reg = DMAC_BTCTRL_SRCINC | DMAC_BTCTRL_BEATSIZE_BYTE | DMAC_BTCTRL_BLOCKACT_INT | DMAC_BTCTRL_VALID; // 8 bit beat size, enable interrupt after transfer is done

  DMAC->CHCTRLA.bit.ENABLE = 1;

  NVIC_EnableIRQ(DMAC_IRQn); // enable interrupt request handler
}

void setup() {
  init_PORT();
  init_GCLK();
  init_PM();
  init_EVSYS();
  init_TC();
  init_ADC();
  init_DAC();
  init_DMAC();
}

void loop() {
  if (!adc_buf_used) { 
    adc_buf_used = true;
    memcpy((void*)dac_buf_active, (void*)adc_buf_active, DATA_BUF_SIZE); // copy sampled adc buffer to adc buffer
  }
}