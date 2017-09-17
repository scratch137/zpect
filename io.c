/**
  \file

   Low-level input/output routines.

   $Id: io.c,v 1.35 2014/03/21 15:24:57 rauch Exp $
*/
#include <constants.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "io.h"
#include "zpec.h"

#include <../mod5270/system/sim5270.h>
#include <cfinter.h>

/** Wrapper allowing usage of macro names in ISR header. */
#define ZPEC_INTERRUPT(x, y) INTERRUPT(x, y)


/**
  Global I/O buffers, etc.

  This data structure is used to pass information between the the low-level
  C routines and the high-level C++ infrastructure.
*/
global_io_t gio;

/**
  CPLD memory map base address.

  \note The base address is that used by the netburner dev kit, see
        $NBROOT/MOD5270/system/ioboard.c.
*/
#define CPLD_BASE_ADDR (0xF0000000U)

/** Pointer to base of CPLD address space. */
volatile cpld_mem_t *const cpld = (cpld_mem_t *)CPLD_BASE_ADDR;

/** Local cache of CPLD register values. */
cpld_mem_t cpld_cache[2];


/**
   ADC interrupt procedure.
\verbatim
   name: ADC_isr
   masking level: The value of the ColdFire SR during the interrupt:

   use 0x2700 to mask all interrupts.
       0x2500 to mask levels 1-5 etc...
       0x2100 to mask level 1
\endverbatim
*/
ZPEC_INTERRUPT(ADC_isr, ZPEC_ADC_IRQ_MASK)
{
  /*
     WARNING WARNING WARNING

     Only a very limited set of RTOS functions can be called from within an
     interrupt service routine. Basically, only OS POST functions and LED
     functions should be used. No I/O (read, write or printf may be called),
     since they can block.

     Zpectrometer data is organized into frames. A complete frame consists of
     two fields of opposite phase. This ISR subtracts the quadrature
     field counts from the positive field before passing the completed frame
     to the higher level data accumulation routine. The CPLD buffers two
     samples per band for each interrupt triggered; therefore, the number of
     samples gathered is twice adc_count.

     A Zpectrometer contains two ADC banks; this ISR collates the data
     on-the-fly into contiguous buffers.

     epfr is the flag register
  */
  const unsigned nLags = gio.adc_part_end - gio.adc_part_begin;
  unsigned chan, first, frame;
  lag_count_t adc[2][2];
  cpld_mem_t reg0;
  int irq;

  /* Clear the interrupt edge. */
  irq = (gio.mode == MODE_DEVKIT ? 1 : ZPEC_ADC_IRQ);
  sim.eport.epfr = (1U << irq);

  /* Cache framing and sample data. */
  reg0      = cpld[0];
  adc[0][0] = cpld[CPLD_ADC0_0];
  adc[1][0] = cpld[CPLD_ADC1_0];
  adc[0][1] = cpld[CPLD_ADC0_1];
  adc[1][1] = cpld[CPLD_ADC1_1];

  first = ((reg0&CPLD_FIRST_FRAME) != 0);
  frame = ((reg0&CPLD_FRAME) != 0);
  ++gio.adc_calls;

  /* For first sample, ignore interrupts until frame start. */
  if (gio.adc_count == 0) {
    if (first==0 && gio.mode!=MODE_DEVKIT) { return; }
    else { gio.adc_p = gio.adc_part_begin; }
  }

  /* Save framing bits to support debugging. */
  {
    unsigned word = (gio.adc_count >> 5) &
                    (sizeof(gio.first)/sizeof(gio.first[0])-1),
	     bit  = (gio.adc_count & 31);
    gio.first[word] = (gio.first[word] & ~(1U << bit)) | (first << bit);
    gio.frame[word] = (gio.frame[word] & ~(1U << bit)) | (frame << bit);
  }

  /*
     Frame signal should be high the first field, low the second.
     First in frame should equal frame for channel 0 and zero otherwise
     if the noise diode is inactive; otherwise, channel 0 should go high
     only every other frame. Channel calculation is for the first of the two
     samples per band per interrupt.
  */
  chan = (2*gio.adc_count) & (nLags-1);
  gio.err_frame += (frame == (gio.adc_field & 1));
  if ((cpld[CPLD_REG0_RD] & CPLD_NOISE_DIODE0) && (gio.adc_field & 3) == 2) {
    gio.err_first += (first != 0);
  } else {
    gio.err_first += (first != (chan==0 ? frame : 0));
  }

  switch (gio.mode) {
    case MODE_PATTERN:
      /* Assign known pattern. */
      adc[0][0] = 1*gio.adc_field+(chan+0)+1;
      adc[0][1] = 1*gio.adc_field+(chan+1)+1;
      adc[1][0] = 2*gio.adc_field+(chan+0)+1;
      adc[1][1] = 2*gio.adc_field+(chan+1)+2;
      /* drop... */
    case MODE_NORMAL:
    case MODE_PPHASE:
    case MODE_NPHASE:
      if ((gio.adc_field&1) == 0) {
	if (gio.mode == MODE_NPHASE) {
	  /* Ignore positive phase switch. */
	  gio.adc_p[0]       = 0;
	  gio.adc_p[1]       = 0;
	  gio.adc_p[nLags+0] = 0;
	  gio.adc_p[nLags+1] = 0;
	} else {
	  /* Set samples during first field. */
	  gio.adc_p[0]       = adc[0][0];
	  gio.adc_p[1]       = adc[0][1];
	  gio.adc_p[nLags+0] = adc[1][0];
	  gio.adc_p[nLags+1] = adc[1][1];
        }
      } else if (gio.mode != MODE_PPHASE) {
	/* Subtract samples during second field (unless ignored). */
	gio.adc_p[0]       -= adc[0][0];
	gio.adc_p[1]       -= adc[0][1];
	gio.adc_p[nLags+0] -= adc[1][0];
	gio.adc_p[nLags+1] -= adc[1][1];
      }
      gio.adc_p += 2;
      break;
    case MODE_DEVKIT:
      /* Set channel0 (each band); one interrupt per frame. */
      gio.adc_p[0]     = 1;
      gio.adc_p[nLags] = 2;
      gio.err_first = 0;
      gio.err_frame = 0;
      gio.adc_p = gio.adc_part_end;
      ++gio.adc_field;
      break;
    case MODE_EXTREME:
      if ((gio.adc_field&1) == 1) {
	/* Assign known pattern; first: -65535, -65525, ..., 65525, 65535. */
	gio.adc_p[0]       = -65540 + 5*(gio.adc_field % 26216);
	gio.adc_p[1]       = (gio.adc_field&2 ? 40000 : -40000);
	gio.adc_p[nLags+0] =  65534 + ((gio.adc_field&2)>>1);
	gio.adc_p[nLags+1] = -65535;
      }
      gio.adc_p += 2;
      break;
    default:
      ++gio.err_mode;
      gio.adc_p = gio.adc_part_end;
      break;
  }
  ++gio.adc_count;

  if (gio.adc_p == gio.adc_part_end) {
    if ((gio.adc_field&1) == 1) {  /* Frame is complete. */
      /* Swap ADC input buffers. */
      ZPEC_SWAP(lag_count_t*, gio.adc_full_begin, gio.adc_part_begin);
      ZPEC_SWAP(lag_count_t*, gio.adc_full_end,   gio.adc_part_end);

      /* Inform accumulation task that new data is available (@ full_begin). */
      OSSemPost(&gio.AdcIsrSem);
    }

    /* Reset the input pointer and field count. */
    gio.adc_p = gio.adc_part_begin;
    ++gio.adc_field;
  }
}


/**
  Initializes interrupts from an IRQ pin.

  Although initialized, the IRQ is left disabled; call zpec_enable_irq() to
  enable the IRQ.

  \param irq        IRQ level (1-6).
  \param trigger    Trigger type.
  \param direction  Pin direction.
  \param isr        Interrupt service routine.
*/
void zpec_setup_irq(unsigned irq, ep_trigger_t trigger, ep_direction_t
  direction, void (*isr)(void))
{
  /*
     First set up the Eport module; cf. Coldfire reference manual chapter 15.
     Leave IRQ disabled to allow synchronized enabling.

     eppar is the pin assignment register
     epddr is the data direction register
  */
  zpec_disable_irq(irq);
  sim.eport.eppar = (sim.eport.eppar & ~(3U << 2*irq)) | (trigger << 2*irq);
  sim.eport.epddr = (sim.eport.epddr & ~(1U << irq))   | (direction << irq);

  /*
     Now enable the actual interrupt controller.
     See the ColdFire reference manual chapter 13 for more information.
     Note that the IRQ pins have hard-wired interrupt level and priority.
     The ICRx (interrupt control register) is read-only for these sources.
     Mask the interrupt, assign an ISR, then enable the interrupt source.

     imrl is the interrupt mask register (lower 32 bits)
  */
  sim.intc.imrl |= (1U << irq);
  vector_base.table[64 + irq] = (long )isr;
  sim.intc.imrl &= 0xFFFFFFFE & ~(1U << irq);
}


/**
  Enables interrupts from an IRQ pin.

  \pre zpec_setup_irq() must have been previously called on \a irq.

  \param irq IRQ level (1-6).
*/
void zpec_enable_irq(unsigned irq)
{
  /*
     epier is the interrupt enable register
     epfr  is the flag register
  */
  sim.eport.epfr  = (1U << irq); /* Clear the interrupt edge. */
  sim.eport.epier = (sim.eport.epier & ~(1U << irq)) | (EP_ENABLE_IRQ << irq);
}


/**
  Disables interrupts from an IRQ pin.

  \param irq IRQ level (1-6).
*/
void zpec_disable_irq(unsigned irq)
{
  /*
     epier is the interrupt enable register
     epfr  is the flag register
  */
  sim.eport.epier = (sim.eport.epier & ~(1U << irq)) | (EP_DISABLE_IRQ << irq);
  sim.eport.epfr  = (1U << irq); /* Clear the interrupt edge. */
}


/**
  Initializes CPLD device memory map (chip select module).
*/
void zpec_setup_mmap()
{
  /*
     csar is the chip select address register
     cscr is the chip select control register
     csmr is the chip select mask    register
  */
  sim.cs[1].csar = (CPLD_BASE_ADDR >> 16);
  sim.cs[1].cscr = 0x0180;  /* 00 0000 0 1 10 0 0 0 000 */
  sim.cs[1].csmr = 0x00000001;
}


/**
  Initializes watchdog timer.
  \warn Currently non-functional (write to WCR ignored).
  \todo Determine how to properly enable watchdog timer.
*/
void zpec_setup_watchdog()
{
  /*
     wcr   is the watchdog control register
     wmr   is the watchdog modulus register
     wcntr is the watchdog counter register
     wsr   is the watchdog service register
  */
  int i;

  sim.wtm.wcr = 0x1;

  for (i=0; i<1; ++i) {
    OSTimeDly(1);
    zpec_debug("WCR = %hu, WMR = %hu, WCNTR = %hu, WSR = %hu",
	       sim.wtm.wcr, sim.wtm.wmr, sim.wtm.wcntr, sim.wtm.wsr);
  }
}


/**
  Writes a specific CPLD register bit.

  Updates a bit in the local register variable and writes the new word to the
  CPLD. All other bits are left unchanged. The associated register is encoded
  as the high word of the input bit mask (cf. #cpld_laddr_t).

  \param bit   Register address/mask.
  \param value Bit value (zero to clear, non-zero to set).
*/
void cpld_set_bit(cpld_laddr_t bit, cpld_mem_t value)
{
  cpld_mem_t mask = (bit & 65535U), reg = (bit >> 16);

  if (!value) { cpld_cache[reg] &= ~mask; }
  else        { cpld_cache[reg] |=  mask; }

  cpld[reg] = cpld_cache[reg];
}


/**
  Reads a specific CPLD register bit.

  Reads a specific register bit from the CPLD (reads the associated register
  and masks off all other bits). The associated register is encoded
  as the high word of the input bit mask (cf. #cpld_laddr_t).

  \param bit Register address/mask.

  \return Zero if the bit clear, else the non-zero bit mask representing the
	  (set) bit.
*/
cpld_mem_t cpld_get_bit(cpld_laddr_t bit)
{
  cpld_mem_t mask = (bit & 65535U), reg = (bit >> 16);
  return(cpld[reg] & mask);
}


/**
  Sets step attenuator value.

  The Zpectrometer step attenuator hardware (HMC307 and HMC270) accepts a
  7-bit value, encoding attenuations in the range 0-31 dB, plus a maximum
  attenuation setting. Other WASP hardware variants (RLT) use an HMC235 in
  place of the HMC307.

  \warning This routine is not thread-safe due to the low-level hardware
	   access involved.

  \param band  Band to which to apply attenuation setting (-1 for all).
  \param atten Attenuation setting in dB (0-31; 32+ sets maximum attenuation).
  \param hw    Backend hardware variant.
*/
void cpld_set_atten(int band, cpld_mem_t atten, zpec_hw_t hw)
{
  cpld_mem_t setup, mask, begin, end;
  unsigned tick;

  switch (hw) {
    case ZPEC_HW_GBT:
      /*
	 HMC307 setup bits, written LSB to MSB order:

	 MSB  A1   engage 16 dB step (active low)
	      A2   engage  8 dB step (active low)
	      A3   engage  4 dB step (active low)
	      A4   engage  2 dB step (active low)
	      A5   engage  1 dB step (active low)
	      SA   0 --> use A5-A1; 1 --> engage max attenuation (> 60 dB)
	 LSB  SB  ~SA
      */
      setup = (atten > 31 ? 2U : (~atten << 2) + 1U);
      begin = 1;
      end   = 1U << 7;
      break;
    case ZPEC_HW_RLT:
      /*
	 HMC235 setup bits, written MSB to LSB order:

	 MSB  SB  ~SA
	      SA   0 --> use A1-A5; 1 --> engage max attenuation (> 60 dB)
	      A1   engage 16 dB step (active high)
	      A2   engage  8 dB step (active high)
	      A3   engage  4 dB step (active high)
	      A4   engage  2 dB step (active high)
	 LSB  A5   engage  1 dB step (active high)
      */
      setup = (atten > 31 ? 63U : 64+atten);
      begin = 1U << 7;
      end   = 0;
      break;
    default:
      return;
  }

  /* Select attenuator. */
  cpld_set_bit(CPLD_PERIPH_ATTEN0, (band<0 || band==0 ? 0 : 1));
  cpld_set_bit(CPLD_PERIPH_ATTEN1, (band<0 || band==1 ? 0 : 1));

  /*
     Write attenuation bits, LSB to MSB order, using ~100 kHz serial clock.
     [Due to overhead, tick = {0 1 2} usec produces ~{90 70 60} kHz.]
     Attenuators read bits on the rising clock edge.
  */
  tick = 0;
  for (mask=begin; mask != end; mask=(end>begin ? mask<<1 : mask>>1)) {
    cpld_set_bit(CPLD_PERIPH_DOUT, setup&mask);

    /* Pulse clock one cycle. */
    cpld_set_bit(CPLD_PERIPH_CLK, 0);
    zpec_usleep(tick);
    cpld_set_bit(CPLD_PERIPH_CLK, 1);
    zpec_usleep(tick);
  }

  /* Deselect attenuators. */
  cpld_set_bit(CPLD_PERIPH_ATTEN0, 1);
  cpld_set_bit(CPLD_PERIPH_ATTEN1, 1);
}


/**
  Convert ADC counts to current.

  This method converts between ADC counts and current for the power supply
  monitor points (hardware type ZPEC_HW_POW).

  \param counts  ADC counts.
  \param channel ADC channel.
  \return Line current in mA.

*/
int Ipow(int counts, adc_channel_t channel)
{
  return(counts);
  int mV = 3.43f * counts;
  int Imax = (channel == ADC_POW_P_5A ? 60 : ADC_POW_P_15A ? 14 : 35);
  return(mV < 3300 ? (Imax*(mV-2000))/13 : (Imax*(mV-3000))/3);
}


/**
  Reads a monitor ADC channel.

  \warning This routine is not thread-safe due to the low-level hardware
	   access involved.

  \param channel ADC channel to read.

  \return A scaled integer representing 1000 times the floating-point
          monitor point value.
*/
int cpld_read_mon_adc(adc_channel_t channel)
{
  /* Channel select bits (= ADCchan(1) & ADCchan(2) & ADCchan(0)). */
  static unsigned char csel[ADC_NCHAN] = {
      /* GBT: */  0, 1, 4, 5, 2, 3, 6, 7,
      /* RLT: */  0, 1, 4, 5, 2, 3, 7,
      /* POW: */  0, 1, 4, 5, 3, 2, 6, 7, 2, 3, 0, 1, 4, 5, 6, 7
  };

  /* ADC setup byte:
     bit 0: 1     (start)
     bit 1: csel2 (chan select LSB)
     bit 2: csel1
     bit 3: csel0 (chan select MSB)
     bit 4: pol   (0 = bipolar, 1 = unipolar)
     bit 5: 1     (single-ended)
     bit 6: 1
     bit 7: 0     (internal clock mode)

  */
  unsigned counts, tick, unipolar;
  unsigned char setup;
  cpld_mem_t mask;
  cpld_laddr_t adc, dclk, din, dout;

  /* Initialize ADC setup byte. */
  switch (channel) {
    case ADC_MINUS_5V:
    case ADC_RLT_AVGT1:
    case ADC_RLT_AVGT2:
    case ADC_RLT_EXTIN:
    case ADC_POW_TOTPWR:
    case ADC_POW_SWPWR:
    case ADC_POW_P_3V:
    case ADC_POW_P_5V:
    case ADC_POW_N_5V:
    case ADC_POW_P_15V:
    case ADC_POW_P_3A:
    case ADC_POW_P_5A:
    case ADC_POW_N_5A:
    case ADC_POW_P_15A:
      unipolar=0;
      setup = 0x61 + (csel[channel] << 1); /* Bipolar output. */
      break;
    default:
      unipolar=1;
      setup = 0x71 + (csel[channel] << 1); /* Unipolar output. */
      break;
  }

  /* Select monitor ADC. */
  adc  = CPLD_PERIPH_ADC;
  dclk = CPLD_PERIPH_CLK;
  din  = CPLD_PERIPH_DIN;
  dout = CPLD_PERIPH_DOUT;
  if ((channel >= ADC_POW_BEGIN && channel < ADC_POW_END)) {
    adc  = (unipolar || channel==ADC_POW_TOTPWR || channel==ADC_POW_SWPWR ?
            CPLD_POW_ADC0 : CPLD_POW_ADC1);
    dclk = CPLD_POW_DCLK;
    din  = (unipolar || channel==ADC_POW_TOTPWR || channel==ADC_POW_SWPWR ?
	    CPLD_POW_DIN0 : CPLD_POW_DIN1);
    dout = CPLD_POW_DOUT;
  }
  cpld_set_bit(adc, 0);

  /*
     Write setup byte, LSB to MSB order, using ~100 kHz serial clock.
     [Due to overhead, tick = {0 1 2} usec produces ~{90 70 60} kHz.]
     ADC reads bits on the rising clock edge.
  */
  tick = 0;
  for (mask=1; mask<(1U<<8); mask<<=1) {
    cpld_set_bit(dout, setup&mask);

    /* Pulse clock one cycle. */
    cpld_set_bit(dclk, 0);
    zpec_usleep(tick);
    cpld_set_bit(dclk, 1);
    zpec_usleep(tick);
  }

  /*
     Read ADC output: 12-bit two's complement in MSB to LSB order.
     ADC writes bits on the falling edge, after one clock initialization.
     Unipolar values are signed; bipolar are two's complement.
  */
  for (counts=0, mask=0x1000; mask!=0; mask>>=1) {
    /* Pulse clock one cycle. */
    cpld_set_bit(dclk, 0);
    zpec_usleep(tick);
    cpld_set_bit(dclk, 1);
    zpec_usleep(tick);

    /* No data on first clock. */
    if (mask != 0x1000) {
      if (cpld_get_bit(din)) { counts |= mask; }
    }
  }

  /* Deselect monitor ADC. */
  cpld_set_bit(adc, 1);

  /*
     Scale results to standard three-decimal fixed-point integer.
     Note: Conversion of result to type int assumes no representation change.
  */

  /* Sign-extend bipolar values. */
  if (!unipolar && (counts&0x0800)) { counts |= 0xFFFFF000; }
  int icounts = (int )counts;

  switch (channel) {
    case ADC_AUX_TEMP:
    case ADC_CORL_TEMP:
    case ADC_AMP1_TEMP:
    case ADC_AMP2_TEMP:
    case ADC_RLT_CORR7:
    case ADC_RLT_CORR0:
    case ADC_RLT_AMOD:
    case ADC_RLT_TAMB:
      return(100*(icounts-500));

    case ADC_PLUS_3_3V:
    case ADC_RLT_AVGT1:
    case ADC_RLT_AVGT2:
    case ADC_RLT_EXTIN:
      return(icounts);

    case ADC_PLUS_5V:
      return((1000*icounts)/589);

    case ADC_MINUS_5V:
      return((1000*icounts)/204);

    case ADC_PLUS_15V:
      return((1000*icounts)/204);

    case ADC_POW_ZTEMP:
    case ADC_POW_ZGND:
    case ADC_POW_PTEMP:
      return(101*icounts-50500);

    case ADC_POW_TILTX:
      return(icounts < 794 ? -90000 : icounts > 2380 ? 90000 :
             57.295e3f*asinf(2.0f-1.26e-3f*icounts));

    case ADC_POW_TILTY:
      return(icounts < 794 ? 90000 : icounts > 2380 ? -90000 :
             57.295e3f*asinf(1.26e-3f*icounts-2.0f));

    case ADC_POW_TOTPWR:
    case ADC_POW_SWPWR:
      return(6.556f*icounts);

    case ADC_POW_Z_12V:
      return(3.43f*icounts);

    case ADC_POW_P_3V:
    case ADC_POW_P_5V:
    case ADC_POW_N_5V:
      return(2.91f*icounts);

    case ADC_POW_P_15V:
      return(8.32f*icounts);

    case ADC_POW_P_3A:
    case ADC_POW_P_5A:
    case ADC_POW_N_5A:
    case ADC_POW_P_15A:
      return(Ipow(icounts, channel));

    default:
      return(0);
  }
}


/** Returns location of least-significant non-zero bit in word. */
static unsigned short find_bit(unsigned word)
{
  unsigned char bit = 0;
  while (word!=0 && (word&1)==0) { ++bit; word>>=1; }
  return(bit);
}

/**
  Initializes the correlator ADCs.

  This routine supports Burr-Brown DDC101 ADCs. See data sheet for an
  explanation of parameters. Bits for each parameter are sent to the ADC in
  MSB to LSB order.

  \warning This routine is not thread-safe due to the low-level hardware
	   access involved.

  \param K Acquisition Time Control value
  \param M Oversampling Control, Samples/Integration value
  \param L Multiple Integration Control, Integration/Conversion value
  \param I Input Range value (0 = unipolar, 1 = bipolar)
  \param O Output Format value (0 = unsigned, 1 = two's complement)

  \return Zero on success, else the number of ADCs returning invalid setup
	  data.
*/
int cpld_init_corl_adc(unsigned short K, unsigned short M, unsigned short L,
		       unsigned short I, unsigned short O)
{
  static const char *fn = "init_adc";
  unsigned short mask, setup, tick;
  int adc, rtn = 0;

  setup = ((K == 16 ? 2U : K==32 ? 3U : K&1) << 10) | (find_bit(M) << 6) |
	  (find_bit(L) << 2) | ((I&1) << 1) | (O&1);

  /* Initialize setup. */
  tick = 0;
  cpld_set_bit(CPLD_OSC_ENABLE,     0);
  cpld_set_bit(CPLD_ADC_RST_SYS,    0);
  cpld_set_bit(CPLD_ADC_READ_SETUP, 1);
  cpld_set_bit(CPLD_ADC_RST_SETUP,  0);
  zpec_usleep(tick);
  cpld_set_bit(CPLD_ADC_RST_SETUP,  1);

  /* Send 12-bit setup string. */
  for (mask = 1U<<11; mask != 0; mask >>= 1) {
    /* Set data bit. */
    cpld_set_bit(CPLD_ADC_SETUP_DIN, setup&mask);

    /* Pulse clock one cycle. */
    cpld_set_bit(CPLD_ADC_SETUP_CLK, 0);
    zpec_usleep(tick);
    cpld_set_bit(CPLD_ADC_SETUP_CLK, 1);
    zpec_usleep(tick);
  }

  /*
     Read back setup string for verification.
     First register the setup word, then read out the serial data.
     A Zpectrometer chassis contains two banks of 8 daisy-chained cards
     containing 16 ADCs each.
  */
  cpld_set_bit(CPLD_ADC_READ_SETUP, 0);
  cpld_set_bit(CPLD_ADC_SETUP_CLK, 0);
  zpec_usleep(tick);
  cpld_set_bit(CPLD_ADC_SETUP_CLK, 1);
  zpec_usleep(tick);

  /*
     A Zpectrometer chassis contains 8 correlator cards per band.
     One correlator card contains 16 ADCs.
  */
  for (adc = 0; adc<8*16; ++adc) {
    unsigned short readback[2] = {0, 0};

    for (mask = 1U<<11; mask != 0; mask >>= 1) {
      /* Pulse clock one cycle. */
      cpld_set_bit(CPLD_ADC_SETUP_CLK, 0);
      zpec_usleep(tick);
      cpld_set_bit(CPLD_ADC_SETUP_CLK, 1);
      zpec_usleep(tick);

      /* Read data bits. */
      if (cpld_get_bit(CPLD_SERIAL_DOUT1)) { readback[1] |= mask; }
      if (cpld_get_bit(CPLD_SERIAL_DOUT0)) { readback[0] |= mask; }
    }
    if (readback[0] != setup || readback[1] != setup) {
      zpec_error_fn("ADC setup failure: ADC%3d = {0x%03X, 0x%03X}, exp. 0x%03X",
		    adc, readback[0], readback[1], setup);
      rtn += (readback[0] != setup)+(readback[1] != setup);
    }
  }

  /* Begin operation. */
  cpld_set_bit(CPLD_ADC_READ_SETUP, 1);
  cpld_set_bit(CPLD_ADC_RST_SYS,    1);
  cpld_set_bit(CPLD_OSC_ENABLE,     1);
  return(rtn);
}


/**
  Return operating mode string name.

  \param mode Operating mode.
  \return String name of \a mode ("????" if mode is invalid).
*/
const char *zpec_mode_name(zpec_mode_t mode)
{
  switch (mode) {
    case MODE_NORMAL:  return "NORMAL";
    case MODE_PATTERN: return "PATTERN";
    case MODE_DEVKIT:  return "DEVKIT";
    case MODE_PPHASE:  return "PPHASE";
    case MODE_NPHASE:  return "NPHASE";
    case MODE_EXTREME: return "EXTREME";
    default:           return "????";
  }
}
