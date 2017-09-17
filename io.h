#ifndef IO_H
#define IO_H
/**
  \file
  \author Kevin P. Rauch
  \brief  Low-level I/O declarations.

  $Id: io.h,v 1.35 2014/03/21 21:40:21 rauch Exp $
*/
#include <basictypes.h>
#include <ucos.h>

#include "zpec.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ZPEC_ADC_IRQ  5          ///< ADC readout IRQ level.
#define ZPEC_ADC_IRQ_MASK 0x2700 ///< ADC readout IRQ mask (inside ISR).
#define ZPEC_ADC_PRIO 30         ///< ADC readout task priority.

/** Operating mode. */
typedef enum mode_type_enum {
   MODE_NORMAL = 0,  /**< Normal operation. */
   MODE_PATTERN= 1,  /**< ADC samples replaced with test pattern. */
   MODE_DEVKIT = 2,  /**< Test mode (Netburner development kit). */
   MODE_PPHASE = 3,  /**< Ignore negative-phase ADC readback fields. */
   MODE_NPHASE = 4,  /**< Ignore positive-phase ADC readback fields. */
   MODE_EXTREME= 5,  /**< Fixed and circulating extreme value test pattern. */
   MODE_NMODES       /**< Invalid/placeholder mode (keep last). */
} zpec_mode_t;

/** EPORT module IRQ trigger. */
typedef enum ep_trigger_enum {
  EP_ACTIVE_LOW = 0,
  EP_RISING_EDGE = 1,
  EP_FALLING_EDGE = 2,
  EP_RISING_OR_FALLING_EDGE = 3
} ep_trigger_t;

/** EPORT module pin direction. */
typedef enum ep_direction_enum {
  EP_INPUT_PIN = 0,
  EP_OUTPUT_PIN = 1
} ep_direction_t;

/** EPORT module IRQ enable. */
typedef enum ep_enable_enum {
  EP_DISABLE_IRQ = 0,
  EP_ENABLE_IRQ = 1
} ep_enable_t;

/** Logical CPLD memory locations and register bit masks. */
typedef enum cpld_laddr_enum {
  CPLD_STATUS        = 0,    /**< Status registers (read-only). */
  CPLD_FIRST_FRAME   = 0x00001,  /**< First readout frame indicator. */
  CPLD_FRAME         = 0x00002,  /**< Frame signal. */
  CPLD_PERIPH_DIN    = 0x00004,  /**< Peripherals serial input data. */
  CPLD_SERIAL_DOUT0  = 0x00008,  /**< ADC serial data (bank 0). */
  CPLD_SERIAL_DOUT1  = 0x00010,  /**< ADC serial data (bank 1). */
  CPLD_EXT_BLANK     = 0x00020,  /**< External blank (slave reads master). */
  CPLD_EXT_NOD_B     = 0x00040,  /**< External nod B (slave reads master). */
  CPLD_EXT_NOD_A     = 0x00080,  /**< External nod A (slave reads master). */
  CPLD_EXT_CHOP      = 0x00100,  /**< External chop  (slave reads master). */
  CPLD_JUMPER0       = 0x00200,  /**< PCB jumper J0 setting (active-low).  */
  CPLD_JUMPER1       = 0x00400,  /**< PCB jumper J0 setting (active-low).  */
  CPLD_JUMPER2       = 0x00800,  /**< PCB jumper J0 setting (active-low).  */
  CPLD_VERSION0      = 0x01000,  /**< CPLD code version bit 0. */
  CPLD_VERSION1      = 0x02000,  /**< CPLD code version bit 0. */
  CPLD_VERSION2      = 0x04000,  /**< CPLD code version bit 0. */
  CPLD_VERSION3      = 0x08000,  /**< CPLD code version bit 0. */
  //
  CPLD_POW_STATE     = 0x00001,  /**< Power supply state. */
  CPLD_POW_DIN0      = 0x00004,  /**< ADC0 serial input data. */
  CPLD_POW_DIN1      = 0x00008,  /**< ADC1 serial input data. */
  CPLD_POW_SW_ON     = 0x00400,  /**< Front panel switch-on state. */
  CPLD_POW_SW_OFF    = 0x00800,  /**< Front panel switch-on state. */

  CPLD_ADC0_0        = 1,    /**< ADC bank 0 sample 0 (read-only). */
  CPLD_ADC1_0        = 2,    /**< ADC bank 1 sample 0 (read-only). */
  CPLD_ADC0_1        = 3,    /**< ADC bank 0 sample 1 (read-only). */
  CPLD_ADC1_1        = 4,    /**< ADC bank 1 sample 1 (read-only). */

  CPLD_REG0          = 0,    /**< Low  control register word (write-only). */
  CPLD_MASTER        = 0x00001,  /**< Master/slave mode (0 = slave). */
  CPLD_SYNC_ENABLE   = 0x00002,  /**< Synchronize ADC system. */
  CPLD_READ_LOW_BITS = 0x00004,  /**< ADC readout mode (0 = normal). */
  CPLD_NOISE_DIODE0  = 0x00008,  /**< Noise diode 0 control (1 = fire). */
  CPLD_QUAD_PHASE_SW = 0x00010,  /**< Quadrature switch mode (1 = quad sw). */
  CPLD_PERIPH_CLK    = 0x00020,  /**< Peripherals serial clock. */
  CPLD_PERIPH_DOUT   = 0x00040,  /**< Peripherals serial output data. */
  CPLD_PERIPH_ATTEN0 = 0x00080,  /**< Attenuator 0 select bit (active-low). */
  CPLD_PERIPH_ATTEN1 = 0x00100,  /**< Attenuator 1 select bit (active-low). */
  CPLD_PERIPH_ADC    = 0x00200,  /**< Monitor  ADC select bit (active-low). */
  CPLD_PERIPH_DAC    = 0x00400,  /**< External DAC select bit (active-low). */
  CPLD_DIODE0_JAM    = 0x00400,  /**< Noise diode 0 override (1 = fire). */
  CPLD_BLANK         = 0x00800,  /**< Blanking status (0 = acquiring data). */
  CPLD_NOD_B         = 0x01000,  /**< CSO: nod signal for beam B. */
  CPLD_NOD_A         = 0x02000,  /**< CSO: nod signal for beam A. */
  CPLD_CHOP          = 0x04000,  /**< CSO: chop switching signal. */
  CPLD_INT_RESET     = 0x08000,  /**< CPLD internal reset. */
  //
  CPLD_POW_ON        = 0x00001,  /**< Main power supply on.  */
  CPLD_POW_OFF       = 0x00002,  /**< Main power supply off. */
  CPLD_POW_MANUAL    = 0x00004,  /**< Front panel switch enable (active-low). */
  CPLD_POW_DCLK      = 0x00020,  /**< Peripherals serial clock. */
  CPLD_POW_DOUT      = 0x00040,  /**< Peripherals serial output data. */
  CPLD_POW_DAC       = 0x00100,  /**< DAC  select bit (active-low). */
  CPLD_POW_ADC0      = 0x00200,  /**< ADC0 select bit (active-low). */
  CPLD_POW_ADC1      = 0x00400,  /**< ADC1 select bit (active-low). */

  CPLD_REG1          = 1,    /**< High control register word (write-only). */
  CPLD_ADC_SETUP_CLK = 0x10001,  /**< CPU generates ADC setup clock. */
  CPLD_ADC_RST_SYS   = 0x10002,  /**< ADC system reset (active-low). */
  CPLD_ADC_RST_SETUP = 0x10004,  /**< ADC internal reset (active-low). */
  CPLD_ADC_READ_SETUP= 0x10008,  /**< ADC system read data/setup mode. */
  CPLD_ADC_SETUP_DIN = 0x10010,  /**< ADC system setup data. */
  CPLD_OSC_ENABLE    = 0x10020,  /**< CPLD 2 MHz oscillator enable. */
  CPLD_EXT_PHASE_SW  = 0x10040,  /**< CPLD external phase switch enable. */
  CPLD_INT_PHASE_SW  = 0x10080,  /**< CPLD internal phase switch enable. */
  //
  CPLD_POW_OVERRIDE  = 0x80004,  /**< Front-panel manual override state. */

  CPLD_REG0_RD       = 8,    /**< High control register word (read-only). */
  CPLD_REG1_RD       = 9,    /**< High control register word (read-only). */
} cpld_laddr_t;

/** Monitor ADC (logical) channels. */
typedef enum adc_channel_enum {
  ADC_GBT_BEGIN =  0,  /**< First logical ADC channel for GBT hardware. */
  ADC_AUX_TEMP  =  0,  /**< Auxilliary interface card temperature. */
  ADC_CORL_TEMP =  1,  /**< Central correlator card temperature. */
  ADC_AMP1_TEMP =  2,  /**< Amplifier module 1 temperature. */
  ADC_AMP2_TEMP =  3,  /**< Amplifier module 2 temperature. */
  ADC_PLUS_3_3V =  4,  /**<  +3.3 V power supply voltage. */
  ADC_PLUS_5V   =  5,  /**<  +5   V power supply voltage. */
  ADC_MINUS_5V  =  6,  /**<  -5   V power supply voltage. */
  ADC_PLUS_15V  =  7,  /**< +15   V power supply voltage. */
  ADC_GBT_END   =  8,  /**< Last logical ADC channel for GBT hardware + 1. */

  ADC_RLT_BEGIN =  8,  /**< First logical ADC channel for RLT hardware. */
  ADC_RLT_CORR7 =  8,  /**< Correlator #7 temperature (C). */
  ADC_RLT_CORR0 =  9,  /**< Correlator #0 temperature (C). */
  ADC_RLT_AVGT1 = 10,  /**< Weighted temperature T1 (mV). */
  ADC_RLT_AVGT2 = 11,  /**< Weighted temperature T2 (mV). */
  ADC_RLT_AMOD  = 12,  /**< Amplifier module temperature (C). */
  ADC_RLT_TAMB  = 13,  /**< Ambient temperature (C). */
  ADC_RLT_EXTIN = 14,  /**< External input voltage (mV). */
  ADC_RLT_END   = 15,  /**< Last logical ADC channel for RLT hardware + 1. */

  ADC_POW_BEGIN = 15,  /**< First logical ADC channel for POW hardware. */
  /* ADC0 channels (bipolar mode) */
  ADC_POW_TOTPWR= 15,  /**< Total RF power (V). */
  ADC_POW_SWPWR = 16,  /**< Switching(?) RF power (V). */
  /* ADC0 channels (unipolar mode) */
  ADC_POW_ZTEMP = 17,  /**< IF processor temperature (C). */
  ADC_POW_ZGND  = 18,  /**< IF processor temperature offset (C). */
  ADC_POW_TILTX = 19,  /**< Power supply X-axis tilt (deg). */
  ADC_POW_TILTY = 20,  /**< Power supply Y-axis tilt (deg). */
  ADC_POW_Z_12V = 21,  /**< IF processor +12 V rail voltage (V). */
  ADC_POW_PTEMP = 22,  /**< Power supply temperature (C). */
  /* ADC1 channels (bipolar mode) */
  ADC_POW_P_3V  = 23,  /**< Power supply  +3.3 V rail voltage (V). */
  ADC_POW_P_3A  = 24,  /**< Power supply  +3.3 V rail current (A). */
  ADC_POW_P_5V  = 25,  /**< Power supply  +5   V rail voltage (V). */
  ADC_POW_P_5A  = 26,  /**< Power supply  +5   V rail current (A). */
  ADC_POW_N_5V  = 27,  /**< Power supply  -5   V rail voltage (V). */
  ADC_POW_N_5A  = 28,  /**< Power supply  -5   V rail current (A). */
  ADC_POW_P_15V = 29,  /**< Power supply +15   V rail voltage (V). */
  ADC_POW_P_15A = 30,  /**< Power supply +15   V rail current (A). */
  ADC_POW_END   = 31,  /**< Last logical ADC channel for POW hardware + 1. */

  ADC_NCHAN     = 31,  /**< Total number of logical ADC channels. */
} adc_channel_t;

/** Lag count type. */
typedef long lag_count_t;

/**
  Low-level I/O buffer pointers.

  The ISR and frame readout buffers are swapped by ADC_isr() when a
  complete frame (consisting of two fields of opposite phase) has been filled,
  at which time AdcIsrSem is posted. This gives the waiting task a full frame
  cycle for processing before the readout frame begins to get overwritten.
*/
typedef struct global_io_struct {
  volatile int
    err_first,  /**< First in frame signal error indicator. */
    err_frame,  /**< Frame signal error indicator. */
    err_mode;   /**< Operating mode error indicator. */

  volatile unsigned
    adc_calls,  /**< Number of entries into ADC_isr(). */
    adc_count,  /**< Number of handled data interrupts since last enable. */
    adc_field;  /**< Number of complete fields filled since last enable. */

  unsigned
    first[4*512], /**< First-in-frame signal history bitset (recent frames). */
    frame[4*512]; /**< Frame signal history bitset (recent frames). */

  volatile zpec_mode_t
    mode;       /**< Operating mode. */

  lag_count_t
    *adc_full_begin, /**< Start of latest full ADC readout buffer. */
    *adc_full_end,   /**< End   of latest full ADC readout buffer (last+1). */
    *adc_part_begin, /**< Start of current partial ADC_isr() buffer. */
    *adc_part_end,   /**< End   of current partial ADC_isr() buffer (last+1).*/
    *adc_p;          /**< Write location of next ADC_isr() input sample. */

  OS_SEM
    AdcIsrSem;       /**< ADC_isr() completed frame notification channel. */

  OS_CRIT
    periph_lock;     /**< Peripheral I/O mutex. */
} global_io_t;

extern global_io_t gio;

/** CPLD memory location type. */
typedef unsigned short cpld_mem_t;

extern volatile cpld_mem_t *const cpld;
extern cpld_mem_t cpld_cache[2];

extern void cpld_set_bit(cpld_laddr_t bit, cpld_mem_t value);
extern void cpld_set_atten(int band, cpld_mem_t atten, zpec_hw_t hw);

extern cpld_mem_t cpld_get_bit(cpld_laddr_t bit);

extern int cpld_read_mon_adc(adc_channel_t channel);
extern int cpld_init_corl_adc(unsigned short K, unsigned short M,
	       unsigned short L, unsigned short I, unsigned short O);

extern void zpec_setup_mmap();
extern void zpec_setup_watchdog();
extern void zpec_setup_irq(unsigned irq, ep_trigger_t trigger,
			   ep_direction_t direction, void (*isr)(void));
extern void zpec_disable_irq(unsigned irq);
extern void zpec_enable_irq(unsigned irq);

extern const char *zpec_mode_name(zpec_mode_t mode);

/** ADC data interrupt service routine. */
extern void ADC_isr(void);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  /* IO_H */
