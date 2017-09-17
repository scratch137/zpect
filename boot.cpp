/**
  \file

   Correlator initialization methods.

   $Id: boot.cpp,v 1.24 2014/03/21 18:51:19 rauch Exp $
*/
#include <ctype.h>
#include <stdlib.h>
#include <string.h>

#include "argus.h"
#include "control.h"


/**
  Initializes correlator hardware, services, and control state.

  This routine initializes correlator peripherals and internal buffers and
  variables to their start-up values, and starts the IP services. The caller
  of this routine becomes the control server.

  \param shell Command interpreter attached to this instance.
  \param io    Low-level I/O structure (C-callable).

  \pre
  Networking must be active. The command shell dictionary must have been
  initialized.

  \warning
  This routine does not return.
*/
void Correlator::boot(Correlator::shell_type *shell, global_io_t *io)
{
  // Low-level initialization.
  io_ = io;
  initHardware();

  // Start internet services.
  zpec_info("Starting services..");
  messageServer.start();
  dataServer.start();
  monitorServer.start();

  // Initialize help facility.
  createHelpSummary(shell);

  // We are the control task; execute the control service directly.
  controlServer.startSameTask();  // Does not return.
}


/// Initializes state variables.
void Correlator::initState()
{
  // Initialize mutex.
  OSCritInit(&lock_);
}


/// Initializes hardware.
void Correlator::initHardware()
{
  // Determine hardware variant.
  flash_t flashData;
  zpec_readFlash(&flashData);
  hw_ = flashData.hw;

  // Initialize memory map (chip select module).
  if (hw_ != ZPEC_HW_ARG) {
    zpec_setup_mmap();

    // Set default values of CPLD registers.
    switch (hw_) {
      case ZPEC_HW_GBT:
      case ZPEC_HW_RLT:
        cpld_set_bit(CPLD_INT_RESET, 1);
        cpld[0] = cpld_cache[0] = 0x0b80;
        cpld[1] = cpld_cache[1] = 0x00fc;
        break;

      case ZPEC_HW_POW:
        cpld[0] = cpld_cache[0] = 0x0780;
        cpld_cache[1] = 0xBADD;
        break;

      default:
        break;
    }
  }

  // Create ADC ISR input buffers.
  adcBuffer_.reserve(2*nBands()*nLags());
  adcBuffer_.resize(2*nBands()*nLags());

  // Initialize peripheral mutex.
  OSCritInit(&io_->periph_lock);

  // Initialize ADC ISR and readout task semaphores.
  OSSemInit(&io_->AdcIsrSem, 0);

  // Initialize watchdog task.
  zpec_setup_watchdog();

  // Set monitor point channels.
  switch (hw_) {
    case ZPEC_HW_GBT:
      hw_adc_begin_ = ADC_GBT_BEGIN;
      hw_adc_end_   = ADC_GBT_END;
      break;

    case ZPEC_HW_RLT:
      hw_adc_begin_ = ADC_RLT_BEGIN;
      hw_adc_end_   = ADC_RLT_END;
      break;

    case ZPEC_HW_POW:
      hw_adc_begin_ = ADC_POW_BEGIN;
      hw_adc_end_   = ADC_POW_END;
      break;

    default:
      hw_adc_begin_ = ADC_NCHAN;
      hw_adc_end_   = ADC_NCHAN;
      break;
  }

  // Miscellaneous hardware-specific initialization.
  switch (hw_) {
    case ZPEC_HW_GBT:
    case ZPEC_HW_RLT:
      // Set maximum attenuation.
      for (unsigned i=0; i<nBands(); ++i) {
	(void )band_[i].control.setAttenuation(63);
      }
      cpld_set_atten(-1, 63, hw_);

      // Initialize correlator ADCs.
      error_.initADC = cpld_init_corl_adc(16, 128, 1, 0, 0);
      break;

    case ZPEC_HW_ARG:
      argus_init(&flashData);
      break;

    default:
      break;
  }
}
