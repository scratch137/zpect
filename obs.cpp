/**
  \file
  \author Kevin P. Rauch
  \brief Correlator data-taking command functionality.

   $Id: obs.cpp,v 1.41 2008/02/20 00:25:27 rauch Exp $
*/
#include <iosys.h>
#include <stdlib.h>
#include <string.h>

#include <basictypes.h>
#include <utils.h>

#include <algorithm>
#include <limits>

#include "control.h"


/**
  Accumulates ADC data.

  This method enables ADC data interrupts and processes frames until the
  desired number of frames is collected, or any new user input (such as ^C)
  is detected. Data interrupts are then disabled.  Frames consist of two
  halves (fields), with phase switched between them. The second field is
  subtracted from the first to generate the net lag counts.

  \warning
  Only observations compatible with BasicObservation are repeatable.

  \param obs     Observation object for processing frames.
  \param nFrames Number of ADC frames to accumulate.
  \param arg     Command arguments (for I/O descriptors).
  \param status  Command return status buffer.
  \param nRepeat Number of times to repeat observation.

  \return Number of frames accumulated (processed).
*/
unsigned Correlator::readADCs(Observation *obs, unsigned nFrames,
  argument_type& arg, return_type status, unsigned nRepeat)
{
  static const char *fn = "readADCs";
  unsigned iRepeat, irq, nAccum, nTicks;

  // Buffers initialized only once per call.
  adcBuffer_.assign(adcBuffer_.size(), 0);
  memset(io_->first, 0, sizeof(io_->first));
  memset(io_->frame, 0, sizeof(io_->frame));
  nAccum = 0;

  if (zpec_change_prio(ZPEC_ADC_PRIO) != OS_NO_ERR) {
    zpec_warn_fn("Could not raise task priority");
  }

  while (OSSemPendNoWait(&io_->AdcIsrSem) == OS_NO_ERR) {
    zpec_info_fn("Cleared one pending ADC semaphore");
  }

  if (obs->getMode() == MODE_DEVKIT) {
    zpec_debug("Press the board IRQ button to integrate");
    irq = 1;
  } else {
    irq = ZPEC_ADC_IRQ;
  }

  nTicks = ::TimeTick;
  zpec_info("IRQ%d  enabled at boot time %u.%02u s",
            irq, nTicks/TICKS_PER_SECOND,
	    (nTicks % TICKS_PER_SECOND)*(100/TICKS_PER_SECOND));

  for (iRepeat = 0; iRepeat < nRepeat && isAccumulating(); iRepeat++) {
    io_->adc_part_begin = &adcBuffer_[0];
    io_->adc_part_end   = io_->adc_part_begin + nLags();
    io_->adc_full_begin = io_->adc_part_end;
    io_->adc_full_end   = io_->adc_full_begin + nLags();
    io_->adc_calls      = 0;
    io_->adc_count      = 0;
    io_->adc_field      = 0;
    io_->err_first      = 0;
    io_->err_frame      = 0;
    io_->err_mode       = 0;
    io_->mode           = obs->getMode();

    if (iRepeat == 0) { nTicks = ::TimeTick; }
    zpec_setup_irq(irq, EP_FALLING_EDGE, EP_INPUT_PIN, ADC_isr);
    cpld_set_bit(CPLD_BLANK, 0);
    zpec_enable_irq(irq);
      for (nAccum=0; nAccum < nFrames && isAccumulating(); ) {
	if (OSSemPend(&io_->AdcIsrSem, 1) == OS_NO_ERR) {
	  obs->processFrame(io_->adc_full_begin, io_->adc_full_end);
	  ++nAccum;
	} else if (zpec_interrupt(arg.fdRead)) {
	  iRepeat = nRepeat - 1;
	  break;
	}
      }
    zpec_disable_irq(irq);
    cpld_set_bit(CPLD_BLANK, 1);
    if (iRepeat == nRepeat-1) { nTicks = ::TimeTick - nTicks; }

    if (nAccum < nFrames) {
      zpec_warn_fn("Integration terminated early (accumulated %u of %u frames)",
		   nAccum, nFrames);
    }
    if (OSSemPendNoWait(&io_->AdcIsrSem) == OS_NO_ERR) {
      zpec_error_fn("Data integrity failure: readout frame already pending");
    }
    if (io_->err_first) {
      zpec_error_fn("Data integrity failure: ISR first-in-frame error");
    }
    if (io_->err_frame) {
      zpec_error_fn("Data integrity failure: ISR framing error");
    }
    if (io_->err_mode) {
      zpec_error_fn("Data integrity failure: ISR operating mode error");
    }

    nFrames = io_->adc_field >> 1;
    if (nAccum != nFrames) {
      zpec_error_fn("Data integrity failure: readout frame count mismatch "
		    "(%u processed, %u posted)", nAccum, nFrames);
    }

    // Process and publish intermediate integrations ASAP.
    if (iRepeat != nRepeat-1 && isAccumulating()) {
      // Repeating observations must be compatible with BasicObservation.
      BasicObservation *bobs = static_cast<BasicObservation *>(obs);
      collateData(bobs->lagData(), bobs->nBuffers());
      bobs->init(bobs->getMode(), bobs->nLags(), bobs->nBuffers());

      unsigned len = setIntegStatus(status, nAccum, nFrames, false);
      len += siprintf(status+len, "%s", ControlService::prompt);
      zpec_write_retry(arg.fdWrite, status, len, fn);
    }
  }

  zpec_change_prio(ZPEC_CONTROL_PRIO);

  zpec_info("IRQ%d disabled at boot time %u.%02u s (used %u of %u interrupts)"
            "\r\nElapsed time: %u.%02u s for %u frames",
             irq, ::TimeTick/TICKS_PER_SECOND,
	     (::TimeTick % TICKS_PER_SECOND)*(100/TICKS_PER_SECOND),
	     io_->adc_count, io_->adc_calls,
             nTicks/TICKS_PER_SECOND,
	     (nTicks % TICKS_PER_SECOND)*(100/TICKS_PER_SECOND),
	     nAccum*nRepeat);

  return nAccum;
}


/**
  Collates accumulated lag data.
  
  Separates integration results into band-specific correlator output buffers.

  \note
  If no frames were accumulated, no collation is done and previous data will
  be left intact.

  \param lags     Accumulated lag data to collate.
  \param nBuffers Number of buffers to collate.
*/
void Correlator::collateData(const LagData& lags, unsigned nBuffers)
{
  lock();
      for (unsigned iBand=0; iBand<nBands(); iBand++) {
	LagData::iterator p = band_[iBand].lags.begin();

	for (unsigned iBuffer=0; iBuffer<nBuffers; iBuffer++) {
	  if (lags.getFrames(iBuffer) > 0) {
	    LagData::const_iterator
	      bLags = lags.begin()+nLags()*(iBuffer*nBands()+iBand);

	    memcpy(&p[0], &bLags[0], nLags()*sizeof(LagData::value_type));
	    p += nLags();
	  }
	  band_[iBand].lags.setFrames(iBuffer, lags.getFrames(iBuffer));
	  band_[iBand].lags.setTime(iBuffer, lags.getTime(iBuffer));
	}
      }
  unlock();
}


/**
  \brief Sets return status for a generic integration.

  Examines the internal readout buffers and error status flags and 
  constructs a summary integration status message.

  \param status  Storage buffer for return status (should contain at least
                 ControlService::maxLine characters).
  \param nAccum  Number of readout frames accumulated.
  \param nFrames Number of readout frames requested.
  \param checkFraming  Whether to analyze accumulated framing data.

  \return The number of characters written to status.
*/
unsigned Correlator::setIntegStatus(return_type status, unsigned nAccum,
  unsigned nFrames, bool checkFraming)
{
  unsigned len = 0;

  if (io_->err_first || io_->err_frame || io_->err_mode) {
    len += siprintf(status+len, "%s", statusERR);
    if (io_->err_mode) {
      len += siprintf(status+len, "ISR: operating mode error (mode = %d)\r\n  ",
                      static_cast<int>(getMode()));
    } else {
      if (io_->err_first) {
	len += siprintf(status+len,
	                "ISR: %u first-frame error(s) detected\r\n  ",
			io_->err_first);
      }
      if (io_->err_frame) {
	len += siprintf(status+len,
	                "ISR: %u framing error(s) detected\r\n  ",
			io_->err_frame);
      }
    }
  } else {
    len += siprintf(status+len, "%s", nAccum < nFrames ? statusWARN : statusOK);
  }

  len += siprintf(status+len, "Integration %s @ %s; processed %d frames.\r\n",
		  nAccum < nFrames ? "halted" : "complete",
		  timestamp(), nAccum);

  if (checkFraming && getMode() != MODE_DEVKIT) {
    /*
       Diagnose framing errors by displaying anomalous frames.
       Expected pattern for 128 channels per field (2 fields per frame),
       at two samples per interrupt, is: 1/FFFFFFFF 0/FFFFFFFF 0/0 0/0
       Exception: when noise diode(s) are utilized, the 1/FFFFFFFF will
       alternate with 0/FFFFFFFF.
    */
    const unsigned words = nLags() >> 5;  // words per frame
    cpld_mem_t diode = (cpld[CPLD_REG0_RD] & CPLD_NOISE_DIODE0);

    if (io_->err_first || io_->err_frame) {  // There should be frame output.
      len += siprintf(status+len, "\r\n");
    }
    for (unsigned i=0; i<sizeof(io_->first)/sizeof(io_->first[0]) &&
		       i<words*nAccum; i+=words) {
      if (len > ControlService::maxLine - 20*words) { break; }

      unsigned j, frame = i/words;
      for (j=0; j<words; j++) {
	if (io_->first[i+j] != (j==0 && (!diode || !(frame&1)) ? 1 : 0) ||
	    io_->frame[i+j] != (j<(words>>1) ? ~0U : 0)) { break; }
      }
      if (j==words) { continue; }  // data ok

      len += siprintf(status+len, "frame %3u:", frame);
      for (j=0; j<words; j++) {
	len += siprintf(status+len, " %X/%X", io_->first[i+j], io_->frame[i+j]);
      }
      len += siprintf(status+len, "\r\n");
    }
  }

  return len;
}


/**
  \brief Performs a total power integration.

  This method obtains a total power integration for a specified number of
  readout frames.

  \param status Storage buffer for return status (should contain at least
                ControlService::maxLine characters).
  \param arg    Argument list: NFRAMES
*/
void Correlator::execTotalPower(return_type status, argument_type arg)
{
  static const char *usage =
  "NFRAMES [REPEAT]\r\n"
  "  Perform a total power integration.\r\n"
  "  NFRAMES Number of ADC readout frames to accumulate.\r\n"
  "  REPEAT  Number of integrations to perform (default: 1).\r\n";

  unsigned nFrames, nRepeat;

  // Do error handling first.
  if (arg.help || !arg.str || 1 != sscanf(arg.str, "%u", &nFrames)) {
    longHelp(status, usage, &Correlator::execTotalPower);
    if (!arg.help) { status[0] = *statusERR; }
    return;
  }
  if (setAccumulating(true)) {
    siprintf(status, "%sIntegration already in progress.\r\n", statusERR);
    return;
  }
  nRepeat = 1;
  sscanf(arg.str, "%*u%u", &nRepeat);

  // At this point, we have exclusive ADC readout access.
  static BasicObservation obs;
  obs.init(getMode(), nBands()*nLags(), 1);

  // OK, do the integration and collate band data.
  unsigned nAccum = readADCs(&obs, nFrames, arg, status, nRepeat);
  collateData(obs.lagData());

  // Clean up.
  setAccumulating(false);
  setIntegStatus(status, nAccum, nFrames);
}


/**
  \brief Performs a zero offset integration.

  This method obtains a zero offset integration using a specified number of
  readout frames. The microwave power is switched off, a total power
  integration is performed, and the original attenuator settings are restored.

  \param status Storage buffer for return status (should contain at least
                ControlService::maxLine characters).
  \param arg    Argument list: NFRAMES
*/
void Correlator::execZero(return_type status, argument_type arg)
{
  static const char *usage =
  "NFRAMES [REPEAT]\r\n"
  "  Perform a zero offset integration.\r\n"
  "  NFRAMES Number of ADC readout frames to accumulate.\r\n"
  "  REPEAT  Number of integrations to perform (default: 1).\r\n";

  unsigned nFrames, nRepeat;

  // Do error handling first.
  if (arg.help || !arg.str || 1 != sscanf(arg.str, "%u", &nFrames)) {
    longHelp(status, usage, &Correlator::execZero);
    if (!arg.help) { status[0] = *statusERR; }
    return;
  }
  if (setAccumulating(true)) {
    siprintf(status, "%sIntegration already in progress.\r\n", statusERR);
    return;
  }
  nRepeat = 1;
  sscanf(arg.str, "%*u%u", &nRepeat);

  // At this point, we have exclusive ADC readout access.
  static BasicObservation obs;
  obs.init(getMode(), nBands()*nLags(), 1);

  // Set attenuators to maximum (assume 4 bands max).
  unsigned short atten[4];
  for (unsigned i=0; i<nBands(); ++i) {
    atten[i] = band_[i].control.setAttenuation(63); 
  }
  periph_lock();
    cpld_set_atten(-1, 63, hw_);
  periph_unlock();

  // OK, do the integration and collate band data.
  unsigned nAccum = readADCs(&obs, nFrames, arg, status, nRepeat);
  collateData(obs.lagData());

  // Restore attenuator values.
  for (unsigned i=0; i<nBands(); ++i) {
    (void )band_[i].control.setAttenuation(atten[i]); 
    periph_lock();
      cpld_set_atten(i, atten[i], hw_);
    periph_unlock();
  }

  // Clean up.
  setAccumulating(false);
  setIntegStatus(status, nAccum, nFrames);
}


/**
  \brief Performs a lag statistics integration.

  This method determines the lag mean and variance based on the specified
  number of integrations.

  \param status Storage buffer for return status (should contain at least
                ControlService::maxLine characters).
  \param arg    Argument list: NFRAMES [NSEND]
*/
void Correlator::execStatsObs(return_type status, argument_type arg)
{
  static const char *usage =
  "NFRAMES [NSEND]\r\n"
  "  Perform a lag statistics integration.\r\n"
  "  NFRAMES Readout frames to accumulate (max 32768; 0 to redisplay).\r\n"
  "  NSEND   Channels to display upon completion (default: 20).\r\n";

  const int scaleFixed = 1000;
  unsigned nFrames, nSend = 20;

  // Do error handling first.
  if (arg.help || !arg.str || 1 > sscanf(arg.str, "%u%u", &nFrames, &nSend)) {
    longHelp(status, usage, &Correlator::execStatsObs);
    if (!arg.help) { status[0] = *statusERR; }
    return;
  }
  if (setAccumulating(true)) {
    siprintf(status, "%sIntegration already in progress.\r\n", statusERR);
    return;
  }
  nSend = std::min(nSend, nLags());

  // At this point, we have exclusive ADC readout access.
  static const char *fn = "execStatsObs";
  static StatsObservation obs;
  unsigned len, maxBytes = ControlService::maxLine - 40;

  if (nFrames > 0) {
    obs.init(getMode(), nLags()*nBands(), 1, scaleFixed);
    if (nFrames > 32768) { nFrames = 32768; }  // Prevent overflow.

    // OK, do the integration.
    unsigned nAccum = readADCs(&obs, nFrames, arg, status);
    obs.computeStats();

    // Format output.
    len = setIntegStatus(status, nAccum, nFrames);
  } else {
    len = siprintf(status, "%s\r\n", statusOK);
  }
  setAccumulating(false);

  len += siprintf(status+len, " # Chan");
  for (unsigned iBand=0; iBand<nBands(); ++iBand) {
    zpec_write_if_full(arg.fdWrite, status, &len, maxBytes, fn);
    len += siprintf(status+len, "%9sMean%u %8sVar%u", "", iBand, "", iBand);
  }
  len += siprintf(status+len, "\r\n");

  for (unsigned iChan=0; iChan<nSend; ++iChan) {
    len += siprintf(status+len, "%7u", iChan);
    for (unsigned iBand=0; iBand<nBands(); ++iBand) {
      lag_count_t mean  = obs.Mean()[iBand*nLags()+iChan],
                  var   = obs.Variance()[iBand*nLags()+iChan];
      char strMean[16], strVar[16];

      zpec_write_if_full(arg.fdWrite, status, &len, maxBytes, fn);
      len += siprintf(status+len, "  %12s %s%11s",
	       zpec_print_fixed(strMean,    mean, scaleFixed),
	       (var == -std::numeric_limits<int>::max() ? ">" : " "),
	       zpec_print_fixed(strVar, abs(var), (var<0 ? 1 : scaleFixed)));
    }
    len += siprintf(status+len, "\r\n");
  }
}


/**
  Perform an oscilloscope integration.

  This method tracks the complete sample history of a specific set of ADC
  channels. 

  \param status Storage buffer for return status (should contain at least
                ControlService::maxLine characters).
  \param arg    Argument list: CHANSPEC NACCUM NPOINTS
*/
void Correlator::execScopeObs(return_type status, argument_type arg)
{
  static const char *usage =
  "CHANSPEC NACCUM NPOINTS\r\n"
  "  Perform an oscilloscope integration.\r\n"
  "  CHANSPEC Comma-separated list of sampling channels (e.g., 0-15,255).\r\n"
  "  NACCUM   Number of frames to accumulate per sample point.\r\n"
  "  NPOINTS  Number of samples to collect (subject to buffer limits).\r\n";
  unsigned nChannels, nAccum, nPoints;

  // Do error handling first.
  if (arg.help || !arg.str ||
      2 != sscanf(arg.str, "%*s%u%u", &nAccum, &nPoints)) {
    longHelp(status, usage, &Correlator::execScopeObs);
    if (!arg.help) { status[0] = *statusERR; }
    return;
  }
  if (setAccumulating(true)) {
    siprintf(status, "%sIntegration already in progress.\r\n", statusERR);
    return;
  }

  // At this point, we have exclusive ADC readout access.
  static const char *fn = "execScopeObs";
  static const unsigned maxChannels = 256;
  static int iLag[maxChannels];
  static ScopeObservation obs;

  // Parse parameters and initialize observation.
  nChannels = zpec_parse_spec(arg.str, iLag, 0, nBands()*nLags()-1,
                              maxChannels);
  if (nChannels == 0) { iLag[nChannels++] = 0; }
  obs.init(getMode(), nBands()*nLags(), nAccum, nChannels, iLag);

  // Avoid buffer overflow.
  nPoints = std::min(nPoints, ScopeObservation::maxSamples/nChannels);

  // OK, do the integration.
  unsigned nFrames = readADCs(&obs, nAccum*nPoints, arg, status);
  setAccumulating(false);

  // Format output.
  unsigned len = setIntegStatus(status, nFrames, nAccum*nPoints),
           maxBytes = ControlService::maxLine - 20;

  len += siprintf(status+len, " # Channel: ");
  for (unsigned iChannel=0; iChannel<nChannels; ++iChannel) {
    zpec_write_if_full(arg.fdWrite, status, &len, maxBytes, fn);
    len += siprintf(status+len, "%3u%9s", iLag[iChannel], "");
  }
  len += siprintf(status+len, "\r\n");

  for (unsigned iSample=0; iSample<nPoints; ++iSample) {
    len += siprintf(status+len, "%3s", "");
    for (unsigned iChannel=0; iChannel<nChannels; ++iChannel) {
      zpec_write_if_full(arg.fdWrite, status, &len, maxBytes, fn);
      len += siprintf(status+len, "%12d", obs.sample(iChannel, iSample));
    }
    len += siprintf(status+len, "\r\n");
  }
}


/**
  \brief Gets/sets attenuation, or performs a level set integration.

  This method displays or changes the attenuator setting. Given zero
  arguments, the current setting is returned; given one, the attenuation is
  set to the specified value. Two arguments are interpreted as the ADC channel
  number and count target for the auto-level set procedure.

  The level auto-set algorithm is a bounded iteration. The counts C should
  vary with the attenuation A according to C(A) = C0 + C1*10^(-A/10). The
  range of the attenuator is 0-31 dB (32+ switches microwave power off).
  After estimating C0 (using A=63) and C1 (using A=15 and C0), iteration
  proceeds until counts within 1 dB of the target are obtained. A bracket is
  maintained to ensure convergence. If the tolerance cannot be met, a warning
  is returned.

  \param status Storage buffer for return status (should contain at least
                ControlService::maxLine characters).
  \param arg    Argument list: [ATTEN] | CHANNEL COUNTS [NFRAMES]
*/
void Correlator::execLevel(return_type status, argument_type arg)
{
  static const char *usage =
  "[ATTEN] | CHANNEL COUNTS [NFRAMES]\r\n"
  "  Form one:\r\n"
  "    Get or set absolute attenuation setting.\r\n"
  "    ATTEN  Attenuation in dB (0-31; 32+: max; default: display current).\r\n"
  "\r\n"
  "  Form two:\r\n"
  "    Auto-set attenuation level to achieve specified channel counts.\r\n"
  "    CHANNEL  ADC channel to monitor.\r\n"
  "    COUNTS   Target count value for CHANNEL.\r\n"
  "    NFRAMES  Number of frames to accumulate per iteration (default: 4).\r\n";

  // Do error handling first.
  if (arg.help) {
    longHelp(status, usage, &Correlator::execLevel);
    if (!arg.help) { status[0] = *statusERR; }
    return;
  }

  long arg2;
  unsigned len, arg1, newAtten, nFrames = 4,
	   nArg = (!arg.str ?
	           0 : sscanf(arg.str, "%u%ld%u", &arg1, &arg2, &nFrames));

  if (nArg == 0) {
    siprintf(status, "%sCurrent attenuation is %u dB.\r\n",
	     statusOK, band_[0].control.getAttenuation());
    return;
  }
 
  if (nArg == 1) {
    // Set absolute attenuation value.
    len = 0;
    newAtten = (arg1 > 31 ? 63 : arg1);
  } else {
    // Auto-set attenuation level.
    if (setAccumulating(true)) {
      siprintf(status, "%sIntegration already in progress.\r\n", statusERR);
      return;
    }

    // At this point, we have exclusive ADC readout access.
    static BasicObservation obs;
    obs.init(getMode(), nBands()*nLags());

    // OK, execute the level set algorithm.
    const int dB = (1<<16)/10;
    unsigned iChannel = arg1;
    lag_count_t C, Cmin, Cmax, Cbest, C0, Cgoal = std::min(arg2, 65535L);
    int A, Amin, Amax, Abest, logC, logCmin, logCmax, logCbest, logC1, logCgoal;

    // Determine C0 (zero offset).
    len = 0;
    C0 = -10;  // Just use crude estimate; it doesn't make much difference.
        // findCounts(status, &len, &obs, 60, iChannel, nFrames, arg.fdRead);
    if (len > 0) {
      setAccumulating(false);
      return;  // findCounts() failed.
    }
    logCgoal = zpec_log10_fix16(Cgoal-C0);

    // Estimate logC1 (all logarithms are scaled by 2^16).
    A = 15;
    len = 0;
    C = findCounts(status, &len, &obs, A, iChannel, nFrames, arg.fdRead);
    if (len > 0) {
      setAccumulating(false);
      return;  // findCounts() failed.
    }
    logC  = zpec_log10_fix16(C-C0);
    logC1 = (A<<16)/10 + logC;

    // Initialize bracket to (impossible) maximum range.
    // The placeholder values serve to constrain the search.
    if (C < Cgoal) {
      Amin=-1;        Amax=A;
      Cmin=1000000;   Cmax=C;
      logCmin=6<<16;  logCmax=logC;
    } else {
      Amin=A;        Amax=32;
      Cmin=C;        Cmax= -1000000;
      logCmin=logC;  logCmax= -6<<16;
    }
    Abest=A;  Cbest=C;  logCbest=logC;

    // Iterate until 1 dB tolerance met, or bracket values are adjacent.
    while (abs(logCbest-logCgoal) > dB &&  Amax-Amin > 1) {
      // Next estimate (rounded); maintain bracket.
      int A1 = (10*(logC1-logCgoal)) >> 15;
      A = (A1>>1)+(A1&1);
      if (A <= Amin) { A=Amin+1; } else if (A >= Amax) { A=Amax-1; }

      // Evaluate new counts.
      len = 0;
      C = findCounts(status, &len, &obs, A, iChannel, nFrames, arg.fdRead);
      if (len > 0) {
	setAccumulating(false);
	return;  // findCounts() failed.
      }
      logC  = zpec_log10_fix16(C-C0);
      logC1 = (A<<16)/10 + logC;

      // Update best estimate and bracket.
      if (abs(logC-logCgoal) < abs(logCbest-logCgoal)) {
	Abest=A;  Cbest=C;  logCbest=logC;
      }
      //
      if ((Cmin<=Cgoal && C<=Cgoal) || (Cmin>=Cgoal && C>=Cgoal)) {
        Amin=A;  Cmin=C;  logCmin=logC;
      } else {
        Amax=A;  Cmax=C;  logCmax=logC;
      }
    }
    setAccumulating(false);

    if (abs(logCbest-logCgoal) > dB) {
      len = siprintf(status, "%sPoor convergence in level set", statusWARN);
    } else {
      len = siprintf(status, "%sLevel set succeeded", statusOK);
    }
    // Note: retain the following ':' to aid automated parsing of results.
    len += siprintf(status+len, " (channel %u counts: %ld @ %d dB).\r\n",
		    iChannel, Cbest, Abest);
    newAtten = Abest;
  }

  unsigned oldAtten = 0;
  for (unsigned i=0; i<nBands(); ++i) {
    oldAtten = band_[i].control.setAttenuation(newAtten);
  }
  periph_lock();
    cpld_set_atten(-1, newAtten, hw_);
  periph_unlock();

  // Note: retain the following '=' to aid automated parsing of results.
  siprintf(status+len, "%sSet attenuation = %u dB (was %u dB).\r\n",
	   (nArg == 1 ? statusOK : "  "), newAtten, oldAtten);
}


/**
  Estimate average counts per frame for an ADC channel.

  If the associated integration fails, its status is reported; otherwise,
  \a status is not altered.

  \param status  Return status for failed integrations.
  \param len     Number of characters written to \a status.
  \param obs     Observation for data accumulation.
  \param atten   Attenuation setting applied during observation.
  \param channel ADC channel for which to report counts.
  \param nFrames Number of readout frames used to estimate average counts.
  \param fdRead  Open, readable file descriptor for detecting user interrupt.

  \return Average counts per frame for \a channel.
*/
lag_count_t Correlator::findCounts(return_type status, unsigned *len,
  BasicObservation *obs, int atten, unsigned channel, unsigned nFrames,
  int fdRead)
{
  if (obs->getMode() == MODE_PATTERN) {
    const lag_count_t dB = 52000;
    lag_count_t rtn = 40000;
    for (int i=0; i<atten; i++) { rtn = (rtn*dB)>>16; }
    return -10+rtn;
  } else {
    periph_lock();
      cpld_set_atten(-1, atten, hw_);
    periph_unlock();
    argument_type arg(0, fdRead);
    unsigned nAccum = readADCs(obs, nFrames, arg, status),
	     len1 = setIntegStatus(status, nAccum, nFrames);

    if (strncmp(status, statusOK, strlen(statusOK))) { *len = len1; }
    if (nAccum == 0) { nAccum = 1; }
    return obs->lagData()[channel]/(lag_count_t )nAccum;
  }
}


/**
  \brief Performs a switching noise diode integration.

  This method obtains a switched noise diode integration for a specified
  number of readout frames per diode state. In this observation mode, readout
  frames alternate between the noise diode being on and off, where the first
  frames received after enabling interrupts is guaranteed to correspond to the
  active noise diode state.

  To support arbitrary repeating integration frame counts, the NFRAMES
  parameter refers to the total integration, \e NOT the number of frames per
  diode state. If NFRAMES is odd, the second state will accumulate one less
  frame than the first. In this case, counts in the second state will be
  depressed due to the one cycle deficit in integration time. It is up to the
  client to correct for this (since it specified NFRAMES, it will always be
  aware of the problem).

  \param status Storage buffer for return status (should contain at least
                ControlService::maxLine characters).
  \param arg    Argument list: NFRAMES
*/
void Correlator::execDiodeObs(return_type status, argument_type arg)
{
  static const char *usage =
  "NFRAMES [REPEAT]\r\n"
  "  Perform a switching noise diode integration.\r\n"
  "  NFRAMES Number of ADC readout frames to accumulate (*total*).\r\n"
  "  REPEAT  Number of integrations to perform (default: 1).\r\n";

  const unsigned nStates = 2;
  unsigned nFrames, nRepeat;

  // Do error handling first.
  if (arg.help || !arg.str || 1 != sscanf(arg.str, "%u", &nFrames)) {
    longHelp(status, usage, &Correlator::execDiodeObs);
    if (!arg.help) { status[0] = *statusERR; }
    return;
  }
  if (setAccumulating(true)) {
    siprintf(status, "%sIntegration already in progress.\r\n", statusERR);
    return;
  }
  nRepeat = 1;
  sscanf(arg.str, "%*u%u", &nRepeat);

  // At this point, we have exclusive ADC readout access.
  static BasicObservation obs;
  obs.init(getMode(), nBands()*nLags(), nStates);

  // Initialize noise diode.
  cpld_mem_t dbit, jbit;
  dbit = (cpld[CPLD_REG0_RD] & CPLD_NOISE_DIODE0);
  jbit = (cpld[CPLD_REG0_RD] & CPLD_DIODE0_JAM);
  cpld_set_bit(CPLD_NOISE_DIODE0, 1);
  cpld_set_bit(CPLD_DIODE0_JAM, 0);

  // OK, do the integration and collate band data.
  unsigned nAccum = readADCs(&obs, nFrames, arg, status, nRepeat);
  collateData(obs.lagData(), nStates);

  // Clean up. NOTE: must set status before turning off diode.
  setIntegStatus(status, nAccum, nFrames);
  cpld_set_bit(CPLD_DIODE0_JAM, jbit);
  cpld_set_bit(CPLD_NOISE_DIODE0, dbit);
  setAccumulating(false);
}
