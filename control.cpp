/**
  \file
  \author Kevin P. Rauch
  \brief Correlator command and control functionality.

   $Id: control.cpp,v 1.48 2014/06/04 18:36:44 harris Exp $
*/
#include <basictypes.h>
#include <bsp.h>
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <algorithm>

#include "control.h"
#include "argus.h"

const char *Correlator::statusEOF = "**EOF**";
const char *Correlator::statusERR = "! ";
const char *Correlator::statusWARN= "? ";
const char *Correlator::statusOK  = "# ";
const char *Correlator::monitorPointName[ADC_NCHAN] = {
  "Taux",   ///<  0: Auxilliary interface card temperature (C).
  "Tcorl",  ///<  1: Central correlator card temperature (C).
  "Tamp1",  ///<  2: Amplifier module 1 temperature (C).
  "Tamp2",  ///<  3: Amplifier module 2 temperature (C).
  "+3.3_V", ///<  4:  +3.3 V power supply voltage (V).
  "+5_V",   ///<  5:  +5   V power supply voltage (V).
  "-5_V",   ///<  6:  -5   V power supply voltage (V).
  "+15_V",  ///<  7: +15   V power supply voltage (V).
  //
  "Tcorr7", ///<  8: Correlator card #7 temperature (C).
  "Tcorr0", ///<  9: Correlator card #0 temperature (C).
  "AvgT1",  ///< 10: Weighted average temperature T1 (mV).
  "AvgT2",  ///< 11: Weighted average temperature T2 (mV).
  "Tampmod",///< 12: Amplifier module temperature (C).
  "Tamb",   ///< 13: Ambient temperature (C).
  "ExtInp", ///< 14: External input (V).
  //
  "TotPwr", ///< 15: Total RF power (V).
  "SwPwr",  ///< 16: Switching(?) RF power (V).
  "Tzon",   ///< 17: IF processor temperature (C).
  "Tgnd",   ///< 18: IF processor temperature offset (C).
  "TiltX",  ///< 19: Power supply X-axis tilt (deg).
  "TiltY",  ///< 20: Power supply Y-axis tilt (deg).
  "+12Vzon",///< 21: IF processor +12 V rail (V).
  "Tpow",   ///< 22: Power supply temperature (C).
  "+3.3_V", ///< 23: Power supply  +3.3 V rail voltage (V).
  "+3.3_A", ///< 24: Power supply  +3.3 V rail current (A).
  "+5_V",   ///< 25: Power supply  +5   V rail voltage (V).
  "+5_A",   ///< 26: Power supply  +5   V rail current (A).
  "-5_V",   ///< 27: Power supply  -5   V rail voltage (V).
  "-5_A",   ///< 28: Power supply  -5   V rail current (A).
  "+15_V",  ///< 29: Power supply +15   V rail voltage (V).
  "+15_A",  ///< 30: Power supply +15   V rail current (A).
};

const char *Correlator::monitorPointUnit[ADC_NCHAN] = {
  "degC",  ///<  0
  "degC",  ///<  1
  "degC",  ///<  2
  "degC",  ///<  3
  "V",     ///<  4
  "V",     ///<  5
  "V",     ///<  6
  "V",     ///<  7
  //
  "degC",  ///<  8
  "degC",  ///<  9
  "V",     ///< 10
  "V",     ///< 11
  "degC",  ///< 12
  "degC",  ///< 13
  "V",     ///< 14
  //
  "V",     ///< 15
  "V",     ///< 16
  "degC",  ///< 17
  "degC",  ///< 18
  "deg",   ///< 19
  "deg",   ///< 20
  "V",     ///< 21
  "degC",  ///< 22
  "V",     ///< 23
  "A",     ///< 24
  "V",     ///< 25
  "A",     ///< 26
  "V",     ///< 27
  "A",     ///< 28
  "V",     ///< 29
  "A",     ///< 30

};


/**
  \brief Sets current value of one or all monitor points.

  \warning This method assumes the caller holds the control lock.

  \param channel Specific monitoring channel to update (-1 for "all").
  \param useLock Whether to obtain the zpectrometer lock before updating.
*/
void Correlator::setMonitorPoint(int channel, bool useLock)
{
  if (useLock) { periph_lock(); }
    int chan0 = (channel<0 ? monitorBegin() : channel),
	chan1 = (channel<0 ? monitorEnd()   : channel+1);

    for (int chan=chan0; chan<chan1; ++chan) {
	int fixval = cpld_read_mon_adc((adc_channel_t )chan);
	char str[16];
	monPoints_.set(monitorPointName[chan],
	               zpec_print_fixed(str, fixval, 1000),
	               monitorPointUnit[chan]);
    }
  if (useLock) { periph_unlock(); }
}


/// Returns operating mode.
zpec_mode_t Correlator::getMode() const
{
  zpec_mode_t rtn;
  lock();
    rtn = mode_;
  unlock();
  return rtn;
}

/**
  Set operating mode.

  \param mode New operating mode.
  \return Previous operating mode.
*/
zpec_mode_t Correlator::setMode(zpec_mode_t mode)
{
  zpec_mode_t rtn;
  lock();
    rtn = mode_;
    mode_ = mode;
  unlock();
  return rtn;
}


/// Returns log verbosity level.
int Correlator::getVerbosity() const
{
  int rtn;
  lock();
    rtn = verbose_;
  unlock();
  return rtn;
}


/**
  \brief Generate FITS Y2K-format date/time string.

  \note
  The current system time is used if both \a utc_sec and \a utc_csec are zero.

  \param utc_sec  UTC seconds since the Epoch.
  \param utc_csec Fractional centiseconds.
  \return Pointer to (static) date string.
*/
const char *Correlator::timestamp(unsigned utc_sec, unsigned utc_csec)
{
  static char date[32];

  if (utc_sec + utc_csec == 0) {
    unsigned long long now = bootTicks_ + ::TimeTick;
    utc_sec  =  now / TICKS_PER_SECOND;
    utc_csec = (now % TICKS_PER_SECOND) * (100U/TICKS_PER_SECOND);
  }

  time_t tsec = utc_sec;
  struct tm *gm = gmtime(&tsec);
  siprintf(date, "%d-%02d-%02dT%02d:%02d:%02d.%02d",
           1900+gm->tm_year, 1+gm->tm_mon, gm->tm_mday,
           gm->tm_hour, gm->tm_min, gm->tm_sec, utc_csec);

  return date;
}


/**
  Set log verbosity level.

  \param level New verbosity level.
  \return Previous verbosity level.
*/
int Correlator::setVerbosity(int level)
{
  int rtn;
  lock();
    rtn = verbose_;
    verbose_ = level;
  unlock();
  return rtn;
}


/// Returns whether ADC data is being accumulated.
bool Correlator::isAccumulating() const
{
  bool rtn;
  lock();
    rtn = accumulating_;
  unlock();
  return rtn;
}

/**
  Set state of data accumulating flag.

  \param state New data accumulation state.
  \return Previous data accumulation state.
*/
bool Correlator::setAccumulating(bool state)
{
  bool rtn;
  lock();
    rtn = accumulating_;
    accumulating_ = state;
  unlock();
  return rtn;
}


/// Returns whether this correlator is the master.
bool Correlator::isMaster() const
{
  bool rtn;
  lock();
    rtn = master_;
  unlock();
  return rtn;
}

/**
  Sets the correlator master/slave state.
  Also resets the CPLD so that slaves obey their local clock.

  \param state New master/slave state.
  \return Previous master/slave state.
*/
bool Correlator::setMaster(bool state)
{
  bool rtn;
  lock();
    rtn = master_;
    master_ = state;
    cpld_set_bit(CPLD_INT_RESET, 1);
    cpld_set_bit(CPLD_INT_RESET, 0);
    cpld_set_bit(CPLD_MASTER, state);
  unlock();
  return rtn;
}


/**
  \brief Gets or sets master/slave mode for chop/nod signals.

  \param status Storage buffer for return status (should contain at least
                ControlService::maxLine characters).
  \param arg    Argument list: [MASTER]
*/
void Correlator::execBoss(return_type status, argument_type arg)
{
  static const char *usage =
  "[MASTER]\r\n"
  "  Get or set master/slave status.\r\n"
  "  MASTER  Designate as master (if non-zero) or slave (if zero).\r\n";

  if (!arg.help) {
    if (arg.str) {
      setMaster(atoi(arg.str));
      siprintf(status, "%sSet master/slave mode to %s.\r\n", statusOK,
               isMaster() ? "master" : "slave");
    } else {
      siprintf(status, "%sCurrently acting as %s.\r\n", statusOK,
               isMaster() ? "master" : "slave");
    }
  } else {
    longHelp(status, usage, &Correlator::execBoss);
  }
}


/**
  \brief Get or set operating mode.

  See zpec_mode_t for a description of defined operating modes.

  \param status Storage buffer for return status (should contain at least
                ControlService::maxLine characters).
  \param arg    Argument list: [MODE]
*/
void Correlator::execMode(return_type status, argument_type arg)
{
  static const char *usage =
  "[MODE]\r\n"
  "  Get or set operating ('evaluation') mode (0-5).\r\n"
  "  MODE  Operating mode (0: Normal, 1: Test Pattern, 2: DevKit,\r\n"
  "                        3: + phase, 4: - phase; 5: Extreme Pattern\r\n"
  "                        default: 0).\r\n";

  if (!arg.help) {
    if (arg.str) {
      zpec_mode_t m0 = setMode(static_cast<zpec_mode_t>(atoi(arg.str)));
      zpec_mode_t m1 = getMode();
      siprintf(status, "%sNew operating mode is %d: %s (was %d: %s).\r\n",
		       statusOK, static_cast<int>(m1), zpec_mode_name(m1),
				 static_cast<int>(m0), zpec_mode_name(m0));
    } else {
      zpec_mode_t m = getMode();
      siprintf(status, "%sCurrent operating mode is %d: %s.\r\n",
		       statusOK, static_cast<int>(m), zpec_mode_name(m));
    }
  } else {
    longHelp(status, usage, &Correlator::execMode);
  }
}


/**
  \brief Read/write user flash memory.

  Reads or writes the user flash memory structure.

  \param status Storage buffer for return status (should contain at least
                ControlService::maxLine characters).
  \param arg    Argument list: [write SERIALNO HWTYPE].
*/
void Correlator::execFlash(return_type status, argument_type arg)
{
  static const char *usage =
  "[write KEYWORD [VALUE]]\r\n"
  "  Read or write contents of user flash memory.\r\n"
  "  Key = SERIALNO  CPU serial number (unsigned short).\r\n"
  "  Key = HWTYPE    Hardware subtype (0 = GBT, 1 = RLT, 2 = POW, 3 = ARG).\r\n"
  "  Key = GVDIV     Gate voltage divider ratio (0 < GVDIV <=1) for LNAs\r\n"
  "  Key = YIGSLOPE  Linear fit slope parameter for YIG filter\r\n"
  "  Key = YIGINT    Linear fit intercept parameter for YIG filter\r\n"
  "  Key = SETS      Keep current LNA bias settings as preset for LNAs \r\n";

  if (!arg.help) {
    flash_t flashData;
    char keywd[10] = {0};
    float value;

    zpec_readFlash(&flashData);  // read to keep unchanged values as defaults

    if (arg.str) {
    	int narg = sscanf(arg.str, "%5s%9s%f", status, keywd, &value);
    	if (narg == 2 || narg == 3) {
    		  if (strcasecmp(status, "write") != 0) {
    		    longHelp(status, usage, &Correlator::execFlash);
    		    return;
    		  }

    		  if (strcasecmp(keywd, "serialno") == 0) flashData.serialNo = (short)value;
    		  else if (strcasecmp(keywd, "hwtype") == 0) flashData.hw = (zpec_hw_t)value;
    		  else if (strcasecmp(keywd, "gvdiv") == 0) // check for out of range values in argus_init
    		    flashData.gvdiv = (float)value;
    		  else if (strcasecmp(keywd, "yigslope") == 0) // YIG tuning slope
    		    flashData.yigFit[0] = (float)value;
    		  else if (strcasecmp(keywd, "yigint") == 0) // YIG tuning intercept
    		    flashData.yigFit[1] = (float)value;
    		  else if (strcasecmp(keywd, "sets") == 0){ ;
    		    // Move LNA bias settings to flashData; storage is g, d, m  (matches argus_LNApresets)
    		    short i, j, k;
    		    k = 2*NSTAGES + NMIX;
    		    for (i=0; i<NRX; i++) {
    		      for (j=0; j<NSTAGES; j++){
    		    	  if (rxPar[i].LNAsets[j+NSTAGES] >= VDMIN && rxPar[i].LNAsets[j+NSTAGES] <= VDMAX) {
    		    		  flashData.LNAsets[i*k+j+NSTAGES] = rxPar[i].LNAsets[j+NSTAGES];
    		    	  } else {
    		    		  flashData.LNAsets[i*k+j+NSTAGES] = VDSTART;
    		    	  }
    		    	  if (rxPar[i].LNAsets[j] >= VGMIN && rxPar[i].LNAsets[j] <= VGMAX) {
    		    		  flashData.LNAsets[i*k+j] = rxPar[i].LNAsets[j];
    		    	  } else {
    		    		  flashData.LNAsets[i*k+j] = VGSTART;
    		    	  }
    		      }
    		      if (NMIX > 0) {
    		    	  for (j=0; j<NMIX; j++){
    		    		  if (rxPar[i].LNAsets[j+NSTAGES+NMIX] >= VMMIN && rxPar[i].LNAsets[j+NSTAGES+NMIX] <= VMMAX){
    		    			  flashData.LNAsets[i*k+j+NSTAGES+NMIX] = rxPar[i].LNAsets[j+NSTAGES+NMIX];
    		    		  } else {
    		    			  flashData.LNAsets[i*k+j+NSTAGES+NMIX] = VMSTART;
    		    		  }
    		    	  }
    		      }
    		    }
    		    if (NWIFBOX > 0){
    		    	for (i=0; i<NRX; i++) {
    		    		flashData.atten[i] = wifPar.atten[i];
    		    		flashData.sb[i] = wifPar.sb[i];
    		    	}
    		    }
    		  } else {
    	            // no valid selection, quit
    		    longHelp(status, usage, &Correlator::execFlash);
    		    return;
    		  }

    	          // survived test for valid data, so write to flash
    	 	  flashData.signature = FLASH_SIGNATURE;
    		  flashData.valid = 1;
    		  int rtn = zpec_writeFlash(&flashData);
    		  switch (rtn) {
    		  case 0:
    		    siprintf(status, "%sFlash write succeeded "
    			     "(reboot to use new settings).\r\n", statusOK);
    		    break;
    		  default:
    		    siprintf(status, "%sFlash write failed: %s error.\r\n",
    			     statusERR, rtn==1 ? "system" :
    			     rtn==2 ? "invalid structure" : "unknown");
    		    break;
    		  }
    	} else {
		    longHelp(status, usage, &Correlator::execFlash);
    	}
    } // end if (arg.str) true
    else {  // if (arg.str) not true
  	  unsigned len = 0;

        //zpec_readFlash(&flashData);
  	  if (flashData.valid) {
  		  len += siprintf(status+len, "%sUser flash contents:\r\n", statusOK);
  		  len += siprintf(status+len,
  				  "  Serial number:  %2hu\r\n", flashData.serialNo);
  		  len += siprintf(status+len,
  				  "  Hardware type:  %2d (%s)\r\n",
  				  (int )flashData.hw,
  				  flashData.hw == ZPEC_HW_GBT ? "GBT" :
			  	  flashData.hw == ZPEC_HW_RLT ? "RLT" :
			          flashData.hw == ZPEC_HW_POW ? "POW" :
			          flashData.hw == ZPEC_HW_ARG ? "ARG" :
			          "UNKNOWN"
			  	  	  );
  		  len += sprintf(status+len,
  				  "  Gate voltage divisor %f\r\n", flashData.gvdiv);
  		  len += sprintf(status+len,
  				  "  YIG fit slope %f, int %f\r\n", flashData.yigFit[0], flashData.yigFit[1]);
  	  } else {
  		  siprintf(status, "%sUser flash area invalid (or uninitialized).\r\n",
  				  statusERR);
  	  }
    }

  } // end if (!arg.help) is true
  else {  // if (!arg.help) is not true
	  longHelp(status, usage, &Correlator::execFlash);
  }  // end for (!arg.help) not true

}

/**
  \brief Halts the current integration.

  \param status Storage buffer for return status (should contain at least
                ControlService::maxLine characters).
  \param arg    Argument list (unused).
*/
void Correlator::execHalt(return_type status, argument_type arg)
{
  static const char *usage =
  "\r\n"
  "  Halt accumulation of correlator ADC data.\r\n";

  if (!arg.help) {
    if (setAccumulating(false)) {
      siprintf(status, "%sIntegration halted.\r\n", statusOK);
    } else {
      siprintf(status, "%sNo integration in progress.\r\n", statusWARN);
    }
  } else {
    longHelp(status, usage, &Correlator::execHalt);
  }
}


/**
  \brief Initializes correlator ADCs.

  \param status Storage buffer for return status (should contain at least
                ControlService::maxLine characters).
  \param arg    Argument list: [K] [L] [M] [I] [O]

  \see cpld_setup_corl_adc()
*/
void Correlator::execInitADCs(return_type status, argument_type arg)
{
  static const char *usage =
  "[K] [L] [M] [I] [O]\r\n"
  "  Initialize correlator ADCs.\r\n"
  "  [K]  Acquisition Time Control (0, 1, 16, 32; default: 16).\r\n"
  "  [L]  Oversampling Control         (1-256; default: 128).\r\n"
  "  [M]  Multiple Integration Control (1-256; default: 1).\r\n"
  "  [I]  Input Range   (0 = unipolar, 1 = bipolar; default: 0).\r\n"
  "  [O]  Output Format (0 = unsigned, 1 = two's complement; default: 0).\r\n";

  if (!arg.help) {
    unsigned short K, L, M, I, O;
    int nArg = 0;

    if (arg.str) {
      nArg = sscanf(arg.str, "%hu%hu%hu%hu%hu", &K, &L, &M, &I, &O);
    }
    switch (nArg) {  /* Fill in default values as needed. */
      case 0:  K = 16;   /* and drop... */
      case 1:  L = 128;  /* and drop... */
      case 2:  M = 1;    /* and drop... */
      case 3:  I = 0;    /* and drop... */
      case 4:  O = 0;    /* and drop... */
      default: break;
    }

    periph_lock();
      error_.initADC = cpld_init_corl_adc(K, L, M, I, O);
    periph_unlock();
    siprintf(status, "%sInitialized correlator ADCs: "
             "K=%hu, L=%hu, M=%hu, I=%hu, O=%hu  (%d ADCs failed)\r\n",
	     error_.initADC==0 ? statusOK : statusERR, K, L, M, I, O,
	     error_.initADC);
  } else {
    longHelp(status, usage, &Correlator::execInitADCs);
  }
}


/**
  \brief Reads and displays the current contents of CPLD memory.

  \param status Storage buffer for return status (should contain at least
                ControlService::maxLine characters).
  \param arg    Argument list (unused).
*/
void Correlator::execPeek(return_type status, argument_type arg)
{
  static const char *usage =
  "\r\n"
  "  Display the current contents of CPLD memory.\r\n";

  if (!arg.help) {
    int len = siprintf(status, "%sCPLD 0x0-0x7:", statusOK);
    for (int i=0; i<16; i++) {
      len += siprintf(status+len, " 0x%0*X",
                      2*sizeof(cpld_mem_t), cpld[i]);
      if (i==7 || i==15) { len += siprintf(status+len, "\r\n"); }
      if (i==7) { len += siprintf(status+len, "       0x8-0xF:"); }
    }
  } else {
    longHelp(status, usage, &Correlator::execPeek);
  }
}


/**
  \brief Writes a location in CPLD memory.

  The \a ADDRESS and \a VALUE arguments can be specified
  in decimal, hexadecimal (leading 0x), or octal (leading 0) notation.

  \param status Storage buffer for return status (should contain at least
                ControlService::maxLine characters).
  \param arg    Argument list: ADDRESS[.BIT] VALUE
*/
void Correlator::execPoke(return_type status, argument_type arg)
{
  static const char *usage =
  "ADDRESS[.BIT] VALUE\r\n"
  "  Write a word (or bit) to a CPLD memory location.\r\n"
  "  ADDRESS  CPLD memory address to poke (0x0-0x1).\r\n"
  "  BIT      Specific bit (0-15) within ADDRESS to change.\r\n"
  "  VALUE    Value to write (16-bit decimal, hexadecimal, or octal).\r\n";

  if (!arg.help && arg.str) {
    int address, bit, value;

    if (3 == sscanf(arg.str, "%i.%d%i", &address, &bit, &value)) {
      cpld_set_bit(static_cast<cpld_laddr_t>((address<<16)+(1<<bit)), value);
      siprintf(status, "%sSet CPLD[%#x.%d] = %d.\r\n",
               statusOK, address, bit, value ? 1 : 0);
      return;
    }
    if (2 == sscanf(arg.str, "%i%i", &address, &value)) {
      cpld[address] = cpld_cache[address] = value;
      siprintf(status, "%sSet CPLD[%#x] = 0x%04X.\r\n",
               statusOK, address, value);
      return;
    }
  }

  longHelp(status, usage, &Correlator::execPoke);
  if (!arg.help) { status[0] = *statusERR; }
}


/**
  \brief Turn main power supply on or off.

  This method controls the state of the main power supply and front panel
  on/off switch. The latter can be electronically disabled to guarantee
  observance of the remotely commanded state. A syntax error aborts the
  command immediately (state left unchanged).

  \note
  This command is available on power supply modules only.

  \param status Storage buffer for return status (should contain at least
                ControlService::maxLine characters).
  \param arg    Argument list: [STATE] [MANUAL]
*/
void Correlator::execPower(return_type status, argument_type arg)
{
  static const char *usage =
  "[STATE] [MANUAL]\r\n"
  "  Set or query power supply state.\r\n"
  "  STATE   1 or ON to power on, 0 or OFF to power off, - to leave as is.\r\n"
  "  MANUAL  1 or ENABLE to activate switch, 0 or DISABLE to deactivate it.\r\n";

  if (!arg.help) {
    if (arg.str) {
      /* Set new state. */
      char state[8] = {0}, manual[8] = {0};
      unsigned len;

      sscanf(arg.str, "%7s%7s", state, manual);
      len = siprintf(status, "%sInvalid power state '%s'.\r\n",
		     statusERR, state);

      if (!strcmp(state, "1") || !strcasecmp(state, "ON")) {
	cpld_set_bit(CPLD_POW_ON, 1);
	zpec_usleep(0);
	cpld_set_bit(CPLD_POW_ON, 0);
	len = siprintf(status, "%sSet power state to ON.\r\n", statusOK);
      } else if (!strcmp(state, "0") || !strcasecmp(state, "OFF")) {
	cpld_set_bit(CPLD_POW_OFF, 1);
	zpec_usleep(0);
	cpld_set_bit(CPLD_POW_OFF, 0);
	len = siprintf(status, "%sSet power state to OFF.\r\n", statusOK);
      } else if (!strcmp(state, "-")) {
	len = siprintf(status, "%sPower state not modified (currently %s).\r\n",
		       statusOK, cpld_get_bit(CPLD_POW_STATE) ? "ON" : "OFF");
      }

      if (!strcmp(manual, "1") || !strcasecmp(manual, "ENABLE")) {
	cpld_set_bit(CPLD_POW_MANUAL, 0);
	strcpy(status+len, "  Manual switch has been ENABLED.\r\n");
      } else if (!strcmp(manual, "0") || !strcasecmp(manual, "DISABLE")) {
	cpld_set_bit(CPLD_POW_MANUAL, 1);
	strcpy(status+len, "  Manual switch has been DISABLED.\r\n");
      } else {
	siprintf(status+len,
		 "  Manual switch setting not modified (currently %s).\r\n",
		 cpld_get_bit(CPLD_POW_OVERRIDE) ? "DISABLED" : "ENABLED");
      }
    } else {
      /* Query state. */
      siprintf(status, "%sPower supply is %s; front panel switch is %s.\r\n",
	       statusOK, cpld_get_bit(CPLD_POW_STATE) ? "ON" : "OFF",
	                 cpld_get_bit(CPLD_POW_OVERRIDE) ?
					"DISABLED" : "ENABLED");
    }
    return;
  }

  longHelp(status, usage, &Correlator::execPower);
  if (!arg.help) { status[0] = *statusERR; }
}


/**
  \brief Queries monitor ADC values.

  Returns a formatted list of current temperature and voltage monitor points.

  \warning It is assumed the complete list does not exceed the space limits of
           \a status.

  \param status Storage buffer for return status (should contain at least
                ControlService::maxLine characters).
  \param arg    Argument list: [INTERVAL] [COUNT].
*/
void Correlator::execQuery(return_type status, argument_type arg)
{
  static const char *usage =
  "[INTERVAL] [COUNT]\r\n"
  "  Query (or repeatedly sample) current monitor point values.\r\n"
  "  INTERVAL  Sampling interval in dsec (0 to disable; default: 0).\r\n"
  "  COUNT     Number of samples (0 repeat indefinitely; default: 0).\r\n";

  if (!arg.help) {
    int interval = 0, count = 0;
    FILE *client = fdopen(arg.fdWrite, "w");
    if (arg.str) { sscanf(arg.str, "%i%i", &interval, &count); }

    unsigned len;
    if (client || interval<=0 || count==1) {
      len = siprintf(status, "%s\r\n", statusOK);
    } else {
      len = siprintf(status, "%sfdopen() failed; cannot iterate.\r\n",
		     statusWARN);
    }
    status[len] = '\0';

    for (int iter=0; iter==0 || (interval>0 && iter!=count); ++iter) {
      // WARNING: for unknown reasons, locking this call can corrupt ADC
      // interrupt handling.
      setMonitorPoint(-1, true);

      unsigned lenName  = monPoints_.getMaxPoint(),
	       lenValue = monPoints_.getMaxValue();

      if (interval<=0 || 1+iter==count || !client ||
          zpec_interrupt(arg.fdRead)) {
        // Last iteration: return status, truncating contents if iterating.
	if (iter > 0) { len = 0; }
	periph_lock();
	  for (unsigned chan=monitorBegin(); chan<monitorEnd(); ++chan) {
	    const char *point = monitorPointName[chan],
		       *value = monPoints_.getValue(point),
		       *units = monPoints_.getUnit(point);
	    len += siprintf(status+len, " %*s = %*s %s\r\n",
			    lenName, point, lenValue, value, units);
	  }
	periph_unlock();
	break;  // Prevent iteration on failure conditions.
      } else {
	if (iter == 0) { fputs(status, client); }
	periph_lock();
	  for (unsigned chan=monitorBegin(); chan<monitorEnd(); ++chan) {
	    const char *point = monitorPointName[chan],
		       *value = monPoints_.getValue(point),
		       *units = monPoints_.getUnit(point);
	    fprintf(client, " %*s = %*s %s\r\n",
		    lenName, point, lenValue, value, units);
	  }
	periph_unlock();
	fprintf(client, "\r\n");
	fflush(client);
	OSTimeDly((interval*TICKS_PER_SECOND)/10);
      }
    }
  } else {
    longHelp(status, usage, &Correlator::execQuery);
  }
}


/**
  \brief Quits a control shell (returns end-of-file indicator).

  Returns end-of-file (EOF), causing a ControlService server to terminate the
  associated connection.

  \param status Storage buffer for return status (should contain at least
                ControlService::maxLine characters).
  \param arg    Argument list (unused).
*/
void Correlator::execQuit(return_type status, argument_type arg)
{
  static const char *usage =
  "\r\n"
  "  Exit control shell and close connection.\r\n";

  if (!arg.help) {
    strcpy(status, statusEOF);
  } else {
    longHelp(status, usage, &Correlator::execQuit);
  }
}


/**
  \brief Reboots the CPU.

  All connections and activity will terminate immediately.

  \param status Storage buffer for return status (should contain at least
                ControlService::maxLine characters).
  \param arg    Argument list (unused).
*/
void Correlator::execReboot(return_type status, argument_type arg)
{
  static const char *usage =
  "\r\n"
  "  Reboot the CPU (terminating all active connections).\r\n";

  if (!arg.help) {
#ifdef ARGUS_H  // execute only if Argus software is included
	if (lnaPwrState==1) argus_lnaPower(0);
#endif
	ForceReboot();
  } else {
    longHelp(status, usage, &Correlator::execReboot);
  }
}


/**
  \brief Sends (writes) plain text lag data to a file.

  Lag data for the most recent, completed integration will be output, in the
  following format:
  \verbatim
  # iBuffer = ..., nLags = ..., nBands = ..., nFrames = ..., Time = ...
  # Lag  Band0  [Band1...]
     0    123   [456...]
     1    789   [234...]
    ...   ...    ...
  \endverbatim

  \note
  The system can actively collect new lag data while serving data from a
  previous integration.

  \param status Storage buffer for return status (should contain at least
                ControlService::maxLine characters).
  \param arg    Argument list: [NLAGS] [BUFFER]
*/
void Correlator::execSend(return_type status, argument_type arg)
{
  static const char *usage =
  "[NLAGS] [BUFFER]\r\n"
  "  Send lag counts for each band from the most recent integration.\r\n"
  "  NLAGS  Number of lags to output, starting with 0 (default: all).\r\n"
  "  BUFFER Integration buffer to output (0-1; default: 0).\r\n";

  if (!arg.help) {
    unsigned nSend = nLags(), iBuffer = 0;
    if (arg.str) { sscanf(arg.str, "%u%u", &nSend, &iBuffer); }
    nSend    = std::min(nSend, nLags());
    iBuffer &= (nBuffers() - 1);

    lock();
      unsigned maxMsg = ControlService::maxLine - 20,
	       nMsg = siprintf(status,
	         "%siBuffer = %u; nLags = %u; nBands = %u; nFrames = %u;"
		 " Time = %u.%02u\r\n %sLag",
	         statusOK, iBuffer, nLags(), nBands(),
		 band_[0].lags.getFrames(iBuffer),
		 band_[0].lags.getTime(iBuffer)/TICKS_PER_SECOND,
		(band_[0].lags.getTime(iBuffer)%TICKS_PER_SECOND)*
		(100/TICKS_PER_SECOND),
		 statusOK);
      for (unsigned iBand=0; iBand<nBands(); ++iBand) {
	nMsg += siprintf(status+nMsg, "%7sBand%u", "", iBand);
      }
      nMsg += siprintf(status+nMsg, "\r\n");

      for (unsigned iLag=0; iLag<nSend; ++iLag) {
	nMsg += siprintf(status+nMsg, "%6u", iLag);
	for (unsigned iBand=0; iBand<nBands(); ++iBand) {
	  zpec_write_if_full(arg.fdWrite, status, &nMsg, maxMsg, "execSend");
	  nMsg += siprintf(status+nMsg, "%12ld",
	                   band_[iBand].lags[nLags()*iBuffer+iLag]);
	}
	nMsg += siprintf(status+nMsg, "\r\n");
      }
    unlock();
  } else {
    longHelp(status, usage, &Correlator::execSend);
  }
}


/**
  \brief Returns summary of correlator status.

  Determines the status of various correlator settings, such as start-up
  errors, integration activity, operating mode, etc. If anomalies are
  detected, an error status is returned.

  \param status Storage buffer for return status (should contain at least
                ControlService::maxLine characters).
  \param arg    Argument list (unused).
*/
void Correlator::execStatus(return_type status, argument_type arg)
{
  static const char *usage =
  "\r\n"
  "  Display current correlator status.\r\n";

  if (!arg.help) {
    unsigned len = 0;
    int serialNo;
    flash_t flashData;

    zpec_readFlash(&flashData);
    serialNo = (flashData.valid ? flashData.serialNo : -1);

    len += siprintf(status+len,
	      "%s\r\n"
	      "  Hardware variant:      %d (%s)\r\n"
	      "  CPLD code version:     0x%X\r\n"
	      "  Serial number:         %d\r\n"
	      "  System uptime:         %u seconds\r\n\r\n"
	      ,
	      (cpld[8] != cpld_cache[0]) || (cpld[9] != cpld_cache[1]) ||
		error_.initADC ?  statusERR : statusOK,
	      hw_, hw_ == ZPEC_HW_GBT ? "GBT" :
		   hw_ == ZPEC_HW_RLT ? "RLT" :
		   hw_ == ZPEC_HW_POW ? "POW" :
		   hw_ == ZPEC_HW_ARG ? "ARG" :
		   "UNKNOWN",
	      cpld[0] >> 12,
	      serialNo,
	      ::Secs
	    );

    switch (hw_) {
      case ZPEC_HW_GBT:
      case ZPEC_HW_RLT:
      {
	zpec_mode_t mode = getMode();
	len += siprintf(status+len,
		 "  ADC hardware:          %u bands, %lu lags each\r\n"
		 "  ADC initialization:    %d ADCs failed\r\n"
		 "  Attenuator setting:    %hu\r\n"
		 "  Integration status:    %s (err_first=%u, err_frame=%u)\r\n"
		 "  Jumper settings:       S1:%s S2:%s S4:%s\r\n"
		 "  Master/slave setting:  %s\r\n"
		 "  Operating mode:        %d: %s\r\n"
		 ,
		 nBands(), nLags(),
		 error_.initADC,
		 band_[0].control.getAttenuation(),
		 isAccumulating() ? "collecting data" : "idle",
		 io_->err_first, io_->err_frame,
		 cpld_get_bit(CPLD_JUMPER0) ? "open" : "closed",
		   cpld_get_bit(CPLD_JUMPER1) ? "open" : "closed",
		   cpld_get_bit(CPLD_JUMPER2) ? "open" : "closed",
		 isMaster() ? "master" : "slave",
		 static_cast<int>(mode), zpec_mode_name(mode)
	       );
      }
	break;
      case ZPEC_HW_POW:
	len += siprintf(status+len,
		  "  Power supply state:    %s\r\n"
		  "  Manual On/Off switch:  %s\r\n"
		  "  Manual On  contact:    %s\r\n"
		  "  Manual Off contact:    %s\r\n"
		  ,
		  cpld_get_bit(CPLD_POW_STATE) ? "on" : "off",
		  cpld_get_bit(CPLD_POW_OVERRIDE) ? "disabled" : "enabled",
		  cpld_get_bit(CPLD_POW_SW_ON) ? "closed" : "open",
		  cpld_get_bit(CPLD_POW_SW_OFF) ? "closed" : "open"
                );
        break;
      default:
        break;
    }

    for (int reg=0; reg<2; reg++) {
      len += siprintf(status+len,
	      "  CPLD ctrl register %d:  0x%04X  (cached value: 0x%04X)\r\n",
	      reg, cpld[8+reg], cpld_cache[reg]);
    }
  } else {
    longHelp(status, usage, &Correlator::execStatus);
  }
}


/**
  \brief Synchronizes ADC clocks with the master.

  Instructs the CPLD to synchronize its logic with signals provided by
  the external master.

  \param status Storage buffer for return status (should contain at least
                ControlService::maxLine characters).
  \param arg    Argument list (unused).
*/
void Correlator::execSync(return_type status, argument_type arg)
{
  static const char *usage =
  "\r\n"
  "  Synchronize ADC logic with external reference (master).\r\n";

  if (!arg.help) {
    if (isMaster()) {
      siprintf(status, "%sCannot synchronize in master mode.\r\n", statusWARN);
    } else {
      cpld_set_bit(CPLD_SYNC_ENABLE, 1);
      zpec_usleep(25000);
      cpld_set_bit(CPLD_SYNC_ENABLE, 0);
      siprintf(status, "%sSynchronization enabled for 25 ms.\r\n", statusOK);
    }
  } else {
    longHelp(status, usage, &Correlator::execSync);
  }
}


/**
  \brief Get or set the current time.

  Times are in seconds since the Epoch (00:00:00 UTC, Jan 1, 1970).
  The input time is rounded to the nearest tick, with a slight upward bias
  to offset network latency.

  \param status Storage buffer for return status (should contain at least
                ControlService::maxLine characters).
  \param arg    Argument list: [TIME]
*/
void Correlator::execTime(return_type status, argument_type arg)
{
  static const char *usage =
  "[TIME]\r\n"
  "  Get or set the current time.\r\n"
  "  TIME  Current time (seconds since 00:00:00 UTC, Jan 1, 1970).\r\n";

  if (!arg.help) {
    if (arg.str) {
      bootTicks_ = (long long )(0.6+atof(arg.str)*TICKS_PER_SECOND) -::TimeTick;
    }

    unsigned long long now = bootTicks_ + ::TimeTick;
    siprintf(status, "%sCurrent time is %u.%02u.\r\n",
		     statusOK,
		     (unsigned )(now/TICKS_PER_SECOND),
		     (unsigned )(now%TICKS_PER_SECOND)*(100/TICKS_PER_SECOND));
  } else {
    longHelp(status, usage, &Correlator::execTime);
  }
}


/**
  \brief Returns version string.

  \param status Storage buffer for return status (should contain at least
                ControlService::maxLine characters).
  \param arg    Argument list (unused).
*/
void Correlator::execVersion(return_type status, argument_type arg)
{
  static const char *usage =
  "\r\n"
  "  Print version string.\r\n";

  if (!arg.help) {
    siprintf(status, "%sZpectrometer %s [CPLD v. %hX].\r\n",
             statusOK, zpec_version(), cpld[0]>>12);
  } else {
    longHelp(status, usage, &Correlator::execVersion);
  }
}


/**
  \brief Control command help facility.

  This method implements control command help. Invoked without arguments,
  a list summarizing the syntax of all recognized commands is returned.
  Optionally, detailed help on a specific command can be obtained by supplying
  the command name (any alias) as an argument.

  \param status Storage buffer for return status (should contain at least
                ControlService::maxLine characters).
  \param arg    Argument list: [COMMAND]
*/
void Correlator::execHelp(return_type status, argument_type arg)
{
  static const char *fn = "execHelp";
  static const char *usage =
  "[COMMAND]\r\n"
  "  Summarize syntax for all commands, or provide detailed help on one.\r\n"
  "  COMMAND  Command name (case-insensitive) for extended help display.\r\n";

  if (!arg.help) {
    if (arg.str) {
      if (shell_) {
	char key[32], *p;

	// Remove trailing whitespace from argument.
	for (p = arg.str; *p !='\0' && !isspace(*p); ++p) { ; }
	*p = '\0';

	strncpy(key, arg.str, sizeof(key)-1);
	key[sizeof(key)-1] = '\0';

	// Key is a case-insensitive command name, convert to lower case.
	for (char *p = key; *p != '\0'; ++p) { *p = tolower(*p); }

	Correlator::shell_type::const_iterator iCmd = shell_->find(key);
	if (iCmd != shell_->end() &&
	    (iCmd->second != &Correlator::execHelp || *arg.str != '\0')) {
	  // Return detailed help on the command.
	  (this->*(iCmd->second))(status, 0);
	} else {
	  // Command unknown: return summary list.
	  zpec_write_strcpy(arg.fdWrite, status, helpSummary_.c_str(),
	                    ControlService::maxLine, fn);
	}
      } else {
	zpec_warn_fn("help summary not initialized.");
	siprintf(status, "%sSorry, help has not been initialized.\r\n",
	         statusERR);
      }
    } else {
      // No argument supplied: return summary list.
      zpec_write_strcpy(arg.fdWrite, status, helpSummary_.c_str(),
			ControlService::maxLine, fn);
    }
  } else {
    longHelp(status, usage, &Correlator::execHelp);
  }
}


/**
  \brief Get or set log message verbosity level.

  Possbile log message priorities are as follows:
  - 1 Error
  - 2 Warning
  - 3 Info
  - 4 Debug

  Setting the verbosity level to \p L will cause zpec_log() to discard
  messages whose priority is greater (lower) than \p L.

  \param status Storage buffer for return status (should contain at least
                ControlService::maxLine characters).
  \param arg    Argument list: LEVEL
*/
void Correlator::execVerbose(return_type status, argument_type arg)
{
  static const char *usage =
  "[LEVEL]\r\n"
  "  Get or set log message verbosity level (1-4, higher is more verbose).\r\n"
  "  [LEVEL]  Logging level (0 -> none, 4 -> all; default: 3).\r\n";

  if (!arg.help) {
    if (arg.str) {
      int v0 = setVerbosity(atoi(arg.str));
      siprintf(status, "%sNew log message verbosity is %d (was %d).\r\n",
		       statusOK, getVerbosity(), v0);
    } else {
      siprintf(status, "%sCurrent log message verbosity is %d.\r\n",
		       statusOK, getVerbosity());
    }
  } else {
    longHelp(status, usage, &Correlator::execVerbose);
  }
}


/**
  \brief Generates extended help string for an executable command.

  This method finds the alias list for the given command, prepends it to
  its extended help message, then copies it to the supplied buffer.

  \param status Help message storage space.
  \param help   Raw extended help fragment for \a member.
  \param member Shell-executable member function for which to generate help.
*/
void Correlator::longHelp(return_type status, const char *help,
  void (Correlator::*member)(return_type, argument_type)) const
{
  alias_container_type::const_iterator iAlias;

  for (iAlias = aliasMap_.begin();
       iAlias != aliasMap_.end() && iAlias->second != member; ++iAlias) ;

  strcat(strcat(strcat(strcpy(status, statusOK),
                iAlias != aliasMap_.end() ? iAlias->first.c_str()
		                          : "{UNKNOWN}"), " "), help);
}


/**
  \brief Creates the summary list of control commands.

  This method processes the detailed help messages returned by each command in
  the CorrelatorShell command dictionary, extracts the first line (which
  should summarize the arguments it accepts), and, for commands with multiple
  aliases, combines synonymous entries into a single listing (one per line).
  The format of the help string
  The format of the final output is:
  \verbatim
  alias1, alias2, ... aliasN ARGUMENTS
  \endverbatim

  This method must be called after the command dictionary (CorrelatorShell)
  has been initialized, and any time its contents are modified thereafter,
  for the summary command usage information returned by execHelp() to be
  accurate.

  \warning This method is not thread-safe.
*/
void Correlator::createHelpSummary(Correlator::shell_type *shell)
{
  // Attach current shell.
  shell_ = shell;

  // Create alias map.
  aliasMap_.clear();
  for (Correlator::shell_type::const_iterator iCmd = shell->begin();
       iCmd != shell->end(); ++iCmd) {
    const char *p;
    for (p = iCmd->first.c_str(); isgraph(*p); ++p) ;

    // Add only printable command names to alias list.
    if (iCmd->first.size() > 0 && *p == '\0') {
      alias_container_type::iterator iAlias;

      for (iAlias  = aliasMap_.begin();
	   iAlias != aliasMap_.end() && iAlias->second != iCmd->second;
	   iAlias++) ;

      if (iAlias == aliasMap_.end()) {
	// Add new command.
	aliasMap_[iCmd->first] = iCmd->second;
      } else {
	// Extend alias list.
	std::string aliases = iAlias->first;
	aliases += ", ";
	aliases += iCmd->first;

	// Replace old aliases with new.
	aliasMap_[aliases] = iAlias->second;
	aliasMap_.erase(iAlias);
      }
    }
  }

  // Form help string.
  char helpstr[ControlService::maxLine];
  //
  helpSummary_.clear();
  for (alias_container_type::const_iterator iAlias = aliasMap_.begin();
       iAlias != aliasMap_.end(); ++iAlias) {

    // Get help string and truncate to first line.
    char *newline;
    (this->*(iAlias->second))(helpstr, 0);
    newline = strchr(helpstr, '\n');
    if (newline) { *newline = '\0'; }

    // Enforce return status convention: no '#' at start of line 2 onward.
    if (iAlias !=  aliasMap_.begin()) { helpstr[0] = '|'; }

    // Append a line of summary help.
    helpSummary_ += helpstr;
    helpSummary_ += "\n";
  }

  // Append detailed help on help command itself.
  execHelp(helpstr, 0);
  helpstr[0] = ' ';
  helpSummary_ += "\r\n";
  helpSummary_ += helpstr;
}


/**
  Zpectrometer command parser.
  This method implements the Zpectrometer command parser. Input consists
  of whitespace-separated tokens, the first being the command name and the
  remainder (if any) representing command-specific options. Command names are
  case-insensitive.

  \note The input command line will be modified during parsing.
*/
template <>
void Correlator::shell_type::parse(Correlator::key_type& key,
    Correlator::argument_type& arg, Correlator::command_type& cmd) const
{
  const char *delim = " \t\r\n";
  char *lastToken, *c_key;

  c_key = strtok_r(cmd.str, delim, &lastToken);
  if (c_key) {
    // Command names are case insensitive: convert to lower case.
    for (char *p = c_key; *p != '\0'; ++p) { *p = tolower(*p); }
    key = c_key;
  } else {
    key = "";
  }

  arg.help    = false;
  arg.fdRead  = cmd.fdRead;
  arg.fdWrite = cmd.fdWrite;
  arg.str     = strchr(cmd.str, '\0') + 1;

  // Skip leading whitespace; if no arguments present, set to NULL.
  while (isspace(*arg.str)) { ++arg.str; }
  if (*arg.str == '\0') { arg.str = 0; }
}


/**
  Command shell error parser.
  This method implements Zpectrometer command shell error reporting. It is
  called whenever exec() is passed a command not found in the command
  dictionary.
*/
template <>
void Correlator::shell_type::cmdNotFound(Correlator::return_type rtn,
    Correlator::key_type& key, Correlator::argument_type& arg,
    Correlator::command_type& cmd) const
{
  for (const char *ckey = key.c_str(); *ckey; ++ckey) {
    zpec_debug("shell input: 0x%02x\r\n", *ckey + 256*(*ckey < 0));
  }

  siprintf(rtn, "! Unrecognized command '%s'. To see the list, type: help\r\n",
           key.c_str());
}
