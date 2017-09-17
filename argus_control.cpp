/**
  \file
  \author Andy Harris
  \brief Argus control command infrastructure.

  $Id: argus_control.cpp,v 1.2 2014/06/04 18:35:21 harris Exp $
*/

#include "argus.h"
#include "control.h"
#include "math.h"

// names for cryostat test points
//char *cnames[] = {"T0:", "T1:", "T2:", "T3:", "T4:", "T5:", "Pressure:"};
char *cnames[] = {"20K cold head: ",
		          "NC:            ",
		          "20K plate:     ",
		          "70 K plate:    ",
		          "70 K cold head:",
		          "Card cage:     ",
		          "Pressure:      "};

// decimal points for display in exexArgusMonPts
int d1 = 1, d2 = 2;

/********************************************************************************/
/**
  \brief Argus test command.

  This method is a template for testing control command infrastructure.

  \param status Storage buffer for return status (should contain at least
                ControlService::maxLine characters).
  \param arg    Argument list: LEVEL
*/
void Correlator::execArgusTest(return_type status, argument_type arg)
{
  static const char *usage =
  "ARG1 [ARG2]\r\n"
  "  Do something with arguments ARG1 and (optional) ARG2.\r\n"
  "  ARG1  The first  argument (integer).\r\n"
  "  ARG2  The second argument (float; default: 1.0).\r\n";

  if (!arg.help) {
    int   arg1 = 0;
    float arg2 = 1.0;
    if (arg.str) {
      // Command called with one or more arguments.

      int narg = sscanf(arg.str, "%d%f", &arg1, &arg2);
      if (narg == 0) {
        // Too few arguments; return help string.
        longHelp(status, usage, &Correlator::execArgusTest);
      } else {
        // Execute the command.
        int rtn = argus_test(arg1, arg2);
        sprintf(status, "%sargus_test(%d, %g) returned status %d.\r\n",
                         (rtn==0 ? statusOK : statusERR), arg1, arg2, rtn);
      }
    } else {
      // Command called without arguments. In this example, that's an error.
      longHelp(status, usage, &Correlator::execArgusTest);
    }
  } else {
    longHelp(status, usage, &Correlator::execArgusTest);
  }
}

/**
  \brief Argus freeze command.

  This method sets a bit to freeze the system state, generally meant for during integrations.

  \param status Storage buffer for return status (should contain at least
                ControlService::maxLine characters).
  \param arg    Argument list: LEVEL
*/
void Correlator::execArgusFreeze(return_type status, argument_type arg)
{
  static const char *usage =
  "\r\n"
  "  Freeze system state (see thaw) to prevent changes to settings.\r\n";


  if (!arg.help && !arg.str) {
	  freezeSys = 1;
	  freezeCtr += 1;
	  sprintf(status, "%sfreezeSys = %u\r\n", statusOK, freezeSys);
  } else {
	    longHelp(status, usage, &Correlator::execArgusFreeze);
  }
}

/**
  \brief Argus thaw command.

  This method clears a bit to unfreeze the system state, generally meant for not during integrations.

  \param status Storage buffer for return status (should contain at least
                ControlService::maxLine characters).
  \param arg    Argument list: LEVEL
*/
void Correlator::execArgusThaw(return_type status, argument_type arg)
{
  static const char *usage =
  "\r\n"
  "  Thaw system state (see freeze) to permit changes to settings.\r\n";

  if (!arg.help && !arg.str) {
	  freezeSys = 0;
	  thawCtr += 1;
	  sprintf(status, "%sfreezeSys = %u\r\n", statusOK, freezeSys);
  } else {
    	longHelp(status, usage, &Correlator::execArgusThaw);
  }
}

/**
  \brief Argus instrument health.

  This method produces an instrument health readout.
  Values returned on the first line should all be zeros indicating nominal operation.  These check
  words are in order of power systems, output IF power, thermal system, then LNA bias cards.
  Information on succeeding lines is in a form easier for humans to decode.

  \param status Storage buffer for return status (should contain at least
                ControlService::maxLine characters).
  \param arg    Argument list: LEVEL
*/
void Correlator::execArgusRxHealth(return_type status, argument_type arg)
{
  static const char *usage =
  "\r\n"
  "  Get instrument health error summary, results to screen.\r\n"
  "  Zero value words indicate no errors.\r\n";

  if (!arg.help) {
	  OSTimeDly(CMDDELAY);
	  argus_readAllSystemADCs();
	  int rtnPow = argus_powCheck();
	  int rtnIF = argus_ifCheck();
	  int rtnTherm = argus_thermCheck();
	  int rtnRx = argus_biasCheck();
	  sprintf(status, "%s %d %d %d %d\r\n"
			  "Power errors 0x%04x\r\n"
			  "IF output power errors 0x%04x\r\n"
			  "Thermal errors 0x%04x\r\n"
			  "LNA and mixer bias errors 0x%04x\r\n"
			  "Individual receiver bias errors:\r\n"
			  "0x%04x 0x%04x 0x%04x 0x%04x\r\n"
			  "0x%04x 0x%04x 0x%04x 0x%04x\r\n"
			  "0x%04x 0x%04x 0x%04x 0x%04x\r\n"
			  "0x%04x 0x%04x 0x%04x 0x%04x\r\n\r\n",
              (!freezeSys ? statusOK : statusERR), rtnPow, rtnIF, rtnTherm,
              rtnRx, rtnPow, rtnIF, rtnTherm, rtnRx,
              biasSatus[0], biasSatus[1], biasSatus[2],
              biasSatus[3], biasSatus[4], biasSatus[5], biasSatus[6], biasSatus[7], biasSatus[8],
              biasSatus[9], biasSatus[10], biasSatus[11], biasSatus[12], biasSatus[13], biasSatus[14],
              biasSatus[15]);
	  /*
	   * also provide check words for power supplies, yig, cal blade in obs(?).
	   * bias board power not needed because failure will show in gate, drain etc bias.
	  */
  } else {
    longHelp(status, usage, &Correlator::execArgusRxHealth);
  }
}

/**
  \brief Argus LO YIG filter control.

  This method covers the YIG filter tuning for cleaning up the Argus LO.

  \param status Storage buffer for return status (should contain at least
                ControlService::maxLine characters).
  \param arg    Argument list: LEVEL
*/
void Correlator::execArgusYIG(return_type status, argument_type arg)
{
  static const char *usage =
  "[KEWORD [VALUE]]\r\n"
  "  KEYWORD  A keyword, either FREQ, PEAK, or SET.\r\n"
  "  VALUE    A value: for FREQ, the target frequency in GHz\r\n"
  "                    for PEAK, optional step size in integer (10 typ)\r\n"
  "                    for SET, an integer 0-65535.\r\n";

  if (!arg.help) {
    if (arg.str) {
    	// Command called with one or no arguments.
    	char kw[15] = {0};
    	float val;

    	int narg = sscanf(arg.str, "%s%f", kw, &val);
        if (narg == 2) {
          // Execute the command.
        	if (!strcasecmp(kw, "set")) {
        		OSTimeDly(CMDDELAY);
        		int rtn = argus_openSubbusC(MUBOX_I2CADDR);
        		if (rtn) {
        			sprintf(status, "%sI2C bus error, status %d\r\n", statusERR, rtn);
        		} else {
        			rtn = yig_set((unsigned int)val);
        			rtn += argus_closeSubbusC();
        			sprintf(status, "%syig_set to %u, status %d\r\n", (rtn==0 ? statusOK : statusERR),
        					muBoxPar.setval, rtn);
        		}
        	}
        	else if (!strcasecmp(kw, "freq")) {
        		OSTimeDly(CMDDELAY);
        		int rtn = yig_freq(val, YIGSEARCHSTEP, YIGMAXN, YIGPTHRESH);
        		// Do a fine sweep once a tone is found
        		if (rtn == 0) rtn = yig_peak(muBoxPar.setval, YIGTUNESTEP, YIGMAXN, YIGPTHRESH);
        		sprintf(status, "%syig_freq for %6.4f GHz, det. voltage %5.3f, tune word %u, status %d\r\n",
        				(rtn==0 ? statusOK : statusERR),
        				val, muBoxPar.adcv[0], muBoxPar.setval, rtn);
        	}
        	else if (!strcasecmp(kw, "peak")) {
        		OSTimeDly(CMDDELAY);
        		int rtn = yig_peak(muBoxPar.setval, (unsigned short int)val, YIGMAXN, YIGPTHRESH);
        		sprintf(status, "%syig_peak for steps of %u; final det. voltage %5.3f, tune word %u, status %d\r\n",
        				(rtn==0 ? statusOK : statusERR),
        				(unsigned short int)val, muBoxPar.adcv[0], muBoxPar.setval, rtn);
        	} else {
        		longHelp(status, usage, &Correlator::execArgusYIG);
        	}
        } else if (narg==1 && !strcasecmp(kw, "peak")) {  // peak with default argument stepval
        	OSTimeDly(CMDDELAY);
        	int rtn = yig_peak(muBoxPar.setval, YIGTUNESTEP, YIGMAXN, YIGPTHRESH);
        	sprintf(status, "%syig_peak for steps of %u; final det. voltage %5.3f, tune word %u, status %d\r\n",
        			(rtn==0 ? statusOK : statusERR),
        			YIGTUNESTEP, muBoxPar.adcv[0], muBoxPar.setval, rtn);
      	} else {
        // Wrong number of arguments; return help string.
      		longHelp(status, usage, &Correlator::execArgusYIG);
      	}
    } else {
      // Command called without arguments; either display or make as an error.
    	OSTimeDly(CMDDELAY);
    	int rtn = read_all_muBox_ADC();  // update values in muBoxPar structure
    	float fextrapol = (muBoxPar.setval-muBoxPar.intercept)/muBoxPar.slope;
    	sprintf(status, "%sYIG values:\r\n"
    			"LO power detector voltage:  %8.3f V (max voltage: %5.3f, threshold %5.3f)\r\n"
    			"Mii heater current readout: %8.3f V\r\n"
    			"YIG temperature readout:    %8.1f C\r\n"
    			"Interface card voltage:     %8.3f V (15 V nominal)\r\n"
    			"YIG set word %u\r\n"
    			"Command freq %6.3f, estimated freq %6.3f GHz, delta %6.3f\r\n"
    			"Status %d\r\n",
    			( (rtn==0  &&  muBoxPar.adcv[YIGPWRCHAN] > YIGPTHRESH ) ? statusOK : statusERR),  //check pwr ok
    			muBoxPar.adcv[0], muBoxPar.pdetVmax, YIGPTHRESH,
    			muBoxPar.adcv[1], muBoxPar.adcv[2]*100, muBoxPar.adcv[3],
    			muBoxPar.setval, muBoxPar.freq, fextrapol, fextrapol-muBoxPar.freq, rtn);
    }
  } else {
    longHelp(status, usage, &Correlator::execArgusYIG);
  }
}

/**
  \brief Argus vane control.

  This method covers the vane and cal sys control.

  \param status Storage buffer for return status (should contain at least
                ControlService::maxLine characters).
  \param arg    Argument list: LEVEL
*/
void Correlator::execArgusVane(return_type status, argument_type arg)
{
  static const char *usage =
  "[KEYWORD] \r\n"
  "  KEYWORD  A keyword, either CAL, OBS, STOP, or STATS.\r\n";

  int rtn = 0;
  if (!arg.help) {
    if (arg.str) {
      // Command called with one or more arguments.
    	char kw[15] = {0};
        int narg = sscanf(arg.str, "%s", kw);
        if (narg == 1) {
          // Execute the command.
        	int rtn = 0;
         	if (!strcasecmp(kw, "obs")) {
        		// uses vmax from either cal or stats
         		OSTimeDly(CMDDELAY);
          		rtn = argus_openSubbusC(VANE_I2CADDR);
        		if (rtn) {
        			sprintf(status, "%sI2C bus error, status %d\r\n", statusERR, rtn);
        		} else {
        			// first find max, min angles
        			rtn += argus_driveVane(VANERUN, VANENMAX, 0., VANEDELTA);
        			// then drive to obs position
        			rtn += argus_driveVane(VANERUN, VANENMAX, calSysPar.maxAngle, VANEDELTA);
        			rtn += readCalSysADC(VANEANGLEADC);
        			rtn += argus_closeSubbusC();
        			sprintf(status, "%sVane to obs position: %s, angle = %.2f (status %d).\r\n",
        				(rtn==0 ? statusOK : statusERR), calSysPar.state, calSysPar.maxAngle, rtn);
        			if (!strcmp(calSysPar.state, "found")) calSysPar.state = "obs";
        		}
         	}
        	else if (!strcasecmp(kw, "cal")) {
        		rtn = argus_openSubbusC(VANE_I2CADDR);
        		if (rtn) {
        			sprintf(status, "%sI2C bus error, status %d\r\n", statusERR, rtn);
        		} else {
        			// first find max, min angles
        			rtn += argus_driveVane(VANERUN, VANENMAX, 0., VANEDELTA);
        			// then drive to cal position
        			// multiply by fudge factor to get vane centered over aperture
        			// changed from 1.15 to 1.07 17.01.25 AH
        			rtn += argus_driveVane(VANERUN, VANENMAX, calSysPar.minAngle*1.07, VANEDELTA);
        			rtn += readCalSysADC(VANEANGLEADC);
        			rtn += argus_closeSubbusC();
        			sprintf(status, "%sVane to cal position: %s, angle = %.2f (status %d).\r\n",
        				(rtn==0 ? statusOK : statusERR), calSysPar.state, calSysPar.minAngle, rtn);
        			if (!strcmp(calSysPar.state, "found")) calSysPar.state = "cal";
        		}
        	}
        	else if (!strcasecmp(kw, "stop")) {
        		rtn = argus_openSubbusC(VANE_I2CADDR);
        		if (rtn) {
        			sprintf(status, "%sI2C bus error, status %d\r\n", statusERR, rtn);
        		} else {
        			rtn += argus_setVaneBits(VANESTOP);
        			rtn += argus_closeSubbusC();
        			sprintf(status, "%sVane stop: stop, angle = %.2f (status %d).\r\n",
         				(rtn==0 ? statusOK : statusERR), calSysPar.adcv[VANEANGLEADC], rtn);
        		}
        	}
        	else if (!strcasecmp(kw, "stats")) {
        		rtn = argus_openSubbusC(VANE_I2CADDR);
        		if (rtn) {
        			sprintf(status, "%sI2C bus error, status %d\r\n", statusERR, rtn);
        		} else {
        			rtn += argus_driveVane(VANERUN, VANENMAX, 0., VANEDELTA);
        			rtn += argus_closeSubbusC();
        			sprintf(status, "%sVane stats: %s (status %d).\r\n",
         				(rtn==0 ? statusOK : statusERR), calSysPar.state, rtn);
        		}
        	}
        	else {
        		longHelp(status, usage, &Correlator::execArgusVane);
        	}
      	} else {
        // Wrong number of arguments; return help string.
      		longHelp(status, usage, &Correlator::execArgusVane);
      	}
    } else {
    	// first update angle, current, temp:
    	rtn = argus_readAllCalSysADC();
    	sprintf(status, "%sVane parameters:\r\n"
    			"Temperature: %.1f [C] \r\nState: %s \r\n"
    			"Angle: %.3f; min: %.3f, max: %.3f [V] \r\n"
    			"Curr: %.2f; mean: %.2f, max: %.2f, stdev: %.2f [A] \r\n"
    			"Supply to interface: %.2f (%.2f extrapolated)\r\n",
    			( (rtn==0 && calSysPar.adcv[0] < 10.) ? statusOK : statusERR),  // check for ADC 99s
     		   calSysPar.adcv[2], calSysPar.state,
    		   calSysPar.adcv[0], calSysPar.minAngle, calSysPar.maxAngle,
    		   calSysPar.adcv[1], calSysPar.meanCurr, calSysPar.maxCurr, sqrt(calSysPar.varCurr),
    		   calSysPar.adcv[3], calSysPar.adcv[3]+VANEVINOFFS);
   }
  } else {
    longHelp(status, usage, &Correlator::execArgusVane);
  }
}

/**
  \brief Set engineering options.

  Set engineering options such as power supply limit bypass for Argus.

  \param status Storage buffer for return status (should contain at least
                ControlService::maxLine characters).
  \param arg    Argument list: LEVEL
*/
void Correlator::execArgusEngr(return_type status, argument_type arg)
{
  static const char *usage =
  "[KEYWORD [VALUE]]\r\n"
  "  Set engineering mode functions.\r\n"
  "    KEYWORD         VALUE:\r\n"
  "    bypassLNApsLim  x   magic number x to bypass LNA power supply limits.\r\n"
  "    bypassCIFpsLim  x   magic number x bypass cold IF power supply limits.\r\n"
  "    bypassLNAlims   y   magic number y to bypass soft limits on LNA biases.\r\n"
  "    stopVaneOnStall x   0 to ignore vane auto-stop when vane stalled.\r\n"
  "    sendVane        z   send vane drive hardware integer z.\r\n"
  "    dec             n   n decimal places for MON LNA, MON MIX, MON SETS display\r\n"
  "    clearBus            clear I2C bus busy bit, open main bus switches.\r\n"
  "    clrCtr              clear counters for I2C bus and freeze/thaw.\r\n"
		  ;

  if (!arg.help) {
    if (arg.str) {
      // Command called with one or more arguments.
   	  char kw[15] = {0};
   	  int val;
      int narg = sscanf(arg.str, "%s%d", kw, &val);

      if (narg == 2) {
        // Execute the command.
      	if (!strcasecmp(kw, "bypassLNApsLim")) lnaPSlimitsBypass = (val == 37 ? 1 : 0);
      	else if (!strcasecmp(kw, "bypassCIFpsLim")) cifPSlimitsBypass = (val == 37 ? 1 : 0);
      	else if (!strcasecmp(kw, "bypassLNAlims"))  lnaLimitsBypass = (val == 74 ? 1 : 0);
      	else if (!strcasecmp(kw, "stopVaneOnStall"))  lnaLimitsBypass = (val == 0 ? 0 : 1);
      	else if (!strcasecmp(kw, "sendVane")) {
      		OSTimeDly(CMDDELAY);
      		if (!argus_openSubbusC(VANE_I2CADDR)) {
      			argus_setVaneBits((BYTE)(val-200));
      			argus_closeSubbusC();
      		}
      	}
      	else if (!strcasecmp(kw, "dec")) {
    		if (val > 2) {
    			d1 = d2 = val;
    		} else {
    			d1 = 1;
    			d2 = 2;
    		}
      	}
    	else longHelp(status, usage, &Correlator::execArgusEngr);
      	sprintf(status,"\r");
      	}
      else if (narg == 1) {
    	  if (!strcasecmp(kw, "clearBus")) {
    		  OSTimeDly(CMDDELAY);
    		  int rtn = argus_clearBus();
    		  sprintf(status, "%sclearBus found status %d, SDA/SCL before and after 0x%x, 0x%x.\r\n",
    				  (rtn==0 ? statusOK : statusERR), rtn, i2cState[0], i2cState[1]);
    	  }
    	  else if (!strcasecmp(kw, "clrCtr")) {
        	  busLockCtr = 0;
        	  busNoLockCtr = 0;
        	  freezeCtr = 0;
        	  thawCtr = 0;
        	  freezeErrCtr = 0;
              sprintf(status,"\r");
          }
    	  else longHelp(status, usage, &Correlator::execArgusEngr);
      }
      else {
      		longHelp(status, usage, &Correlator::execArgusEngr);
      	}
    } else {
    	OSTimeDly(CMDDELAY);
    	sprintf(status, "%sEngineering:\r\n"
    		"  i2cBusBusy = %d, freeze = %u\r\n"
            "  successful and unsuccessful I2C bus lock requests since clrCtr = %u and %u\r\n"
            "  freeze and thaw requests since clrCtr = %u and %u, denials while frozen = %u\r\n"
    		"  bypassLNApsLim = %d\r\n"
    		"  bypassCIFpsLim = %d\r\n"
       		"  bypassLNAlims = %d\r\n"
       		"  stopVaneOnStall = %d\r\n"
    		"  decimal points: %d, %d\r\n"
       		"  power control PIO byte = 0x%02x\r\n"
    		"  version %s\r\n",
    		statusOK, i2cBusBusy, freezeSys,
    		busLockCtr, busNoLockCtr, freezeCtr, thawCtr, freezeErrCtr,
    		lnaPSlimitsBypass, cifPSlimitsBypass, lnaLimitsBypass,
    		stopVaneOnStall, d1, d2, argus_lnaPowerPIO(), VER);
    }
  } else {
	  longHelp(status, usage, &Correlator::execArgusEngr);
  }
}

/**
  \brief Argus setting limits.

  Return values of bias setting limits when queried.  This keeps the GBT Manager
  synchronized with the firmware limits.

  \param status Storage buffer for return status (should contain at least
                ControlService::maxLine characters).
  \param arg    Argument list: LEVEL
*/
void Correlator::execArgusLimits(return_type status, argument_type arg)
{
  static const char *usage =
  "\r\n"
  "  Return bias setting limits in order:\r\n"
  "  VDGMAX, VGMIN, VGMAX, VDMIN, VDMAX, VMMIN, VMMAX [V], IDMIN, IDMAX, IMMIN, IMMAX [mA],\r\n"
  "  MAXATTEN [dB], YIGf_min, YIGf_max [GHz]\r\n"
		  ;

  if (!arg.help) {
	  sprintf(status, "%s %5.1f %5.1f %5.1f %5.1f %5.1f %5.1f %5.1f %5.1f %5.1f %5.1f %5.1f %d %7.3f %7.3f\r\n",
	      				statusOK, VDGMAX, VGMIN, VGMAX, VDMIN, VDMAX, VMMIN, VMMAX,
	      				IDMIN, IDMAX, IMMIN, IMMAX, MAXATTEN,
	      				YIGFMIN, YIGFMAX);
  } else {
    longHelp(status, usage, &Correlator::execArgusLimits);
  }
}

/**
  \brief Argus individual drain bias control.

  Set or read a single LNA drain bias for Argus.

  \param status Storage buffer for return status (should contain at least
                ControlService::maxLine characters).
  \param arg    Argument list: LEVEL
*/
void Correlator::execArgusDrain(return_type status, argument_type arg)
{
  static const char *usage =
  "[M N V]\r\n"
  "  Set an LNA drain voltage.\r\n"
  "  M is the Mth receiver to set.\r\n"
  "  N is the Nth stage within receiver to set.\r\n"
  "  V is the voltage in V to set.\r\n"
		  ;

  if (!arg.help) {
    int m, n;
    float v = 0.0;
    if (arg.str) {
      // Command called with one or more arguments.
      int narg = sscanf(arg.str, "%d%d%f", &m, &n, &v);
      if (narg < 3) {
        // Too few arguments; return help string.
        longHelp(status, usage, &Correlator::execArgusDrain);
      } else {
        // Execute the command.
    	if (m > 0 && m <= NRX && n > 0 && n <= NSTAGES){
    		// convert from user's 1-base to code's 0-base
    		OSTimeDly(CMDDELAY);
    		int rtn = argus_setLNAbias("d", m-1, n-1, v, 0);
    		if (rtn == -10) {
        		sprintf(status, "%sLNA cards are not powered, returned status %d.\r\n",
        				statusERR, rtn);
    		} else {
    			sprintf(status, "%sargus_setLNAbias(%d, %d, %f, 0) returned status %d.\r\n",
    				(rtn==0 ? statusOK : statusERR), m, n, v, rtn);
    		}
    	} else {
    		sprintf(status, "%sReceiver or stage number out of range\r\n", statusERR);
    	}
     }
  } else {
      // Command called without arguments; read drain values
      longHelp(status, usage, &Correlator::execArgusDrain);  //here dummy
     }
  } else {
    longHelp(status, usage, &Correlator::execArgusDrain);
  }
}

/**
  \brief Argus individual gate bias control.

  Set or read a single LNA gate bias for Argus.

  \param status Storage buffer for return status (should contain at least
                ControlService::maxLine characters).
  \param arg    Argument list: LEVEL
*/
void Correlator::execArgusGate(return_type status, argument_type arg)
{
  static const char *usage =
  "[M N V]\r\n"
  "  Set an LNA gate voltage.\r\n"
  "  M is the Mth receiver to set.\r\n"
  "  N is the Nth stage within receiver to set.\r\n"
  "  V is the voltage in V to set.\r\n"
		  ;

  if (!arg.help) {
    int m, n;
    float v = 0.0;
    if (arg.str) {
      // Command called with one or more arguments.
      int narg = sscanf(arg.str, "%d%d%f", &m, &n, &v);
      if (narg < 3) {
        // Too few arguments; return help string.
        longHelp(status, usage, &Correlator::execArgusGate);
      } else {
        // Execute the command.
    	if (m > 0 && m <= NRX && n > 0 && n <= NSTAGES){
    		// convert from user's 1-base to code's 0-base
    		OSTimeDly(CMDDELAY);
    		int rtn = argus_setLNAbias("g", m-1, n-1, v, 0);
    		if (rtn == -10) {
        		sprintf(status, "%sLNA cards are not powered, returned status %d.\r\n",
        				statusERR, rtn);
    		} else {
    			sprintf(status, "%sargus_setLNAbias(%d, %d, %f, 0) returned status %d.\r\n",
    					(rtn==0 ? statusOK : statusERR), m, n, v, rtn);
    		}
    	} else {
    		sprintf(status, "%sReceiver or stage number out of range\r\n", statusERR);
    	}
     }
  } else {
      // Command called without arguments; read gate values
      longHelp(status, usage, &Correlator::execArgusGate);  //here dummy
     }
  } else {
    longHelp(status, usage, &Correlator::execArgusGate);
  }
}

/**
  \brief Argus individual mixer bias control.

  Set or read a single LNA mixer bias for Argus.

  \param status Storage buffer for return status (should contain at least
                ControlService::maxLine characters).
  \param arg    Argument list: LEVEL
*/
void Correlator::execArgusMixer(return_type status, argument_type arg)
{
  static const char *usage =
  "[M N V]\r\n"
  "  Set an LNA mixer voltage.\r\n"
  "  M is the Mth receiver to set.\r\n"
  "  N is the Nth stage within receiver to set.\r\n"
  "  V is the voltage in V to set.\r\n"
		  ;

  if (!arg.help) {
    int m, n;
    float v = 0.0;
    if (arg.str) {
      // Command called with one or more arguments.
      int narg = sscanf(arg.str, "%d%d%f", &m, &n, &v);
      if (narg < 3) {
        // Too few arguments; return help string.
        longHelp(status, usage, &Correlator::execArgusMixer);
      } else {
        // Execute the command.
    	if (m > 0 && m <= NRX && n > 0 && n <= NSTAGES){
    		// convert from user's 1-base to code's 0-base
    		OSTimeDly(CMDDELAY);
    		int rtn = argus_setLNAbias("m", m-1, n-1, v, 0);
    		if (rtn == -10) {
        		sprintf(status, "%sLNA cards are not powered, returned status %d.\r\n",
        				statusERR, rtn);
    		} else {
    		sprintf(status, "%sargus_setLNAbias(%d, %d, %f, 0) returned status %d.\r\n",
    				(rtn==0 ? statusOK : statusERR), m, n, v, rtn);
    		}
    	} else {
    		sprintf(status, "%sReceiver or stage number out of range\r\n", statusERR);
    	}
     }
  } else {
      // Command called without arguments; read mixer values
      longHelp(status, usage, &Correlator::execArgusMixer);  //here dummy
     }
  } else {
    longHelp(status, usage, &Correlator::execArgusMixer);
  }
}

/**
  \brief Argus individual receiver attenuator control.

  Set a single receiver's warm IF attenuation for Argus.

  \param status Storage buffer for return status (should contain at least
                ControlService::maxLine characters).
  \param arg    Argument list: LEVEL
*/
void Correlator::execArgusAtten(return_type status, argument_type arg)
{
  static const char *usage =
  "[M A]\r\n"
  "  Set a receiver warm IF attenuation.\r\n"
  "  M is the Mth receiver to set.\r\n"
  "  A is the attenuation in dB to set.\r\n"
		  ;

  if (!arg.help) {
    int m, a;
   if (arg.str) {
      // Command called with one or more arguments.
      int narg = sscanf(arg.str, "%d%d", &m, &a);
      if (narg < 2) {
        // Too few arguments; return help string.
        longHelp(status, usage, &Correlator::execArgusAtten);
      } else {
        // Execute the command.
    	if (m > 0 && m <= NRX){
    		// convert from user's 1-base to code's 0-base
    		OSTimeDly(CMDDELAY);
    		int rtn = argus_setWIFswitches("a", m-1, a, 0);
   			sprintf(status, "%sargus_setWIFswitches(a, %d, %d, 0) returned status %d.\r\n",
    					(rtn==0 ? statusOK : statusERR), m, a, rtn);
    	} else {
    		sprintf(status, "%sReceiver number out of range\r\n", statusERR);
    	}
     }
  } else {
      // Command called without arguments; read gate values
      longHelp(status, usage, &Correlator::execArgusAtten);  //here dummy
     }
  } else {
    longHelp(status, usage, &Correlator::execArgusAtten);
  }
}

/**
  \brief Argus individual receiver sideband control.

  Set a single receiver's warm IF sideband for Argus.

  \param status Storage buffer for return status (should contain at least
                ControlService::maxLine characters).
  \param arg    Argument list: LEVEL
*/
void Correlator::execArgusSB(return_type status, argument_type arg)
{
  static const char *usage =
  "[M S]\r\n"
  "  Set a receiver sideband.\r\n"
  "  M is the Mth receiver to set.\r\n"
  "  S is the sideband: 0 for LSB, 1 for USB.\r\n"
		  ;

  if (!arg.help) {
    int m, s;
   if (arg.str) {
      // Command called with one or more arguments.
      int narg = sscanf(arg.str, "%d%d", &m, &s);
      if (narg < 2) {
        // Too few arguments; return help string.
        longHelp(status, usage, &Correlator::execArgusSB);
      } else {
        // Execute the command.
    	if (m > 0 && m <= NRX){
    		// convert from user's 1-base to code's 0-base
    		OSTimeDly(CMDDELAY);
    		int rtn = argus_setWIFswitches("s", m-1, s, 0);
   			sprintf(status, "%sargus_setWIFswitches(s, %d, %d, 0) returned status %d.\r\n",
    					(rtn==0 ? statusOK : statusERR), m, s, rtn);
    	} else {
    		sprintf(status, "%sReceiver number out of range\r\n", statusERR);
    	}
     }
  } else {
      // Command called without arguments; read gate values
      longHelp(status, usage, &Correlator::execArgusSB);  //here dummy
     }
  } else {
    longHelp(status, usage, &Correlator::execArgusSB);
  }
}

/**
  \brief Argus: set all gate, drain, or mixer biases to a common value.

  Set all gate, drain, or mixer biases to a common value.

  \param status Storage buffer for return status (should contain at least
                ControlService::maxLine characters).
  \param arg    Argument list: LEVEL
*/
void Correlator::execArgusSetAll(return_type status, argument_type arg)
{
  static const char *usage =
  "[KEYWORD VALUE]\r\n"
  "  Set choice of LNA mixer gate/drain/mixer bias voltages or \r\n"
  "  receiver warm IF attenuations or sidebands to a common value.\r\n"
  "  Keywords are:\r\n"
  "    G  gate [V].\r\n"
  "    D  drain [V].\r\n"
  "    M  mixer [V].\r\n"
  "    A  attenuation [dB].\r\n"
  "    S  sideband [0 = LSB, 1 = USB].\r\n"
  "  Value V is the set value.\r\n"
		  ;

  if (!arg.help) {
    float v = 0.0;
    char inp[2] = {0};

    if (arg.str) {
      // Command called with one or more arguments.
      int narg = sscanf(arg.str, "%1s%f", inp, &v);
      if (narg < 2) {
        // Too few arguments; return help string.
        longHelp(status, usage, &Correlator::execArgusSetAll);
      } else if (!strcmp(inp, "a") || !strcmp(inp, "s")) {
    	// Set atten, sb
   		OSTimeDly(CMDDELAY);
        int rtn = argus_setAllWIFswitches(inp, (char)v);
		sprintf(status, "%sargus_setAlWIFswitches(%s, %d) returned status %d.\r\n",
					(rtn==0 ? statusOK : statusERR), inp, (char)v, rtn);
      } else {
        // Set G, D, M biases
    	OSTimeDly(CMDDELAY);
    	int rtn = argus_setAllBias(inp, v, 0);
    	if (rtn == -10) {
        	sprintf(status, "%sLNA cards are not powered, returned status %d.\r\n",
        			statusERR, rtn);
    	} else {
    		sprintf(status, "%sargus_setAllBias(%s, %f) returned status %d.\r\n",
    				(rtn==0 ? statusOK : statusERR), inp, v, rtn);
    	}
      }

  } else {
      // Command called without arguments; read mixer values
      longHelp(status, usage, &Correlator::execArgusSetAll);  //here dummy
     }
  } else {
    longHelp(status, usage, &Correlator::execArgusSetAll);
  }
}

/**
  \brief Argus cryostat monitoring.

  Monitor temperatures and aux inputs from the cryo board.

  \param status Storage buffer for return status (should contain at least
                ControlService::maxLine characters).
  \param arg    Argument list: LEVEL
*/
void Correlator::execArgusCryo(return_type status, argument_type arg)
{
  static const char *usage =
  "\r\n"
  "  Read all cryostat monitor points, return values to screen.\r\n"
		  ;

  if (!arg.help) {
	OSTimeDly(CMDDELAY);
   	int rtn = argus_readThermADCs();
    sprintf(status, "%sCryostat:\r\n%s%8.1f K\r\n%s%8.1f K\r\n%s%8.1f K\r\n"
    			    "%s%8.1f K\r\n%s%8.1f K\r\n%s%8.1f K\r\n%s%8.1e Torr (%4.3f V)\r\n",
    		(rtn==0 ? statusOK : statusERR), cnames[0], cryoPar.cryoTemps[0], cnames[1], cryoPar.cryoTemps[1],
    		cnames[2], cryoPar.cryoTemps[2], cnames[3], cryoPar.cryoTemps[3], cnames[4], cryoPar.cryoTemps[4],
    		cnames[5], cryoPar.cryoTemps[5], cnames[6],
    		(cryoPar.auxInputs[0] > 1. ? powf(10., cryoPar.auxInputs[0]-6.) : 0.),
    		cryoPar.auxInputs[0]);
  } else {
	longHelp(status, usage, &Correlator::execArgusCryo);
  }

}

/**
  \brief Use Argus LNA presets.

  Use Argus LNA presets from flash memory.

  \param status Storage buffer for return status (should contain at least
                ControlService::maxLine characters).
  \param arg    Argument list: LEVEL
*/
void Correlator::execArgusPresets(return_type status, argument_type arg)
{
  static const char *usage =
  "\r\n"
  "  Set LNA biases to values stored in memory.\r\n"
  "  (see FLASH command to set).\r\n "
		  ;

  if (!arg.help) {
	flash_t flashData;
	zpec_readFlash(&flashData);
	OSTimeDly(CMDDELAY);
	int rtn = argus_LNApresets(&flashData);
    sprintf(status, "%sSet LNA to preset bias values, status %d\r\n",
    		(rtn==0 ? statusOK : statusERR), rtn);
  } else {
	longHelp(status, usage, &Correlator::execArgusPresets);
  }

}

/**
  \brief Argus LNA power control.

  Turn LNA power on and off for Argus.

  \param status Storage buffer for return status (should contain at least
                ControlService::maxLine characters).
  \param arg    Argument list: LEVEL
*/
void Correlator::execArgusPwrCtrl(return_type status, argument_type arg)
{
  static const char *usage =
  "[STATE]\r\n"
  "  Sequence LNA power on or off, query LNA power supply.\r\n"
  "  STATE  ON or 1 to sequence LNA power on.\r\n"
  "         OFF or 0 to sequence LNA power off.\r\n"
  "  No argument returns power supply voltages at power control card.\r\n"
		  ;

  if (!arg.help) {
	  if (arg.str) {  // argument present, set new state.
		  char state[5] = {0};

      // Command called with one or more arguments.
      int narg = sscanf(arg.str, "%4s", state);
      if (narg < 1) {
        // Too few arguments; return help string.
        longHelp(status, usage, &Correlator::execArgusPwrCtrl);
      } else {
        // Execute the command.
      	if (!strcmp(state, "1") || !strcasecmp(state, "ON")) {
      		OSTimeDly(CMDDELAY);
      		int rtn = argus_lnaPower(1);
            sprintf(status, "%sLNA power commanded on, status %d.\r\n",
                             (rtn==0 ? statusOK : statusERR), rtn);
      	}
      	else if (!strcmp(state, "0") || !strcasecmp(state, "OFF")) {
      		OSTimeDly(CMDDELAY);
      		int rtn = argus_lnaPower(0);
            sprintf(status, "%sLNA power commanded off, status %d.\r\n",
                             (rtn==0 ? statusOK : statusERR), rtn);
      	}
      	else {
      		longHelp(status, usage, &Correlator::execArgusPwrCtrl);
      	}
      }
    } else {
      // Command called without arguments; echo power state and key values
      int rtn = argus_readPwrADCs();

      sprintf(status, "%sLNA power state %s.\r\n +15V: %6.2f V\r\n -15V: %6.2f V\r\n  +5V: %6.2f V\r\n",
    		  (rtn==0 ? statusOK : statusERR), (lnaPwrState==1 ? "ON" : "OFF"),
    		  pwrCtrlPar[2], pwrCtrlPar[1], pwrCtrlPar[0]);
    }
  } else {
    longHelp(status, usage, &Correlator::execArgusPwrCtrl);
  }
}

/**
  \brief Argus cold IF power control.

  Turn cold IF power on and off for Argus.

  \param status Storage buffer for return status (should contain at least
                ControlService::maxLine characters).
  \param arg    Argument list: LEVEL
*/
void Correlator::execArgusCIFPwrCtrl(return_type status, argument_type arg)
{
  static const char *usage =
  "[STATE]\r\n"
  "  Turn cold IF power on or off.\r\n"
  "  STATE  1 or ON to turn cold IF power on.\r\n"
  "         0 or OFF to turn cold IF power off.\r\n"
  "  No argument returns power supply state.\r\n"
		  ;

  if (!arg.help) {
	  if (arg.str) {  // argument present, set new state.
		  char state[5] = {0};

      // Command called with one or more arguments.
      int narg = sscanf(arg.str, "%4s", state);
      if (narg < 1) {
        // Too few arguments; return help string.
        longHelp(status, usage, &Correlator::execArgusCIFPwrCtrl);
      } else {
        // Execute the command.
    	if (lnaPwrState) {
            sprintf(status, "%sNo change in state allowed with LNA bias on.\r\n",
                             statusERR);
    	} else {
    		if (!strcmp(state, "1") || !strcasecmp(state, "ON")) {
    			OSTimeDly(CMDDELAY);
    			int rtn = argus_cifPower(1);
    			sprintf(status, "%sCold IF power commanded on, status %d.\r\n",
                             (rtn==0 ? statusOK : statusERR), rtn);
    		}
    		else if (!strcmp(state, "0") || !strcasecmp(state, "OFF")) {
    			OSTimeDly(CMDDELAY);
    			int rtn = argus_cifPower(0);
    			sprintf(status, "%sCold IF power commanded off, status %d.\r\n",
                             (rtn==0 ? statusOK : statusERR), rtn);
    		} else {
    			longHelp(status, usage, &Correlator::execArgusCIFPwrCtrl);
    		}
    	}
      }
    } else {
      // Command called without arguments; echo power state and key values
      OSTimeDly(CMDDELAY);
      int rtn = argus_readPwrADCs();
      sprintf(status, "%sCold IF power state %s.\r\n"
	          "Supply voltage:  %6.2f V\r\n"
	          "Output voltage:  %6.2f V, current: %6.2f A\r\n",
    		  (rtn==0 ? statusOK : statusERR), (cifPwrState==1 ? "ON" : "OFF"),
    		  pwrCtrlPar[5], pwrCtrlPar[6], pwrCtrlPar[7]);
    }
  } else {
    longHelp(status, usage, &Correlator::execArgusCIFPwrCtrl);
  }

}
/**
  \brief Argus warm IF monitor.

  Get and list status of the Argus warm IF system.

  \param status Storage buffer for return status (should contain at least
                ControlService::maxLine characters).
  \param arg    Argument list: LEVEL
*/
void Correlator::execArgusWIFCtrl(return_type status, argument_type arg)
{
  static const char *usage =
	"\r\n"
	"  Read all warm IF monitor points, return values to screen.\r\n"
 		  ;

  if (!arg.help) {
	  OSTimeDly(CMDDELAY);
	  int rtn = argus_readWIF();     // update total power and temperature in status table
      rtn += argus_readWIFpsADCs();  // update power supply voltage in status table
      sprintf(status, "%sWarm IF:\r\n"
    		  "Supply voltages: %5.2f V, %5.2f V\r\n"
    		  "Ch    TotPow       Atten    SB    Card T\r\n"
    		  " 1   %8.4f V    %2d dB    %1d    %5.1f C\r\n"
    		  " 2   %8.4f V    %2d dB    %1d    %5.1f C\r\n"
    		  " 3   %8.4f V    %2d dB    %1d    %5.1f C\r\n"
    		  " 4   %8.4f V    %2d dB    %1d    %5.1f C\r\n"
    		  " 5   %8.4f V    %2d dB    %1d    %5.1f C\r\n"
    		  " 6   %8.4f V    %2d dB    %1d    %5.1f C\r\n"
    		  " 7   %8.4f V    %2d dB    %1d    %5.1f C\r\n"
    		  " 8   %8.4f V    %2d dB    %1d    %5.1f C\r\n"
    		  " 9   %8.4f V    %2d dB    %1d    %5.1f C\r\n"
    		  "10   %8.4f V    %2d dB    %1d    %5.1f C\r\n"
    		  "11   %8.4f V    %2d dB    %1d    %5.1f C\r\n"
    		  "12   %8.4f V    %2d dB    %1d    %5.1f C\r\n"
    		  "13   %8.4f V    %2d dB    %1d    %5.1f C\r\n"
    		  "14   %8.4f V    %2d dB    %1d    %5.1f C\r\n"
    		  "15   %8.4f V    %2d dB    %1d    %5.1f C\r\n"
    		  "16   %8.4f V    %2d dB    %1d    %5.1f C\r\n",
    		  (rtn==0 ? statusOK : statusERR), wifPar.psv[0], wifPar.psv[1],
    		  wifPar.totPow[0], wifPar.atten[0], wifPar.sb[0], wifPar.cardTemp[0],
    		  wifPar.totPow[1], wifPar.atten[1], wifPar.sb[1], wifPar.cardTemp[1],
    		  wifPar.totPow[2], wifPar.atten[2], wifPar.sb[2], wifPar.cardTemp[2],
    		  wifPar.totPow[3], wifPar.atten[3], wifPar.sb[3], wifPar.cardTemp[3],
    		  wifPar.totPow[4], wifPar.atten[4], wifPar.sb[4], wifPar.cardTemp[4],
    		  wifPar.totPow[5], wifPar.atten[5], wifPar.sb[5], wifPar.cardTemp[5],
    		  wifPar.totPow[6], wifPar.atten[6], wifPar.sb[6], wifPar.cardTemp[6],
    		  wifPar.totPow[7], wifPar.atten[7], wifPar.sb[7], wifPar.cardTemp[7],
    		  wifPar.totPow[8], wifPar.atten[8], wifPar.sb[8], wifPar.cardTemp[8],
    		  wifPar.totPow[9], wifPar.atten[9], wifPar.sb[9], wifPar.cardTemp[9],
    		  wifPar.totPow[10], wifPar.atten[10], wifPar.sb[10], wifPar.cardTemp[10],
    		  wifPar.totPow[11], wifPar.atten[11], wifPar.sb[11], wifPar.cardTemp[11],
    		  wifPar.totPow[12], wifPar.atten[12], wifPar.sb[12], wifPar.cardTemp[12],
    		  wifPar.totPow[13], wifPar.atten[13], wifPar.sb[13], wifPar.cardTemp[13],
    		  wifPar.totPow[14], wifPar.atten[14], wifPar.sb[14], wifPar.cardTemp[14],
    		  wifPar.totPow[15], wifPar.atten[15], wifPar.sb[15], wifPar.cardTemp[15]);
  } else {
    longHelp(status, usage, &Correlator::execArgusWIFCtrl);
  }
}

/**
  \brief Read and display Argus monitor points.

  Read and display Argus monitor points.

  \param status Storage buffer for return status (should contain at least
                ControlService::maxLine characters).
  \param arg    Argument list: LEVEL
*/
void Correlator::execArgusMonPts(return_type status, argument_type arg)
{
  static const char *usage =
		  "[KEYWORD]\r\n"
		  "  Read and display monitor and set points.\r\n"
		  "  KEYWORD:\r\n"
		  "    LNA for measured LNA bias values.\r\n"
		  "    MIX for measured LNA mixer bias values.\r\n"
		  "    SETS for LNA set points.\r\n"
		  "    POW for power supply values and card power monitor points.\r\n"
		  "    WIF for warm IF monitor points.\r\n"
		  "    CRYO for cryostat monitor points.\r\n"
		  "    YIG for LO YIG filter status.\r\n"
		  "    VANE for cal system vane data.\r\n"
		  "    PRESETS for stored bias values.\r\n"
		  "  No KEYWORD returns LNA bias values.\r\n"
		  ;

  int rtn = 0;
  if (!arg.help) {
	  if (arg.str) {  // argument present, set new state.
		  char state[5] = {0};
      // Command called with an argument.
	      int narg = sscanf(arg.str, "%4s", state);
	      if (narg == 1) {
	    	  if (!strcasecmp(state, "lna")) {
	     		  if (lnaPwrState) {
	     			  OSTimeDly(CMDDELAY);
	     			  rtn += argus_readLNAbiasADCs("vg");
	    	      	  rtn += argus_readLNAbiasADCs("vd");
	    	      	  rtn += argus_readLNAbiasADCs("id");
	    	      	  rtn += argus_readPwrADCs();
	    	      	  sprintf(status, "%sLNA power state %s.\r\nSupplies: +15V: %5.2f V; "
	    	      			  "-15V: %5.2f V; +5V: %5.2f V\r\n"
	    	      			  "Voltages in [V], currents in [mA]\r\n\r\n"
	    	      			  "          1               2               3               4\r\n"
	    	      			  "VG: %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f\r\n"
	    	      			  "VD: %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f\r\n"
	    	      			  "ID: %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f\r\n\r\n"
	    	      			  "          5               6               7               8\r\n"
	    	      			  "VG: %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f\r\n"
	    	      			  "VD: %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f\r\n"
	    	      			  "ID: %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f\r\n\r\n"
	    	      			  "          9               10              11              12\r\n"
	    	      			  "VG: %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f\r\n"
	    	      			  "VD: %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f\r\n"
	    	      			  "ID: %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f\r\n\r\n"
	    	      			  "          13              14              15              16\r\n"
	    	      			  "VG: %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f\r\n"
	    	      			  "VD: %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f\r\n"
	    	      			  "ID: %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f\r\n\r\n",
	    	      			  (rtn==0 ? statusOK : statusERR), (lnaPwrState==1 ? "ON" : "OFF"),
	    	      			  pwrCtrlPar[2], pwrCtrlPar[1], pwrCtrlPar[0],  //pv, nv, vds
	    	      			  d2, rxPar[0].LNAmonPts[0], d2, rxPar[0].LNAmonPts[1], d2, rxPar[1].LNAmonPts[0], d2, rxPar[1].LNAmonPts[1],
	    	      			  d2, rxPar[2].LNAmonPts[0], d2, rxPar[2].LNAmonPts[1], d2, rxPar[3].LNAmonPts[0], d2, rxPar[3].LNAmonPts[1],
	    	      			  d1, rxPar[0].LNAmonPts[2], d1, rxPar[0].LNAmonPts[3], d1, rxPar[1].LNAmonPts[2], d1, rxPar[1].LNAmonPts[3],
	    	      			  d2, rxPar[2].LNAmonPts[2], d2, rxPar[2].LNAmonPts[3], d2, rxPar[3].LNAmonPts[2], d2, rxPar[3].LNAmonPts[3],
	    	      			  d2, rxPar[0].LNAmonPts[4], d2, rxPar[0].LNAmonPts[5], d2, rxPar[1].LNAmonPts[4], d2, rxPar[1].LNAmonPts[5],
	    	      			  d1, rxPar[2].LNAmonPts[4], d1, rxPar[2].LNAmonPts[5], d1, rxPar[3].LNAmonPts[4], d1, rxPar[3].LNAmonPts[5],

	    	      			  d2, rxPar[4].LNAmonPts[0], d2, rxPar[4].LNAmonPts[1], d2, rxPar[5].LNAmonPts[0], d2, rxPar[5].LNAmonPts[1],
	    	      			  d2, rxPar[6].LNAmonPts[0], d2, rxPar[6].LNAmonPts[1], d2, rxPar[7].LNAmonPts[0], d2, rxPar[7].LNAmonPts[1],
	    	      			  d1, rxPar[4].LNAmonPts[2], d1, rxPar[4].LNAmonPts[3], d1, rxPar[5].LNAmonPts[2], d1, rxPar[5].LNAmonPts[3],
	    	      			  d2, rxPar[6].LNAmonPts[2], d2, rxPar[6].LNAmonPts[3], d2, rxPar[7].LNAmonPts[2], d2, rxPar[7].LNAmonPts[3],
	    	      			  d2, rxPar[4].LNAmonPts[4], d1, rxPar[4].LNAmonPts[5], d1, rxPar[5].LNAmonPts[4], d1, rxPar[5].LNAmonPts[5],
	    	      			  d1, rxPar[6].LNAmonPts[4], d2, rxPar[6].LNAmonPts[5], d2, rxPar[7].LNAmonPts[4], d2, rxPar[7].LNAmonPts[5],

	    	      			  d2, rxPar[8].LNAmonPts[0],  d2, rxPar[8].LNAmonPts[1],  d2, rxPar[9].LNAmonPts[0],  d2, rxPar[9].LNAmonPts[1],
	    	      			  d2, rxPar[10].LNAmonPts[0], d2, rxPar[10].LNAmonPts[1], d2, rxPar[11].LNAmonPts[0], d2, rxPar[11].LNAmonPts[1],
	    	      			  d1, rxPar[8].LNAmonPts[2],  d1, rxPar[8].LNAmonPts[3],  d1, rxPar[9].LNAmonPts[2],  d1, rxPar[9].LNAmonPts[3],
	    	      			  d2, rxPar[10].LNAmonPts[2], d2, rxPar[10].LNAmonPts[3], d2, rxPar[11].LNAmonPts[2], d2, rxPar[11].LNAmonPts[3],
	    	      			  d2, rxPar[8].LNAmonPts[4],  d2, rxPar[8].LNAmonPts[5],  d2, rxPar[9].LNAmonPts[4],  d2, rxPar[9].LNAmonPts[5],
	    	      			  d1, rxPar[10].LNAmonPts[4], d1, rxPar[10].LNAmonPts[5], d1, rxPar[11].LNAmonPts[4], d1, rxPar[11].LNAmonPts[5],

	    	      			  d2, rxPar[12].LNAmonPts[0], d2, rxPar[12].LNAmonPts[1], d2, rxPar[13].LNAmonPts[0], d2, rxPar[13].LNAmonPts[1],
	    	      			  d2, rxPar[14].LNAmonPts[0], d2, rxPar[14].LNAmonPts[1], d2, rxPar[15].LNAmonPts[0], d2, rxPar[15].LNAmonPts[1],
	    	      			  d1, rxPar[12].LNAmonPts[2], d1, rxPar[12].LNAmonPts[3], d1, rxPar[13].LNAmonPts[2], d1, rxPar[13].LNAmonPts[3],
	    	      			  d2, rxPar[14].LNAmonPts[2], d2, rxPar[14].LNAmonPts[3], d2, rxPar[15].LNAmonPts[2], d2, rxPar[15].LNAmonPts[3],
	    	      			  d2, rxPar[12].LNAmonPts[4], d2, rxPar[12].LNAmonPts[5], d2, rxPar[13].LNAmonPts[4], d2, rxPar[13].LNAmonPts[5],
	    	      			  d1, rxPar[14].LNAmonPts[4], d1, rxPar[14].LNAmonPts[5], d1, rxPar[15].LNAmonPts[4], d1, rxPar[15].LNAmonPts[5]);
	    		  } else {
	    			  sprintf(status, "%sNo report: LNA power is not on.\r\n", statusERR);
	    		  }
	    	  } else if (!strcasecmp(state, "mix")) {
	    		  OSTimeDly(CMDDELAY);
	    		  rtn += argus_readLNAbiasADCs("vm");
    	      	  rtn += argus_readLNAbiasADCs("im");
    	      	  rtn += argus_readPwrADCs();
	    		  if (lnaPwrState) {
	    	      	  sprintf(status, "%sLNA power state %s.\r\nSupplies: +15V: %5.2f V; "
	    	      			  "-15V: %5.2f V; +5V: %5.2f V\r\n"
	    	      			  "Voltages in [V], currents in [mA]\r\n\r\n"
	    	      			  "          1               2               3               4\r\n"
	    	      			  "VM: %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f\r\n"
	    	      			  "IM: %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f\r\n\r\n"
	    	      			  "          5               6               7               8\r\n"
	    	      			  "VM: %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f\r\n"
	    	      			  "IM: %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f\r\n\r\n"
	    	      			  "          9               10              11              12\r\n"
	    	      			  "VM: %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f\r\n"
	    	      			  "IM: %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f\r\n\r\n"
	    	      			  "          13              14              15              16\r\n"
	    	      			  "VM: %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f\r\n"
	    	      			  "IM: %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f\r\n\r\n",
	    	      			  (rtn==0 ? statusOK : statusERR), (lnaPwrState==1 ? "ON" : "OFF"),
	    	      			  pwrCtrlPar[2], pwrCtrlPar[1], pwrCtrlPar[0],  //pv, nv, vds
	    	      			  d2, rxPar[0].LNAmonPts[6], d2, rxPar[0].LNAmonPts[7], d2, rxPar[1].LNAmonPts[6], d2, rxPar[1].LNAmonPts[7],
	    	      			  d2, rxPar[2].LNAmonPts[6], d2, rxPar[2].LNAmonPts[7], d2, rxPar[3].LNAmonPts[6], d2, rxPar[3].LNAmonPts[7],
	    	      			  d1, rxPar[0].LNAmonPts[8], d1, rxPar[0].LNAmonPts[9], d1, rxPar[1].LNAmonPts[8], d1, rxPar[1].LNAmonPts[9],
	    	      			  d1, rxPar[2].LNAmonPts[8], d1, rxPar[2].LNAmonPts[9], d1, rxPar[3].LNAmonPts[8], d1, rxPar[3].LNAmonPts[9],

	    	      			  d2, rxPar[4].LNAmonPts[6], d2, rxPar[4].LNAmonPts[7], d2, rxPar[5].LNAmonPts[6], d2, rxPar[5].LNAmonPts[7],
	    	      			  d2, rxPar[6].LNAmonPts[6], d2, rxPar[6].LNAmonPts[7], d2, rxPar[7].LNAmonPts[6], d2, rxPar[7].LNAmonPts[7],
	    	      			  d1, rxPar[4].LNAmonPts[8], d1, rxPar[4].LNAmonPts[9], d1, rxPar[5].LNAmonPts[8], d1, rxPar[5].LNAmonPts[9],
	    	      			  d1, rxPar[6].LNAmonPts[8], d1, rxPar[6].LNAmonPts[9], d1, rxPar[7].LNAmonPts[8], d1, rxPar[7].LNAmonPts[9],

	    	      			  d2, rxPar[8].LNAmonPts[6], d2, rxPar[8].LNAmonPts[7], d2, rxPar[9].LNAmonPts[6], d2, rxPar[9].LNAmonPts[7],
	    	      			  d2, rxPar[10].LNAmonPts[6], d2, rxPar[10].LNAmonPts[7], d2, rxPar[11].LNAmonPts[6], d2, rxPar[11].LNAmonPts[7],
	    	      			  d1, rxPar[8].LNAmonPts[8], d1, rxPar[8].LNAmonPts[9], d1, rxPar[9].LNAmonPts[8], d1, rxPar[9].LNAmonPts[9],
	    	      			  d1, rxPar[10].LNAmonPts[8], d1, rxPar[10].LNAmonPts[9], d1, rxPar[11].LNAmonPts[8], d1, rxPar[11].LNAmonPts[9],

	    	      			  d2, rxPar[12].LNAmonPts[6], d2, rxPar[12].LNAmonPts[7], d2, rxPar[13].LNAmonPts[6], d2, rxPar[13].LNAmonPts[7],
	    	      			  d2, rxPar[14].LNAmonPts[6], d2, rxPar[14].LNAmonPts[7], d2, rxPar[15].LNAmonPts[6], d2, rxPar[15].LNAmonPts[7],
	    	      			  d1, rxPar[12].LNAmonPts[8], d1, rxPar[12].LNAmonPts[9], d1, rxPar[13].LNAmonPts[8], d1, rxPar[13].LNAmonPts[9],
	    	      			  d1, rxPar[14].LNAmonPts[8], d1, rxPar[14].LNAmonPts[9], d1, rxPar[15].LNAmonPts[8], d1, rxPar[15].LNAmonPts[9]);
	    		  } else {
	    			  sprintf(status, "%sNo report: LNA power is not on.\r\n", statusERR);
	              }
	    	  } else if (!strcasecmp(state, "sets")) {
	    		  if (1) {     //(lnaPwrState != 0) {
	    	      	  sprintf(status, "%sSet values:\r\n"
	    	      			  "Voltages in [V], currents in [mA]\r\n\r\n"
	    	      			  "         1               2               3               4\r\n"
	    	      			  "G: %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f\r\n"
	    	      			  "D: %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f\r\n"
	    	      			  "M: %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f\r\n\r\n"
	    	      			  "         5               6               7               8\r\n"
	    	      			  "G: %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f\r\n"
	    	      			  "D: %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f\r\n"
	    	      			  "M: %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f\r\n\r\n"
	    	      			  "         9               10              11              12\r\n"
	    	      			  "G: %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f\r\n"
	    	      			  "D: %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f\r\n"
	    	      			  "M: %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f\r\n\r\n"
	    	      			  "         13              14              15              16\r\n"
	    	      			  "G: %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f\r\n"
	    	      			  "D: %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f\r\n"
	    	      			  "M: %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f,   %5.*f, %5.*f\r\n\r\n"

	    	      			  "A: %2d, %2d, %2d, %2d,   %2d, %2d, %2d, %2d\r\n"
	    	      			  "S: %2d, %2d, %2d, %2d,   %2d, %2d, %2d, %2d\r\n"
	    	      			  "A: %2d, %2d, %2d, %2d,   %2d, %2d, %2d, %2d\r\n"
	    	      			  "S: %2d, %2d, %2d, %2d,   %2d, %2d, %2d, %2d\r\n\r\n",

	    	      			  statusOK,

	    	      			  d2, rxPar[0].LNAsets[0], d2, rxPar[0].LNAsets[1], d2, rxPar[1].LNAsets[0], d2, rxPar[1].LNAsets[1],
	    	      			  d2, rxPar[2].LNAsets[0], d2, rxPar[2].LNAsets[1], d2, rxPar[3].LNAsets[0], d2, rxPar[3].LNAsets[1],
	    	      			  d1, rxPar[0].LNAsets[2], d1, rxPar[0].LNAsets[3], d1, rxPar[1].LNAsets[2], d1, rxPar[1].LNAsets[3],
	    	      			  d1, rxPar[2].LNAsets[2], d1, rxPar[2].LNAsets[3], d1, rxPar[3].LNAsets[2], d1, rxPar[3].LNAsets[3],
	    	      			  d2, rxPar[0].LNAsets[4], d2, rxPar[0].LNAsets[5], d2, rxPar[1].LNAsets[4], d2, rxPar[1].LNAsets[5],
	    	      			  d2, rxPar[2].LNAsets[4], d2, rxPar[2].LNAsets[5], d2, rxPar[3].LNAsets[4], d2, rxPar[3].LNAsets[5],

	    	      			  d2, rxPar[4].LNAsets[0], d2, rxPar[4].LNAsets[1], d2, rxPar[5].LNAsets[0], d2, rxPar[5].LNAsets[1],
	    	      			  d2, rxPar[6].LNAsets[0], d2, rxPar[6].LNAsets[1], d2, rxPar[7].LNAsets[0], d2, rxPar[7].LNAsets[1],
	    	      			  d1, rxPar[4].LNAsets[2], d1, rxPar[4].LNAsets[3], d1, rxPar[5].LNAsets[2], d1, rxPar[5].LNAsets[3],
	    	      			  d1, rxPar[6].LNAsets[2], d1, rxPar[6].LNAsets[3], d1, rxPar[7].LNAsets[2], d1, rxPar[7].LNAsets[3],
	    	      			  d2, rxPar[4].LNAsets[4], d2, rxPar[4].LNAsets[5], d2, rxPar[5].LNAsets[4], d2, rxPar[5].LNAsets[5],
	    	      			  d2, rxPar[6].LNAsets[4], d2, rxPar[6].LNAsets[5], d2, rxPar[7].LNAsets[4], d2, rxPar[7].LNAsets[5],

	    	      			  d2, rxPar[8].LNAsets[0], d2, rxPar[8].LNAsets[1], d2, rxPar[9].LNAsets[0], d2, rxPar[9].LNAsets[1],
	    	      			  d2, rxPar[10].LNAsets[0], d2, rxPar[10].LNAsets[1], d2, rxPar[11].LNAsets[0], d2, rxPar[11].LNAsets[1],
	    	      			  d1, rxPar[8].LNAsets[2], d1, rxPar[8].LNAsets[3], d1, rxPar[9].LNAsets[2], d1, rxPar[9].LNAsets[3],
	    	      			  d1, rxPar[10].LNAsets[2], d1, rxPar[10].LNAsets[3], d1, rxPar[11].LNAsets[2], d1, rxPar[11].LNAsets[3],
	    	      			  d2, rxPar[8].LNAsets[4], d2, rxPar[8].LNAsets[5], d2, rxPar[9].LNAsets[4], d2, rxPar[9].LNAsets[5],
	    	      			  d2, rxPar[10].LNAsets[4], d2, rxPar[10].LNAsets[5], d2, rxPar[11].LNAsets[4], d2, rxPar[11].LNAsets[5],

	    	      			  d2, rxPar[12].LNAsets[0], d2, rxPar[12].LNAsets[1], d2, rxPar[13].LNAsets[0], d2, rxPar[13].LNAsets[1],
	    	      			  d2, rxPar[14].LNAsets[0], d2, rxPar[14].LNAsets[1], d2, rxPar[15].LNAsets[0], d2, rxPar[15].LNAsets[1],
	    	      			  d1, rxPar[12].LNAsets[2], d1, rxPar[12].LNAsets[3], d1, rxPar[13].LNAsets[2], d1, rxPar[13].LNAsets[3],
	    	      			  d1, rxPar[14].LNAsets[2], d1, rxPar[14].LNAsets[3], d1, rxPar[15].LNAsets[2], d1, rxPar[15].LNAsets[3],
	    	      			  d2, rxPar[12].LNAsets[4], d2, rxPar[12].LNAsets[5], d2, rxPar[13].LNAsets[4], d2, rxPar[13].LNAsets[5],
	    	      			  d2, rxPar[14].LNAsets[4], d2, rxPar[14].LNAsets[5], d2, rxPar[15].LNAsets[4], d2, rxPar[15].LNAsets[5],

		    	      			wifPar.atten[0], wifPar.atten[1], wifPar.atten[2], wifPar.atten[3],
		    	      			wifPar.atten[4], wifPar.atten[5], wifPar.atten[6], wifPar.atten[7],
		    	      			wifPar.sb[0], wifPar.sb[1], wifPar.sb[2], wifPar.sb[3],
		    	      			wifPar.sb[4], wifPar.sb[5], wifPar.sb[6], wifPar.sb[7],
		    	      			wifPar.atten[8], wifPar.atten[9], wifPar.atten[10], wifPar.atten[11],
		    	      			wifPar.atten[12], wifPar.atten[13], wifPar.atten[14], wifPar.atten[15],
		    	      			wifPar.sb[8], wifPar.sb[9], wifPar.sb[10], wifPar.sb[11],
		    	      			wifPar.sb[12], wifPar.sb[13], wifPar.sb[14], wifPar.sb[15]);
	    		  } else {
	    			  sprintf(status, "%sNo report: LNA power is not on.\r\n", statusERR);
	              }
	    	  } else if (!strcasecmp(state, "wif")) {
      			  OSTimeDly(CMDDELAY);
	    	      rtn = argus_readWIF();         // update total power and temperature in status table
	    	      rtn += argus_readWIFpsADCs();  // update power supply voltage in status table
	    	      sprintf(status, "%sWarm IF:\r\n"
	    	    		  "Supply voltages: %5.2f V, %5.2f V\r\n"
	    	    		  "Ch    TotPow       Atten    SB    Card T\r\n"
	    	    		  " 1   %8.4f V    %2d dB    %1d    %5.1f C\r\n"
	    	    		  " 2   %8.4f V    %2d dB    %1d    %5.1f C\r\n"
	    	    		  " 3   %8.4f V    %2d dB    %1d    %5.1f C\r\n"
	    	    		  " 4   %8.4f V    %2d dB    %1d    %5.1f C\r\n"
	    	    		  " 5   %8.4f V    %2d dB    %1d    %5.1f C\r\n"
	    	    		  " 6   %8.4f V    %2d dB    %1d    %5.1f C\r\n"
	    	    		  " 7   %8.4f V    %2d dB    %1d    %5.1f C\r\n"
	    	    		  " 8   %8.4f V    %2d dB    %1d    %5.1f C\r\n"
	    	    		  " 9   %8.4f V    %2d dB    %1d    %5.1f C\r\n"
	    	    		  "10   %8.4f V    %2d dB    %1d    %5.1f C\r\n"
	    	    		  "11   %8.4f V    %2d dB    %1d    %5.1f C\r\n"
	    	    		  "12   %8.4f V    %2d dB    %1d    %5.1f C\r\n"
	    	    		  "13   %8.4f V    %2d dB    %1d    %5.1f C\r\n"
	    	    		  "14   %8.4f V    %2d dB    %1d    %5.1f C\r\n"
	    	    		  "15   %8.4f V    %2d dB    %1d    %5.1f C\r\n"
	    	    		  "16   %8.4f V    %2d dB    %1d    %5.1f C\r\n",
	    	    		  (rtn==0 ? statusOK : statusERR), wifPar.psv[0], wifPar.psv[1],
	    	    		  wifPar.totPow[0], wifPar.atten[0], wifPar.sb[0], wifPar.cardTemp[0],
	    	    		  wifPar.totPow[1], wifPar.atten[1], wifPar.sb[1], wifPar.cardTemp[1],
	    	    		  wifPar.totPow[2], wifPar.atten[2], wifPar.sb[2], wifPar.cardTemp[2],
	    	    		  wifPar.totPow[3], wifPar.atten[3], wifPar.sb[3], wifPar.cardTemp[3],
	    	    		  wifPar.totPow[4], wifPar.atten[4], wifPar.sb[4], wifPar.cardTemp[4],
	    	    		  wifPar.totPow[5], wifPar.atten[5], wifPar.sb[5], wifPar.cardTemp[5],
	    	    		  wifPar.totPow[6], wifPar.atten[6], wifPar.sb[6], wifPar.cardTemp[6],
	    	    		  wifPar.totPow[7], wifPar.atten[7], wifPar.sb[7], wifPar.cardTemp[7],
	    	    		  wifPar.totPow[8], wifPar.atten[8], wifPar.sb[8], wifPar.cardTemp[8],
	    	    		  wifPar.totPow[9], wifPar.atten[9], wifPar.sb[9], wifPar.cardTemp[9],
	    	    		  wifPar.totPow[10], wifPar.atten[10], wifPar.sb[10], wifPar.cardTemp[10],
	    	    		  wifPar.totPow[11], wifPar.atten[11], wifPar.sb[11], wifPar.cardTemp[11],
	    	    		  wifPar.totPow[12], wifPar.atten[12], wifPar.sb[12], wifPar.cardTemp[12],
	    	    		  wifPar.totPow[13], wifPar.atten[13], wifPar.sb[13], wifPar.cardTemp[13],
	    	    		  wifPar.totPow[14], wifPar.atten[14], wifPar.sb[14], wifPar.cardTemp[14],
	    	    		  wifPar.totPow[15], wifPar.atten[15], wifPar.sb[15], wifPar.cardTemp[15]);
	    	  } else if (!strcasecmp(state, "cryo")) {
	    		  OSTimeDly(CMDDELAY);
	    		  rtn = argus_readThermADCs();
	    		  sprintf(status, "%sCryostat:\r\n%s%8.1f K\r\n%s%8.1f K\r\n%s%8.1f K\r\n"
	    		    			   "%s%8.1f K\r\n%s%8.1f K\r\n%s%8.1f K\r\n%s%8.1e Torr (%4.3f V)\r\n",
	    		    	    (rtn==0 ? statusOK : statusERR), cnames[0], cryoPar.cryoTemps[0], cnames[1], cryoPar.cryoTemps[1],
	    		    		cnames[2], cryoPar.cryoTemps[2], cnames[3], cryoPar.cryoTemps[3], cnames[4], cryoPar.cryoTemps[4],
	    		    		cnames[5], cryoPar.cryoTemps[5],
	    		    		cnames[6], (cryoPar.auxInputs[0] > 1 ? powf(10., cryoPar.auxInputs[0]-6.) : 0.),
	    		    		cryoPar.auxInputs[0]);
	    	  } else if (!strcasecmp(state, "pow")) {
	    		  OSTimeDly(CMDDELAY);
	    		  rtn = argus_readPwrADCs();
	    		  rtn += argus_readBCpsV();
	    		  rtn += argus_readWIFpsADCs();
	    		  sprintf(status, "%sPower control card:\r\n"
	    				  "Amplifiers +15V: %5.2f V;  -15V:     %5.2f V\r\n"
	    				  "Digital +5V:     %5.2f V;  Drains:    %5.2f V\r\n"
	    				  "Cold IF suppl.:  %5.2f V;  Switched:  %5.2f V; Curr.: %5.2f A\r\n"
	    				  "Warm IF #1 +5V:  %5.2f V;  Warm IF #2 %5.2f V\r\n"
	    				  "Cal sys supply:  %5.2f V\r\n"
	    				  "Chassis temp.:   %5.1f C\r\n\r\n"
	    				  "Bias card power monitor points in [V]:\r\n"
	    				  "         Card 1            Card 2            Card 3            Card 4\r\n"
    	      			  "+15: % 5.2f, % 5.2f,   % 5.2f, % 5.2f,   % 5.2f, % 5.2f,   % 5.2f, % 5.2f\r\n"
    	      			  "-15: % 5.2f, % 5.2f,   % 5.2f, % 5.2f,   % 5.2f, % 5.2f,   % 5.2f, % 5.2f\r\n"
    	      			  "VCC: % 6.2f, % 6.2f,   % 6.2f, % 6.2f,   % 6.2f, % 6.2f,   % 6.2f, % 6.2f\r\n"
    	      			  "VDS: % 6.2f, % 6.2f,   % 6.2f, % 6.2f,   % 6.2f, % 6.2f,   % 6.2f, % 6.2f\r\n\r\n",
	    				  (rtn==0 ? statusOK : statusERR), pwrCtrlPar[2], pwrCtrlPar[1], pwrCtrlPar[3],
	    				  pwrCtrlPar[0], pwrCtrlPar[5], pwrCtrlPar[6], pwrCtrlPar[7],
	    				  wifPar.psv[0], wifPar.psv[1],
	    				  pwrCtrlPar[4], pwrCtrlPar[8],
	    				  bcPar[0].v[0], bcPar[0].v[1], bcPar[1].v[0], bcPar[1].v[1],
	    				  bcPar[2].v[0], bcPar[2].v[1], bcPar[3].v[0], bcPar[3].v[1],
	    				  bcPar[0].v[2], bcPar[0].v[3], bcPar[1].v[2], bcPar[1].v[3],
	    				  bcPar[2].v[2], bcPar[2].v[3], bcPar[3].v[2], bcPar[3].v[3],
	    				  bcPar[0].v[4], bcPar[0].v[5], bcPar[1].v[4], bcPar[1].v[5],
	    				  bcPar[2].v[4], bcPar[2].v[5], bcPar[3].v[4], bcPar[3].v[5],
	    				  bcPar[0].v[6], bcPar[0].v[7], bcPar[1].v[6], bcPar[1].v[7],
	    				  bcPar[2].v[7], bcPar[2].v[7], bcPar[3].v[6], bcPar[3].v[7]);
    	  } else if (!strcasecmp(state, "pres")) {
    		  flash_t flashData;
    		  zpec_readFlash(&flashData);
	      	  sprintf(status, "%sStored bias values.  Voltages in [V]\r\n\r\n"
	      			  "          1               2               3               4\r\n"
	      			  "VG: %5.2f, %5.2f,   %5.2f, %5.2f,   %5.2f, %5.2f,   %5.2f, %5.2f\r\n"
	      			  "VD: %5.2f, %5.2f,   %5.2f, %5.2f,   %5.2f, %5.2f,   %5.2f, %5.2f\r\n"
	      			  "VM: %5.2f, %5.2f,   %5.2f, %5.2f,   %5.2f, %5.2f,   %5.2f, %5.2f\r\n\r\n"
	      			  "          5               6               7               8\r\n"
	      			  "VG: %5.2f, %5.2f,   %5.2f, %5.2f,   %5.2f, %5.2f,   %5.2f, %5.2f\r\n"
	      			  "VD: %5.2f, %5.2f,   %5.2f, %5.2f,   %5.2f, %5.2f,   %5.2f, %5.2f\r\n"
	      			  "VM: %5.2f, %5.2f,   %5.2f, %5.2f,   %5.2f, %5.2f,   %5.2f, %5.2f\r\n\r\n"
	      			  "          9               10              11              12\r\n"
	      			  "VG: %5.2f, %5.2f,   %5.2f, %5.2f,   %5.2f, %5.2f,   %5.2f, %5.2f\r\n"
	      			  "VD: %5.2f, %5.2f,   %5.2f, %5.2f,   %5.2f, %5.2f,   %5.2f, %5.2f\r\n"
	      			  "VM: %5.2f, %5.2f,   %5.2f, %5.2f,   %5.2f, %5.2f,   %5.2f, %5.2f\r\n\r\n"
	      			  "          13              14              15              16\r\n"
	      			  "VG: %5.2f, %5.2f,   %5.2f, %5.2f,   %5.2f, %5.2f,   %5.2f, %5.2f\r\n"
	      			  "VD: %5.2f, %5.2f,   %5.2f, %5.2f,   %5.2f, %5.2f,   %5.2f, %5.2f\r\n"
	      			  "VM: %5.2f, %5.2f,   %5.2f, %5.2f,   %5.2f, %5.2f,   %5.2f, %5.2f\r\n\r\n"

	      			  "A: %2d, %2d, %2d, %2d,   %2d, %2d, %2d, %2d\r\n"
	      			  "S: %2d, %2d, %2d, %2d,   %2d, %2d, %2d, %2d\r\n"
	      			  "A: %2d, %2d, %2d, %2d,   %2d, %2d, %2d, %2d\r\n"
	      			  "S: %2d, %2d, %2d, %2d,   %2d, %2d, %2d, %2d\r\n\r\n",

	      			  statusOK,
	      			  flashData.LNAsets[0], flashData.LNAsets[1], flashData.LNAsets[6], flashData.LNAsets[7],
	      			  flashData.LNAsets[12], flashData.LNAsets[13], flashData.LNAsets[18], flashData.LNAsets[19],
	      			  flashData.LNAsets[2], flashData.LNAsets[3], flashData.LNAsets[8], flashData.LNAsets[9],
	      			  flashData.LNAsets[14], flashData.LNAsets[15], flashData.LNAsets[20], flashData.LNAsets[21],
	      			  flashData.LNAsets[4], flashData.LNAsets[5], flashData.LNAsets[10], flashData.LNAsets[11],
	      			  flashData.LNAsets[16], flashData.LNAsets[17], flashData.LNAsets[22], flashData.LNAsets[23],

	      			  flashData.LNAsets[24], flashData.LNAsets[25], flashData.LNAsets[30], flashData.LNAsets[31],
	      			  flashData.LNAsets[36], flashData.LNAsets[37], flashData.LNAsets[42], flashData.LNAsets[43],
	      			  flashData.LNAsets[26], flashData.LNAsets[27], flashData.LNAsets[32], flashData.LNAsets[33],
	      			  flashData.LNAsets[38], flashData.LNAsets[39], flashData.LNAsets[44], flashData.LNAsets[45],
	      			  flashData.LNAsets[28], flashData.LNAsets[29], flashData.LNAsets[34], flashData.LNAsets[35],
	      			  flashData.LNAsets[40], flashData.LNAsets[41], flashData.LNAsets[46], flashData.LNAsets[47],
			  
	      			  flashData.LNAsets[48], flashData.LNAsets[49], flashData.LNAsets[54], flashData.LNAsets[55],
	      			  flashData.LNAsets[60], flashData.LNAsets[61], flashData.LNAsets[66], flashData.LNAsets[67],
	      			  flashData.LNAsets[50], flashData.LNAsets[51], flashData.LNAsets[56], flashData.LNAsets[57],
	      			  flashData.LNAsets[62], flashData.LNAsets[63], flashData.LNAsets[68], flashData.LNAsets[69],
	      			  flashData.LNAsets[52], flashData.LNAsets[53], flashData.LNAsets[58], flashData.LNAsets[59],
	      			  flashData.LNAsets[64], flashData.LNAsets[65], flashData.LNAsets[70], flashData.LNAsets[71],
			  
	      			  flashData.LNAsets[72], flashData.LNAsets[73], flashData.LNAsets[78], flashData.LNAsets[79],
	      			  flashData.LNAsets[84], flashData.LNAsets[85], flashData.LNAsets[90], flashData.LNAsets[91],
	      			  flashData.LNAsets[74], flashData.LNAsets[75], flashData.LNAsets[80], flashData.LNAsets[81],
	      			  flashData.LNAsets[86], flashData.LNAsets[87], flashData.LNAsets[92], flashData.LNAsets[93],
	      			  flashData.LNAsets[76], flashData.LNAsets[77], flashData.LNAsets[82], flashData.LNAsets[83],
	      			  flashData.LNAsets[88], flashData.LNAsets[89], flashData.LNAsets[94], flashData.LNAsets[95],

	      			  flashData.atten[0], flashData.atten[1], flashData.atten[2], flashData.atten[3],
	      			  flashData.atten[4], flashData.atten[5], flashData.atten[6], flashData.atten[7],
	      			  flashData.sb[0], flashData.sb[1], flashData.sb[2], flashData.sb[3],
	      			  flashData.sb[4], flashData.sb[5], flashData.sb[6], flashData.sb[7],
	      			  flashData.atten[8], flashData.atten[9], flashData.atten[10], flashData.atten[11],
	      			  flashData.atten[12], flashData.atten[13], flashData.atten[14], flashData.atten[15],
	      			  flashData.sb[8], flashData.sb[9], flashData.sb[10], flashData.sb[11],
	      			  flashData.sb[12], flashData.sb[13], flashData.sb[14], flashData.sb[15]);


	      } else if (!strcasecmp(state, "yig")) {
	    	  OSTimeDly(CMDDELAY);
	    	  int rtn = read_all_muBox_ADC();  // update values in muBoxPar structure
	    	  float fextrapol = (muBoxPar.setval-muBoxPar.intercept)/muBoxPar.slope;
	      	  sprintf(status, "%sYIG values:\r\n"
	      			"LO power detector voltage:  %8.3f V (max voltage: %5.3f, threshold %5.3f)\r\n"
	      			"Mii heater current readout: %8.3f V\r\n"
	      			"YIG temperature readout:    %8.1f C\r\n"
	      			"Interface card voltage:     %8.3f V (15 V nominal)\r\n"
	      			"YIG set word %u\r\n"
	      			"Command freq %6.3f, estimated freq %6.3f GHz, delta %6.3f\r\n"
	      			"Status %d\r\n",
	      			( (rtn==0  &&  muBoxPar.adcv[YIGPWRCHAN] > YIGPTHRESH ) ? statusOK : statusERR),   //check pwr ok
	      			muBoxPar.adcv[0], muBoxPar.pdetVmax, YIGPTHRESH,
	      			muBoxPar.adcv[1], muBoxPar.adcv[2]*100, muBoxPar.adcv[3],
	      			muBoxPar.setval, muBoxPar.freq, fextrapol, fextrapol-muBoxPar.freq, rtn);
	      } else if (!strcasecmp(state, "vane")) {
	    	OSTimeDly(CMDDELAY);
	      	rtn = argus_readAllCalSysADC();
	      	sprintf(status, "%sVane parameters:\r\n"
	      			"Temperature: %.1f [C] \r\nState: %s \r\n"
	      			"Angle: %.3f [V] \r\n"
	      			"Curr: %.2f [A] \r\n",
	      			( (rtn==0 && calSysPar.adcv[0] < 10.) ? statusOK : statusERR),  // check for ADC 99s
	       		   calSysPar.adcv[2], calSysPar.state,
	      		   calSysPar.adcv[0],
	      		   calSysPar.adcv[1]);
	      } else {  // invalid argument
    	  	  longHelp(status, usage, &Correlator::execArgusMonPts);
	      }
	      }
	      } else {  // no argument; send LNA bias readout to screen
     		  if (lnaPwrState) {
     			  OSTimeDly(CMDDELAY);
    	      	  rtn += argus_readPwrADCs();
    			  rtn += argus_readLNAbiasADCs("vg");
    	      	  rtn += argus_readLNAbiasADCs("vd");
    	      	  rtn += argus_readLNAbiasADCs("id");

    	      	  sprintf(status, "%sLNA power state %s.\r\nSupplies: +15V: %5.2f V; "
    	      			  "-15V: %5.2f V; +5V: %5.2f V\r\n"
    	      			  "Voltages in [V], currents in [mA]\r\n\r\n"
    	      			  "          1               2               3               4\r\n"
    	      			  "VG: %5.2f, %5.2f,   %5.2f, %5.2f,   %5.2f, %5.2f,   %5.2f, %5.2f\r\n"
    	      			  "VD: %5.2f, %5.2f,   %5.2f, %5.2f,   %5.2f, %5.2f,   %5.2f, %5.2f\r\n"
    	      			  "ID: %5.1f, %5.1f,   %5.1f, %5.1f,   %5.1f, %5.1f,   %5.1f, %5.1f\r\n\r\n"
    	      			  "          5               6               7               8\r\n"
    	      			  "VG: %5.2f, %5.2f,   %5.2f, %5.2f,   %5.2f, %5.2f,   %5.2f, %5.2f\r\n"
    	      			  "VD: %5.2f, %5.2f,   %5.2f, %5.2f,   %5.2f, %5.2f,   %5.2f, %5.2f\r\n"
    	      			  "ID: %5.1f, %5.1f,   %5.1f, %5.1f,   %5.1f, %5.1f,   %5.1f, %5.1f\r\n\r\n"
    	      			  "          9               10              11              12\r\n"
    	      			  "VG: %5.2f, %5.2f,   %5.2f, %5.2f,   %5.2f, %5.2f,   %5.2f, %5.2f\r\n"
    	      			  "VD: %5.2f, %5.2f,   %5.2f, %5.2f,   %5.2f, %5.2f,   %5.2f, %5.2f\r\n"
    	      			  "ID: %5.1f, %5.1f,   %5.1f, %5.1f,   %5.1f, %5.1f,   %5.1f, %5.1f\r\n\r\n"
    	      			  "          13              14              15              16\r\n"
    	      			  "VG: %5.2f, %5.2f,   %5.2f, %5.2f,   %5.2f, %5.2f,   %5.2f, %5.2f\r\n"
    	      			  "VD: %5.2f, %5.2f,   %5.2f, %5.2f,   %5.2f, %5.2f,   %5.2f, %5.2f\r\n"
    	      			  "ID: %5.1f, %5.1f,   %5.1f, %5.1f,   %5.1f, %5.1f,   %5.1f, %5.1f\r\n\r\n",
    	      			  (rtn==0 ? statusOK : statusERR), (lnaPwrState==1 ? "ON" : "OFF"),
    	      			  pwrCtrlPar[2], pwrCtrlPar[1], pwrCtrlPar[0],  //pv, nv, vds
    	      			  rxPar[0].LNAmonPts[0], rxPar[0].LNAmonPts[1], rxPar[1].LNAmonPts[0], rxPar[1].LNAmonPts[1],
    	      			  rxPar[2].LNAmonPts[0], rxPar[2].LNAmonPts[1], rxPar[3].LNAmonPts[0], rxPar[3].LNAmonPts[1],
    	      			  rxPar[0].LNAmonPts[2], rxPar[0].LNAmonPts[3], rxPar[1].LNAmonPts[2], rxPar[1].LNAmonPts[3],
    	      			  rxPar[2].LNAmonPts[2], rxPar[2].LNAmonPts[3], rxPar[3].LNAmonPts[2], rxPar[3].LNAmonPts[3],
    	      			  rxPar[0].LNAmonPts[4], rxPar[0].LNAmonPts[5], rxPar[1].LNAmonPts[4], rxPar[1].LNAmonPts[5],
    	      			  rxPar[2].LNAmonPts[4], rxPar[2].LNAmonPts[5], rxPar[3].LNAmonPts[4], rxPar[3].LNAmonPts[5],

    	      			  rxPar[4].LNAmonPts[0], rxPar[4].LNAmonPts[1], rxPar[5].LNAmonPts[0], rxPar[5].LNAmonPts[1],
    	      			  rxPar[6].LNAmonPts[0], rxPar[6].LNAmonPts[1], rxPar[7].LNAmonPts[0], rxPar[7].LNAmonPts[1],
    	      			  rxPar[4].LNAmonPts[2], rxPar[4].LNAmonPts[3], rxPar[5].LNAmonPts[2], rxPar[5].LNAmonPts[3],
    	      			  rxPar[6].LNAmonPts[2], rxPar[6].LNAmonPts[3], rxPar[7].LNAmonPts[2], rxPar[7].LNAmonPts[3],
    	      			  rxPar[4].LNAmonPts[4], rxPar[4].LNAmonPts[5], rxPar[5].LNAmonPts[4], rxPar[5].LNAmonPts[5],
    	      			  rxPar[6].LNAmonPts[4], rxPar[6].LNAmonPts[5], rxPar[7].LNAmonPts[4], rxPar[7].LNAmonPts[5],

    	      			  rxPar[8].LNAmonPts[0],  rxPar[8].LNAmonPts[1],  rxPar[9].LNAmonPts[0],  rxPar[9].LNAmonPts[1],
    	      			  rxPar[10].LNAmonPts[0], rxPar[10].LNAmonPts[1], rxPar[11].LNAmonPts[0], rxPar[11].LNAmonPts[1],
    	      			  rxPar[8].LNAmonPts[2],  rxPar[8].LNAmonPts[3],  rxPar[9].LNAmonPts[2],  rxPar[9].LNAmonPts[3],
    	      			  rxPar[10].LNAmonPts[2], rxPar[10].LNAmonPts[3], rxPar[11].LNAmonPts[2], rxPar[11].LNAmonPts[3],
    	      			  rxPar[8].LNAmonPts[4],  rxPar[8].LNAmonPts[5],  rxPar[9].LNAmonPts[4],  rxPar[9].LNAmonPts[5],
    	      			  rxPar[10].LNAmonPts[4], rxPar[10].LNAmonPts[5], rxPar[11].LNAmonPts[4], rxPar[11].LNAmonPts[5],

    	      			  rxPar[12].LNAmonPts[0], rxPar[12].LNAmonPts[1], rxPar[13].LNAmonPts[0], rxPar[13].LNAmonPts[1],
    	      			  rxPar[14].LNAmonPts[0], rxPar[14].LNAmonPts[1], rxPar[15].LNAmonPts[0], rxPar[15].LNAmonPts[1],
    	      			  rxPar[12].LNAmonPts[2], rxPar[12].LNAmonPts[3], rxPar[13].LNAmonPts[2], rxPar[13].LNAmonPts[3],
    	      			  rxPar[14].LNAmonPts[2], rxPar[14].LNAmonPts[3], rxPar[15].LNAmonPts[2], rxPar[15].LNAmonPts[3],
    	      			  rxPar[12].LNAmonPts[4], rxPar[12].LNAmonPts[5], rxPar[13].LNAmonPts[4], rxPar[13].LNAmonPts[5],
    	      			  rxPar[14].LNAmonPts[4], rxPar[14].LNAmonPts[5], rxPar[15].LNAmonPts[4], rxPar[15].LNAmonPts[5]);
    		  } else {
    			  sprintf(status, "%sNo report: LNA power is not on.\r\n", statusERR);
    		  }
	      }
	  } else {
		  longHelp(status, usage, &Correlator::execArgusMonPts);
	  }
}

/*************************************************************************************/
/**
  \brief Argus lock test command.

  This method tests the lockout scheme.

  \param status Storage buffer for return status (should contain at least
                ControlService::maxLine characters).
  \param arg    Argument list: LEVEL
*/

void Correlator::execArgusLock(return_type status, argument_type arg)
{
  static const char *usage =
  "\r\n";

  int rtn;

  if (!arg.help) {
	  int busy = 1;  // i2cBusBusy value for all tests
	  int lnaps = 0; // LNA power state; set to 1 to reach bus lock tests for LNA, 0 for CIF

	  i2cBusBusy = busy;
	  rtn = argus_readAllSystemADCs();
	  iprintf("i2cBusBusy = %u, rtn = %d for argus_readAllSystemADCs()\r\n", i2cBusBusy, rtn);

	  i2cBusBusy = busy;
	  rtn = argus_readPwrADCs();
	  iprintf("i2cBusBusy = %u, rtn = %d for argus_readPwrADCs();\r\n", i2cBusBusy, rtn);

	  i2cBusBusy = busy;
	  rtn = argus_readBCpsV();
	  iprintf("i2cBusBusy = %u, rtn = %d for argus_readBCpsV();\r\n", i2cBusBusy, rtn);

	  i2cBusBusy = busy;
	  rtn = argus_readThermADCs();
	  iprintf("i2cBusBusy = %u, rtn = %d for argus_readThermADCs();\r\n", i2cBusBusy, rtn);

	  i2cBusBusy = busy;
	  rtn = argus_readLNAbiasADCs("vg");
	  iprintf("i2cBusBusy = %u, rtn = %d for argus_readLNAbiasADCs(vg);\r\n", i2cBusBusy, rtn);

	  i2cBusBusy = busy;
	  rtn = argus_readLNAbiasADCs("vd");
	  iprintf("i2cBusBusy = %u, rtn = %d for argus_readLNAbiasADCs(vd);\r\n", i2cBusBusy, rtn);

	  i2cBusBusy = busy;
	  rtn = argus_readLNAbiasADCs("id");
	  iprintf("i2cBusBusy = %u, rtn = %d for argus_readLNAbiasADCs(id);\r\n", i2cBusBusy, rtn);

	  i2cBusBusy = busy;
	  rtn = argus_readLNAbiasADCs("vm");
	  iprintf("i2cBusBusy = %u, rtn = %d for argus_readLNAbiasADCs(vm);\r\n", i2cBusBusy, rtn);

	  i2cBusBusy = busy;
	  rtn = argus_readLNAbiasADCs("im");
	  iprintf("i2cBusBusy = %u, rtn = %d for argus_readLNAbiasADCs(im);\r\n", i2cBusBusy, rtn);

	  i2cBusBusy = busy;
	  rtn = argus_readAllCalSysADC();
	  iprintf("i2cBusBusy = %u, rtn = %d for argus_readAllCalSysADC();\r\n", i2cBusBusy, rtn);

	  i2cBusBusy = busy;
	  rtn = read_all_muBox_ADC();
	  iprintf("i2cBusBusy = %u, rtn = %d for read_all_muBox_ADC()\r\n", i2cBusBusy, rtn);

	  i2cBusBusy = busy;
	  rtn = argus_readWIF();
	  iprintf("i2cBusBusy = %u, rtn = %d for argus_readWIF()\r\n", i2cBusBusy, rtn);

	  i2cBusBusy = busy;
	  rtn = yig_freq(100, 10, 10, .02);
	  iprintf("i2cBusBusy = %u, rtn = %d for yig_freq(100, 10, 10, .02)\r\n", i2cBusBusy, rtn);

	  i2cBusBusy = busy;
	  rtn = yig_peak(100, 10, 10, .02);
	  iprintf("i2cBusBusy = %u, rtn = %d for yig_peak(100, 10, 10, .02)\r\n", i2cBusBusy, rtn);

	  i2cBusBusy = busy;
	  rtn = argus_setWIFswitches("a", 3, 20, 0);
	  iprintf("i2cBusBusy = %u, rtn = %d for argus_setWIFswitches(a, 3, 20, 0)\r\n", i2cBusBusy, rtn);

	  i2cBusBusy = busy;
	  rtn = argus_setWIFswitches("s", 2, 1, 0);
	  iprintf("i2cBusBusy = %u, rtn = %d for argus_setWIFswitches(s, 2, 1, 0) \r\n", i2cBusBusy, rtn);

	  i2cBusBusy = busy;
	  rtn = argus_setAllWIFswitches("a", 10);
	  iprintf("i2cBusBusy = %u, rtn = %d for argus_setAllWIFswitches(a, 10)\r\n", i2cBusBusy, rtn);

	  lnaPwrState = lnaps;
	  i2cBusBusy = busy;
	  rtn = argus_setLNAbias("d", 2, 1, .5, 0);
	  iprintf("i2cBusBusy = %u, rtn = %d for argus_setLNAbias(d, 2, 1, .5, 0)\r\n", i2cBusBusy, rtn);

	  lnaPwrState = lnaps;
	  i2cBusBusy = busy;
	  rtn = argus_setLNAbias("g", 2, 1, .5, 0);
	  iprintf("i2cBusBusy = %u, rtn = %d for argus_setLNAbias(g, 2, 1, .5, 0)\r\n", i2cBusBusy, rtn);

	  lnaPwrState = lnaps;
	  i2cBusBusy = busy;
	  rtn = argus_setLNAbias("m", 2, 1, .5, 0);
	  iprintf("i2cBusBusy = %u, rtn = %d for argus_setLNAbias(m, 2, 1, .5, 0)\r\n", i2cBusBusy, rtn);

	  lnaPwrState = lnaps;
	  i2cBusBusy = busy;
	  rtn = argus_setAllBias("d", 0.5, 0);
	  iprintf("i2cBusBusy = %u, rtn = %d for argus_setAllBias(d, 0.5, 0)\r\n", i2cBusBusy, rtn);

	  lnaPwrState = lnaps;
	  i2cBusBusy = busy;
	  rtn = argus_lnaPower(1);
	  iprintf("i2cBusBusy = %u, rtn = %d for argus_lnaPower(1)\r\n", i2cBusBusy, rtn);

	  lnaPwrState = lnaps;
	  i2cBusBusy = busy;
	  rtn = argus_cifPower(1);
	  iprintf("i2cBusBusy = %u, rtn = %d for argus_cifPower(1)\r\n", i2cBusBusy, rtn);

	  sprintf(status, "# I2C bus lock test results output to UART0.\r\n");

  } else {
    longHelp(status, usage, &Correlator::execArgusLock);
  }
}
