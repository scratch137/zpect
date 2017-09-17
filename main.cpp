/**
  \file

  Top level Netburner application file.

  $Id: main.cpp,v 1.36 2014/06/04 18:36:44 harris Exp $
*/
#include "predef.h"

#include <stdio.h>
#include <string.h>

#include <startnet.h>
#include <autoupdate.h>
#include <dhcpclient.h>
#include <taskmon.h>
#include <smarttrap.h>
#include <utils.h>

#include "control.h"
#include "zpec.h"


/// Netburner application name.
const char *AppName = "Zpectrometer";

/// The Zpectrometer (two bands per CPU, 2 buffers of 128 lags per band).
 Correlator zpectrometer(2, 2, 128);

/**
  \var Correlator::shell_type zpecShell
  Zpectrometer command interpreter.
*/
Correlator::shell_type zpecShell(&zpectrometer);


/// Initializes networking.
void startNetwork()
{
  static const char *fn = "startNetwork";

  // Initialize TCP/IP.
  InitializeStack();
  if (EthernetIP == 0) {
    char ipstr[16];

    zpec_info("Obtaining IP address..");
    if (GetDHCPAddress() == DHCP_OK) {
      zpec_info("DHCP assigned IP address %s", zpec_iptostr(ipstr, EthernetIP));
    } else {
      zpec_error_fn("DHCP failed; no IP address");
    }
  }

  // Enable updates and debugging via network, and start web server.
  EnableAutoUpdate();
  EnableTaskMonitor();
  EnableSmartTraps();
  StartHTTP();
}

/// Adds control functions to the command dictionary.
void initCommandShell()
{
  /*
     The hardware type is used to initialize only those commands which
     are meaningful for the particular hardware variant.
  */
  flash_t flashData;
  zpec_readFlash(&flashData);
  zpec_hw_t hw = (flashData.valid ? flashData.hw : ZPEC_HW_GBT);

  switch (hw) {
    case ZPEC_HW_GBT:
    case ZPEC_HW_RLT:
      ::zpecShell["b"]         = &Correlator::execBoss;
      ::zpecShell["boss"]      = &Correlator::execBoss;
      ::zpecShell["master"]    = &Correlator::execBoss;

      ::zpecShell["d"]         = &Correlator::execDiodeObs;
      ::zpecShell["dobs"]      = &Correlator::execDiodeObs;

      ::zpecShell["e"]         = &Correlator::execMode;
      ::zpecShell["eval"]      = &Correlator::execMode;
      ::zpecShell["mode"]      = &Correlator::execMode;

      ::zpecShell["halt"]      = &Correlator::execHalt;

      ::zpecShell["i"]         = &Correlator::execInitADCs;
      ::zpecShell["initADCs"]  = &Correlator::execInitADCs;

      ::zpecShell["l"]         = &Correlator::execLevel;
      ::zpecShell["level"]     = &Correlator::execLevel;

      ::zpecShell["m"]         = &Correlator::execStatsObs;
      ::zpecShell["meanvar"]   = &Correlator::execStatsObs;

      ::zpecShell["o"]         = &Correlator::execScopeObs;
      ::zpecShell["scope"]     = &Correlator::execScopeObs;

      ::zpecShell["s"]         = &Correlator::execSend;
      ::zpecShell["send"]      = &Correlator::execSend;

      ::zpecShell["sync"]      = &Correlator::execSync;

      ::zpecShell["t"]         = &Correlator::execTotalPower;
      ::zpecShell["totpwr"]    = &Correlator::execTotalPower;

      ::zpecShell["z"]         = &Correlator::execZero;
      ::zpecShell["zero"]      = &Correlator::execZero;
      break;

    case ZPEC_HW_POW:
      ::zpecShell["power"]     = &Correlator::execPower;
      break;

    case ZPEC_HW_ARG:
      //::zpecShell["test"]      = &Correlator::execArgusTest;
      ::zpecShell["limits"]    = &Correlator::execArgusLimits;
      ::zpecShell["all"]       = &Correlator::execArgusSetAll;
      ::zpecShell["g"]         = &Correlator::execArgusGate;
      ::zpecShell["d"]         = &Correlator::execArgusDrain;
      ::zpecShell["m"]         = &Correlator::execArgusMixer;
      ::zpecShell["lna"]       = &Correlator::execArgusPwrCtrl;
      ::zpecShell["cryo"]      = &Correlator::execArgusCryo;
      ::zpecShell["c"]         = &Correlator::execArgusCryo;
      ::zpecShell["mon"]       = &Correlator::execArgusMonPts;
      ::zpecShell["presets"]   = &Correlator::execArgusPresets;
      ::zpecShell["cif"]       = &Correlator::execArgusCIFPwrCtrl;
      ::zpecShell["engr"]      = &Correlator::execArgusEngr;
      ::zpecShell["wif"]       = &Correlator::execArgusWIFCtrl;
      ::zpecShell["a"]         = &Correlator::execArgusAtten;
      ::zpecShell["s"]         = &Correlator::execArgusSB;
      ::zpecShell["yig"]       = &Correlator::execArgusYIG;
      ::zpecShell["vane"]      = &Correlator::execArgusVane;
      ::zpecShell["check"]     = &Correlator::execArgusRxHealth;
      ::zpecShell["freeze"]    = &Correlator::execArgusFreeze;
      ::zpecShell["thaw"]      = &Correlator::execArgusThaw;
      //::zpecShell["lock"]    = &Correlator::execArgusLock;
      break;

    default:
      break;
  }

  // The following commands are available on all hardware variants.
  ::zpecShell["flash"]     = &Correlator::execFlash;

  ::zpecShell[""]          = &Correlator::execHelp;
  ::zpecShell["?"]         = &Correlator::execHelp;
  ::zpecShell["h"]         = &Correlator::execHelp;
  ::zpecShell["help"]      = &Correlator::execHelp;

  if (hw != ZPEC_HW_ARG) {
    ::zpecShell["peek"]      = &Correlator::execPeek;
    ::zpecShell["poke"]      = &Correlator::execPoke;

    ::zpecShell["q"]         = &Correlator::execQuery;
    ::zpecShell["query"]     = &Correlator::execQuery;

    ::zpecShell["status"]    = &Correlator::execStatus;

    ::zpecShell["v"]         = &Correlator::execVersion;
    ::zpecShell["version"]   = &Correlator::execVersion;
  }

  ::zpecShell["\x04"]      = &Correlator::execQuit;  // ^D
  ::zpecShell["\xFF\x0C"]  = &Correlator::execQuit;  // Telnet ^D [#1]
  ::zpecShell["\xFF\xEC"]  = &Correlator::execQuit;  // Telnet ^D [#2]
  ::zpecShell["quit"]      = &Correlator::execQuit;

  ::zpecShell["reboot"]    = &Correlator::execReboot;

  ::zpecShell["time"]      = &Correlator::execTime;

  ::zpecShell["verbose"]   = &Correlator::execVerbose;
}

extern "C" {

/**
  Netburner application entry point.

  \param pd uC/OS program input data (not used).
*/
void UserMain(void *pd)
{
  // Initialize correlator structure.
  ::zpectrometer.initState();

  zpec_info("Zpectrometer initializing..");

  // Start networking and obtain IP address if necessary.
  startNetwork();

  // Load command dictionary.
  initCommandShell();

  // Initialize hardware and services, and begin operation.
  // Does not return.
  ::zpectrometer.boot(&::zpecShell, &::gio);

  // Shouldn't get here.
  zpec_error("Sorry, I've lost control!! It's been %3u s since boot", ::Secs);
}

}  // extern "C"
