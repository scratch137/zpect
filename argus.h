#ifndef ARGUS_H
#define ARGUS_H
/*
   \file
   \author Andy Harris
   \brief  Argus backend declarations.

   $Id: argus.h,v 1.5 2014/06/04 18:35:05 harris Exp $
*/
#include "zpec.h"
#include "argusHardwareStructs.h"

#ifdef __cplusplus
  // Any C++ declarations can go here...
extern struct receiverParams rxPar[];
extern struct cryostatParams cryoPar;
extern struct biasCardParams bcPar[];
extern struct warmIFparams wifPar;
extern struct muBoxParams muBoxPar;
extern struct calSysParams calSysPar;
extern float pwrCtrlPar[];
extern struct chSet *chSetPtr;
extern struct chRead *chReadPtr;
extern struct chRead2 *chRead2Ptr;

extern "C" {
#endif

/* All C declarations in this region. */

extern char lnaPwrState;  // LNA power supply state
extern char cifPwrState;  // Cold IF power state
extern unsigned char lnaPSlimitsBypass; // bypass LNA power supply limits when = 1
extern unsigned char cifPSlimitsBypass; // bypass cold IF power supply limits when = 1
extern unsigned char lnaLimitsBypass;   // bypass soft limits on LNA bias when = 1
extern unsigned char stopVaneOnStall;   // bypass timeout on vane stall = 0
extern float gvdiv;                     // Gate voltage divider factor
extern unsigned char i2cBusBusy;        // I2C bus is busy when = 1
extern unsigned int busLockCtr;         // I2C successful bus lock request counter
extern unsigned int busNoLockCtr;       // I2C unsuccessful bus lock request counter
extern unsigned char freezeSys;         // freeze system state when = 1
extern unsigned int freezeCtr;          // freeze request counter
extern unsigned int thawCtr;            // thaw request counter
extern unsigned int freezeErrCtr;       // freeze error counter (access request while frozen)
extern short unsigned int biasSatus[NRX]; // receiver status word, see argus_rxCheck(void)
extern int i2cState[];                  // I2C bus SCL (0/1) and SDA (0/2) values

extern void argus_init(const flash_t *flash);
extern int  argus_test(int foo, float bar);
extern int  argus_setLNAbias(char *term, int m, int n, float v, unsigned char busyOverride);
extern int  argus_setAllBias(char *inp, float v, unsigned char busyOverride);
extern int  argus_lnaPower(short state);
extern int  argus_cifPower(short state);
extern int  argus_readLNAbiasADCs(char *sw);
extern int  argus_readPwrADCs(void);
extern int  argus_readBCpsV(void);
extern int  argus_readThermADCs(void);
extern int  argus_LNApresets(const flash_t *flash);
extern unsigned char argus_lnaPowerPIO(void);
extern int  argus_readWIFpsADCs(void);
extern int  argus_setWIFswitches(char *term, int m, char val, unsigned char busyOverride);
extern int  argus_setAllWIFswitches(char *inp, char val);
extern int  argus_readWIF(void);
extern int  argus_openSubbusC(BYTE addr);
extern int  argus_closeSubbusC(void);
extern int  read_all_muBox_ADC(void);
extern int  yig_set(unsigned short int val);
extern int  yig_peak(unsigned short int ctr, unsigned short int step, int maxn, float pThresh);
extern int  yig_freq(float freq, unsigned short int step, int maxn, float pThresh);
extern int  readCalSysADC(int ch);
extern int  argus_readAllCalSysADC(void);
extern int  argus_setVaneBits(BYTE cmd);
extern int  argus_driveVane(BYTE cmd, short int nmax, float stopv, float deltastop);
extern int  argus_clearBus(void);
extern int  argus_readAllSystemADCs(void);
extern int  argus_biasCheck(void);
extern int  argus_powCheck(void);
extern int  argus_thermCheck(void);
extern int  argus_ifCheck(void);

/**************************************************************************/


#ifdef __cplusplus
}  // extern "C"
#endif

#endif  /* ARGUS_H */
