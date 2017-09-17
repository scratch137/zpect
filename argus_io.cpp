/**
  \file
  \author Andy Harris
  \brief  Low-level Argus hardware input/output routines.

  $Id: argus_io.cpp,v 1.2 2014/06/04 18:51:10 harris Exp $
*/
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include <ctype.h>
#include <taskmon.h>
#include <smarttrap.h>

#include "i2cmulti.h"
//Overwrite values in 12cmulti.h
#define I2C_RX_TX_TIMEOUT (5)   // Ticks allowed before timeout of a single byte transmission; default 5
#define I2C_START_TIMEOUT (20)  // Ticks allowed before timeout when attempting start on I2C bus; default 20    

#include <pins.h>  // individual pin manipulation

#include "constants.h"

#include "argus.h"

//I2C setups
BYTE buffer[I2C_MAX_BUF_SIZE];
char I2CInputBuffer[I2C_MAX_BUF_SIZE];   // User created I2C input buffer
char* inbuf = I2CInputBuffer;            // Pointer to user I2C buffer
BYTE address = 0;
int I2CStat = 0;

//Initialize Argus-global state flags and variables
char lnaPwrState = 0;
char cifPwrState = 0;
unsigned char lnaPSlimitsBypass = BYPASS;  // bypass LNA power supply limits when = 1
unsigned char cifPSlimitsBypass = BYPASS;  // bypass cold IF power supply limits when = 1
unsigned char lnaLimitsBypass = BYPASS;    // bypass soft limits on LNA bias when = 1
unsigned char stopVaneOnStall = !BYPASS;   // bypass timeout on vane stall when = 0
float gvdiv;
unsigned char i2cBusBusy = 1;              // set to 1 when I2C bus is busy (clears in argus_init())
unsigned int busLockCtr = 0;               // I2C successful bus lock request counter
unsigned int busNoLockCtr = 0;             // I2C unsuccessful bus lock request counter
unsigned char freezeSys =  0;              // freeze system state when = 1
unsigned int freezeCtr = 0;                // freeze request counter
unsigned int thawCtr = 0;                  // thaw request counter
unsigned int freezeErrCtr = 0;             // freeze error counter (access request while frozen)
int i2cState[2];                           // I2C bus SCL (0/1) and SDA (0/2) values, before and after reset
unsigned int vaneErr;                      // vane motion has failed if vaneErr != 0

//Pointer defs
struct chSet *chSetPtr; // pointer to structure of form chSet
struct chRead *chReadPtr; // pointer to structure of form chRead
struct chRead2 *chRead2Ptr; // pointer to structure of form chRead

// control bits within power control board PIO
BYTE ctlVDS = 0x01;
BYTE ctlnVamp = 0x02;
BYTE ctlpVamp = 0x04;
BYTE ctlVCC = 0x08;
BYTE ctlVIF = 0x10;
BYTE FPLED = 0x20;
BYTE FPOn = 0x40;
BYTE FPOff = 0x80;

// hardware status words
short unsigned int biasSatus[NRX]; // receiver status word (see argus_rxCheck(void))
short unsigned int powStatus;     // power system status word (see argus_powCheck(void))

/****************************************************************************/
// storage structure and array definitions
/*struct receiverParams {
  int cardNo;           // bias card number: 0..3 for Argus (four cards)
  int bcChan[NSTAGES];  // channel no. within a bias card: bcChan 0..7
  float LNAsets[2*NSTAGES+NMIX];     // command values: gate, drain, mixer
  float LNAmonPts[NSTAGES+2*NSTAGES+2*NMIX];  // monitor points: gate V, drain V I, mixer V I
};
two receivers per board, each with
  LNAsets   index: vg 0 1; vd 2 3, vm 4 5
  LNAmonPts index: vg 0 1; vd 2 3; id 4 5; vm 6 7; im 8 9
*/
struct receiverParams rxPar[] = {
  //rx0:
  {0, {4, 5}, {0, 0, 0, 0, 0, 0},
   {99, 99, 99, 99, 99, 99, 99, 99, 99, 99}},
  //rx1:
  {0, {6, 7}, {0, 0, 0, 0, 0, 0},
   {99, 99, 99, 99, 99, 99, 99, 99, 99, 99}},
  //rx2:
  {0, {0, 1}, {0, 0, 0, 0, 0, 0},
   {99, 99, 99, 99, 99, 99, 99, 99, 99, 99}},
  //rx3:
  {0, {2, 3}, {0, 0, 0, 0, 0, 0},
   {99, 99, 99, 99, 99, 99, 99, 99, 99, 99}},
  //rx4:
  {1, {4, 5}, {0, 0, 0, 0, 0, 0},
   {99, 99, 99, 99, 99, 99, 99, 99, 99, 99}},
  //rx5:
  {1, {6, 7}, {0, 0, 0, 0, 0, 0},
   {99, 99, 99, 99, 99, 99, 99, 99, 99, 99}},
  //rx6:
  {1, {0, 1}, {0, 0, 0, 0, 0, 0},
   {99, 99, 99, 99, 99, 99, 99, 99, 99, 99}},
  //rx7:
  {1, {2, 3}, {0, 0, 0, 0, 0, 0},
   {99, 99, 99, 99, 99, 99, 99, 99, 99, 99}},
  //rx8:
  {2, {4, 5}, {0, 0, 0, 0, 0, 0},
   {99, 99, 99, 99, 99, 99, 99, 99, 99, 99}},
  //rx9:
  {2, {6, 7}, {0, 0, 0, 0, 0, 0},
   {99, 99, 99, 99, 99, 99, 99, 99, 99, 99}},
  //rx10:
  {2, {0, 1}, {0, 0, 0, 0, 0, 0},
   {99, 99, 99, 99, 99, 99, 99, 99, 99, 99}},
  //rx11:
  {2, {2, 3}, {0, 0, 0, 0, 0, 0},
   {99, 99, 99, 99, 99, 99, 99, 99, 99, 99}},
  //rx12:
  {3, {4, 5}, {0, 0, 0, 0, 0, 0},
   {99, 99, 99, 99, 99, 99, 99, 99, 99, 99}},
  //rx13:
  {3, {6, 7}, {0, 0, 0, 0, 0, 0},
   {99, 99, 99, 99, 99, 99, 99, 99, 99, 99}},
  //rx14:
  {3, {0, 1}, {0, 0, 0, 0, 0, 0},
   {99, 99, 99, 99, 99, 99, 99, 99, 99, 99}},
  //rx15:
  {3, {2, 3}, {0, 0, 0, 0, 0, 0},
   {99, 99, 99, 99, 99, 99, 99, 99, 99, 99}}
};

/*struct biasCardParams {  // definition in argusHarwareStructs.h
  float v[8];    // pv, nv, dsv, vcc
}; */
struct biasCardParams bcPar[] = {
		  {{99, 99, 99, 99, 99, 99, 99, 99}},
		  {{99, 99, 99, 99, 99, 99, 99, 99}},
		  {{99, 99, 99, 99, 99, 99, 99, 99}},
		  {{99, 99, 99, 99, 99, 99, 99, 99}}
};

/*struct cryostatParams {  // definition in argusHarwareStructs.h
  float cryoTemps[6];     // cryostat temperatures
  float auxInputs[2];     // aux inputs
}; */
struct cryostatParams cryoPar = {
	  {99, 99, 99, 99, 99, 99}, {99, 99},
};

// vds, -15V, +15, vcc, cal sys, cold if in, cold if out, cold if curr, chassis temp
float pwrCtrlPar[] = {99, 99, 99, 99, 99, 99, 99, 99, 99};

/*struct muBoxParams {
  float adcv[8];              // adc voltages
  float pdetVmax;             // maximum detector voltage
  unsigned short int setval;  // tune word for YIG driver
  float slope;          	  // blind tune slope from linear fit
  float intercept;            // blind tune intercept from linear fit
  float freq;                 // requested LO frequency
};*/
struct muBoxParams muBoxPar = {
	{99., 99., 99., 99., 99., 99., 99., 99.}, -99., 32768, 1., 0., 0.
};

/*struct calSysParams {
	float adcv[8]; 		// includes angle [V], temperature [C], and motorMeanI[A]
	float minAngle; 	// minimum vane angle [V]
	float maxAngle; 	// maximum vane angle [V]
	float meanCurr; 	// mean motor current [A]
	float maxCurr;      // maximum motor current [A]
	float varCurr;  	// motor current variance [A]
	char state[15];     // system state
};*/
struct calSysParams calSysPar = {
	{99., 99., 99., 99., 99., 99., 99., 99.}, 99., 99., 99., 99., 99., " "
};

// Warm IF
/*struct warmIFparams {
	float psv[2];       // power supply voltage monitor for interface boards
	float totPow[16];   // total power reading for each IF channel
	float cardTemp[16]; // card temperature
	char atten[16];     // attenuation for each IF channel
	char sb[16];        // sideband state for each IF channel (0 = LSB, 1 = USB)
  (setting atten to max value and sb to USB matches initialization in argus_init() below)
*/
struct warmIFparams wifPar = {
		{99, 99},
		{99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99},
		{99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99},
		{31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31},
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
};
// Warm IF channel ordering
BYTE wifChan_i2caddr[] = {0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80};
//BYTE wifChan_i2caddr[] = {0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01};



/**************************/
// DACs

// drain voltage setups
struct chSet vdSet = {
		{0x40, 0x40, 0x40, 0x40, 0x31, 0x31, 0x31, 0x31},
		{0x31, 0x37, 0x32, 0x36, 0x31, 0x37, 0x32, 0x36},
		1.0, 0.0, 0};

// gate voltage or servo current setups
struct chSet vgSet = {
		{0x41, 0x41, 0x41, 0x41, 0x32, 0x32, 0x32, 0x32},
		{0x31, 0x37, 0x33, 0x36, 0x31, 0x37, 0x33, 0x36},
		0.1470, 0.0, 1};

// mixer voltage setups
struct chSet vmSet = {
		{0x40, 0x40, 0x41, 0x41, 0x31, 0x31, 0x32, 0x32},
		{0x34, 0x33, 0x30, 0x32, 0x34, 0x33, 0x30, 0x32},
		0.42824, -0.27676, 1};

// offset
struct chSet voSet = {
		{0x41, 0x32, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
		{0x35, 0x35, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
		0.001, 2.047, 1};  // offset in mV
/**************************/
//ADCs

// drain voltage monitor points
struct chRead vdRead = {
		{0x19, 0x19, 0x19, 0x19, 0x08, 0x08, 0x08, 0x08},
		{0xf8, 0xa8, 0xb8, 0xe8, 0xf8, 0xa8, 0xb8, 0xe8},
		1.0, 0.0, 0};  // no scaling needed for sc

// drain current monitor points
struct chRead idRead = {
		{0x18, 0x18, 0x18, 0x18, 0x09, 0x09, 0x09, 0x09},
		{0xc0, 0xf0, 0x80, 0xb0, 0xc0, 0xf0, 0x80, 0xb0},
		16.75, 34.15, 1};  // offset is slope*2.048

// gate voltage monitor point
struct chRead vgRead = {
		{0x18, 0x18, 0x18, 0x18, 0x09, 0x09, 0x09, 0x09},
		{0x90, 0xe0, 0xd0, 0xa0, 0x90, 0xe0, 0xd0, 0xa0},
		-6.8, 0.0, 1};

// mixer voltage monitor points
struct chRead vmRead = {
		{0x0b, 0x0b, 0x0b, 0x0b, 0x0a, 0x0a, 0x0a, 0x0a},
		{0xf0, 0x80, 0xa0, 0xd0, 0xf0, 0x80, 0xa0, 0xd0},
		1.564, 2.179, 1};  // with 4.7k shunt at ADC input

// mixer current monitor points
struct chRead imRead = {
		{0x0b, 0x0b, 0x0b, 0x0b, 0x0a, 0x0a, 0x0a, 0x0a},
		{0xb0, 0xc0, 0xe0, 0x90, 0xb0, 0xc0, 0xe0, 0x90},
		2.439, 0.0, 1};

// amplifier positive voltage monitor point on bias card
struct chRead2 pvRead = {
		{0x19, 0x08},
		{0xc8, 0xc8},
		4.727, 0.0, 0};

// amplifier negative voltage monitor point on bias card
struct chRead2 nvRead = {
		{0x19, 0x08},
		{0xd8, 0xd8},
		-4.545, 0.0, 0};

// drains supply voltage monitor point on bias card
struct chRead2 vdsRead = {
		{0x19, 0x08},
		{0x88, 0x88},
		2.0, 0.0, 0};

// vcc supply voltage monitor point on bias card
struct chRead2 vccRead = {
		{0x19, 0x08},
		{0x98, 0x98},
		2.0, 0.0, 0};

// power control board monitor points (pcRead.add used in other locations too)
struct chRead pcRead = {
		{0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08},
		{0x88, 0xc8, 0x98, 0xd8, 0xa8, 0xe8, 0xb8, 0xf8},
		1, 0, 0};

// thermometry board monitor points
struct chRead thRead = {
		{0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08},
		{0xf8, 0xb8, 0xe8, 0x98, 0xc8, 0x88, 0xd8, 0xa8},
		1, 0, 0};

/**************************************************************************/

/**
  \brief Initialize hardware.

  This routine is called automatically at boot (for Argus hardware).

  \param  flash  User flash data structure.
*/
void argus_init(const flash_t *flash)
{

	// start I2C interface
	I2CInit( 0xaa, 0x1a );   // Initialize I2C and set NB device slave address and I2C clock
	// Second argument is clock divisor for I2C bus, freqdiv
	// 0x16 for 97.6 kHz (fastest)
	// 0x17 for 78.1 kHz
	// 0x3b for 73.2 kHz
	// 0x18 for 65.1 kHz
	// 0x19 for 58.6 kHz
	// 0x1a for 48.8 kHz
	// 0x1c for 32.6 kHz
	// 0x1f for 19.5 kHz (slowest)
	//Echo setup
	printf("argus_init: flash serNo = %d, hwType = %d, valid = %d.\n",
	  			  flash->serialNo, flash->hw, flash->valid);

	printf("argus_init: flash vgdiv %f\n", flash->gvdiv);
	gvdiv = flash->gvdiv;  // gvdiv is initialized as a global variable
	if (gvdiv < 0. || gvdiv > 1.) gvdiv = 1.e6;  // protect against uninitialized flash value

	// in case system is already running, set bias levels to safe voltages
	// before power control relays reset open -- code in control.cpp/Correlator::execReboot

	/** Initialize devices on main I2C bus **/
	//// Power control card
	address = I2CSWITCH_BP;
	buffer[0] = PWCTL_I2CADDR;
	I2CSEND1;
	// set default value in PIO
	address = 0x21;
	buffer[0] = 0x01;
	buffer[1] = 0x20;  // all relays off low; FPLED is off high
	I2CSEND2;
	// configure PIO I/O
	address = 0x21;
	buffer[0] = 0x03;
	buffer[1] = 0xc0;
	I2CSEND2;
	//// vane/muBox interface
	address = I2CSWITCH_BP;
	buffer[0] = CALSY_I2CADDR;
	I2CSEND1;
	// set switch on card for vane
	address = I2CSWITCH_VMUB;
	buffer[0] = VANE_I2CADDR;
	I2CSEND1;
	// set default value in vane PIO
	address = 0x21;
	buffer[0] = 0x01;
	buffer[1] = VANESTOP;  // set stop drive bit
	I2CSEND2;
	// configure vane PIO I/O
	buffer[0] = 0x03;  // control byte for PIO configuration
	buffer[1] = 0xf8;  // P0-2 as outputs, others high-z
	I2CSEND2;
	// deselect cal system
	address = I2CSWITCH_VMUB;
	buffer[0] = 0;
	I2CSEND1;
	// (muBox initializes PIO each time in muBox_SPI_bitbang() -- could do it here for consistency)
	//// Clean up: disconnect I2C sub-buses
	address = I2CSWITCH_BP;
	buffer[0] = 0;
	I2CStat = I2CSEND1;

	// Now deal with I2C buses other than main bus
	if (NWIFBOX > 0) {
		// set up warm IF receiver PIOs, start with all bits high (gives max atten, USB)
		int i;
		BYTE i2cSwitchWIF_addr[] = I2CSWITCH_WIF;
		for (i=0; i<NWIFBOX; i++) {  // step through warm IF chassis
			address = i2cSwitchWIF_addr[i];  // main bus I2C address for receiver box buffer card
			buffer[0] = 0xff;  // set switch for all receivers
			I2CSEND1;
			// configure PIOs
			address = 0x21;    // PIO address on individual channel cards
			buffer[0] = 0x03;  // command byte: config buffer
			buffer[1] = 0x00;  // set all pins as outputs
			I2CSEND2;
			buffer[0] = 0x01;  // command byte: output buffer
			buffer[1] = 0x00;  // set all pins low (max atten)
			I2CSEND2;
			// disconnect 12C sub-bus
			address = i2cSwitchWIF_addr[i];
			buffer[0] = 0x00;
			I2CSEND1;
		}
	}

	// Transfer values to structure for YIG tuning with muBox
	muBoxPar.slope = flash->yigFit[0];
	muBoxPar.intercept = flash->yigFit[1];

    // release I2C bus
	i2cBusBusy = 0;

}

/************************************************************************/

/**
  \brief Convert voltage to DAC word

  Function converts voltage to DAC word.  Sets to max or min value for 10-bit DAC if 
  input value is out of range. 

  \param  v   input value (float)
  \param  sc  multiplicitive scaling factor (float)
  \param  offs additive offset (float)
  \param  bip bipolar = 1, unipolar = 0 (char)
  \return Zero on success, else -1.
*/
/*------------------------------------------------------------------
  Convert voltage to DAC word
 ------------------------------------------------------------------*/
unsigned short int v2dac(float v, float sc, float offs, char bip) {

	unsigned int dacw;
	if (bip) {
		dacw = ((v+offs)*sc*65535/4.096 + 0x7fff);
		if (dacw & 0xffff0000) {  // check for values out of range, clip to in-range
			if (v > 0) dacw = 0x0000ffff;
			else dacw = 0x00000000;
		}
	}
	else {
		dacw = ((v+offs)*sc*65535/4.096);
		if (v < 0) dacw = 0x00000000; // if below zero, set to zero
		if (dacw & 0xffff0000) dacw = 0x0000ffff;  // if above max, clip to max
	}
	return (unsigned short int)dacw;
}

/****************************************************************************************/

/**
  \brief Convert ADC word to voltage

  Function converts ADC word to volage in mV.

  \param  adcw  ADC word (short int)
  \param  sc    Multiplicitive scaling factor (float)
  \param  offs  Additive offset (float)
  \param  bip   Bipolar = 1, unipolar = 0 (char)
  \return Zero on success, else -1.
*/
/*------------------------------------------------------------------
  Convert ADC word to voltage
 ------------------------------------------------------------------*/
float adc2v(short int adcw, float sc, float offs, char bip) {

  float v;
    v = adcw*sc*4.096/65535 + offs;
    return v;
}


/********************************************************************/
/**
  \brief Set LNA DAC.

  This command sets the DACs for LNA and mixer bias voltages

  \param  i   terminal: g, d, or m for gate, drain, and mixer (char).
  \param  m   mth receiver.
  \param  n   nth stage within a receiver.
  \param  v   value in V
  \param busy set to 0 to release I2C bus, 1 to retain (1 for loops)
  \return Zero on success, -1 for invalid selection, -10 if bias card power is not on,
              else number of I2C read fails.
*/
int argus_setLNAbias(char *term, int m, int n, float v, unsigned char busyOverride)
{
	unsigned short int dacw;
	short I2CStat;
	int baseAdd; //base address offset for gate, drain, mixer
	char bcard_i2caddr[] = BCARD_I2CADDR;//{0x01, 0x01, 0x01, 0x01, 0xff}; //BCARD_I2CADDR;
	float vDiv;  // voltage divider ratio,  vDiv <= 1

	// return if the LNA boards are not powered
	if (!lnaPwrState) return (-10);

	// check for freeze
	if (freezeSys) {freezeErrCtr += 1; return FREEZEERRVAL;}

	// check that I2C bus is available, else return
	if (i2cBusBusy && !busyOverride) {busNoLockCtr += 1; return I2CBUSERRVAL;}
	i2cBusBusy = 1;
	busLockCtr += 1;

    // Write to device
	// first set I2C bus switch for bias card in backplane
    address = I2CSWITCH_BP;  // bias cards are in Argus backplane
   	buffer[0] = bcard_i2caddr[rxPar[m].cardNo];  // select bias card
	I2CStat = I2CSEND1;    // set i2c bus switch to talk to card

	// then check that voltage is within limits, set address internal to card and channel,
	// convert voltage, send chip i2c address on card, internal address for channel,
	// convert voltage to word for DAC
	if (strcmp(term, "g") == 0) {
		if (lnaLimitsBypass == 0) {   // bypass soft limits on LNA bias when = 1
			if (v > VGMAX) v = VGMAX;
			if (v < VGMIN) v = VGMIN;
			if (rxPar[m].LNAsets[n+NSTAGES] - v > VDGMAX) v = rxPar[m].LNAsets[n+NSTAGES] - VDGMAX;
		}
		v = v/gvdiv;  // convert from gate voltage to bias card output voltage
		address = vgSet.i2c[rxPar[m].bcChan[n]];
		buffer[0] = vgSet.add[rxPar[m].bcChan[n]];
		dacw = v2dac(v, vgSet.sc, vgSet.offset, vgSet.bip);
		baseAdd = 0;
		vDiv = gvdiv;
	} else if (strcmp(term, "d") == 0) {
		if (lnaLimitsBypass == 0) {   // bypass soft limits on LNA bias when = 1
			if (v > VDMAX) v = VDMAX;
			if (v < VDMIN) v = VDMIN;
			if (v - rxPar[m].LNAsets[n] > VDGMAX) v = rxPar[m].LNAsets[n] + VDGMAX;
		} else {
			if (v < 0) v = 0;  // hardware limit
		}
		address = vdSet.i2c[rxPar[m].bcChan[n]];
		buffer[0] = vdSet.add[rxPar[m].bcChan[n]];
		dacw = v2dac(v, vdSet.sc, vdSet.offset, vdSet.bip);
		baseAdd = 2;
		vDiv = 1.;
	} else if (strcmp(term, "m") == 0) {
		if (lnaLimitsBypass == 0) {   // bypass soft limits on LNA bias when = 1
			if (v > VMMAX) v = VMMAX;
			if (v < VMMIN) v = VMMIN;
		}
		address = vmSet.i2c[rxPar[m].bcChan[n]];
		buffer[0] = vmSet.add[rxPar[m].bcChan[n]];
		dacw = v2dac(v, vmSet.sc, vmSet.offset, vmSet.bip);
		baseAdd = 4;
		vDiv = 1.;
	} else {
		// Disconnect I2C sub-bus
		address = I2CSWITCH_BP;
		buffer[0] = 0;
		i2cBusBusy = 0; // release I2C bus
		I2CStat = I2CSEND1;
		return -1;
	}
	// write to DAC
	buffer[2] = BYTE(dacw);
	buffer[1] = BYTE(dacw>>8);
	I2CStat = I2CSEND3;    // send set command
	// write set value v into structure
	if (I2CStat==0) {
        rxPar[m].LNAsets[n+baseAdd] = v*vDiv;
	}
	else {
		if (lnaPSlimitsBypass == 1) rxPar[m].LNAsets[n+baseAdd] = v*vDiv;
		else rxPar[m].LNAsets[n+baseAdd] = 99.;
	}
	// Disconnect I2C sub-bus
	address = I2CSWITCH_BP;
	buffer[0] = 0;
	I2CStat = I2CSEND1;

    // release I2C bus
	i2cBusBusy = busyOverride;

	return I2CStat;
}


/****************************************************************************************/

/**
  \brief Set all LNA bias voltages

  This command sets all LNA bias voltages to a common value.  Useful
  for initialization.

  This command does not set i2cBusBusy semaphore; that should be done outside
  if needed

  \param  inp  Select input: char g, d, m for gate, drain, mixer.
  \param  v    Voltage [V].
  \return 0 on success; -1 for invalid request; -10 if LNA boards have no power,
               else a number giving the number of failed I2C writes.
  */

int argus_setAllBias(char *inp, float v, unsigned char busyOverride){

	int i, j, stat=0;

    // return if the LNA boards are not powered
	if (!lnaPwrState) return (-10);

    // check that I2C bus is available, else return
	if (i2cBusBusy && !busyOverride) {busNoLockCtr += 1; return I2CBUSERRVAL;}
	i2cBusBusy = 1;
	busLockCtr += 1;

 	if (strcmp(inp, "g") == 0 || strcmp(inp, "d") == 0) {
		for (i=0; i<NRX; i++) {
			for (j=0 ; j<NSTAGES ; j++) {
				stat += argus_setLNAbias(inp, i, j, v, 1);
			}
		}
	}
	else if (strcmp(inp, "m") == 0 ) {
		if (NMIX > 0) {
			for (i=0; i<NRX; i++) {
				for (j=0 ; j<NMIX ; j++) {
					stat += argus_setLNAbias("m", i, j, v, 1);
				}
			}
		}
	} else {
		i2cBusBusy = 0; // release I2C bus
		return -1;
	}

    // release I2C bus
	i2cBusBusy = busyOverride;

	return stat;
}


/****************************************************************************************/
/**
  \brief Read LNA monitor points.

  This command reads the LNA voltage monitor points for all receivers.  Results are put in
  the receiver parameter structure, with 99 the value for an unsuccessful read.

  \return Zero on success, else number of failed I2C writes.
*/

int argus_readLNAbiasADCs(char *sw)
{
	int n, m, mmax; //index counters
	int baseAddr;  // offset in values vector
	unsigned short int rawu;
	short int rawb;
	float vDivRatio;
	BYTE bcard_i2caddr[] = BCARD_I2CADDR;
	char idFlag = 0; // set to 1 for drain shunt current correction

	// check for freeze
	if (freezeSys) {freezeErrCtr += 1; return FREEZEERRVAL;}

// check that I2C bus is available, else return
	if (i2cBusBusy) {busNoLockCtr += 1; return I2CBUSERRVAL;}
	i2cBusBusy = 1;
	busLockCtr += 1;

	// set up for particular monitor point: vg, vd, id, vm, im
	// baseAddr offsets correspond to offsets in receiver parameters structure
	if (strcmp(sw, "vg") == 0) {
		chReadPtr = &vgRead;
		baseAddr = 0;
		mmax = NSTAGES;
		vDivRatio = gvdiv;
	} else if (strcmp(sw, "vd") == 0) {
		chReadPtr = &vdRead;
		baseAddr = 2;
		mmax = NSTAGES;
		vDivRatio = 1.;
	} else if (strcmp(sw, "id") == 0) {
		chReadPtr = &idRead;
		baseAddr = 4;
		mmax = NSTAGES;
		vDivRatio = 1.;
		idFlag = 1;  // set to 1 to make shunt current correction
	} else if (strcmp(sw, "vm") == 0) {
		if (NMIX == 0) {
			i2cBusBusy = 0; // release I2C bus
			return -1;
		}
		chReadPtr = &vmRead;
		baseAddr = 6;
		mmax = NMIX;
		vDivRatio = 1.;
	} else if (strcmp(sw, "im") == 0) {
		if (NMIX == 0) {
			i2cBusBusy = 0; // release I2C bus
			return -1;
		}
		chReadPtr = &imRead;
		baseAddr = 8;
		mmax = NMIX;
		vDivRatio = 1.;
	} else {
		i2cBusBusy = 0; // release I2C bus
		return -1;
	}

	// loop over receivers
	if (lnaPwrState) {
		for (n = 0 ; n < NRX; n++) {
			// set I2C bus switch for correct bias card
			address = I2CSWITCH_BP;  // select backplane
			buffer[0] = bcard_i2caddr[rxPar[n].cardNo];  // bias card address in backplane
			I2CStat = I2CSEND1;    // set i2c bus switch to talk to card
			// loop over stages
			for (m = 0 ;  m < mmax ; m++) {
				address = chReadPtr->i2c[rxPar[n].bcChan[m]];    // chip i2c address on card
				buffer[0] = chReadPtr->add[rxPar[n].bcChan[m]];  // internal address for channel
				I2CStat = I2CSEND1;        // send command for conversion
				I2CREAD2;   // read device buffer back
				//I2CStat += I2CREAD2 - 2;   // read device buffer back, accumulate error flag
				// write result to structure; two cases for bipolar and unipolar ADC settings
				// if idFlag correct correct I_D for current from 1k shunt resistor (units are V, mA)
				if (I2CStat == 0) {
					if (chReadPtr->bip == 1) {
						rawb = (short int)(((unsigned char)buffer[0]<<8) | (unsigned char)buffer[1]);
						rxPar[n].LNAmonPts[m+baseAddr] = rawb*chReadPtr->sc*4.096/65535*vDivRatio + chReadPtr->offset;
						if (idFlag) rxPar[n].LNAmonPts[m+baseAddr] = rxPar[n].LNAmonPts[m+baseAddr] - rxPar[n].LNAmonPts[m+2];
					} else {
						rawu =(unsigned short int)(((unsigned char)buffer[0]<<8) | (unsigned char)buffer[1]);
						rxPar[n].LNAmonPts[m+baseAddr] = rawu*chReadPtr->sc*4.096/65535*vDivRatio + chReadPtr->offset;
						if (idFlag) rxPar[n].LNAmonPts[m+baseAddr] = rxPar[n].LNAmonPts[m+baseAddr] - rxPar[n].LNAmonPts[m+2];
					}
				} else rxPar[n].LNAmonPts[m+baseAddr] = 99;
			}
		}
	} else {
		// set value to no-measurement if power is not on
		for (n = 0 ; n < NRX; n++) {
			for (m = 0 ;  m < mmax ; m++) {
				rxPar[n].LNAmonPts[m+baseAddr] = 99;
			}
		}
	}

	// Disconnect I2C sub-bus
	address = I2CSWITCH_BP;
	buffer[0] = 0;
	I2CStat = I2CSEND1;

    // release I2C bus
	i2cBusBusy = 0;

	return I2CStat;
}
/****************************************************************************************/
/**
  \brief Read LNA bias card power monitors.

  This command reads the power supply voltage monitor points on a bias card.  Results are put in
  the bias card structure, with 99 the value for an unsuccessful read.

  \return Zero on success, else number of failed I2C writes.
*/

int argus_readBCpsV(void)
{
	struct chRead2 *bcPsVptr[] = {&pvRead, &nvRead, &vdsRead, &vccRead};  // pointers to bias card struct
	BYTE bcard_i2caddr[] = BCARD_I2CADDR;
	int k, n, m;
	BYTE writeErrs = 0;
	short baseAddr[] = {0, 2, 4, 6};
	unsigned short int rawu;
	short int rawb;
	// offsets correspond to offsets in results vector

	// check for freeze
	if (freezeSys) {freezeErrCtr += 1; return FREEZEERRVAL;}

    // check that I2C bus is available, else return
	if (i2cBusBusy) {busNoLockCtr += 1; return I2CBUSERRVAL;}
	i2cBusBusy = 1;
	busLockCtr += 1;

	if (lnaPwrState) {
		for (k = 0; k < 4; k++) {  // loop over cards
			// set I2C bus switch for correct bias card
			address = I2CSWITCH_BP;  // select backplane
			buffer[0] = bcard_i2caddr[k];  // bias card address in backplane
			writeErrs = I2CSEND1;    // set i2c bus switch to talk to card
			for (m = 0; m < 4; m++) { // loop over monitor points
				for (n = 0; n < 2; n++) {  // loop over bias points
					address = bcPsVptr[m]->i2c[n];    // chip i2c address on card
					buffer[0] = bcPsVptr[m]->add[n];  // internal address for channel
						I2CStat = I2CSEND1;    // send command for conversion
						I2CREAD2;   // read device buffer back, accumulate error flag
					    // write result to structure; two cases for bipolar and unipolar ADC settings
						if (I2CStat == 0) {
							if (bcPsVptr[m]->bip == 1) {
								rawb = (short int)(((unsigned char)buffer[0]<<8) | (unsigned char)buffer[1]);
								bcPar[k].v[n+baseAddr[m]] = rawb*bcPsVptr[m]->sc*4.096/65535 + bcPsVptr[m]->offset;
							} else {
								rawu =(unsigned short int)(((unsigned char)buffer[0]<<8) | (unsigned char)buffer[1]);
								bcPar[k].v[n+baseAddr[m]] = rawu*bcPsVptr[m]->sc*4.096/65535 + bcPsVptr[m]->offset;
							}
						} else {
							bcPar[k].v[n+baseAddr[m]] = 99.;
							writeErrs += 1;
						}
				}
			}
		}
	} else {
		// set value to no-measurement if power is not on
		for (k = 0; k < 4; k++) {  // loop over cards
			for (m = 0; m < 4; m++) { // loop over monitor points
				for (n = 0; n < 2; n++) {  // loop over bias points
					bcPar[k].v[n+baseAddr[m]] = 99.;
				}
			}
		}
		writeErrs = 0;
	}

	// Disconnect I2C sub-bus
	address = I2CSWITCH_BP;
	buffer[0] = 0;
	I2CStat = I2CSEND1;

    // release I2C bus
	i2cBusBusy = 0;

	return writeErrs;
}

/**************************************************************************************/

/**
  \brief Read power control ADC.

  This command reads the ADC on the power control card

  \return Zero on success, else number of failed I2C writes.
*/
int argus_readPwrADCs(void)
{

	short i;
	unsigned short int rawu;
	static float offset[8] = {0};
	static float scale[8] = {2., -4.545, 4.727, 2., 7.818, 2., 2., 1.};
	//float scale[8] = {1, 1, 1, 1, 1, 1, 1, 1};  // for calibration
	// vds, -15, +15, vcc, vcal, vif, swvif, iif

    if (i2cBusBusy) {busNoLockCtr += 1; return I2CBUSERRVAL;}
    i2cBusBusy = 1;
    busLockCtr += 1;

    // Write to device
	// first set I2C bus switch
    address = I2CSWITCH_BP;  // select Argus backplane
    buffer[0] = PWCTL_I2CADDR;  // select power control card
	I2CStat = I2CSEND1;    // set i2c bus switch to talk to card

	//Read all channels of ADC
	for (i = 0 ;  i < 8 ; i++) {
		address = (BYTE)0x08;    // ADC device address on I2C bus
		buffer[0] = (BYTE)pcRead.add[i];        // internal address for channel
		I2CStat = I2CSEND1;  // send command for conversion
		I2CREAD2;  // read device buffer back
		if (I2CStat == 0) {
			rawu =(unsigned short int)(((unsigned char)buffer[0]<<8) | (unsigned char)buffer[1]);
			pwrCtrlPar[i] = rawu*scale[i]*4.096/65535 + offset[i];
		}
		else pwrCtrlPar[i] = 99.;  // error condition
	}

	// Read thermometer chip on power control board
	short int rawtemp;
	address = 0x4f;
	buffer[0] = 0x00;
	I2CStat += I2CSEND1;
	I2CREAD2;
	rawtemp = (short int)(((unsigned char)buffer[0]<<8) | (unsigned char)buffer[1]);
	if (I2CStat == 0) {
		pwrCtrlPar[8] = (float)rawtemp/256.;
	} else {
		pwrCtrlPar[8] = 999.;
	}

	// Disconnect I2C sub-bus
	address = I2CSWITCH_BP;
	buffer[0] = 0;
	I2CStat = I2CSEND1;

    // release I2C bus
	i2cBusBusy = 0;

	return I2CStat;
}

/**************************************************************************************/
/**
  \brief Read LNA power control PIO buffer.

  Function to query LNA power control PIO buffer.
*/

unsigned char argus_lnaPowerPIO(void)
{
	unsigned char pioState;

    // check that I2C bus is available, else return
	if (i2cBusBusy) {busNoLockCtr += 1; return I2CBUSERRVAL;}
	i2cBusBusy = 1;
	busLockCtr += 1;

	// set I2C sub-bus switch for power control card in backplane
	address = I2CSWITCH_BP;
	buffer[0] = PWCTL_I2CADDR;
	I2CStat = I2CSEND1;

	// Read port register to establish present state, store byte for output
	address = 0x21;  // PIO I2C address
	buffer[0] = 0x00;
	I2CStat += I2CSEND1;
	I2CREAD1;
	pioState = buffer[0];

	// Disconnect I2C sub-bus
	address = I2CSWITCH_BP;
	buffer[0] = 0;
	I2CStat = I2CSEND1;

    // release I2C bus
	i2cBusBusy = 0;

	return (pioState);
}


/**************************************************************************************/
/**
  \brief LNA power control.

  This command turns the LNA power on and off in a safe way.  Checks for power supplies
  in range for ON, but OFF executes regardless of power supply values.

  \param  state     Power state (1=on, else off).
  \return Zero on success; else a number giving the number of failed I2C writes or,
          for power supplies out of range, in order of first failure:
            9995 for Vcc
            9996 for -Vamp
            9997 for +Vamp
            9998 for VDS
          */
int argus_lnaPower(short state)
{
	BYTE pioState;

	I2CStat = argus_readPwrADCs();
	// get power supply voltages: returns vds, -15, +15, vcc, vcal, vif, swvif, iif, tamb

	// check for freeze
	if (freezeSys) {freezeErrCtr += 1; return FREEZEERRVAL;}

	// check that I2C bus is available, else return
	if (i2cBusBusy) {busNoLockCtr += 1; return I2CBUSERRVAL;}
	i2cBusBusy = 1;
	busLockCtr += 1;

	if (state == 1) {  // check power supply voltages before on, but skip check for off
		if (lnaPSlimitsBypass != 1) {
			if (pwrCtrlPar[3] < MINVCCV || pwrCtrlPar[3] > MAXVCCV) {
				// Disconnect I2C sub-bus
				address = I2CSWITCH_BP;
				buffer[0] = 0;
				I2CStat = I2CSEND1;
				i2cBusBusy = 0;  // release I2C bus
				return 9995;   //VCC
			}
			if (pwrCtrlPar[1] < -MAXAMPV || pwrCtrlPar[1] > -MINAMPV) {
				// Disconnect I2C sub-bus
				address = I2CSWITCH_BP;
				buffer[0] = 0;
				I2CStat = I2CSEND1;
				i2cBusBusy = 0;  // release I2C bus
				return 9996; //-15V
			}
			if (pwrCtrlPar[2] < MINAMPV || pwrCtrlPar[2] > MAXAMPV) {
				// Disconnect I2C sub-bus
				address = I2CSWITCH_BP;
				buffer[0] = 0;
				I2CStat = I2CSEND1;
				i2cBusBusy = 0;  // release I2C bus
				return 9997;   //+15
			}
			if (pwrCtrlPar[0] < MINVDSV || pwrCtrlPar[0] > MAXVDSV) {
				// Disconnect I2C sub-bus
				address = I2CSWITCH_BP;
				buffer[0] = 0;
				I2CStat = I2CSEND1;
				i2cBusBusy = 0;  // release I2C bus
				return 9996;   //VDS
			}
		}
	}

	// set I2C bus switch for power control card in backplane
	address = I2CSWITCH_BP;
	buffer[0] = PWCTL_I2CADDR;
	I2CStat = I2CSEND1;

	// Read port register to establish present state;
	// store byte for later modification
	address = 0x21;  // PIO I2C address
	buffer[0] = 0x00;
	I2CStat += I2CSEND1;
	I2CREAD1;
	pioState = buffer[0];

	// control power
	if (state==1 && lnaPwrState==0) {
		 // turn on VCC (digital) and allow it to stabilize
		address = 0x21;  // PIO I2C address
		buffer[0] = 0x01;
		buffer[1] = pioState = pioState | ctlVCC;
		I2CStat += I2CSEND2;
		OSTimeDly( TICKS_PER_SECOND * 1 );
		lnaPwrState = 1;  // set state flag

		// initialize DAC values, then return to power control board
		argus_setAllBias("g", VGSTART, 1);
		argus_setAllBias("d", VDSTART, 1);
		argus_setAllBias("m", VMSTART, 1);
		address = I2CSWITCH_BP;
		buffer[0] = PWCTL_I2CADDR;
		I2CStat = I2CSEND1;

		// turn on +/- Vamp (for gates), allow stabilization time
		address = 0x21;  // PIO I2C address
		buffer[0] = 0x01;
		buffer[1] = pioState = pioState | ctlpVamp | ctlnVamp;
		I2CStat += I2CSEND2;
		OSTimeDly( TICKS_PER_SECOND * 1 );

		// turn on VDS (for drains)
		address = 0x21;  // PIO I2C address
		buffer[0] = 0x01;
		buffer[1] = pioState = pioState | ctlVDS;
		I2CStat += I2CSEND2;

		// turn on LED
		address = 0x21;  // PIO I2C address
		buffer[0] = 0x01;
		buffer[1] = pioState = pioState & 0xff^FPLED;
		I2CStat += I2CSEND2;

	}
	else if (state==0 && lnaPwrState==1) {
		// set power supplies to safe voltages for switching
		argus_setAllBias("g", VGSTART, 1);
		argus_setAllBias("d", VDSTART, 1);
		argus_setAllBias("m", VMSTART, 1);

		// set I2C bus switch for power control card in backplane
		address = I2CSWITCH_BP;
		buffer[0] = PWCTL_I2CADDR;
		I2CStat = I2CSEND1;

		// turn off VDS (drains)
		address = 0x21;  // PIO I2C address
		buffer[0] = 0x01;
		buffer[1] = pioState = pioState & 0xff^ctlVDS;
		I2CStat += I2CSEND2;
		OSTimeDly( TICKS_PER_SECOND * 0.5 );

		// turn off +/- Vamp (gates)
		address = 0x21;  // PIO I2C address
		buffer[0] = 0x01;
		buffer[1] = pioState = pioState & 0xff^(ctlpVamp | ctlnVamp);
		I2CStat += I2CSEND2;
		OSTimeDly( TICKS_PER_SECOND * 0.5 );  // wait

		// turn off VCC (digital)
		address = 0x21;  // PIO I2C address
		buffer[0] = 0x01;
		buffer[1] = pioState = pioState & 0xff^ctlVCC;
		I2CStat += I2CSEND2;
		lnaPwrState = 0;  // clear state flag

		// turn off LED
		address = 0x21;  // PIO I2C address
		buffer[0] = 0x01;
		buffer[1] = pioState = pioState | FPLED;
		I2CStat += I2CSEND2;

	}

	// Disconnect I2C sub-bus
	address = I2CSWITCH_BP;
	buffer[0] = 0;
	I2CStat = I2CSEND1;

    // release I2C bus
	i2cBusBusy = 0;

	return (I2CStat);
}

/**************************************************************************************/
/**
  \brief Cold IF power control.

  This command turns the cold IF power on and off.  It will not change power state if
  the LNAs are powered.

  \param  state Power state (char; 1=on, else off).
  \return Zero on success; else for power supply voltage out of range:
               9993 too low
               9994 too high
          or -10 if LNA power is on
          */
int argus_cifPower(short state)
{
	BYTE pioState;

	// don't do anything if LNA power is on
	if (lnaPwrState) return -10;

	// check power supplies
	I2CStat = argus_readPwrADCs();

	// check for freeze
	if (freezeSys) {freezeErrCtr += 1; return FREEZEERRVAL;}

    // check that I2C bus is available, else return
	if (i2cBusBusy) {busNoLockCtr += 1; return I2CBUSERRVAL;}
	i2cBusBusy = 1;
	busLockCtr += 1;

	// return if power supplies out of range
	if (state) {
		if (cifPSlimitsBypass != 1) {
			if (pwrCtrlPar[5] < MINCIFV) {
				// Disconnect I2C sub-bus
				address = I2CSWITCH_BP;
				buffer[0] = 0;
				I2CStat = I2CSEND1;
				i2cBusBusy = 0;  // release I2C bus
				return 9993;
			}
			if (pwrCtrlPar[5] > MAXCIFV) {
				// Disconnect I2C sub-bus
				address = I2CSWITCH_BP;
				buffer[0] = 0;
				I2CStat = I2CSEND1;
				i2cBusBusy = 0;  // release I2C bus
				return 9994;
			}
		}
	}

	// set I2C bus switch for power control card in backplane
	address = I2CSWITCH_BP;
	buffer[0] = PWCTL_I2CADDR;
	I2CStat = I2CSEND1;

	// Read port register to establish present state;
	// store byte for later modification
	address = 0x21;  // PIO I2C address
	buffer[0] = 0x00;
	I2CStat += I2CSEND1;
	I2CREAD1;
	pioState = buffer[0];

	// control power
	if (state==1 && cifPwrState==0) {
		 // turn on cold IF
		buffer[0] = 0x01;
		buffer[1] = pioState = pioState | ctlVIF;
		I2CStat += I2CSEND2;
		cifPwrState = 1;  // set state flag
	} else if (state==0 && cifPwrState==1) {
		// turn off cold IF
		buffer[0] = 0x01;
		buffer[1] = pioState = pioState & 0xff^ctlVIF;
		I2CStat += I2CSEND2;
		cifPwrState = 0;  // clear state flag
	}

	// Disconnect I2C sub-bus
	address = I2CSWITCH_BP;
	buffer[0] = 0;
	I2CStat = I2CSEND1;

    // release I2C bus
	i2cBusBusy = 0;

	return (I2CStat);
}

/****************************************************************************************/

/**
  \brief Covert cryo diode voltage to temperature

  This function converts voltage to temperature for Lakeshore 670-series cryo diodes. Coefficients
  and method from Lakeshore diode curve writeup.

  \param  v  The voltage across a Lakeshore 670-series diode
  \return    The corresponding temperature, or obvious out-of-range values for over- or under-range.
*/
float v2t_670(float v) {
 
  short int i;
  float x, temp, tc[12];

  // strutures with Chebyshev coefficients, different temperatures
  struct coeffs {
    float vl;    // low voltage
    float vh;    // high voltage
    float a[12]; // coeffs (padded with 0 if needed)
  };
  struct coeffs *ptc; // pointer to structure of type coeffs
  // coefficients for 2K to 12K
  static struct coeffs t1 = {1.294390, 1.680000, {6.429274, -7.514262,
			     -0.725882, -1.117846, -0.562041, -0.360239, 
			     -0.229751, -0.135713, -0.068203, -0.029755, 
			     0., 0.}};
  // coefficients for 12K to 24.5K
  static struct coeffs t2 = {1.11230, 1.38373, {17.244846, -7.964373,
			     0.625343, -0.105068, 0.292196, -0.344492,
			     0.271670, -0.151722, 0.121320, -0.035566,
			     0.045966, 0.}};
  // coefficients for 24.5 to 100 K
  static struct coeffs t3 = {0.909416, 1.122751, {82.017868, -59.064244,
			     -1.356615, 1.055396, 0.837341, 0.431875, 
			     0.440840, -0.061588, 0.209414, -0.120882,
			     0.055734, -0.035974}};
  // coefficients for 100K to 475K
  static struct coeffs t4 = {0.07000, 0.99799, {306.592351, -205.393808,
			     -4.695680, -2.031603, -0.071792, -0.437682,
			     0.176352, -0.182516, 0.064687, -0.027019, 
			     0.010019, 0.}};

  // pick correct set of coefficients; first out-of-range tests
  if (v > 1.680) return(-99);  // overvoltage error
  if (v < 0.070) return(99);   // undervoltage error
  if (v >= 1.339) ptc = &t1;       // for 2-12K
  else if (v >= 1.118) ptc = &t2;  // for 12-24.5K
  else if (v >= 0.954) ptc = &t3;  // for 24.5-100K
  else ptc = &t4;                  // for 100-475K

  // compute temperature with Chebyshev recursion
  x = ((v - ptc->vl) - (ptc->vh - v))/(ptc->vh - ptc->vl);
  tc[0] = 1.;
  tc[1] = x;
  temp = ptc->a[0] + ptc->a[1]*x;
  for (i=2; i<12; i++) {
    tc[i] = 2*x*tc[i-1] - tc[i-2];
    temp = temp + ptc->a[i]*tc[i];
  }


  return(temp);
}

/****************************************************************************************/

/**
  \brief Read bias card power monitor points.

  This command reads the supply monitor points on the bias cards.

  \return Zero on success, else number of failed I2C writes.
*/
int argus_readThermADCs(void)
{
	short i;
	unsigned short int rawu;
	static float offset[8] = {0};
    static float scale[8] = {0.4439, 0.4439, 0.4439, 0.4439, 0.4439, 0.4439, 3.7, 3.7};
	//float scale[8] = {1, 1, 1, 1, 1, 1, 1, 1};  // for calibration

    // check that I2C bus is available, else return
	if (i2cBusBusy) {busNoLockCtr += 1; return I2CBUSERRVAL;}
	i2cBusBusy = 1;
	busLockCtr += 1;

    // Write to device
	// first set I2C bus switch
    address = I2CSWITCH_BP;  // select Argus backplane
    buffer[0] = THERM_I2CADDR;  // select thermometry card
	I2CStat = I2CSEND1;    // set i2c bus switch to talk to card

	//Read thermometry channels of ADC
	for (i = 0 ;  i < 6 ; i++) {
		address = thRead.i2c[i];    // ADC device address on I2C bus
		buffer[0] = thRead.add[i];  // internal address for channel
		I2CStat = I2CSEND1;  // send command for conversion
		I2CREAD2;  // read device buffer back
		if (I2CStat == 0) {
			rawu =(unsigned short int)(((unsigned char)buffer[0]<<8) | (unsigned char)buffer[1]);
			cryoPar.cryoTemps[i] = rawu*scale[i]*4.096/65535 + offset[i]; // voltage
			cryoPar.cryoTemps[i] = v2t_670(cryoPar.cryoTemps[i]);         // temperature
			//cryoPar.cryoTemps[i] = cryoPar.cryoTemps[i]*100.;           // for testing: 1.234 V -> 123.4 K
		}
		else cryoPar.cryoTemps[i] = 99;  // error condition
	}

	//Read thermometry card aux input channels of ADC
	for (i = 6 ;  i < 8 ; i++) {
		address = thRead.i2c[i];    // ADC device address on I2C bus
		buffer[0] = thRead.add[i];  // internal address for channel
		I2CStat = I2CSEND1;  // send command for conversion
		I2CREAD2;  // read device buffer back
		if (I2CStat == 0) {
			rawu =(unsigned short int)(((unsigned char)buffer[0]<<8) | (unsigned char)buffer[1]);
			cryoPar.auxInputs[i-6] = rawu*scale[i]*4.096/65535 + offset[i];
		}
		else cryoPar.auxInputs[i-6] = 99;  // error condition
	}

	// Disconnect I2C sub-bus
	address = I2CSWITCH_BP;
	buffer[0] = 0;
	I2CStat = I2CSEND1;

    // release I2C bus
	i2cBusBusy = 0;

	return I2CStat;
}

/****************************************************************************************/
/**
  \brief Set LNA params from flash.

  This command sets the LNA bias values to preset values stored in flash.

  \param  *flash A pointer to a structure of type flash_t.
  \return Zero on success, else number of failed I2C writes.
*/
int argus_LNApresets(const flash_t *flash)
{
// Storage for Arugus is is g1, g2, d1, d2, m1, m2, g3, g4 ... m31, m32
	// Data written in control.cpp, approx line 405
	short i, j, k;
	int rtn = 0;

	// check for freeze
	if (freezeSys) {freezeErrCtr += 1; return FREEZEERRVAL;}

	// check that I2C bus is available, else return
	if (i2cBusBusy) {busNoLockCtr += 1; return I2CBUSERRVAL;}
	i2cBusBusy = 1;
	busLockCtr += 1;

	k = 2*NSTAGES + NMIX;
	for (i=0; i<NRX; i++) {
		for (j=0; j<NSTAGES; j++){  // set drains first, then gates
			rtn += argus_setLNAbias("d", i, j, flash->LNAsets[i*k+j+NSTAGES], 1);
			rtn += argus_setLNAbias("g", i, j, flash->LNAsets[i*k+j], 1);
		}
		if (NMIX > 0) {  // set mixers
			for (j=0; j<NMIX; j++){
				rtn += argus_setLNAbias("m", i, j, flash->LNAsets[i*k+j+NSTAGES+NMIX], 1);
			}
		}
	}
	if (NWIFBOX > 0) {
		for (i=0; i<NRX; i++) {
			rtn += argus_setWIFswitches("a", i, flash->atten[i], 1);
			rtn += argus_setWIFswitches("s", i, flash->sb[i], 1);
		}
	}

    // release I2C bus
	i2cBusBusy = 0;

	return rtn;
}


/**************************************************************************************/

/**
  \brief Read warm IF power supply ADCs.

  This command reads the ADC on the warm IF interface cards

  \return Zero on success, else number of failed I2C writes.
*/
int argus_readWIFpsADCs(void)
{
	BYTE i2cADC_addr[] = I2CADC_WIF;  // Warm IF power mon ADC addresses (jumper selectable)

    // check that I2C bus is available, else return
	if (i2cBusBusy) {busNoLockCtr += 1; return I2CBUSERRVAL;}
	i2cBusBusy = 1;
	busLockCtr += 1;

	short i;
	unsigned short int rawu;
	float scale = 2.;  // scaling for warm IF power supply ADCs; 1/voltdiv ratio on board

	//Read both ADCs
	for (i = 0 ;  i < 2 ; i++) {
		address = i2cADC_addr[i];    // ADC device address on I2C bus
		buffer[0] = (BYTE)0x08;      // setup for LTC2301
		I2CStat = I2CSEND1;  // send command for conversion
		I2CREAD2;  // read device buffer back
		if (I2CStat == 0) {
			rawu =(unsigned short int)(((unsigned char)buffer[0]<<8) | (unsigned char)buffer[1]);
			wifPar.psv[i] = rawu*scale*4.096/65535;
		}
		else wifPar.psv[i] = 99.;  // error condition
	}

	// These devices are on the main I2C bus and do not need a sub-bus close

    // release I2C bus
	i2cBusBusy = 0;

	return I2CStat;
}

/********************************************************************/
/**
  \brief Read warm IF total powers and temperatures.

  This command reads all of the total powers and card temperatures in the warm IF

  \return Zero on success, else number of I2C read fails.
*/
int argus_readWIF(void)
{
	int m;             // loop index
	int I2CStat = 0;   // write status flag
	int chassis;       // index for warm IF chassis
	BYTE i2cSwitchWIF_addr[] = I2CSWITCH_WIF;   // Warm IF buffers switch addresses
	// channel ordering in wifChan_i2caddr[] is defined with other lookups at top of file

	if (NWIFBOX < 1) return -10;  // check that warm IF should be addressed

	// check that I2C bus is available, else return
	if (i2cBusBusy) {busNoLockCtr += 1; return I2CBUSERRVAL;}
	i2cBusBusy = 1;
	busLockCtr += 1;

	// loop over receivers, get total power and card temp. for each
	for (m=0; m<NRX; m++) {
		// Select chassis
		chassis = (int)(m/8);                       // assumes 8 channels per warm IF chassis
		// Select bus switch for warm IF channel
		address = i2cSwitchWIF_addr[chassis];       // main bus I2C address for buffer card
		buffer[0] = wifChan_i2caddr[m % 8];         // set switch for selected receiver
		I2CStat = I2CSEND1;   // set switch on buffer card

		// Set total power ADC to initiate conversion
		address = 0x14;                               // ADC address on individual channel cards
		buffer[0] = 0x00;                             // set ADC mode for LTC5421
		I2CStat = I2CSEND1;     // trigger ADC conversion
		OSTimeDly(1); // wait 50 ms to ensure ADC conversion complete (slow converters)
		I2CREAD2;				// read ADC buffer
		if (I2CStat == 0) {                         // write result to structure
			wifPar.totPow[m] = (float)(((unsigned char)buffer[0]<<8) | (unsigned char)buffer[1]);
			wifPar.totPow[m] = wifPar.totPow[m] * 5./65536.;  // scale output
		} else {
			wifPar.totPow[m] = 99.;
		}
		// Read thermometer chip on power control board
		short int rawtemp;
		address = 0x4f;
		buffer[0] = 0x00;
		I2CStat  += I2CSEND1;
		I2CREAD2;
		rawtemp = (short int)(((unsigned char)buffer[0]<<8) | (unsigned char)buffer[1]);
		if (I2CStat == 0) {
			wifPar.cardTemp[m] = (float)rawtemp/256.;
		} else {
			wifPar.cardTemp[m] = 99.;
		}

		// Deselect bus after reading last warm IF channel
    	address = i2cSwitchWIF_addr[chassis];           // I2C address for buffer card
    	buffer[0] = 0;
    	I2CStat += I2CSEND1;
	    /* more efficient in i2c bus transmissions, only deselect after last channel in each box...
		if (m % 8 == 7) {
	    	address = i2cSwitchWIF_addr[chassis];           // I2C address for buffer card
	    	buffer[0] = 0;
	    	I2CStat += I2CSEND1;
	    }
	    but then maybe add a final deselect after loop just to be safe*/
	}

    // release I2C bus
	i2cBusBusy = 0;

	return I2CStat;
}

/********************************************************************/
/**
  \brief Set warm IF switches.

  This command sets the switches for sideband selection and attenuation in the warm IF

  \param  inp  terminal: select on a for attenuation, s for sideband.
  \param  m    mth receiver.
  \param  val  value.
  \param busy set to 0 to release I2C bus, 1 to retain (for loops)
  \return Zero on success, -1 for invalid selection, else number of I2C read fails.
*/
int argus_setWIFswitches(char *inp, int m, char val, unsigned char busyOverride)
{
	int I2CStat;
	int chassis; // warm IF chassis number
	BYTE i2cSwitchWIF_addr[] = I2CSWITCH_WIF;   // Warm IF buffers switch addresses
	// channel ordering in wifChan_i2caddr[] is defined with other lookups at top of file

	if (NWIFBOX < 1) return -10;  // check that warm IF should be addressed

	// check for freeze
	if (freezeSys) {freezeErrCtr += 1; return FREEZEERRVAL;}

	// check that I2C bus is available, else return
	if (i2cBusBusy && !busyOverride) {busNoLockCtr += 1; return I2CBUSERRVAL;}
	i2cBusBusy = 1;
	busLockCtr += 1;

	// check that values are within limits, provisionally store to memory, establish which IF chassis
	if (strcmp(inp, "s") == 0) {
		if (val > 1) val = 1;
		if (val < 0) val = 0;
		wifPar.sb[m] = val;
	} else if (strcmp(inp, "a") == 0) {
		if (val > MAXATTEN) val = MAXATTEN;
		if (val < 0)  val = 0;
		wifPar.atten[m] = val;
	} else {
		i2cBusBusy = 0;  // release I2C bus
		return -1;
	}
	chassis = (int)(m/8);  // assumes 8 channels per warm IF chassis

    // Select bus switch for warm IF channel
    address = i2cSwitchWIF_addr[chassis];           // main bus I2C address for buffer card
   	buffer[0] = wifChan_i2caddr[m % 8];       // set switch for selected receiver
	I2CStat = I2CSEND1; // set switch on buffer card
	// write to PIO for the receiver
    address = 0x21;                                         // PIO address on individual channel cards
    buffer[0] = 0x01;                                       // command byte: output register
    buffer[1] = (BYTE)((wifPar.atten[m] <<1) | wifPar.sb[m])^0xfe;  // assemble command byte, atten active low
	I2CStat = I2CSEND2;               // set PIO on receiver card
	// if write fails, write invalid values into structure
	if (I2CStat != 0) {
		wifPar.sb[m] = 9;
		wifPar.atten[m] = 99;
	}

	// Disconnect I2C sub-bus
    address = i2cSwitchWIF_addr[chassis];           // I2C address for buffer card
	buffer[0] = 0x00;
	I2CStat = I2CSEND1;

    // release I2C bus
	i2cBusBusy = busyOverride;

	return I2CStat;
}


/********************************************************************/
/**
  \brief Set all warm IF switches.

  This command sets the switches for sideband selection and attenuation in the warm IF

  \param  inp  select on a for attenuation, s for sideband.
  \param  m    mth receiver.
  \param  val  value.
  \return Zero on success, -1 for invalid selection, else number of I2C read fails.
*/
int argus_setAllWIFswitches(char *inp, char val)
{
	if (NWIFBOX < 1) return -10;  // check that warm IF should be addressed

	// check for freeze
	if (freezeSys) {freezeErrCtr += 1; return FREEZEERRVAL;}

	// check that I2C bus is available, else return
	if (i2cBusBusy) {busNoLockCtr += 1; return I2CBUSERRVAL;}
	i2cBusBusy = 1;
	busLockCtr += 1;

	int m;  // loop counter
	int I2CStat = 0;
	for (m=0; m<NRX; m++){
		I2CStat += argus_setWIFswitches(inp, m, val, 1);
	}

	// release I2C bus
	i2cBusBusy = 0;

	return I2CStat;
}

/*******************************************************************/
/**
  \brief Open Argus muBox communication.

  Set up I2C communication with Argus' muBox interface card.

  \return Zero on success, else number of I2C write fails.
*/
int argus_openSubbusC(BYTE addr)
{
	int I2CStat = 0;

	//if (NMUBOX < 1 && addr==MUBOX_I2CADDR) return -1;  // return if no muBox in system

    // check that I2C bus is available, else return
	if (i2cBusBusy) {busNoLockCtr += 1; return I2CBUSERRVAL;}
	i2cBusBusy = 1;
	busLockCtr += 1;

	address = I2CSWITCH_BP;      // select backplane switch
	buffer[0] = CALSY_I2CADDR;   // select vane/muBox interface card
	I2CStat = I2CSEND1; // select
	// select muBox sub-bus from interface card
	address = I2CSWITCH_VMUB;    // select switch on interface card
	buffer[0] = addr;    // select muBox on switch
	I2CStat += I2CSEND1; // select

	if (I2CStat) i2cBusBusy = 0;  // clear bit if write errors on muBox bus

  return I2CStat;
}


/*******************************************************************/
/**
  \brief Close Argus muBox communication.

  Clear I2C communication with Argus' muBox interface card.

  \return Zero on success, else number of I2C write fails.
*/
int argus_closeSubbusC(void)
{
	int I2CStat = 0;

	//fix if (NMUBOX < 1 && addr==MUBOX_I2CADDR) return -1;  // return if no muBox in system

	// deselect muBox sub-bus on interface card
	address = I2CSWITCH_VMUB;      // select switch on interface card
	buffer[0] = 0;   // open all switches
	I2CStat += I2CSEND1; // select
	// deselect vane/muBox interface card
	address = I2CSWITCH_BP;      // select backplane switch
	buffer[0] = 0;   // open all switches
	I2CStat = I2CSEND1; // select

    // release I2C bus
	i2cBusBusy = 0;

	return I2CStat;
}

/*******************************************************************/
/**
  \brief SPI bit-bang to Argus muBox.

  This function converts an unsigned integer to a series of I2C commands
  to the parallel port on the Argus muBox interface.

  Requires call to argus_openSubbusC(MUBOX_I2CADDR) before use, and close_muBox after use, to
  open and close connection to muBox interface.

  \param  val  value to convert and send
  \return Zero on success, else number of I2C write fails.
*/
int muBox_SPI_bitbang(unsigned int val)
{

	if (NMUBOX < 1) {
		argus_closeSubbusC();
		return -1;  // return if no muBox in system
	}
	// variable definitions
	BYTE spi_clk_m =  0x01; // Mask for SPI clock bit (P0)
	BYTE spi_dat_m = 0x02;  // Mask for SPI data bit (P1)
	BYTE spi_cs_m = 0x04;   // Mask for SPI chip select bit (P2)
	BYTE x = 0;  // working byte
	int i;       // loop counter
	int I2CStat = 0;

	address = 0x21;    // I2C address for PIO chip on board
	buffer[0] = 0x03;  // control byte for PIO configuration
	buffer[1] = 0xf8;  // P.0-2 as outputs, others high-z
	I2CStat += I2CSEND2; // write to pio

	buffer[0] = 0x01;  // control byte indicating write to PIO output

	x |= spi_cs_m;    // ensure CS is high at start
	buffer[1] = x;
	I2CStat += I2CSEND2; // write to pio

	x &= ~spi_cs_m; // send CS low to initiate
	buffer[1] = x;
	I2CStat += I2CSEND2; // write to pio

	// step through input word; write data with clock low, then raise clock
	for (i=0; i<16; i++) {
		x &= ~spi_clk_m;   // set clock low
		if (val & 0x8000)  // look at MSB of 16-bit word to set data bit
			x |= spi_dat_m;
		else
			x &= ~spi_dat_m;
		buffer[1] = x;
		I2CStat += I2CSEND2; // write to pio
		x |= spi_clk_m;  // set clock high, data will be valid here
		buffer[1] = x;
		I2CStat += I2CSEND2; // write to pio
		val <<= 1;  // rotate to next-MSB
	}

	x |= spi_cs_m; // when done, set CS high to terminate SPI write
	buffer[1] = x;
	I2CStat += I2CSEND2; // write to pio

	return I2CStat;
}

/*******************************************************************/
/**
  \brief Read muBox ADC channel.

  Read a single ADC channel on the Argus muBox interface card.

  Requires call to argus_openSubbusC(MUBOX_I2CADDR) before use, and close_muBox after use, to
  open and close connection to muBox interface.

  \param  ch  number of ADC channel to read.
  \return Zero on success, -10 if channel is out of range, else number of I2C write fails.  Updates value in muBoxPar.
*/
int read_muBox_ADC(int ch)
{
    if (NMUBOX < 1) {
		argus_closeSubbusC();
		return -1;  //return if no muBox in system
    }
    if (ch < 0 || ch > 7) {
		argus_closeSubbusC();
		return -10;  //return if invalid channel number
    }
    int I2CStat = 0;

    unsigned short int rawu;
    address = 0x08;    // ADC I2C device address on board
    BYTE add[] = {0x88, 0xc8, 0x98, 0xd8, 0xa8, 0xe8, 0xb8, 0xf8};  // channel mapping (see spec sheet Table 1)
    float offset[8] = {0};
    float scale[8] = {1.045, 1.045, 1.045, 8.021, 1.045, 3.2, 8.021, 8.021}; // equal to 1/divider ratio
    //static float scale[8] = {1, 1, 1, 1, 1, 1, 1, 1};  // for calibration
    
    //Read ADC channel
    buffer[0] = add[ch];  // command word for single-ended unipolar conversion
    I2CStat = I2CSEND1;  // send command for conversion
    I2CREAD2;  // read device buffer back
    if (I2CStat == 0) {
      rawu =(unsigned short int)(((unsigned char)buffer[0]<<8) | (unsigned char)buffer[1]);
      muBoxPar.adcv[ch] = rawu*scale[ch]*4.096/65535 + offset[ch];
    }
    else muBoxPar.adcv[ch] = 99.;  // error condition

    return I2CStat;
}

/*******************************************************************/
/**
  \brief Read all muBox ADC channels.

  Read all ADC channels on the Argus muBox interface card.

  \return Zero on success, else number of I2C write fails.  Updates value in muBoxPar array.
*/
int read_all_muBox_ADC(void)
{
	if (NMUBOX < 1) return -1;  //return if no muBox in system
	int ch;

	int I2CStat = argus_openSubbusC(MUBOX_I2CADDR);
	if (I2CStat) return I2CStat;

	for (ch=0; ch<8; ch++) I2CStat += read_muBox_ADC(ch);

	argus_closeSubbusC();

	return I2CStat;
}

/*******************************************************************/
/**
  \brief Set YIG driver.

  Send a integer value to the YIG driver.

  Requires call to argus_openSubbusC(MUBOX_I2CADDR) before use, and close_muBox after use, to
  open and close connection to muBox interface.

  \param  val  value to send to YIG driver
  \return      zero for success (with updated value in muBoxPar), -1 for failure.
*/
int yig_set(unsigned short int val)
{
	// send command and update value if successful
	int I2CStat = muBox_SPI_bitbang(val);
	if (I2CStat==0) {
		muBoxPar.setval = val;  // update stored tuning value
		return 0;
	}
	else {
		muBoxPar.setval = 9;
		return -1;
	}
}

/*******************************************************************/
/**
  \brief Find peak LO tone power.

  Internal function to sweep YIG passband across LO tone to find maximum power and
  set tuning to that value.

  Function requires argus_openSubbusC(MUBOX_I2CADDR) before, close_muBox after.

  \param  ctr     center value to send YIG driver.
  \param  step    step size to take in units of YIG DAC word.
  \param  maxn    maximum number of steps to try in finding center.
  \param  pthresh  threshold power to declare power detected.
  \return         zero for success (with updated value in muBoxPar), -1 for failure.
*/
int yig_pkFind(unsigned short int ctr, unsigned short int step, int maxn, float pThresh)
{

	int I2CStat = 0;          // I2C error count
	unsigned short int cmax = ctr;  // max tuning value
	unsigned short int trial, tmin, tmax;  // tuning trial, max, min

	// guard against potential underflow and overflow
	cmax = (unsigned short int)(maxn/2)*step;  // use this as a dummy variable
	tmin = 0;
	if (ctr > cmax) tmin = ctr - cmax;            // prevent underflow
	tmax = 0xffff;
	if (ctr < (tmax - cmax)) tmax = ctr + cmax;   // prevent overflow

	cmax = ctr;  // fallback if no convergence
	muBoxPar.pdetVmax = -99.;    // initialize

	// sweep to find peak
	for (trial=tmin; trial<=tmax; trial+=step) {
		I2CStat += yig_set(trial);
		OSTimeDly(1); // settling time
		I2CStat += read_muBox_ADC(YIGPWRCHAN);
		if ((muBoxPar.adcv[YIGPWRCHAN] > pThresh) && muBoxPar.adcv[YIGPWRCHAN] > muBoxPar.pdetVmax) {
			// record high value
			cmax = trial;
			muBoxPar.pdetVmax = muBoxPar.adcv[YIGPWRCHAN];
		}
	}

	// set up on peak value
	I2CStat += yig_set(cmax);  // set to peak value
	OSTimeDly(1);          // settling time
	I2CStat += read_muBox_ADC(YIGPWRCHAN);  // record current det voltage
	if (muBoxPar.adcv[YIGPWRCHAN] < pThresh) I2CStat += 1;  // indicate peak failed

	return I2CStat;
}

/*******************************************************************/
/**
  \brief Center YIG center tuning.

  Sweep YIG passband across LO tone, declare peaked for tuning that is one step before
  first power decrease.

  \param  ctr     center value to send YIG driver.
  \param  step    step size to take in units of YIG DAC word.
  \param  maxn    maximum number of steps to try in finding center.
  \param  pthresh  threshold power to declare power detected.
  \return         zero for success (with updated value in muBoxPar), -1 for failure.
*/
int yig_peak(unsigned short int ctr, unsigned short int step, int maxn, float pThresh)
{

	int I2CStat = argus_openSubbusC(MUBOX_I2CADDR);
	if (I2CStat) return I2CStat;
	I2CStat =+ yig_pkFind(ctr, step, maxn, pThresh);
#ifdef SIMULATE
	OSTimeDly(15 * TICKS_PER_SECOND); // simulate a long search delay
#endif
	argus_closeSubbusC();

	return I2CStat;

}

/*******************************************************************/
/**
  \brief Find YIG center tuning from frequency.

  Convert input frequency to integer for YIG driver command using coefficients stored in flash
  memory, then search for LO signal with total power detector.  Stop searching when LO power detector
  value is >= pthresh or when number of steps exceeds maxn (2*maxn, really, since the function searches
  in pairs spaced increasingly above and below the nominal center).

  \param  freq   input   frequency.
  \param  step   step    value in same units to try to either side of center.
  \param  maxn   number  of tries to make.
  \param  pThresh power  threshold value from LO power detector.
  \return         zero for success (with updated value in muBoxPar), -1 for failur, -10 f out of range.
*/
int yig_freq(float freq, unsigned short int step, int maxn, float pThresh)
{

	// check that frequency is within tuning limits
	if (freq < YIGFMIN || freq > YIGFMAX) return -10;

	// check for freeze
	if (freezeSys) {freezeErrCtr += 1; return FREEZEERRVAL;}

    // get access to I2C bus
	int I2CStat = argus_openSubbusC(MUBOX_I2CADDR);
	if (I2CStat) return I2CStat;

	// update requested frequency
	muBoxPar.freq = freq;
	// convert frequency to YIG-driver integer, set YIG to approximate tuning
	unsigned short int ctr = (unsigned short int)(freq * muBoxPar.slope + muBoxPar.intercept);
	I2CStat += yig_set(ctr);


	// search for signal from total power detector
	// step to both sides of starting position in increasing steps
	// here it just stops once it finds a signal above threshold
	// we don't check here for valid ctr values, but that's done in yig_set()
	int i, j;  // loop indices
	unsigned short int inc[] = {0, 0};  // +/- increment values

	inc[0] = inc[1] = 0;  // +/- step offsets
	for (i=0; i<maxn; i++) {
		for (j=0; j<2; j++) {
			yig_set(ctr + inc[j]);
			OSTimeDly(1); // wait 50 ms for settling
			I2CStat += read_muBox_ADC(YIGPWRCHAN);
			if (muBoxPar.adcv[YIGPWRCHAN] > pThresh) {
				// found a signal, peak up and exit cleanly
				muBoxPar.setval = (unsigned short int)(ctr + inc[j]);
				I2CStat += yig_pkFind(muBoxPar.setval, step, maxn, pThresh);
				argus_closeSubbusC();
				return I2CStat;
			}
		}
		// only allow step to new value if no underflow or overflow
		if (0xffff - ctr - inc[0] >= step) {
			inc[0] += step;
		} else {
			inc[0] = 0;
		}
		if (ctr + inc[1] >= step) {
			inc[1] -= step;
		} else {
			inc[1] = 0;
		}
	}

	argus_closeSubbusC();
	if (muBoxPar.adcv[YIGPWRCHAN] < pThresh) I2CStat += 1;  // indicate peak failed
	return I2CStat;

}


/*******************************************************************/
/**
  \brief Read single cal sys ADC channel.

  Read a single ADC channel on the Argus cal system interface card.

  Requires call to open_muBox before use, and close_muBox after use, to
  open and close connection to muBox interface.

  \param  ch  number of ADC channel to read.
  \return Zero on success, -10 if channel is out of range, else number of I2C write fails.  Updates value in muBoxPar.
*/
int readCalSysADC(int ch)
{
	if (NMUBOX < 1) return -1;  //return if no muBox in system
	if (ch < 0 || ch > 7) return -10;  //return if invalid channel number
	int I2CStat = 0;

	unsigned short int rawu;
	address = 0x08;    // ADC I2C device address on board
	BYTE add[] = {0x88, 0xc8, 0x98, 0xd8, 0xa8, 0xe8, 0xb8, 0xf8};  // channel mapping (see spec sheet Table 1)
	float offset[8] = {0, 0, -50, 0, 0, 0, 0, 0};
	float scale[8] = {1.050, 0.375, 105.0, 8.021, 1.045, 3.2, 8.021, 8.021}; // equal to 1/divider ratio (modify for extra 1k; also unit conversion?)
	//static float scale[8] = {1, 1, 1, 1, 1, 1, 1, 1};  // for calibration

	//Read ADC channel
    buffer[0] = add[ch];  // command word for single-ended unipolar conversion
    I2CStat = I2CSEND1;  // send command for conversion
    I2CREAD2;  // read device buffer back
    if (I2CStat == 0) {
      rawu =(unsigned short int)(((unsigned char)buffer[0]<<8) | (unsigned char)buffer[1]);
      calSysPar.adcv[ch] = rawu*scale[ch]*4.096/65535 + offset[ch];
    }
    else calSysPar.adcv[ch] = 99.;  // error condition

    return I2CStat;
}

/*******************************************************************/
/**
  \brief Read all calSys ADC channels.

  Read all ADC channels on the Argus calSys interface card.

  \return Zero on success, else number of I2C write fails.  Updates value in muBoxPar array.
*/
int argus_readAllCalSysADC(void)
{

	int I2CStat = 0;

	I2CStat = argus_openSubbusC(VANE_I2CADDR);
	if (I2CStat) return I2CStat;

	I2CStat = readCalSysADC(VANEANGLEADC);
	I2CStat = readCalSysADC(VANECURRADC);
	I2CStat = readCalSysADC(VANETEMPADC);
	I2CStat = readCalSysADC(3);  // input supply voltage

	I2CStat = argus_closeSubbusC();

	return I2CStat;
}

/*******************************************************************/
/**
  \brief Set vane control bits

  Requires call to open_muBox before use, and close_muBox after use, to
  open and close connection to muBox interface.

  for PIO setup with stop on PO, run on P1, out/in& on P2
  cmd    function
   0     vane clear of beam
   4     vane in beam
   1     vane stop (also cmd = 3)
   2     vane run without stopping

  \return Zero on success, else number of I2C write fails or -1 for invalid cmd.
*/
int argus_setVaneBits(BYTE cmd)
{
	if (cmd > 0x04) return -1;  // invalid command byte

	address = 0x21;         // I2C address for PIO chip on board
	buffer[0] = 0x01;       // output port
	buffer[1] = cmd;        // set drive command bits
	int I2CStat = I2CSEND2; // select

    return I2CStat;
}

/*******************************************************************/
/**
  \brief Drive cal vane in and out of cal position

  Requires call to argus_openSubbusC before use, and argus_closeSubbusC after use, to
  open and close connection to interface.

  \return Zero on success, else number of I2C write fails.
*/
int argus_driveVane(BYTE cmd, short int nmax, float stopv, float deltastop)
{

	// cmd values are linked to functions in #define statements within
	// argusHarwareStructs.h:
	// VANESTOP 0x01      // lock vane motion
	// VANERUN 0x02       // run continuously
	// VANEINBEAM 0x04    // vane blocks beam
	// VANECLEAR 0x00     // vane clear of beam, stowed

	// check for freeze
	if (freezeSys && cmd!=VANESTOP) {freezeErrCtr += 1; return FREEZEERRVAL;}

	// set up err, loops, etc.
	int I2CStat = 0;
	short int n;

	// initialize stats
	float vsum = 0;
	float vsum2 = 0;
	calSysPar.minAngle = 999;
	calSysPar.maxAngle = -999;
	calSysPar.maxCurr = -999;
	calSysPar.meanCurr = 999;
	calSysPar.varCurr = 999;
	calSysPar.adcv[VANEANGLEADC] = 999;
	calSysPar.adcv[VANECURRADC] = 999;
	calSysPar.state = "error";

	// initialize for running sum
 	#define NSUM 20
	int i = 0;
	int j;
	int sum;
	int sumBuf[NSUM] = {99};
	int lastiangle = 0;
	int *psum;  // pointer for running sum
	psum = sumBuf;    // initialize pointer

	// convert to mV for integer comparison
	int istopv = (int)(stopv*1000);
	int ideltastop = (int)(deltastop*1000);

	// command motor controller
	I2CStat += argus_setVaneBits(cmd);
	if (I2CStat != 0) return I2CStat;

	if (cmd & VANESTOP) {
		if (!I2CStat) {
			calSysPar.state = "stop";
			vaneErr = 0;
		} else {
			calSysPar.state = "error";
			vaneErr = VANEPOSERR;
		}
		return I2CStat;
	}

	// loop over nmax measurements of angle and current
	for (n=0; n<nmax; n++) {
	// check current
		I2CStat += readCalSysADC(VANECURRADC);
		if (calSysPar.adcv[VANECURRADC] > calSysPar.maxCurr) calSysPar.maxCurr = calSysPar.adcv[VANECURRADC];
		vsum = vsum + calSysPar.adcv[VANECURRADC];
		vsum2 = vsum2 + calSysPar.adcv[VANECURRADC]*calSysPar.adcv[VANECURRADC];
	// check angle
		I2CStat += readCalSysADC(VANEANGLEADC);
		if (calSysPar.adcv[VANEANGLEADC] > calSysPar.maxAngle) calSysPar.maxAngle = calSysPar.adcv[VANEANGLEADC];
		if (calSysPar.adcv[VANEANGLEADC] < calSysPar.minAngle) calSysPar.minAngle = calSysPar.adcv[VANEANGLEADC];
		int iangle = (int)(calSysPar.adcv[VANEANGLEADC]*1000);
		// stop vane if voltage is within deltastop of requested stop voltage
		if (abs(iangle - istopv) < ideltastop) {
			I2CStat += argus_setVaneBits(VANESTOP);
			if (!I2CStat) {
				calSysPar.state = "found";
				vaneErr = 0;
			} else {
				calSysPar.state = "error";
				vaneErr = VANEPOSERR;
			}
			break;
		}
		// for stall, sample every 20 read cycles (approx. every 48.6 ms; 20 of these is 0.96 sec.)
		if (!(n%20) && stopVaneOnStall) {
			*(psum + i%NSUM) = iangle - lastiangle; // difference from last reading into buffer
			lastiangle = iangle;					// update last reading
			i++;									// increment counter for pointer
			sum = 0;								// initialize sum
			for (j=0; j<NSUM; j++) {				// make sum
				sum += abs(sumBuf[j]);
			}
			if (sum == 0) {						    // test for sum too small (no motion)
				if (stopVaneOnStall) I2CStat += argus_setVaneBits(VANESTOP);
				calSysPar.state = "stall";
				vaneErr = VANEPOSERR;
				break;
			}
		}
	}

    // compute and update parameters
	float div = 1./(float)nmax;
	calSysPar.meanCurr = vsum*div; //mean current
	calSysPar.varCurr =  vsum2*div - calSysPar.meanCurr*calSysPar.meanCurr; //variance
	if (n == nmax) {
		I2CStat += argus_setVaneBits(VANESTOP);
		calSysPar.state = "timeout";
		vaneErr = VANEPOSERR;
	}
	if (cmd == VANECLEAR || cmd == VANEINBEAM) {
		if (!I2CStat) {
			calSysPar.state = "hw_pos";
			vaneErr = 0;
		} else {
			calSysPar.state = "error";
			vaneErr = VANEPOSERR;
		}
	}

   return I2CStat;

}




/*******************************************************************/
/**
  \brief Clear I2C bus.

  Clear lock bit and open switches on main I2C bus

  \return         zero for success, or encoded number of failed I2C writes.
*/
int argus_clearBus(void)
{

	/* return value will be:
	 * i2cBusBusy value +
	 * write failure to spare switch *2 +
	 * write failure to backplane switch * 4 +
	 * write failure to warm IF #1 switch * 8  +
	 * write failure to warm IF #2 switch * 16  +
	 * write failure to spare switch * 16 +
	 * write failure to spare switch * 32
	 * (last two during attempt to make hardware reset of backplane switch)
	 *
	 * So if all switches have write failures, return will be 62
	 * If backplane switch fails, then 4
	 * If both warm IFs fail, then 24
	 * If switches are ok but bus busy semaphore is on, then return is 1
	 */

	// first check the state of the I2C bus:
	i2cState[0] = 0;
	// set pins for GPIO to read
	J2[42].function (PINJ2_42_GPIO);  // configure SCL as GPIO
	J2[39].function (PINJ2_39_GPIO);  // configure SDA as GPIO
	// read pins for BOOL values
	if (J2[42]) i2cState[0] = 1;  // SCL
	if (J2[39]) i2cState[0] += 2; // SDA
	// set pins for I2C again
	J2[42].function (PINJ2_42_SCL);  // configure SCL as GPIO
	J2[39].function (PINJ2_39_SDA);  // configure SDA as GPIO

	// now work on main bus
	BYTE wifaddr[] = I2CSWITCH_WIF;
	int I2CStat = 0;
	// check, then clear I2C bus lock bit
	if (i2cBusBusy) I2CStat += 1;
	i2cBusBusy = 0;
	// try opening all switches
	buffer[0] = 0x00;  // open switches command
	address = I2CSWITCH_SP;    // I2C address for spare switch
	if(I2CSEND1) I2CStat += 2; // write to switch
	address = I2CSWITCH_BP;    // I2C address for backplane
	if(I2CSEND1) I2CStat += 4;     // write to switch
	address = wifaddr[0];      // I2C address for warm IF chassis #1
	if(I2CSEND1) I2CStat += 8;     // write to switch
	address = wifaddr[1];      // I2C address for warm IF chassis #2
	if(I2CSEND1) I2CStat += 16;     // write to switch
	// hardware reset, main sub-bus switch
	// set and clear sends lows to main sub-bus switch to reset it
	address = I2CSWITCH_SP;
	buffer[0] = 0x08;  // ch 3, hardware hack
	if(I2CSEND1) I2CStat += 32;     // write to switch
	buffer[0] = 0x00;
	if(I2CSEND1) I2CStat += 64;     // write to switch

	// wrap up with check of the state of the I2C bus:
	i2cState[1] = 0;
	// set pins for GPIO to read
	J2[42].function (PINJ2_42_GPIO);  // configure SCL as GPIO
	J2[39].function (PINJ2_39_GPIO);  // configure SDA as GPIO
	// read pins for BOOL values
	if (J2[42]) i2cState[1] = 1;  // SCL
	if (J2[39]) i2cState[1] += 2; // SDA
	// set pins for I2C again
	J2[42].function (PINJ2_42_SCL);  // configure SCL as GPIO
	J2[39].function (PINJ2_39_SDA);  // configure SDA as GPIO

	return I2CStat;
}

/**************************************************************************************/
/**
  \brief Read all Argus system ADCs.

  Read all Argus system ADCs.  This fills monitor data point structures with
  up to date values.

  \return Zero.
*/
/*------------------------------------------------------------------
  Read all ADCs
 ------------------------------------------------------------------*/
int argus_readAllSystemADCs(void)
{
	int I2CStat = 0;
	if (argus_readPwrADCs() == I2CBUSERRVAL) I2CStat = I2CBUSERRVAL;
	if (argus_readBCpsV() == I2CBUSERRVAL) I2CStat = I2CBUSERRVAL;
	if (argus_readThermADCs() == I2CBUSERRVAL) I2CStat = I2CBUSERRVAL;
	if (argus_readLNAbiasADCs("vg") == I2CBUSERRVAL) I2CStat = I2CBUSERRVAL;
	if (argus_readLNAbiasADCs("vd") == I2CBUSERRVAL) I2CStat = I2CBUSERRVAL;
	if (argus_readLNAbiasADCs("id") == I2CBUSERRVAL) I2CStat = I2CBUSERRVAL;
	if (argus_readLNAbiasADCs("vm") == I2CBUSERRVAL) I2CStat = I2CBUSERRVAL;
	if (argus_readLNAbiasADCs("im") == I2CBUSERRVAL) I2CStat = I2CBUSERRVAL;
	if (argus_readAllCalSysADC() == I2CBUSERRVAL) I2CStat = I2CBUSERRVAL;
	if (read_all_muBox_ADC() == I2CBUSERRVAL) I2CStat = I2CBUSERRVAL;
	if (argus_readWIF() == I2CBUSERRVAL) I2CStat = I2CBUSERRVAL;
	return I2CStat;
}

/****************************************************************************************/
/**
  \brief Argus LNA bias check

  This command checks request vs. measured bias for LNAs and mixers.

  Within each 16-bit word:
    First stage LNA status in MSN (most significant nibble)
    Second stage LNA in next nibble
    Mixer one in next nibble
    Mixer two in LSN

  Within each nibble:
  For LNAs, V_G error in LSB, then V_D, then I_D
    1 means gate voltage error
    4 means drain current error, but drain voltage ok
    6 means both drain voltage and current errors
  For mixers, V in LSB, I in next
    1 means mixer voltage error
    2 means mixer current error
	3 means both error

  \return Zero on no errors, else summary with all words or-ed together.
*/
int argus_biasCheck(void)
{
	int i;  // loop counter
	int ret = 0;  // return value

	// loop over channels
	// initialize, then compare with set points
	for (i=0; i<NRX; i++) {
		biasSatus[i] = 0;
		// drain i
		if ( (rxPar[i].LNAmonPts[4] < IDMIN) || (rxPar[i].LNAmonPts[4] > IDMAX) )  biasSatus[i] |= 0x4000;
		if ( (rxPar[i].LNAmonPts[5] < IDMIN) || (rxPar[i].LNAmonPts[5] > IDMAX) )  biasSatus[i] |= 0x0400;
		// drain v
		if (fabsf(rxPar[i].LNAmonPts[2] - rxPar[i].LNAsets[2]) > VDEVMAX)  biasSatus[i] |= 0x2000;
		if (fabsf(rxPar[i].LNAmonPts[3] - rxPar[i].LNAsets[3]) > VDEVMAX)  biasSatus[i] |= 0x0200;
		// gate v
		if (fabsf(rxPar[i].LNAmonPts[0] - rxPar[i].LNAsets[0]) > VDEVMAX)  biasSatus[i] |= 0x1000;
		if (fabsf(rxPar[i].LNAmonPts[1] - rxPar[i].LNAsets[1]) > VDEVMAX)  biasSatus[i] |= 0x0100;
		// mixer i
		if ( (rxPar[i].LNAmonPts[8] < IMMIN) || (rxPar[i].LNAmonPts[8] > IMMAX) )  biasSatus[i] |= 0x0020;
		if ( (rxPar[i].LNAmonPts[9] < IMMIN) || (rxPar[i].LNAmonPts[9] > IMMAX) )  biasSatus[i] |= 0x0002;
		// mixer v
		if (fabsf(rxPar[i].LNAmonPts[6] - rxPar[i].LNAsets[4]) > VDEVMAX*2.)  biasSatus[i] |= 0x0010;
		if (fabsf(rxPar[i].LNAmonPts[7] - rxPar[i].LNAsets[5]) > VDEVMAX*2.)  biasSatus[i] |= 0x0001;
		// accumulate return value for error summary
		ret |= (int)biasSatus[i];
	}

	return ret;
}


/****************************************************************************************/
/**
  \brief Argus power check

  This command checks power system against limits defined in header.  Also checks that LO power is present.


  \return Zero on no errors, else int reporting errors.
*/
int argus_powCheck(void)
{
	int ret = 0;  // return value

	// MSN: power supplies
	if ( (pwrCtrlPar[0] < MINVDSV) || (pwrCtrlPar[0] > MAXVDSV) ) ret |= 0x8000;    //VDS
	if ( (pwrCtrlPar[2] < MINAMPV) || (pwrCtrlPar[2] > MAXAMPV) ) ret |= 0x4000;    //+15V
	if ( (pwrCtrlPar[1] < -MAXAMPV) || (pwrCtrlPar[1] > -MINAMPV) ) ret |= 0x2000;  //-15V
	if ( (pwrCtrlPar[3] < MINVCCV) || (pwrCtrlPar[3] > MAXVCCV) ) ret |= 0x1000;    //VCC
	// next nibble CIF
	if ( fabsf(pwrCtrlPar[7] - CIFNOMCURR) > CIFMAXCURRDEV ) ret |= 0x0400;         //cif curr
	if ( (pwrCtrlPar[6] < MINCIFV) || (pwrCtrlPar[6] > MAXCIFV) ) ret |= 0x0200;    //cif out
	if ( (pwrCtrlPar[5] < MINCIFV) || (pwrCtrlPar[5] > MAXCIFV) ) ret |= 0x0100;    //cif in
	// next nibble warm IF
	if ( (wifPar.psv[0] < MINVCCV) || (wifPar.psv[0] > MAXVCCV) ) ret |= 0x0020;    //wif 1
	if ( (wifPar.psv[0] < MINVCCV) || (wifPar.psv[0] > MAXVCCV) ) ret |= 0x0010;    //wif 0
	// LSN: YIG, vane systems
	// VANEVINOFFS volts removed from cal sys mon pt for 750 ohm resistor
	if ( (calSysPar.adcv[3] < MINCSV - VANEVINOFFS) || (calSysPar.adcv[3] > MAXCSV - VANEVINOFFS) ) ret |= 0x0008; //cal sys supply
	if ( (pwrCtrlPar[4] < MINCSV) || (pwrCtrlPar[4] > MAXCSV) ) ret |= 0x0004;         //cal sys supply
	if ( (muBoxPar.adcv[3] < MINAMPV) || (muBoxPar.adcv[3] > MAXAMPV) ) ret |= 0x0002; //+15V
	if ( muBoxPar.adcv[0] < YIGPTHRESH ) ret |= 0x0001;                                //LO power

	return ret;
}


/****************************************************************************************/
/**
  \brief Argus thermal system check

  This command checks thermal system, including cryostat pressure, against limits defined in header.


  \return Zero on no errors, else int reporting errors.
*/
int argus_thermCheck(void)
{
	int ret = 0;  // return value
	int i;        // loop counter

	// compare with limit values
	// MSN: cryostat
	if ( (cryoPar.auxInputs[0] > 1.1) || (cryoPar.auxInputs[0] < 0.5) ) ret |= 0x8000; // pressure
	if ( cryoPar.cryoTemps[4] > MAXINTT ) ret |= 0x4000;    // 77K cold head
	if ( cryoPar.cryoTemps[0] > MAXCOLDT ) ret |= 0x2000;   // 20K cold head
	if ( cryoPar.cryoTemps[5] > MAXCOLDT  ) ret |= 0x1000;  // LNAs warm
	// next nibble: warm elex
	// check max WIF temp, chassis temp
	if ( pwrCtrlPar[8] > MAXELEXT ) ret |= 0x0200;  //chassis temperature
	float maxt = -100.;
	for (i=0; i<16; i++) {
		if ( wifPar.cardTemp[i] > maxt ) maxt = wifPar.cardTemp[i];
	}
	if ( maxt > MAXELEXT ) ret |= 0x0100;           //max wif temperature
	// next nibble: vane position
	ret |= vaneErr;  // value defined by VANEPOSERR
	// LSN: YIG temp
	if ( muBoxPar.adcv[2]*100 > MAXYIGT ) ret |= 0x0001;

	return ret;
}


/****************************************************************************************/
/**
  \brief Argus IF power check

  This command checks the IF power at the warm IF processor power detectors against limits
  defined within this function.


  \return Zero on no errors, else int reporting errors.
*/
int argus_ifCheck(void)
{
	int ret = 0;      // return value
	int mask = 0x01;  // mask for setting bits
	int i;            // loop counter

	// check against these limits, ch0 to ch15
	float lims[] = MINIFPWRDETV;

	// loop over channels, check for low power and no response from det system
	for (i=0; i<16; i++) {
		if ( (wifPar.totPow[i] < lims[i]) || (wifPar.totPow[i]) > MAXIFPWRDETV) ret |= mask;
		mask <<= 1;
	}

	return ret;
}


/****************************************************************************************/
/**
  \brief Argus test command.

  This command "succeeds" iff the sum of the arguments is at least unity.

  \param  foo  A signed integer.
  \param  bar  A floating-point value.
  \return Zero on success, else -1.
*/
int argus_test(int foo, float bar)
{
  int rtn = (foo+bar >= 1.0 ? 0 : -1);
  printf("argus_test: foo=%d, bar=%g, rtn=%d.\n", foo, bar, rtn);

  if (0) {
	  *buffer = 0x000;   // initialize buffer
	  address = 0x41;    // DAC address for VGI2
	  buffer[0] = 0x32;  // internal address for VGI2
	  buffer[1] = 0x01;  // first data byte
	  buffer[2] = 0x02;  // second data byte
	  printf("sending\n");
	  I2CStat = I2CSendBuf(address, buffer, (strlen( (const char*)buffer ) ));
	  printf("done\n");
  }

  if (0){
	  argus_readAllSystemADCs();
  }

  if (0) {
	  *buffer = 0x000;   // initialize buffer
	  buffer[0] = (BYTE)0xff;
	  //buffer[0] = (BYTE)0x21;
	  address = (BYTE)0x70;  // addresses 0x70, 0x71
	  //while (1) I2CSEND1;  // infinite loop to make warm IF switch test
  }

  return rtn;
}


