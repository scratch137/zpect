#ifndef ARGUSHARDWARESTRUCTS_H
#define ARGUSHARDWARESTRUCTS_H


/* Structures containing address, scaling, etc. information for 
Argus bias monitor boards.

For biasBoardB cards
First bank A (lower on card, CHSel = F), then bank B (higher on card, CHSel = L)

AH 2014.07.01
*/

// Version label
#define VER "argus_v3.0_20170125"

// Run with hardware or standalone by commenting/uncommenting #define SIMULATE
//#define SIMULATE
#ifdef SIMULATE
#define I2CSEND1 0
#define I2CSEND2 0
#define I2CSEND3 0
#define I2CREAD1 0
#define I2CREAD2 0
#define BYPASS 1
#else
#define I2CSEND1 (int)I2CSendBuf(address, buffer, 1)
#define I2CSEND2 (int)I2CSendBuf(address, buffer, 2)
#define I2CSEND3 (int)I2CSendBuf(address, buffer, 3)
#define I2CREAD1 (int)I2CReadBuf(address, buffer, 1)
#define I2CREAD2 (int)I2CReadBuf(address, buffer, 2)
#define BYPASS 0
#endif

// Misc parameters
#define CMDDELAY 1        // pause before executing command, in units of 50 ms
#define I2CBUSERRVAL -100 // value to return for I2C bus lock error
#define FREEZEERRVAL -200 // value to return for system freeze violation error

// Hardware parameters -- must match structure definitions in argusHardwareStructs.h!
#define NUM_ELEM(x) (sizeof(x) / sizeof(*(x)))
#define NRX 16      // number of receivers
#define NRXPERBC 4  // number of receivers per bias card
#define NSTAGES 2   // number of amplifier stages in each receiver
#define NMIX 2      // number of mixers in each receiver
#define NBCMP 2     // number of monitor point sets for each bias card
#define NWIFBOX 2   // number of warm IF chassis; set to zero if none
#define NMUBOX 1    // number of muBoxes for YIG; set to zero if none

// Software limits for bias settings
#define VDGMAX 1.7    // Max drain-gate voltage [V]
#define VGMIN -0.3    // Min allowable gate voltage [V]
#define VGMAX 0.3     // Max allowable gate voltage [V]
#define VDMIN 0.0     // Min allowable drain voltage [V]
#define VDMAX 1.8     // Max allowable drain voltage [V]
#define VMMIN -.25    // Min allowable mixer voltage [V]
#define VMMAX 5.0     // Max allowable mixer voltage [V]
#define IDMIN 10.0    // Min operating drain current [mA]
#define IDMAX 30.0    // Max operating drain current [mA]
#define IMMIN 0.1     // Min operating mixer current [mA]
#define IMMAX 5.0     // Max operating mixer current [mA]
#define VDEVMAX 0.1   // Max allowable deviation from setpoint, error check [V]

// Startup voltages for gates, drains, mixers
#define VGSTART -0.2   // Gate
#define VDSTART 0.0    // Drain
#define VMSTART 0.0    // Mixer

// Power supply limits
#define MINVCCV 4.75       // Min digital vcc voltage
#define MAXVCCV 5.25       // Max digital vcc voltage
#define MINVDSV MINVCCV    // Max drain supply voltage
#define MAXVDSV MAXVCCV    // Min drain supply voltage
#define MINAMPV 10.0       // Min amplifier supply voltage (absolute)
#define MAXAMPV 15.5       // Max amplifier supply voltage (absolute)
#define MINCIFV 1.6        // Min cold IF supply voltage
#define MAXCIFV 2.         // Max cold IF supply voltage
#define CIFNOMCURR 0.4     // nominal CIF current [A]
#define CIFMAXCURRDEV 0.1  // Allowable CIF current deviation
#define MINCSV 26.         // Minimum cal sys voltage (28V nom)
#define MAXCSV 29.         // Maximum cal sys voltage (28V nom, 30V abs max)
#define MINIFPWRDETV {1.1, 1.02, 0.94, 1.15, 1.15, 1.06, 1.00, 1.12, 0.65, 0.60, 0.62, 0.59, 0.56, 0.66, 0.69, 0.53}
                           // Minimum IF totpwr detector voltages for operation (set to voltage with LO off + 0.1 V)
#define MAXIFPWRDETV 3.    // Maximum valid IF totpwr detector voltage

// Over-temperature limits
#define MAXCOLDT 40.   // Maximum nom 20K temperature [K]
#define MAXINTT  80.   // Maximum nom 77K temperature [K]
#define MAXELEXT 60.   // Maximum warm electronics temperature [C]
#define MAXYIGT  80.   // Maximum YIG temperature [C]

//Warm IF limits
#define MAXATTEN 31  // max attenuation, 5-bit, 1 dB/step, including 0

//YIG filter parameters
#define YIGPWRCHAN 0       // muBox ADC channel for YIG total power detector
#define YIGTUNESTEP 10     // default step value for YIG peak routine
#define YIGSEARCHSTEP 100  // search step for YIG freq routine; should be larger than YIGTUNESTEP
#define YIGMAXN 100        // number of steps to search
#define YIGPTHRESH 0.02    // threshold voltage for power detection (originally 0.05)
#define YIGFMIN (500-muBoxPar.intercept)/muBoxPar.slope     // YIG min frequency
#define YIGFMAX (65035-muBoxPar.intercept)/muBoxPar.slope   // YIG max frequency

//Vane definitions
#define VANESTOP 0x01      // lock vane motion
#define VANERUN 0x02       // run continuously
#define VANEINBEAM 0x04    // vane blocks beam
#define VANECLEAR 0x00     // vane clear of beam, stowed
#define VANENMAX 2300      // max no ADC readouts before timeout; 2300 is about 5.5s timeout
#define VANEDELTA 0.10     // delta in volts that means "close enough" to target
#define VANEANGLEADC 0     // angle ADC channel
#define VANECURRADC 1      // motor current ADC channel
#define VANETEMPADC 2      // vane temperature ADC channel
#define VANEVINOFFS 10.5   // vane input voltage offset
#define VANEPOSERR 0x0010  // position error flag for "check" command

// I2C bus mapping
#define I2CSWITCH_BP 0x77      // low bus nos. on micro, Argus warm elex chassis backplane
#define I2CSWITCH_SP 0x73      // high bus nos. on micro  (unused hardware)
#define I2CSWITCH_WIF {0x71, 0x70} // Warm IF switch addresses (jumper selectable)
#define I2CADC_WIF {0x28, 0x1a}    // Warm IF power mon ADC  (jumper selectable)

//I2C mapping off warm electronics chassis backplane
#define THERM_I2CADDR 0x10 // thermometry, off backplane
#define CALSY_I2CADDR 0x20 // cal sys, off backplane
#define SPARE_I2CADDR 0x40 // spare, off backplane
#define PWCTL_I2CADDR 0x80 //power control; 0x01 in test board, 0x80 in backplane
#define BCARD_I2CADDR {0x01, 0x02, 0x04, 0x08} // bias card addresses, off backplane
#define ALLBCARD_I2CADDR 0x0f // all bias card addresses, off backplane
#define I2CSWITCH_VMUB 0x72   // Vane and muBox switch addresses (sub-bus)
#define MUBOX_I2CADDR 0x01  // muBox sub-bus from vane/muBox interface card in backplane
#define VANE_I2CADDR 0x02   // vane sub bus from vane/muBox interface card in backplane


/****************************************/
// Parameter structure definitions

// Receiver parameter array of structures
struct receiverParams {
  int cardNo;           // bias card number: 0..3 for Argus (four cards)
  int bcChan[NSTAGES];  // channel no. within a bias card: bcChan 0..7
  float LNAsets[NSTAGES*2*(2 + NMIX/2)];     // command values, two per rx: gate, drain, mixer
  float LNAmonPts[NSTAGES*2*(1 + 2 + NMIX)];  // monitor points, two per rx: gate V, drain V I, mixer V I
};

struct biasCardParams {
  float v[8];    // pv, nv, dsv, vcc
};

struct cryostatParams { 
  float cryoTemps[6];     // cryostat temperatures
  float auxInputs[2];     // aux inputs
};

struct muBoxParams {
	float adcv[8];        // adc voltages
	float pdetVmax;       // maximum voltage
    unsigned int setval;  // tune word for YIG driver
    float slope, intercept; // slope and intercept for linear approx.
    float freq;           // requested LO frequency
};

struct calSysParams {
	float adcv[8]; 		// includes angle [V], temperature [C], and motorMeanI[A]
	float minAngle; 	// minimum vane angle [V]
	float maxAngle; 	// maximum vane angle [V]
	float meanCurr; 	// mean motor current [A]
	float maxCurr;      // maximum motor current [A]
	float varCurr;  	// motor current variance [A]
	char *state;     	// system state
};

struct warmIFparams {
	float psv[2];       // power supply voltage monitor for interface boards
	float totPow[16];   // total power reading for each IF channel
	float cardTemp[16]; // card temperature
	char atten[16];     // attenuation for each IF channel
	char sb[16];        // sideband state for each IF channel (0 = LSB, 1 = USB)
};

/*********************************************************************/

// Structures for DACs and ADCs on bias boards

struct chSet {  // set DACs
  char i2c[8];  // board-level i2c addr
  char add[8];  // addr in device
  float sc;     // scale for setting
  float offset; // offset for setting
  char bip;     // bipolar or unipolar, bipolar = 1
};

struct chRead { // read ADCs 
  char i2c[8];  // board-level i2c addr 
  char add[8];  // addr in device
  float sc;     // scale for reading
  float offset; // offset for reading
  char bip;     // bipolar or unipolar, bipolar = 1
};

struct chRead2 { // read ADCs
  char i2c[2];  // board-level i2c addr 
  char add[2];  // addr in device
  float sc;     // scale for reading
  float offset; // offset for reading
  char bip;     // bipolar or unipolar, bipolar = 1
};

#endif
