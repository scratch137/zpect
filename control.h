#ifndef CONTROL_H
#define CONTROL_H
/**
  \file

  Zpectrometer command and control functionality.

  $Id: control.h,v 1.33 2014/06/04 18:36:44 harris Exp $

*/
#include "data.h"
#include "io.h"
#include "observations.h"
#include "services.h"


/**
  \brief Executes single-argument class member functions for arbitrary types.

  This class implements a simple command dispatcher. It is essentially an
  associative array with enhanced functionality. Specific instantiations
  need to implement the parse() and cmdNotFound() methods, which determine how
  a command is parsed into a command name (key) and argument. The key is used
  in exec() to look up the associated member function, which is called with
  the command argument on the object supplied at construction.

  \param Obj_t Type of object whose member functions will be called.
  \param Cmd_t Raw command input type.
  \param Rtn_t Member function return type
               (normally a pointer or reference type).
  \param Arg_t Member function argument type.
  \param Key_t Dictionary key type (must support operator< and operator>).
*/
template<typename Obj_t, typename Cmd_t,
         typename Rtn_t, typename Arg_t, typename Key_t>
class CommandInterpreter
{
public:
  /// Container type.
  typedef std::map<Key_t, void (Obj_t::*)(Rtn_t, Arg_t)> container_type;

  /// Constant iterator type.
  typedef typename container_type::const_iterator const_iterator;

  /// Constructor.
  CommandInterpreter(Obj_t* obj) : obj_(obj) { }

  /// Indexing operator.
  typename container_type::mapped_type& operator[](const Key_t& key) {
    return cmdMap_[key];
  }

  /// Start of array.
  const_iterator begin() const { return cmdMap_.begin(); }

  /// End of array.
  const_iterator end() const { return cmdMap_.end(); }

  /// Finds a command.
  const_iterator find(const Key_t& key) const { return cmdMap_.find(key); }

  /**
    \brief Executes a command.
    \param rtn Return status.
    \param cmd Raw command input.
  */
  void exec(Rtn_t rtn, Cmd_t cmd)
  {
    Key_t key;
    Arg_t arg;
    parse(key, arg, cmd);
    typename container_type::const_iterator iCmd = cmdMap_.find(key);
    if (iCmd != cmdMap_.end()) { (obj_->*(iCmd->second))(rtn, arg); }
    else                       { cmdNotFound(rtn, key, arg, cmd); }
  }

  /**
    \brief Extracts the key (command name) and argument from a command.
    \param key Dictionary key.
    \param arg Member function argument.
    \param cmd Raw command input.
    \post \a key and \a arg arguments must be set by the method.
  */
  void parse(Key_t& key, Arg_t& arg, Cmd_t& cmd) const;

  /**
    \brief Return value for unrecognized commands.
    \param rtn Return value.
    \param key Dictionary key.
    \param arg Member function argument.
    \param cmd Raw command input.
  */
  void cmdNotFound(Rtn_t rtn, Key_t& key, Arg_t& arg, Cmd_t& cmd) const;

private:
  /// Class instance whose member functions are to be executed.
  Obj_t* obj_;

  /// Command dictionary.
  container_type cmdMap_;
};


/**
  Top-level Zpectrometer component. This class encapsulates the complete set
  of data and control components residing in a single chassis---i.e.,
  controlled by a single CPU. It is assumed each band contains the same number
  of lags. A template is used to allow static (compile-time) instantiation of
  the composite band objects and because the number of bands is also static.

  Control functions are implemented by methods whose names begin with \p exec.
  These commands can be made available for remote execution (via calls to
  CorrelatorShell::exec) by the ControlService. All such methods take
  C-strings as input (specifying their arguments, if any) and return a
  C-string status message for output. Each also contains a help string
  summarizing usage that is returned when a null pointer is passed as input.
  The form of the help messages are as follows (cf. createHelpSumary()):
  \verbatim
  REQUIRED_ARGUMENT ... [OPTIONAL_ARGUMENT ...]\r\n
  Statement of purpose.\r\n
  ARGUMENT1 Summary of argument...\r\n
  ARGUMENT2 Summary of argument...\r\n
  ...
  \endverbatim
  Each of these elements must reside on a single line (of arbitrary length);
  i.e., newline characters should be inserted as indicated. Note that the
  command name is \b not included above as commands can (in principal) have
  many aliases (cf. longHelp()).
*/
class Correlator
{
public:
  /// Band container type.
  typedef std::vector<BandData> container_type;

  /// Control shell command type.
  struct command_type {
    char *str;    ///< Command string (with arguments).
    int  fdRead,  ///< Open, readable client file descriptor (-1 if none).
         fdWrite; ///< Open, writable client file descriptor (-1 if none).
  };

  /// Control shell argument type.
  struct argument_type {
    char *str;    ///< Argument string.
    int  fdRead,  ///< Open, readable client file descriptor (-1 if none).
         fdWrite; ///< Open, writable client file descriptor (-1 if none).
    bool help;    ///< Whether to generate help message (only).

    /// Default constructor.
    argument_type(char *s = 0, int fdRead = -1, int fdWrite = -1,
		  bool h = true) :
	str(s), fdRead(fdRead), fdWrite(fdWrite), help(h) { }
  };

  /// Control shell return type.
  typedef char *return_type;

  /// Control shell key type.
  typedef std::string key_type;

  /// Control shell type.
  typedef CommandInterpreter<Correlator, command_type, return_type,
			     argument_type, key_type> shell_type;

  /// Error indicators type.
  struct error_type {
    int initADC;    ///< cpld_init_corl_adc() return status
  };

  static const char *statusEOF; ///< Control function EOF     result.
  static const char *statusERR; ///< Control function failure prefix.
  static const char *statusOK;  ///< Control function success prefix.
  static const char *statusWARN;///< Control function warning prefix.
  static const char *monitorPointName[ADC_NCHAN];  ///< Monitor point names.
  static const char *monitorPointUnit[ADC_NCHAN];  ///< Monitor point units.

  ControlService controlServer;  ///< Control server.
  DataService    dataServer;     ///< Lag data server.
  MonitorService monitorServer;  ///< Monitor data server.
  MessageService messageServer;  ///< Log message server.

  /// Constructor.
  Correlator(unsigned nBands, unsigned nBuffers, unsigned nLags) :
    accumulating_(false), band_(nBands), io_(0), master_(false), bootTicks_(0),
    mode_(MODE_NORMAL), nBuffers_(nBuffers), shell_(0), verbose_(ZPEC_LOG_INFO)
  {
    for (unsigned i=0; i<nBands; ++i) {
      band_[i].lags.setResolution(nLags, nBuffers);
    }
  }

  // Initialization.
  void boot(shell_type *shell, global_io_t *io);
  void initState();

  /// Number of contained bands.
  unsigned nBands() const { return band_.size(); }

  /// Number of lag data buffers per band.
  unsigned nBuffers() const { return nBuffers_; }

  /// Number of lags per band.
  unsigned nLags() const { return band_[0].lags.size()/nBuffers(); }

  /**
    Returns the band object for band # \a bandNo. The value of bandNo is
    zero-based and should which should be either a BCD (binary-coded decimal)
    integer, 0x0 - 0x9, or an ASCII digit, '0' - '9'. If an invalid value (or
    non-existent band) is requested, band #0 will be returned.

    \param bandNo Desired band number.
  */
  BandData& Band(unsigned bandNo)
  {
    if (bandNo >= '0') { bandNo -= '0'; }  // ASCII --> BCD
    return band_[bandNo < nBands() ? bandNo : 0];
  }

  /// Lock the main mutex.
  void lock() const { OSCritEnter(&lock_, 0); }

  /// Unlock the main mutex.
  void unlock() const { OSCritLeave(&lock_); }

  /// Lock the peripheral mutex.
  void periph_lock() const { OSCritEnter(&io_->periph_lock, 0); }

  /// Unlock the peripheral mutex.
  void periph_unlock() const { OSCritLeave(&io_->periph_lock); }

  zpec_mode_t getMode() const;
  zpec_mode_t setMode(zpec_mode_t mode);

  int getVerbosity() const;
  int setVerbosity(int level);

  bool  isAccumulating() const;
  bool setAccumulating(bool state);

  bool  isMaster() const;
  bool setMaster(bool state);

  /// Set and return monitor points (read-only).
  const MonitorData& monitorPoints(bool useLock) {
    setMonitorPoint(-1, useLock);
    return monPoints_;
  }

  /// First hardware-specific monitor ADC (logical) channel.
  unsigned monitorBegin() { return hw_adc_begin_; }

  /// First hardware-specific monitor ADC (logical) channel.
  unsigned monitorEnd()   { return hw_adc_end_; }

  const char *timestamp(unsigned utc_sec = 0, unsigned utc_csec = 0);

  // Control commands (available via control telnet server).
  void execBoss(return_type status, argument_type arg);
  void execDiodeObs(return_type status, argument_type arg);
  void execFlash(return_type status, argument_type arg);
  void execHalt(return_type status, argument_type arg);
  void execHelp(return_type status, argument_type arg);
  void execInitADCs(return_type status, argument_type arg);
  void execLevel(return_type status, argument_type arg);
  void execMode(return_type status, argument_type arg);
  void execPeek(return_type status, argument_type arg);
  void execPoke(return_type status, argument_type arg);
  void execPower(return_type status, argument_type arg);
  void execQuery(return_type status, argument_type arg);
  void execQuit(return_type status, argument_type arg);
  void execReboot(return_type status, argument_type arg);
  void execScopeObs(return_type status, argument_type arg);
  void execSend(return_type status, argument_type arg);
  void execStatsObs(return_type status, argument_type arg);
  void execStatus(return_type status, argument_type arg);
  void execSync(return_type status, argument_type arg);
  void execTime(return_type status, argument_type arg);
  void execTotalPower(return_type status, argument_type arg);
  void execVerbose(return_type status, argument_type arg);
  void execVersion(return_type status, argument_type arg);
  void execZero(return_type status, argument_type arg);
  //
  // Argus-specific commands.
  //
  void execArgusTest(return_type status, argument_type arg);
  void execArgusLimits(return_type status, argument_type arg);
  void execArgusDrain(return_type status, argument_type arg);
  void execArgusGate(return_type status, argument_type arg);
  void execArgusMixer(return_type status, argument_type arg);
  void execArgusPwrCtrl(return_type status, argument_type arg);
  void execArgusCIFPwrCtrl(return_type status, argument_type arg);
  void execArgusWIFCtrl(return_type status, argument_type arg);
  void execArgusSetAll(return_type status, argument_type arg);
  void execArgusCryo(return_type status, argument_type arg);
  void execArgusMonPts(return_type status, argument_type arg);
  void execArgusPresets(return_type status, argument_type arg);
  void execArgusEngr(return_type status, argument_type arg);
  void execArgusAtten(return_type status, argument_type arg);
  void execArgusSB(return_type status, argument_type arg);
  void execArgusYIG(return_type status, argument_type arg);
  void execArgusVane(return_type status, argument_type arg);
  void execArgusRxHealth(return_type status, argument_type arg);
  void execArgusFreeze(return_type status, argument_type arg);
  void execArgusThaw(return_type status, argument_type arg);
  void execArgusLock(return_type status, argument_type arg);

private:
  /// Alias map container type.
  typedef std::map<std::string,
                   void (Correlator::*)(return_type, argument_type)>
          alias_container_type;

  mutable OS_CRIT lock_;    ///< Main correlator mutex.
  bool accumulating_;       ///< Whether some task is accumulating ADC data.
  container_type band_;     ///< Observing bands.
  error_type error_;        ///< Error indicators.
  std::string helpSummary_; ///< Summary list of control commands.
  global_io_t *io_;         ///< Low-level I/O pointers (C-callable).
  bool master_;             ///< Whether this correlator is the sync master.
  zpec_hw_t hw_;            ///< Hardware variant.
  unsigned  hw_adc_begin_;  ///< First monitor ADC channel (hardware-specific).
  unsigned  hw_adc_end_;    ///< Last  monitor ADC channel (hardware-specific).
  unsigned long long
            bootTicks_;     ///< Boot time in ticks since the Unix Epoch.
  zpec_mode_t mode_;        ///< Evaluation (operating) mode.
  unsigned nBuffers_;       ///< Number of lag data buffers per band.
  shell_type *shell_;       ///< Attached command interpreter.
  int verbose_;             ///< Log message verbosity level.

  std::vector<lag_count_t>
              adcBuffer_;  ///< ADC_isr() input sample storage.
  MonitorData monPoints_;  ///< Monitor point dictionary.


  /// Command alias map (cf. createHelpSummary()).
  alias_container_type aliasMap_;

  void createHelpSummary(shell_type *shell);

  void longHelp(return_type status, const char *usage,
                void (Correlator::*member)(return_type, argument_type)) const;

  void initHardware();
  void setMonitorPoint(int channel, bool useLock);

  void collateData(const LagData &lags, unsigned nBuffers = 1);
  unsigned readADCs(Observation *obs, unsigned nFrames,
		    argument_type& arg, return_type status,
		    unsigned nRepeat = 1);
  unsigned setIntegStatus(return_type status,
                          unsigned nAccum, unsigned nFrames,
			  bool checkFraming = true);
  lag_count_t findCounts(return_type status, unsigned *len,
                         BasicObservation *obs, int atten,
			 unsigned channel, unsigned nFrames, int fdRead);
};

extern Correlator zpectrometer;
extern Correlator::shell_type zpecShell;

#endif  // CONTROL_H
