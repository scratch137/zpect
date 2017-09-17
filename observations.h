#ifndef OBSERVATIONS_H
#define OBSERVATIONS_H
/**
  \file

  Observation management.

  $Id: observations.h,v 1.11 2007/10/04 23:48:20 rauch Exp $
*/
#include "data.h"


/**
  Base observation class. This abstract class encapsulates the data and data
  processing corresponding to a specific type of observation. An observation
  receives frames of ADC data taken in an indicated operating mode, which it
  analyzes frame by frame according to the concrete observation implementation.
*/
class Observation
{
public:
  /// Constructor.
  Observation(zpec_mode_t mode) : mode_(mode) { }

  /// Destructor.
  virtual ~Observation() { }

  /// Sets assumed hardware operating mode.
  void setMode(zpec_mode_t mode) { mode_ = mode; }

  /// Returns assumed hardware operating mode.
  zpec_mode_t getMode() const { return mode_; }

  /// Processes a single frame of ADC data.
  virtual void processFrame(const lag_count_t *begin,
                            const lag_count_t *end) = 0;

private:
  zpec_mode_t mode_;  ///< Hardware operating mode.
};

/**
  A basic observation. Basic observations simply accumulate the results of
  individual frames into one or more buffers. Multiple buffers accumulate 
  alternately in round-robin fashion, starting with buffer 0.
*/
class BasicObservation : public Observation
{
public:
  /// Default constructor.
  BasicObservation(zpec_mode_t mode = MODE_NORMAL, unsigned nLags = 128,
                   unsigned nBuffers = 2) :
      Observation(mode), lagBuffer_(nLags, nBuffers),
      nBuffers_(nBuffers), nLags_(nLags) { }

  /// Destructor.
  virtual ~BasicObservation() { }

  /// Accumulated lag data (read-only).
  const LagData& lagData() const { return lagBuffer_; }

  /// Number of accumulation buffers.
  unsigned nBuffers() const { return nBuffers_; }

  /// Number of lags per buffer.
  unsigned nLags() const { return nLags_; }

  /// Sets number of lags.
  void setResolution(unsigned nLags, unsigned nBuffers) {
    lagBuffer_.setResolution(nLags, nBuffers); 
    nBuffers_ = nBuffers;
    nLags_    = nLags;
  }

  /// (Re)initialize the observation.
  void init(zpec_mode_t mode, unsigned nLags, unsigned nBuffers = 1) {
    setMode(mode);
    setResolution(nLags, nBuffers);
    lagBuffer_.assign(nLags*nBuffers, 0);
    for (unsigned i=0; i<nBuffers; ++i) {
      lagBuffer_.setFrames(i, 0);
      lagBuffer_.setTime(i, 0);
    }
    nFrames_ = 0;
  }

  virtual void processFrame(const lag_count_t *begin,
                            const lag_count_t *end);

private:
  LagData lagBuffer_;   ///< The lag accumulation buffer.
  unsigned nBuffers_,   ///< Number of accumulation buffers.
           nFrames_,    ///< Number of frames accumulated (all buffers).
	   nLags_;      ///< Number of lags per buffer.
};


/**
  A lag statistics observation. Lag statistics observations estimate the count
  mean and variance for each individual lag. An initial block of frames is
  used to crudely estimate the mean, for use in offset subtraction while
  accumulating the variance sums (the variance is typically small compared to
  the mean, so mean subtraction should be done in situ to avoid loss of
  precision). Statistics are calculated using fixed-point scaled integers,
  with decimal fraction precision determined by scaleFixed.
*/
class StatsObservation : public Observation
{
public:
  /// Default constructor.
  StatsObservation(zpec_mode_t mode = MODE_NORMAL, unsigned nLags = 128,
		   unsigned nBuffers = 2, unsigned scaleFixed = 1000) :
      Observation(mode), scaleFixed_(scaleFixed), sumBuffer_(nLags, nBuffers),
      sum2lsbBuffer_(nLags, nBuffers), sum2msbBuffer_(nLags, nBuffers) { }

  /// Destructor.
  virtual ~StatsObservation() { }

  /// (Re)initialize the observation.
  void init(zpec_mode_t mode, unsigned nLags, unsigned nBuffers,
            unsigned scaleFixed)
  {
    setMode(mode);
    setResolution(nLags, nBuffers);
    setScale(scaleFixed);
    sumBuffer_.setFrames(0, 0);
    sumBuffer_.assign(nLags*nBuffers, 0);
    sum2lsbBuffer_.assign(nLags*nBuffers, 0);
    sum2msbBuffer_.assign(nLags*nBuffers, 0);
  }

  /// Lag mean statistics.
  LagData& Mean() { return sumBuffer_; }

  /// Lag variance statistics.
  LagData& Variance() { return sum2lsbBuffer_; }

  /// Set fixed-point integer scale factor.
  void setScale(int scaleFixed) { scaleFixed_ = scaleFixed; }

  /// Set number of lags.
  void setResolution(unsigned nLags, unsigned nBuffers) {
        sumBuffer_.setResolution(nLags, nBuffers); 
    sum2lsbBuffer_.setResolution(nLags, nBuffers); 
    sum2msbBuffer_.setResolution(nLags, nBuffers); 
  }

  void computeStats();

  virtual void processFrame(const lag_count_t *begin, const lag_count_t *end);

private:
  unsigned scaleFixed_;   ///< Default fixed-point integer scaling factor.
  LagData    sumBuffer_,  ///< The scaled lag mean Sum(x).
	 sum2lsbBuffer_,  ///< The scaled lag variance Sum(x^2) LSBs.
	 sum2msbBuffer_;  ///< Sum(x^2) MSBs.

   int computeMean(int sum, unsigned nSamp);
   int computeVar(int sum, int sum2msb, int sum2lsb, unsigned nSamp);

   /// Return high digit (base 2^16).
   unsigned hi(unsigned x) { return (x >> 16); }

   /// Return low digit (base 2^16).
   unsigned lo(unsigned x) { return (x & 0xFFFF); }

   /// Return whether 64-bit value overflow type int.
   bool intOverflow(const unsigned *x) {
     return (x[1] != 0) || (x[0] & (1U<<31));
   }

   void div_64bit(unsigned *quot, const unsigned *x, unsigned y);
   void mul_64bit(unsigned *prod, unsigned x, unsigned y);

   /// Compute 64-bit difference x-y. x = 2^32 x1 + x0, y = 2^32 y1 + y0.
   void sub_64bit(unsigned *diff, const unsigned *x, const unsigned *y) {
     diff[1] = (x[1] - y[1]) - (x[0] < y[0]);
     diff[0] = (x[0] - y[0]);
   }
};


/**
  An "oscilloscope" observation. This observation records the complete sample
  history of a specified set of lags. A fixed-size buffer holds all samples
  collected. A sample is the accumulation of a specified number of ADC frames.
  The user specifies the number of sampling channels, which implicitly
  determines the maximum number of samples per channel. A sampling channel
  consists of a band number and lag number (all 0-based); the user must
  provide two arrays defining the sampling channels to record.
*/
class ScopeObservation : public Observation
{
public:
  /// Sample buffer size.
  static const unsigned maxSamples = 1U << 17;  // 512KB

  /// Default constructor.
  ScopeObservation(zpec_mode_t mode = MODE_NORMAL, unsigned nLags = 256,
		   unsigned nAccum = 1, unsigned nChannels = 8,
		   const int iLag[] = 0) :
      Observation(mode), nLags_(nLags), nAccum_(nAccum),
      nChannels_(nChannels), nSamples_(0), iLag_(iLag) { }

  /// Destructor.
  virtual ~ScopeObservation() { }

  /// (Re)initialize the observation.
  void init(zpec_mode_t mode, unsigned nLags, unsigned nAccum,
            unsigned nChannels, const int iLag[])
  {
    setMode(mode);
    nLags_     = nLags;
    nAccum_    = nAccum;
    nChannels_ = nChannels;
    nSamples_  = 0;
    iLag_      = iLag;
    memset(sampBuffer_, 0, sizeof(sampBuffer_));
  }

  /// Number of (complete) samples collected.
  unsigned nSamples() const { return nSamples_/nAccum_; }

  /// Sample buffer (read-only).
  const lag_count_t *sampBuffer() const { return sampBuffer_; }

  /// Sample \a iSample of channel \a iChannel.
  lag_count_t sample(unsigned iChannel, unsigned iSample) const {
    return sampBuffer_[nChannels_*iSample + iChannel];
  }

  virtual void processFrame(const lag_count_t *begin,
                            const lag_count_t *end);

private:
  /// The sample buffer.
  lag_count_t sampBuffer_[maxSamples];

  unsigned nLags_,   ///< Combined total number of lags in all bands.
	  nAccum_,   ///< Number of frames to accumulate per sample point.
       nChannels_,   ///< Number of ADC channels (lags) to sample.
	nSamples_;   ///< Current number of samples collected per channel.

  const int *iLag_;  ///< Lag corresponding to each sampling channel.
};

#endif  // OBSERVATIONS_H
