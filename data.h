#ifndef DATA_H
#define DATA_H
/**
  \file

  Band data management declarations.

  $Id: data.h,v 1.18 2007/10/03 22:06:29 rauch Exp $
*/
#include <basictypes.h>
#include <string.h>
#include <utils.h>

#include <map>
#include <string>
#include <vector>

#include "io.h"


/**
  Lag data management.  This class allows the raw lag data for a band to be
  manipulated and output. It supports multiple output buffers (each of the
  same length), implemented as a single vector.
*/
class LagData
{
public:
  /// Lag data container type.
  typedef std::vector<lag_count_t> container_type; 

  /// Lag container size type.
  typedef container_type::size_type size_type;

  /// Lag container value type.
  typedef container_type::value_type value_type;

  /// Lag iterator type.
  typedef container_type::iterator iterator;

  /// Lag constant iterator type.
  typedef container_type::const_iterator const_iterator;

  /// Default constructor.
  LagData(unsigned nLags = 128, unsigned nBuffers = 2) :
      lags_(nLags*nBuffers), frames_(nBuffers), time_(nBuffers) { }

  /// Returns number of lag buffers.
  unsigned getBuffers() const { return time_.size(); }

  /// Returns number of frames of data accumulated.
  unsigned getFrames(unsigned iBuffer) const { return frames_[iBuffer]; }

  /// Returns number of frames of data accumulated.
  void setFrames(unsigned iBuffer, unsigned frames) {
    frames_[iBuffer] = frames;
  }

  /// Returns the total number of lags.
  int getResolution() const { return size(); }

  /// Sets the lag buffer size.
  void setResolution(unsigned nLags, unsigned nBuffers) {
    unsigned n = nLags*nBuffers;
    if (n != size()) {
      lags_.reserve(n+(n&1));
      lags_.resize(n); 
    }
    if (nBuffers != getBuffers()) {
      frames_.reserve(nBuffers);
      frames_.resize(nBuffers);
      time_.reserve(nBuffers);
      time_.resize(nBuffers);
    }
  }

  /// Returns the lag data timestamp.
  DWORD getTime(unsigned iBuffer) const { return time_[iBuffer]; }

  /// Sets the lag data timestamp.
  void setTime(unsigned iBuffer, DWORD time) { time_[iBuffer] = time; }

  /// Iterator to start of container.
  iterator begin() { return lags_.begin(); }

  /// Iterator to start of container.
  const_iterator begin() const { return lags_.begin(); }

  /// Iterator to end of container.
  iterator end() { return lags_.end(); }

  /// Iterator to end of container.
  const_iterator end() const { return lags_.end(); }

  /// Assign operator.
  void assign(size_type n, const value_type& elem) { lags_.assign(n, elem); }

  /// Assign operator.
  void assign(const_iterator begin, const_iterator end) {
    lags_.assign(begin, end);
  }

  /// Index operator.
  container_type::reference operator[](unsigned idx)
  { return lags_[idx]; }

  /// Index operator (const).
  container_type::const_reference operator[](unsigned idx) const
  { return lags_[idx]; }

  /// Number of elements in all buffers.
  container_type::size_type size() const { return lags_.size(); }

  void write(int fd, unsigned iBuffer, unsigned nLags) const;

  /// C vector assignment operator.
  LagData& operator=(const container_type::value_type *lags) {
    memmove(&lags_[0], lags, lags_.size()*sizeof(container_type::value_type)); 
    return *this;
  }

private:
  container_type        lags_;   ///< Lag counts (32-bit unsigned).
  std::vector<unsigned> frames_; ///< Number of accumulated frames in lags_.
  std::vector<DWORD>    time_;   ///< Data acquisition time (ticks since boot).

  /// Initialize lags with ASCII "deadbeef" pattern.
  void deadbeef() {
    for (unsigned i=0; i<lags_.size(); i+=2) {
      lags_[i] = 0x44656164;  lags_[i+1] = 0x42656566;
    }
  }
};


/**
  Monitor data management. This class allows all available monitor points for
  a band to be queried. Monitor point names, values, and units should not
  contain whitespace characters. Dimensionless points can be defined by
  omitting the (optional) unit string, or setting it to "".
*/
class MonitorData
{
public:
  /// Monitor point dictionary type.
  typedef std::map<std::string, std::string> container_type; 

  /// Default constructor.
  MonitorData() : maxPoint_(0), maxValue_(0) { }

  size_t getMaxPoint() { return maxPoint_; }  ///< Maximum  name string length.
  size_t getMaxValue() { return maxValue_; }  ///< Maximum value string length.

  /// Retrieves a monitor point's value, or NULL if the point doesn't exist.
  const char *getValue(const char *key) const { return get(valueMap_, key); }

  /// Retrieves a monitor point's units, or NULL if the point doesn't exist.
  const char *getUnit(const char *key) const { return get(unitMap_, key); }

  /// Sets the current value of a monitor point.
  void set(const char *point, const char *value, const char *units = 0) {
    if (point) {
      valueMap_[point] = value; 
       unitMap_[point] = (units ? units : "");
      maxPoint_ = std::max(maxPoint_, strlen(point));
      maxValue_ = std::max(maxValue_, strlen(value));
    }
  }

  void write(int fd, const char *point = 0) const;

private:
  container_type valueMap_,  ///< Monitor point value dictionary.
                  unitMap_;  ///< Monitor point unit  dictionary.

  size_t maxPoint_,  ///< Maximum monitor point string length.
	 maxValue_;  ///< Maximum monitor value string length.

  /// Retrieve map value.
  const char *get(const container_type& map, const char *key) const {
    container_type::const_iterator entry = (key ? map.find(key) : map.end());
    return (entry != map.end() ? entry->second.c_str() : "????");
  }
};


/**
  Control parameter management. This class allows all available control points
  for an individual band to be set or queried.
*/
class ControlData
{
public:
  /// Default constructor.
  ControlData() : attenuation_(0) { }

  /// Returns the current attenuation setting.
  unsigned short getAttenuation() { return attenuation_; }
  unsigned short setAttenuation(unsigned short attenuation);

private:
  unsigned short attenuation_;  ///< Attenuator setting.
};


/**
  Data for single observing band. Some common control parameters may be
  duplicated between bands (the Zpectrometer design places two ADC sections 
  controlled by one CPU in a single chassis).
*/
class BandData
{
public:
  LagData     lags;     ///< Lag data.
  MonitorData monitor;  ///< Monitor data.
  ControlData control;  ///< Control parameters.

  /// Default constructor.
  BandData(unsigned nLags = 128, unsigned nBuffers = 2) :
      lags(nLags, nBuffers) { }
};

#endif  // DATA_H
