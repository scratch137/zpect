/**
  \file

   Routines for processing lag data.

   $Id: data.cpp,v 1.9 2007/10/03 22:06:18 rauch Exp $
*/
#include <basictypes.h>
#include <constants.h>
#include <iosys.h>
#include <stdio.h>
#include <string.h>

#include "data.h"
#include "zpec.h"


/**
  \brief Writes lag data to a file descriptor.

  This method writes binary lag data to an open file. The data block includes
  a header and is formatted as follows:\n
  \verbatim
    # iBuffer nLags nFrames Time
    lag_0...lag_(nLags-1)
  \endverbatim

  The \p iBuffer, \p nLags, \p nFrames, and \p Time fields will be set to the
  (starting) buffer number, the total number of lags written (possibly
  spanning multiple buffers), the number of readout frames accumulated
  per lag, and the time (in seconds since reboot) at which the lags were
  collected, each formatted as decimal ASCII integers. Lags are returned as a
  contiguous block of 32-bit signed (twos-complement) binary integers, each in
  standard network byte order.

  \note Multiple buffers
  \param fd      Open file descriptor to receive output.
  \param iBuffer Lag buffer to output.
  \param nLags   Number of lags to output (0 means "all in this buffer").

  \warning The current implementation assumes native byte order is the same as
	   network byte order, as is true for Zpectrometer (Coldfire) CPUs.
*/
void LagData::write(int fd, unsigned iBuffer, unsigned nLags) const
{
  static const char *fn = "LagData::write";
  unsigned nChars, nWrite;
  char header[32];

  nWrite = (nLags < 1 ? size()/getBuffers() :
            nLags > size() ? size() : nLags);
  nChars = siprintf(header, "# %u %u %u %u.%02u\n",
                    iBuffer, nWrite, frames_[iBuffer],
		    time_[iBuffer]/TICKS_PER_SECOND,
		    (time_[iBuffer] % TICKS_PER_SECOND)*(100/TICKS_PER_SECOND));
  zpec_write_retry(fd, header, nChars, fn);

  // Assumes native byte ordering matches standard network byte order.
  zpec_write_retry(fd, &lags_[0],
                   nWrite*sizeof(container_type::value_type), fn);
}


/**
  Outputs a monitor point. If \a point is NULL, all monitor points are output.
  The format of the output is:
  \verbatim
    Point = Value Units
  \endverbatim

  \param fd    Open file descriptor to receive output.
  \param point Monitor point name (can be NULL).

  \note If the value is empty (or has not been set), UNKNOWN is printed.
*/
void MonitorData::write(int fd, const char *point) const
{
  static const char *fn = "MonitorData::write";
  container_type::const_iterator iUnit, iValue, endValue;
  
  if (point) {
     iUnit   = unitMap_.find(point);
    iValue   = valueMap_.find(point);
    endValue = iValue;
    if (iValue != valueMap_.end()) { ++endValue; }
  } else {
     iUnit   = unitMap_.begin();
    iValue   = valueMap_.begin();
    endValue = valueMap_.end();
  }

  for (; iValue != endValue; ++iUnit, ++iValue) {
    size_t len;
    char str[256];

    len = siprintf(str, "%*s = %*s %s\n", 
              (int )maxPoint_, iValue->first.c_str(), (int )maxValue_,
	      iValue->second.length() ? iValue->second.c_str() : "UNKNOWN",
	      iUnit->second.c_str());
    zpec_write_retry(fd, str, len, fn);
  }
}


/**
  Sets the attenuation to a new value.

  \param attenuation New attenuation setting.
  \return Previous attenuation setting.
*/
unsigned short ControlData::setAttenuation(unsigned short attenuation) 
{
  unsigned short old_atten = attenuation_;
  attenuation_ = attenuation;
  return old_atten;
}
