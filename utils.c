/**
  \file

   Miscellaneous utility functions.

   $Id: utils.c,v 1.34 2008/02/20 00:25:44 rauch Exp $
*/
#include <ctype.h>
#include <iosys.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <system.h>

#include "zpec.h"

/**
  Read user parameters from flash memory. The maximum data size is 8K.
  Validity of the data is flagged by verifying the signature field.

  \param flashData  Data structure to receive flash contents.
*/
void zpec_readFlash(flash_t *flashData)
{
  *flashData = *(flash_t *)GetUserParameters();
  flashData->valid = 1;

  /* Current revision. */
  if (flashData->signature == 0x12345679U) {
    return;
  }

  /* hw field was added in revision 0x12345679U. */
  flashData->hw = ZPEC_HW_GBT;
  if (flashData->signature == 0x12345678U) {
    return;
  }

  /* Unknown flash structure revision. */
  flashData->valid = 0;
}

/**
  Write a user parameter structure to flash memory.

  \param flashData Data structure to write.

  \return 0 on success, 1 if flash write failed, 2 if input structure not
  valid.
*/
int zpec_writeFlash(const flash_t *flashData)
{
  flash_t tmp = *flashData;
  if (flashData->signature != FLASH_SIGNATURE) { return(2); }
  return(! SaveUserParameters(&tmp, sizeof(tmp)));
}


/**
  Parse an integer range specification to a flat list.

  A range specification consists of a comma-separated list of of integers and
  integer ranges (e.g., 1-9). This routine (destructively) parses the range
  specification string to build a linear array of the set of all integers in
  the range. Elements are restricted to lie between the specified extreme
  values.

  \param spec     Input range specification (overwritten during parsing).
  \param specList Array to receive expanded list of integers in range.
  \param minValue Minimum value allowed in \a specList.
  \param maxValue Maximum value allowed in \a specList.
  \param maxElem  Maximum number of elements \a specList can contain.

  \return The number of elements assigned to \a specList.
*/
unsigned zpec_parse_spec(char *spec, int specList[/*maxElements*/],
                         int minValue, int maxValue, unsigned maxElem)
{
  unsigned j, nElem = 0;

  do {
    while (*spec == ',') ++spec;
    if (!isdigit(*spec) && *spec != '-') break;

    /* Assign next element. */
    specList[nElem++] = strtol(spec, &spec, 10);

    /* Expand range specifications. */
    if (*spec == '-') {
      int i, end = strtol(spec+1, &spec, 10);
      for (i=1+specList[nElem-1]; i<=end && nElem<maxElem; ++i) {
        specList[nElem++] = i;
      }
    }
  } while (nElem < maxElem);

  for (j=0; j < nElem; j++) {
    if (specList[j] < minValue) { specList[j] = minValue; }
    if (specList[j] > maxValue) { specList[j] = maxValue; }
  }

  return(nElem);
}


/**
  Format a fixed-point scaled integer as a fixed-point string.

  Converts a fixed-point integer to a fixed-point string. The true floating
  point value if the input \a fixed is \a fized/scale.

  \param str  Buffer holding formatted value (16 bytes minimum).
  \param fixed The scaled, fixed-point value.
  \param scale The scaling factor applied to \a fixed (must be a power of 10).

  \return The buffer \a str.
*/
char *zpec_print_fixed(char str[/*16*/], int fixed, int scale)
{
  if (scale == 1) {
    siprintf(str, "%d.", fixed);
  } else {
    /*
       Compute number of decimal digits given a scale factor 10^N.
       The algorithm successivley divides by 8 until 0 is obtained;
       the power of 10 is the number of divides minus 1. This works for
       scale factors between 1 and 10^9, inclusive.
    */ 
    div_t qr = div(abs(fixed), scale);
    int nFixed;

    for (nFixed = -1; scale != 0; ++nFixed) { scale >>= 3; }
    siprintf(str, "%s%d.%0*d", (fixed<0 ? "-" : ""), qr.quot, nFixed, qr.rem);
  }
  return(str);
}


/**
  Actively pause (busy-wait) for the specified number of microseconds.
*/
void zpec_usleep(unsigned usec)
{
  /* Number of loop iterations per 1024 us (147.5 MHz Coldfire 5270 CPU). */ 
  static const unsigned loops_per_ms = 2112;
  volatile unsigned long l1, l2, msec, n;

  /* Reduce range to avoid integer overflow. */
  msec = usec >> 10;
  for (l1=0; l1<msec; ++l1) {
    for (l2=0; l2<loops_per_ms; ++l2) ;
  }

  n = (loops_per_ms*(usec & 1023)) >> 10;
  for (l2=0; l2<n; ++l2) ;
}


/**
  Check for user-requested interrupt.

  This routine checks if data is available on \p fd. Any such data is
  taken to signify a soft (^C type) interrupt and discarded.
  
  \param fd Open, readable file descriptor to examine for input.

  \return 0 if no interrupt pending, else 1.

  \note This routine does \b not block.
*/
int zpec_interrupt(int fd)
{
  char line[32];
  int n= 0;

  if (fd >= 0) {
    while (dataavail(fd)) { n+= read(fd, line, sizeof(line)); }
  }

  return(n > 0);
}


/// Returns message priority string given the logging level.
const char *zpec_severity(int level)
{
  switch (level) {
    case ZPEC_LOG_ERROR:  return "Error";
    case ZPEC_LOG_WARN:   return "Warning";
    case ZPEC_LOG_INFO:   return "Info";
    case ZPEC_LOG_DEBUG:  return "Debug";
    default:              return "UNKNOWN";
  }
}


/**
  Change task priority (with retry).

  This routine changes the task priority to \a prio, or the next highest
  (i.e., lowered) priority that is available. If all priorities up to
  IDLE_PRIO are taken, the priority will not be changed.

  \param prio Desired new task priority.

  \return OS_NO_ERROR on success, else OS_PRIO_EXIST.
*/
BYTE zpec_change_prio(BYTE prio)
{
  BYTE rtn = OS_PRIO_EXIST;
  for (; prio < OS_LO_PRIO && rtn == OS_PRIO_EXIST; ++prio) {
    rtn = OSChangePrio(prio);
  }
  return rtn;
}

/**
  Returns the code version string.

  This routine uses the CVS Name tag to form the version string; release tags
  should have the form release-A_B[_C]-YYYY_MM_DD. If absent (i.e., for
  development releases), a manually defined version string is used.

  \return A version string of the form "VERSION (DATE)".
*/
extern const char *zpec_version(void)
{
  static char ver_str[32];
  const char *cvs_ver = "CVS-1.2gamma4",
	     *tag_ver = "$Name:  $";

  sscanf(tag_ver, "%*s%s", ver_str);

  if (ver_str[0] == '$') {
    /* No release tag. */
    return(cvs_ver);
  } else {
    /* Reformat: release-A_B[_C]-YYYY_MM_DD ==> release A.B[.C] (YYYY-MM-DD) */
    char *p = strchr(ver_str, '-');

    if (p) {
      int dot = 1;  /* Whether _ maps to . (else -). */

      *p = ' ';
      while (*++p) {
	if (*p == '-') { *p = ' '; dot = 0; }
	else if (*p == '_') { *p = (dot ? '.' : '-'); }
      }
    }
    return(ver_str);
  }
}


/**
  Logs a message.
  
  This routine sends a message to both the message service and the serial 
  debug port (stderr). Each message is prepended with a descriptive severity
  level string. Recognized message levels are as follows:
  - ZPEC_LOG_ERROR Error
  - ZPEC_LOG_WARN  Warning
  - ZPEC_LOG_INFO  Info
  - ZPEC_LOG_DEBUG Debug

  \param msg   Log message.
  \param level Message severity level (lower is more severe).

  \note
  Debug-level messages are not posted to the message server, both to limit
  volume and allow a channel for debugging the message service itself.

  \warning Messages are limited to 255 bytes (not checked).
*/
void zpec_log(const char *msg, int level)
{
  char buffer[256];

  siprintf(buffer, "%s: %s%s\r\n",
           zpec_severity(level), msg, level<=ZPEC_LOG_ERROR ? "!" : ".");
 
  /* Send to debug port. */
  fputs(buffer, stderr);

  /* Send to message service. */
  if (level < ZPEC_LOG_DEBUG) { zpec_post_msg(buffer, level); }
}


/** Returns the serial number of the backend. */
int zpec_getSerialNo(void)
{
  flash_t flashData;
  zpec_readFlash(&flashData);
  return(flashData.valid ? flashData.serialNo : -1);
}


/**
  Converts a binary IP address to a formatted string. The string \b must
  contain at least 16 characters.

  \param str  String in which to format IP address.
  \param addr Input numeric IP address.

  \return The string \a str.
*/
char *zpec_iptostr(char *str, IPADDR addr)
{
  PBYTE pb = (PBYTE)&addr;
  siprintf(str, "%d.%d.%d.%d",(int)pb[0],(int)pb[1],(int)pb[2],(int)pb[3]);
  return(str);
}


/**
  Write call with built-in retry and error logging. This method makes a second
  call to write() if the first returns an (unknown) error or times out, and
  logs appropriate messages.

  \param fd     Open file descriptor to receive output.
  \param buf    Output data buffer.
  \param nbytes Number of bytes in \a buf.
  \param fn     Name of calling function (for error localization); can be NULL.
*/
void zpec_write_retry(int fd, const void *buf, unsigned nbytes, const char *fn)
{
  unsigned iter = 1, status = 0;

  /* Retry once, and as long as one or more bytes are written. */
  while (nbytes > 0 && (iter <= 2 || status > 0)) {
    status = write(fd, (const char *)buf, nbytes);
    if (status == -3) { return; }  // Connection closed at remote end.

    if (status < nbytes) {
      zpec_warn_fn("write() #%d returned %d (requested %d)",
		    iter, status, nbytes);
    }
    if (status > 0) {
      buf    += status;
      nbytes -= status;
    }
    ++iter;
  }

  if (nbytes > 0) {
    zpec_error_fn("write() failed with code %d", status); 
    iprintf("write() buffer: %.*s\r\n", nbytes, buf);
  }
}


/**
  Write out contents of "nearly" full buffers. This method checks whether
  a buffer has exceeded its soft capacity limit. If so, its contents are
  written to the provided stream and the buffer length is updated. If the
  file descriptor is invalid, the buffer is truncated and an error status is
  appended to indicate the failure.

  \param fd       Open file descriptor to receive output.
  \param buf      Output data buffer.
  \param nbytes   Number of bytes in \a buf.
  \param maxbytes Threshold for flushing current buffer contents.
  \param fn       Name of calling function (for error localization);
                  can be NULL.
*/
void zpec_write_if_full(int fd, void *buf, unsigned *nbytes, unsigned maxbytes,
  const char *fn)
{
  /* If buffer size has exceeded desired limit, empty it. */
  if (*nbytes >= maxbytes) {
    if (fd >= 0) {
      zpec_write_retry(fd, buf, *nbytes, fn);
      *nbytes = 0;
    } else {
      zpec_warn_fn("Bad file descriptor in zpec_write_if_full() [fd=%d]", fd);
      *nbytes = siprintf(buf, "! ..output truncated (fd = %d)!\r\n", fd);
    }
  }
}


/*
  Copy string to buffer with overflow handling. This method determines whether
  the given string will fit into the supplied buffer. If not, the initial
  excess is output to the specified stream. If the stream is invalid, the
  buffer is truncated.

  \param fd       Open file descriptor to receive output.
  \param buf      Output data buffer.
  \param str      Input character string (NULL-terminated).
  \param maxbytes Threshold for flushing current buffer contents.
  \param fn       Name of calling function (for error localization);
                  can be NULL.
*/
void zpec_write_strcpy(int fd, void *buf, const char *str, unsigned maxbytes,
    const char *fn)
{
  unsigned nbytes = strlen(str);

  if (fd >= 0) {
    while (nbytes >= maxbytes) {
      zpec_write_retry(fd, str, maxbytes, fn);
      nbytes -= maxbytes;
      str    += maxbytes;
    }
  } else {
    if (nbytes >= maxbytes) {
      zpec_warn_fn("Bad file descriptor in zpec_write_strcpy) [fd=%d]", fd);
      nbytes = maxbytes - 6;
      memmove(buf+1+nbytes, "...\n", 5);
    }
  }
  memmove(buf, str, 1+nbytes);
}


/**
  Crudely estimate 2^16*log2(x).

  First, \a x is shifted until its MSB is 1; the 4 next MSBs (rounded) are
  then used with a look-up table to estimate the base 2 logarithm of the
  resulting mantissa. The final estimate is good to within 1%.

  \param x Input value.
  \return  2^16*log2(x) to 1% accuracy (-2^16 for x = 0).
*/
int zpec_log2_fix16(unsigned x)
{
  /* mant16[i] = 2^16*log2(1/2 + i/2^5). */
  static const int mant16[17] = {
     -65536, -59804, -54400, -49288, -44438, -39825, -35427, -31224,
     -27200, -23340, -19632, -16064, -12625, -9307, -6102, -3002, 0
  };
  int expon = (32U<<16);

  for (; expon>0 && (x&(1U<<31)) == 0; x<<=1) { expon -= (1U<<16); }
  return(expon+mant16[((x>>27) & 15) + ((x & 0x7FFFFFF) >= 0x4000000)]);
}


/**
  Crudely estimate 2^16*log10(x).

  \param x Input value.
  \return  2^16*log10(x) to 1% accuracy (-2^16 for x = 0).
*/
int zpec_log10_fix16(unsigned x)
{
  const int log10_2_fix12 = 1233; /* 2^12*log10(2) */
  int log2x = zpec_log2_fix16(x);

  return(log2x<=0 ? log2x : (log10_2_fix12*(log2x>>1))>>11);
}
