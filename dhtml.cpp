/**
  \file

   Methods implementing dynamic HMTL content.
   
   $Id: dhtml.cpp,v 1.4 2006/09/12 15:45:38 rauch Exp $

   DHTML callbacks are declared with C linkage for easy external comsumption
   but are C++ code to allow access to the full set of control and data APIs.
*/
#include <iosys.h>
#include <stdio.h>

#include "zpec.h"


extern "C" {

/**
  Writes the serial number of the backend to a socket.

  \param sock Socket descriptor.
  \param url  URL of calling web page (not used).
*/
void dhtml_getSerialNo(int sock, const char *url)
{
  static const char *fn = "dhtml_getSerialNo()";
  int nbytes, serialNo;
  char buf[16];

  serialNo = zpec_getSerialNo();
  if (serialNo >= 0) {
    nbytes = siprintf(buf, "%d", zpec_getSerialNo());
  } else {
    nbytes = siprintf(buf, "??");
  }
  zpec_write_retry(sock, buf, nbytes, fn);
}

}  // extern "C"
