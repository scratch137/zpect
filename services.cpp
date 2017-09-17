/**
  \file

  Implements Zpectrometer TCP services.

  $Id: services.cpp,v 1.31 2007/10/04 19:10:31 rauch Exp $

  Currently three telnet-based services are provided:

  - The DataServer provides the most recent (coherent) set of lag data
    (read-only).
  - The MonitorServer provides the most recent (coherent) set of monitor data
    (read-only).
  - The ControlServer provides full interactive access to embedded control
    functionality.
*/
#include <predef.h> 

#include <stdio.h>
#include <string.h>

#include <startnet.h>
#include <tcp.h>
#include <udp.h>

#include "control.h"
#include "services.h"
#include "zpec.h"

const char *ControlService::ctlName = "Control";
const char *ControlService::prompt  = "uC> ";
const char *DataService::dataName   = "Data";
const char *MonitorService::monName = "Monitor";
const char *MessageService::msgName = "Message";


/**
  Reads a line of input one character at a time. Erase processing is performed.

  \param line    Input buffer.
  \param size    Buffer length.
  \param fd      Input file descriptor.
  \param noBlank Whether to return blank lines.

  \return The number of characters read, else a system error code.
*/
int Service::readLine(char *line, unsigned size, int fd, bool noBlank)
{
  unsigned nTot;
  int nRead;
  char input;

  nTot = 0;
  do {
    nRead = read(fd, &input, 1);
    if (nRead == 1) {
      if (input == '\b') {  // Erase processing.
        if (nTot > 0) { --nTot; }
      } else if (input == ';') { input = '\n'; }
      line[nTot++] = input;
      line[nTot] = '\0';  // Terminate now for strcmp() calls below.

      // End of line or end of input reached?
      // Recognized EOF strings are: "\x04", "\xFF\x0C", "\xFF\xEC".
      if (nTot == size-1 || strchr("\n\x04", input) ||
          (nTot >= 2 && (!strcmp(line+nTot-2, "\xFF\x0C") ||
	                 !strcmp(line+nTot-2, "\xFF\xEC")))) {
	if (strspn(line, " \t") == nTot && noBlank) {
	  nTot = 0;  // Ignore blank lines.
	} else {
	  return (int )nTot;  // Return line.
	}
      }
    }
  } while (nRead == 1);

  return nRead;
}


/**
  \param client Pointer to client to activate.

  \return OS_NO_ERR on success, or an error code if activation failed.
*/
int Service::openClient(Client *client)
{
  static const char *fn = "Service::openClient";
  int status = OS_NO_ERR;

  lock();
    if (nClients_ == maxClients) {
      zpec_warn_fn("%s: client limit exceeded", tag_);
      status = -1;
    } else {
      unsigned id = 0;

      // Find first free index number.
      for (unsigned aset=isActive_; aset&1; aset>>=1, ++id) ;

      if (id >= maxClients) {
	zpec_error_fn("%s: inconsistent member data", tag_);
	status = -1;
      } else {
	zpec_info("%s: attaching client %d", tag_, id);
	++nClients_;
	isActive_ |= (1U << id);  // Mark index as in use.
	client->myId()      = id;
	client->myService() = this;
      }
    }
  unlock();

  return status;
}

/**
  \param client Pointer to client to deactivate.
*/
void Service::closeClient(Client *client)
{
  static const char *fn = "Service::closeClient";
 
  lock();
    if (nClients_ == 0) {
      zpec_error_fn("%s: no active clients", tag_);
    } else {
      zpec_info("%s: detaching client %d", tag_, client->myId());
      --nClients_;
      isActive_ &= ~(1U << client->myId());  // Mark index as free.
      client->myService() = 0;
      delete client;
    }
  unlock();
}

/**
  Creates a service task which waits for new clients.
*/
void Service::start()
{
  static const char *fn = "Service::start";
 
  // Initialize lock here to avoid static initialization issues. 
  OSCritInit(&lock_);

  if (OS_PRIO_EXIST == OSTaskCreate(serviceTask, (void *)this,
				    (void *)&serviceStack_[serviceStackSize],
				    (void *)&serviceStack_[0], prio_)) {
    zpec_error_fn("%s task priority (%d) unavailable", tag_, prio_);
  } else {
    zpec_info("%s task initialized, priority = %d", tag_, prio_);
  }
}

/**
  Service task function. This function must be static; therefore, the instance
  pointer is passed as input to recover access to the concrete service.

  \param vsvc Service instance cast to a void pointer.

  \todo Safe to call multiple times (lock_ init)?
*/
void Service::serviceTask(void *vsvc)
{
  static const char *fn = "Service::serviceTask";

  // Recover access to service data.
  Service *svc = (Service *)vsvc;

  if (svc) {
    zpec_info("Offering service: %s", svc->getName());
    svc->offerService();
  } else {
    zpec_error_fn("Service pointer is NULL");
  }
}

/**
  Spawns a server task (at the next lowest available priority below the
  service task) to handle a new client. No server task with priority higher
  than the ADC readout task will be created.

  \param client Client the new server will handle.

  \return OS_NO_ERR on success, or an error code if the client could not be
	  activated.
*/
int Service::createServer(Client *client)
{
  if (openClient(client) == OS_NO_ERR) {
    int iClient = client->myId();
    BYTE prio = prio_, status = OS_PRIO_EXIST;

    for (prio = prio_-1; prio>ZPEC_ADC_PRIO && status==OS_PRIO_EXIST; --prio) {
      status = OSTaskCreate(serverTask, (void *)client,
			    (void *)&serverStack_[iClient][serverStackSize],
			    (void *)&serverStack_[iClient][0], prio);
      if (status == OS_NO_ERR) {
	zpec_info("%s: client %d priority is %d", tag_, iClient, prio);
      }
    }
    return status;
  } else {
    return -1;
  }
}

/**
  \param vclient Client pointer cast to a void pointer.
*/
void Service::serverTask(void *vclient)
{
  static const char *fn = "Service::serverTask";

  // Recover access to client and pass to server.
  Client *client = (Client *)vclient;
  Service *svc = client->myService();

  if (svc) {
    svc->handleClient(client);
    svc->closeClient(client);
  } else {
    zpec_error_fn("Client not attached to any service");
  }
}


/**
  Main TCP service routine. Listens for new incoming connections and
  connection. The connection is rejected if the client limit has been reached,
  or if no server task could be created.
*/
void TcpService::offerService()
{
  static const char *fn = "TcpService::offerService";

  setAddress();

  fdListen_ = listen(INADDR_ANY, getPort(), maxClients);

  if (fdListen_ >= 0) {
    zpec_info("%s: listening..", getTag(), getPort());

    while (1) {
      TcpClient *client = new TcpClient;
      char ipstr[16];

      client->fd = accept(fdListen_, &client->ip, &client->port, 0);
      (void )zpec_iptostr(ipstr, client->ip);
      zpec_info("%s: connection request from %s", getTag(), ipstr);

      // AH mod for testing: 
      // redirect input, output, and error to telnet session
      if (0) {
    	  //ReplaceStdio(0, client->fd);
    	  //ReplaceStdio(1, client->fd);
    	  //ReplaceStdio(2, client->fd);
      }

      if (client->fd >= 0) {
	if (createServer(client) == OS_NO_ERR) {
	  zpec_info("%s: accepting connection from %s", getTag(), ipstr);
	} else {
	  zpec_warn_fn("%s: rejecting connection from %s", getTag(), ipstr);
	  close(client->fd);
	  delete client;
	}
      } else {
	zpec_warn_fn("%s: accept() returned %d", getTag(), client->fd);
	delete client;
      }
    }
  } else {
    zpec_warn_fn("%s: listen() returned %d", getTag(), fdListen_);
  }
}


/**
  Main UDP service routine. Listens for subscribe/unsubscribe requests and 
  maintains the subscription list.
*/
void UdpService::offerService()
{
  static const char *fn = "UdpService::offerService";
  Client *dummy = new Client;
 
  setAddress();

  // Start a server --- one task handles all subscribers.
  if (createServer(dummy) == OS_NO_ERR) {
    // Create incoming packet FIFO and start listening.
    OSFifoInit(&fifo_);
    RegisterUDPFifo(getPort(), &fifo_);
    zpec_info("%s: listening..", getTag(), getPort());

    while (1) {
      UDPPacket dgram(&fifo_, 0);  // Wait for client request.
      char ipstr[16];

	if (dgram.Validate()) {
	  IPADDR ip = dgram.GetSourceAddress();
	  WORD port = dgram.GetSourcePort();

	  // Ignore blank lines.
	  if (dgram.GetDataSize() > 0 && *dgram.GetDataBuffer() == '\n') {
	    zpec_debug("%s: Ignoring blank line from %s:%hu", 
		       getTag(), zpec_iptostr(ipstr, ip), port);
	    continue;
	  }

	  if (dgram.GetDataSize() == 0 || *dgram.GetDataBuffer() == '0') {
	    // Unsubscribe the client.
	    lock();
	      container_type::iterator client = clientMap_.find(ip),
				       cmend  = clientMap_.end();
	    unlock();
	    if (client != cmend) {
	      zpec_info("%s: unsubscribing %s:%hu", getTag(),
			zpec_iptostr(ipstr, client->first), client->second);
	      send("\n", 2, client->first,  client->second);
	      lock();
		clientMap_.erase(client);
	      unlock();
	    }
	  } else {
	    // Subscribe the client. If the IP already exists, the old port is
	    // replaced with the current port.
	    lock();
	    if (clientMap_.size() < maxClients) {
		clientMap_[ip] = port; 
	      unlock();
	      zpec_info("%s: subscribing %s:%hu", getTag(),
			zpec_iptostr(ipstr, ip), port);
	    } else {
	      unlock();
	      zpec_warn_fn("%s: too many subscribers; "
			   "ignoring request from %s:%hu",
			   getTag(), zpec_iptostr(ipstr, ip), port);
	    }
	  }
	} else {
	  zpec_warn_fn("%s: UDP packet from %s is invalid, dropping..",
		    getTag(), zpec_iptostr(ipstr, dgram.GetSourceAddress()));
	}
    }
  } else {
    zpec_error_fn("%s: server creation failed", getTag());
  }
}


/**
  \brief Sends a UDP datagram.

  \param buffer Data buffer.
  \param nBytes Number of bytes in buffer.
  \param addr   Destination IP address.
  \param port   Destination port number.
*/
void UdpService::send(const void *buffer, WORD nBytes, IPADDR addr, WORD port)
{
  UDPPacket dgram;
  dgram.ResetData();
  dgram.AddData((BYTE *)buffer, nBytes);
  dgram.SetSourcePort(getPort());
  dgram.SetDestinationPort(port);
  dgram.Send(addr);
}


void SimpleTcpService::handleClient(Client *client)
{
  // Recover access to TCP/IP client.
  TcpClient *tcpClient = (TcpClient *)client;

  // Call handleRequest for each line of input.
  char request[1+maxRequest];
  int nRead;
  while ((nRead = readLine(request, sizeof(request), tcpClient->fd, true))
         > 0) {
    handleRequest(tcpClient, request); 
  }

  zpec_info("%s: client %d readLine() returned %d, closing connection",
            getTag(), client->myId(), nRead);
  close(tcpClient->fd);
}

void ControlService::handleClient(Client *client)
{
  static const char *fn = "ControlService::handleClient";
  char cmdLine[maxLine];

  // Recover access to TCP/IP client.
  TcpClient *tcpClient = (TcpClient *)client;

  // Flush telnet initialization string and send initial prompt.
  ReadWithTimeout(tcpClient->fd, cmdLine, sizeof(cmdLine)-1, TICKS_PER_SECOND);
  zpec_write_retry(tcpClient->fd, prompt, strlen(prompt), fn);

  // Read and process commands.
  int nRead;
  while ((nRead = readLine(cmdLine, sizeof(cmdLine), tcpClient->fd, false))
         >= 0) {
    Correlator::command_type cmd = {cmdLine, tcpClient->fd, tcpClient->fd};
    ::zpecShell.exec(cmdLine, cmd);
    if (strcmp(cmdLine, Correlator::statusEOF)) {
      strcat(cmdLine, prompt);
      zpec_write_retry(tcpClient->fd, cmdLine, strlen(cmdLine), fn);
    } else {
      break;
    }
  }

  if (nRead <= 0) {
    zpec_info("%s: client %d readLine() returned %d, closing connection",
	      getTag(), client->myId(), nRead);
  } else {
    zpec_info("%s: client %d EOF detected, closing connection", 
              getTag(), client->myId());
  }
  close(tcpClient->fd);
}


void DataService::handleRequest(TcpClient *client, char *request)
{
  char obs = 't';
  unsigned band = 0, nSend = 0;
  sscanf(request, "%c%u%u", &obs, &band, &nSend);

  unsigned nBuffers = (obs=='d' ? 2 : 1);
  if (nSend == 0) { nSend = ::zpectrometer.nLags()*nBuffers; }

  ::zpectrometer.lock();
    ::zpectrometer.Band(band).lags.write(client->fd, 0, nSend);
  ::zpectrometer.unlock();
}


void MonitorService::handleRequest(TcpClient *client, char *request)
{
  int interval = 0, count = 0;
  sscanf(request, "%i%i", &interval, &count);

  for (int iter=0; iter==0 || (interval>0 && iter!=count &&
			       !zpec_interrupt(client->fd)); ++iter) {
    ::zpectrometer.periph_lock();
      ::zpectrometer.monitorPoints(false).write(client->fd); 
    ::zpectrometer.periph_unlock();

    if (interval>0) {
      write(client->fd, "\r\n", 2);
      if (1+iter != count) { OSTimeDly((interval*TICKS_PER_SECOND)/10); }
    }
  }
}


void MessageService::handleClient(Client *client)
{
  // Initialize queues. Conceptually this should be done in the constructor,
  // but calling lock() and OSQInit() there trapped the CPU, so we defer it
  // until here (uC/OS static initialization order fiasco).
  OSQInit(&osQueue_, osQueuePool_, (BYTE )maxMessages);
  lock();
    for (int i=0; i<maxMessages; ++i) { msgStack_.push(msgPool_[i]); }
  unlock();

  container_type clients;
  while (1) {
    // Wait for a new message.
    BYTE error = OS_NO_ERR;
    char *msg = (char *)OSQPend(&osQueue_, 60*TICKS_PER_SECOND, &error);

    // A "KeepAlive" packet seems necessary to avoid UDP Send() failures.
    if (error == OS_TIMEOUT) { msg = ""; }

    lock();
      clients = clientMap_;
    unlock();

    // Send the message to subscribers.
    for (container_type::const_iterator client = clients.begin();
	  client != clients.end(); ++client) {
      send(msg, strlen(msg)+1, client->first, client->second);
    }

    // Return storage to pool.
    if (error != OS_TIMEOUT) {
      lock();
	msgStack_.push(msg);
      unlock();
    }
  }
}


/**
  \brief Posts a message to the server.

   The message is copied to internal storage before posting.

  \param msg Message to post.

  \return OS_NO_ERR on success, or OS_Q_FULL if the message could not be posted.
*/
int MessageService::post(const char *msg)
{
  char *lmsg = 0;

  lock();
    if (!msgStack_.empty()) {
      lmsg = msgStack_.top();
      msgStack_.pop();
    }
  unlock();

  if (lmsg) {
    // Copy message.
    strncpy(lmsg, msg, maxLength);
    lmsg[maxLength] = '\0';

    // Post it.
    return OSQPost(&osQueue_, lmsg);
  } else {
    return OS_Q_FULL;
  }
}


extern "C" {

/**
  \brief Posts a message to the global message server.

  This method is a C callback used by zpec_log() to post messages to the
  message service.

  \param msg   Message to post to message service.
  \param level Message priority.

  \return OS_NO_ERR on success, or OS_Q_FULL if the message could not be
	  posted.

  \see MessageService::post
*/
int zpec_post_msg(const char *msg, int level)
{
  if (level <= ::zpectrometer.getVerbosity()) {
    return ::zpectrometer.messageServer.post(msg);
  } else {
    return OS_NO_ERR;
  }
}

}  // extern "C"
