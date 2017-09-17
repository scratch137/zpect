#ifndef SERVICES_H
#define SERVICES_H
/**
  \file

  Classes implementing Zpectrometer services.

  $Id: services.h,v 1.20 2014/03/21 15:25:51 rauch Exp $
*/
#include <stdio.h>
#include <string.h>
#include <startnet.h>
#include <ucos.h>

#include <map>
#include <stack>

#include "zpec.h"

// Note: priorities staggered to create holes for associated server tasks.
#define ZPEC_MESSAGE_PRIO (OS_LO_PRIO - 9)  ///< Message service task priority.
#define ZPEC_CONTROL_PRIO (OS_LO_PRIO - 6)  ///< Control service task priority.
#define ZPEC_DATA_PRIO    (OS_LO_PRIO - 4)  ///< Data    service task priority.
#define ZPEC_MONITOR_PRIO (OS_LO_PRIO - 2)  ///< Monitor service task priority.

extern "C" {
  /// C linkage uC/OS task function.
  typedef void (task_fn_t)(void *);
}

class Service;


/**
  Client base class. All clients have an associated identification number.
  Active clients also have a valid reference to a service.
*/
class Client
{
public:
  /// Constructor.
  Client(Service *service = 0, unsigned id = 0) : service_(service), id_(id) { }

  /// Destructor.
  virtual ~Client() { }

  /// Associated service (read/write).
  Service*& myService() { return service_; }

  /// Id number (read/write).
  unsigned& myId() { return id_; }

private:
  Service *service_;  ///< Attached service (if any).
  unsigned id_;       ///< Client identification number (set by service).
};

/// Internet client.
class IpClient: public Client
{
public:
  IPADDR ip;  ///< Client IP address.
  WORD port;  ///< Client port number.

  /// Constructor.
  IpClient(Service *svc = 0, int id = 0, IPADDR addr = 0, WORD num = 0) :
      Client(svc, id), ip(addr), port(num) { }
};

/// TCP/IP client.
class TcpClient: public IpClient
{
public:
  int    fd;  ///< Client socket file descriptor.

  /// Constructor.
  TcpClient(Service *svc = 0, int id = 0, IPADDR addr = 0, WORD port = 0,
           int sock = -1) :
      IpClient(svc, id, addr, port), fd(sock) { }
};


/**
  Abstract system service class. This class is patterned after UNIX init.d
  service scripts. A service is an entity that can be started and stopped.
  Services have a name, an associated priority---in this case, corresponding
  to the OS priority of the service task---and zero or more active clients.
  Each client communicates with an independent server task.

  Concrete services need to implement two methods, offerService(), which
  enables the service, obtains clients, and passes each off to a server via
  a call to createServer(); and handleClient(), which processes communication
  with a single active client.

  \warning offerService() \b must dynamically allocate the client using the \p
	   new operator, and \b must \p delete it \b iff its call to
	   createServer() fails.
*/
class Service
{
public:
  static const unsigned maxClients = 4;  ///< Maximum number of active clients.

  /**
    Constructor.

    \param name       Service name.
    \param prio       Service task priority.

    \note Initialization of lock_ deferred to start() to mitigate static
          initialization order issues.
  */
  Service(const char *name, BYTE prio) :
      prio_(prio), name_(name), nClients_(0), isActive_(0) { setTag(); }

  /// Destructor.
  virtual ~Service() { stop(); }

  /// Initialize (enable) the service.
  virtual void start();

  /// Initialize the service without creating a new task.
  virtual void startSameTask() {
    OSCritInit(&lock_);  OSChangePrio(prio_);
    zpec_info("%s task priority = %d", tag_, prio_);
    serviceTask((void *)this);
  }

  /// Terminate (disable) the service. Disabling a service includes closing
  /// all active instances and releasing any associated resources.
  virtual void stop() { }

  /// Return service name.
  const char *getName() const { return name_; }

  /// Return service tag.
  const char *getTag() const { return tag_; }

protected:
  /// Standardized service description string.
  char tag_[32];

  /// Offer the service to clients.
  virtual void offerService() = 0;

  /// Handle communication with a client.
  virtual void handleClient(Client *) = 0;

  /// Set the service tag.
  virtual void setTag() { siprintf(tag_, "%s service", name_); }

  /// Create a new server task to process a client.
  int createServer(Client *);

  /// Read input using erase processing.
  int readLine(char *line, unsigned size, int fd, bool noBlank);

  /// Lock the mutex.
  void lock()   { OSCritEnter(&lock_, 0); }

  /// Unlock the mutex.
  void unlock() { OSCritLeave(&lock_); }

private:
  static const int
    serviceStackSize = USER_TASK_STK_SIZE,  ///< Service task stack size.
    serverStackSize  = USER_TASK_STK_SIZE;  ///< Server  task stack size.

  /// Service task stack space.
  DWORD serviceStack_[serviceStackSize] __attribute__( ( aligned( 4 ) ) );

  /// Server tasks stack space.
  DWORD serverStack_[maxClients][serverStackSize]
            __attribute__( ( aligned( 4 ) ) );

  OS_CRIT lock_;       ///< Service data mutex.
  BYTE prio_;          ///< OS priority of service task.
  const char *name_;   ///< Service name.
  unsigned nClients_;  ///< Number of active clients.
  unsigned isActive_;  ///< Active client bit set.

  // OS task functions; declared via typedef to force C linkage conventions.
  static task_fn_t serviceTask, serverTask;

  /// Reserve record storage for new client and attach it to service.
  int openClient(Client *);

  /// Release records for a terminated client, then delete it.
  void closeClient(Client *);
};


/**
  Abstract internet service class. An internet service is a service with
  an associated server IP address, and port number. Concrete services should
  call setAddress() early in offerService() to set their IP address.
*/
class IpService: public Service
{
public:
  /// Constructor.
  IpService(const char *name, BYTE prio, WORD port) :
      Service(name, prio), addr_(0), port_(port) { }

  virtual void setTag() {
    char str[16];
    siprintf(tag_, "%s service (%s:%d)",
             getName(), zpec_iptostr(str, addr_), port_);
  }

  /// Returns IP address.
  IPADDR getAddress() { return addr_; }

  /// Return port number.
  WORD getPort() { return port_; }

protected:
  /**
    Sets the IP address.

    \warning Before network initialization, the EthernetIP variable is invalid.
	     Do not call this method before starting the network!
  */
  void setAddress() { addr_ = EthernetIP;  setTag(); }

private:
  IPADDR addr_;  ///< The server IP address.
  WORD   port_;  ///< The server port number.
};


/**
  A generic TCP/IP service. A TCP service is an IP service that has a readable
  socket file descriptor when enabled. The descriptor is used to listen for
  client connection requests. Concrete instances must implement the
  handleClient() method to process client communications.
*/
class TcpService: public IpService
{
public:
  /// Constructor.
  TcpService(const char *name, BYTE prio, WORD port) :
      IpService(name, prio, port), fdListen_(-1) { }

  virtual void offerService();

private:
  int fdListen_;  ///< Listening socket file descriptor.
};

/**
  A "simple" TCP service. A simple service is defined as one in which clients
  communicate via short, newline-terminated requests. Blank lines are ignored,
  and backspace processing is performed. Each request received from the
  client is then passed to TcpService::handleRequest for response, this being
  the only method concrete simple services need to implement.
*/
class SimpleTcpService: public TcpService
{
public:
  /// Maximum request length.
  static const unsigned maxRequest = 255;

  /// Constructor.
  SimpleTcpService(const char *name, BYTE prio, WORD port) :
      TcpService(name, prio, port) { }

  void handleClient(Client *client);

  /**
    Process a single request from a simple TCP client.

    \param client  Attached (active) client.
    \param request Client request string.
  */
  virtual void handleRequest(TcpClient *client, char *request) = 0;
};


/**
  A generic UDP service. A UDP service is an IP service that sends datagrams
  to subscribers as they arrive. Transmission is connectionless and packet
  delivery is not guaranteed. Clients subscribe to the service by sending a
  datagram to the service port. If the payload is the ASCII digit '0' (or
  is empty), the client IP is unsubscribed; otherwise, the client is
  subscribed to the service. Blank lines are ignored (cf. utelnet.c). The
  subscription will be sent to the originating port number. Only one
  subscription per IP address is allowed. Concrete services must implement the
  handleClient() method to service subscribers; this function should never
  return as it would terminate the entire service.
*/
class UdpService: public IpService
{
public:
  /// Client list container type.
  typedef std::map<IPADDR, WORD> container_type;

  /// Constructor.
  UdpService(const char *name, BYTE prio, WORD port) :
      IpService(name, prio, port) { }

  virtual void offerService();

protected:
  /// Active client list. Key is the IP address; value is the port number.
  container_type clientMap_;

  void send(const void *buffer, WORD nBytes, IPADDR addr, WORD port);

private:
  OS_FIFO fifo_;  ///< UDP packet FIFO.
};


/**
  Zpectrometer control service. The control service uses the telnet
  protocol to interactively receive and execute pre-defined control commands.
  It is the local equivalent of a shell interpreter with a built-in command
  set (and nothing else). Typing 'help' (or a blank line) will print a list of
  recognized commands. Service continues until the connection is closed by
  the client.

  The server provides positive feedback to all command requests by sending a
  human-readable return string indicating the return status.  The first
  character returned is an exit code crudely describing the result: '#' for
  success, '?' for warning (unexpected but non-fatal conditions), and '!' for
  failures.  The string may contain multiple lines of text.  Responses are
  crafted such that these characters will never appear as the FIRST character
  of any line beyond the initial status line, allowing multiple, concatenated
  responses to be distinguished by clients.

  \todo Merge line erase processing with that used by the
        SimpleTcpService class.

  \see Zpectrometer
*/
class ControlService: public TcpService
{
public:
  /// Control service task priority.
  static const BYTE  ctlPrio = ZPEC_CONTROL_PRIO;
  static const char *ctlName,         ///< Control service name.
		    *prompt;          ///< Command line prompt (separator).
  static const WORD  ctlPort = 23;    ///< Control service port number.
  static const size_t maxLine = 2048; ///< Maximum input line length.

  /// Constructor.
  ControlService() : TcpService(ctlName, ctlPrio, ctlPort) { }

  void handleClient(Client *client);
};


/**
  Zpectrometer lag data service. The data service uses simple TCP (telnet)
  protocol to send lag data on-demand to connected clients. Clients request
  data transfer by sending a newline-terminated request of the form:
\verbatim
  BUFFER BAND [NSEND] \n
\endverbatim
  \c BUFFER is a single character ('d', 'm', 'o', or 't') denoting the specific
  buffer for which to retrieve data. The character refers to the control
  command used to generate the corresponding data (dobs, meanvar, scope, or
  totpwr).  \c BAND is a decimal integer representing the band for which to
  return data. The optional \c NSEND is the number of data channels (starting
  with 0) to return; the default is to send the entire buffer.

  The server then sends a binary lag data block by calling
  LagData::write(). If a non-existent band is requested, band \#0 is used.
  %Service continues until the connection is closed by the client.

  \todo Implement 'm' and 'o' buffers.

  \see Correlator::execDiodeObs(), Correlator::execStatsObs(),
       Correlator::execScopeObs(), Correlator::execTotalPower()
*/
class DataService: public SimpleTcpService
{
public:
  /// Data service task priority.
  static const BYTE  dataPrio = ZPEC_DATA_PRIO;
  static const char *dataName;               ///< Data service name.
  static const WORD  dataPort = 2112;        ///< Data service port number.

  /// Constructor.
  DataService() : SimpleTcpService(dataName, dataPrio, dataPort), p_(cmd_) { }

  void handleRequest(TcpClient *client, char *request);

private:
  char cmd_[32],   ///< Command line buffer.
      *p_;         ///< End of current (partial) command line.
};


/**
  Zpectrometer monitor data service. The monitor service uses simple TCP
  (telnet) protocol to send monitor data on-demand to connected clients.
  Clients request data transfer by sending a single line (newline-terminated)
  per request. The server then sends the most recent monitor point values to
  the client by calling MonitorData::write().

  Each line shall contain one or two decimal decimal numbers indicating the
  sampling INTERVAL (in dsec) and COUNT, respectively.  If INTERVAL is
  positive, then COUNT sets of readings are returned, each block separated by
  a blank line. If COUNT is zero, the cycle repeats indefinitely until another
  line of input is received (and is discarded). At this point, a new request
  can be made. This behavior is equivalent to the \c query control command.

  %Service continues until the connection is closed by the client.
*/
class MonitorService: public SimpleTcpService
{
public:
  /// Monitor service task priority.
  static const BYTE  monPrio = ZPEC_MONITOR_PRIO;
  static const char *monName;               ///< Monitor service name.
  static const WORD  monPort = 5150;        ///< Monitor service port number.

  /// Constructor.
  MonitorService() : SimpleTcpService(monName, monPrio, monPort) { }

  void handleRequest(TcpClient *client, char *request);
};


/**
  Zpectrometer message service. The message service uses UDP datagrams to send
  log messages to subscribers. See UdpService for details on the subscription
  procedure. Log messages are posted to the server via calls to zpec_log(),
  which uses a ?? to transfer messages. The server waits for new messages,
  then sends a datagram to each subscriber. The payload of the datagram, a
  standard C string, is the message.
*/
class MessageService: public UdpService
{
public:
  static const int maxMessages = 63,  ///< Maximum number of queued messages.
	           maxLength   = 255; ///< Maximum message length.

  /// Message service task priority.
  static const BYTE  msgPrio = ZPEC_MESSAGE_PRIO;
  static const char *msgName;               ///< Message service name.
  static const WORD  msgPort = 1984;        ///< Message service port number.

  /// Constructor. \see MessageService::handleClient
  MessageService() : UdpService(msgName, msgPrio, msgPort) { }

  void handleClient(Client *client);

  int post(const char *msg);

private:
  /// OS queue structure for posting messages.
  OS_Q osQueue_;

  /// OS queue storage area.
  void *osQueuePool_[maxMessages];

  /// Message storage area.
  char msgPool_[maxMessages][maxLength+1];

  /// Stack of free message pointers.
   std::stack<char *> msgStack_;
};

#endif  //  SERVICES_H
