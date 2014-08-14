/********************************************
 *  sshDriver.cpp
 * 
 *  SSH wrapper class for the libssh2 library
 *  This class provides standard read/write
 *  and flush methods for an ssh connection.
 * 
 *  Alan Greer
 *  21 Jan 2013
 * 
 ********************************************/

#include "sshDriver.h"
#include <sys/time.h>

#include <osiUnistd.h>
#include <osiSock.h>

/*
 * Uncomment the DEBUG define and recompile for lots of
 * driver debug messages.
 */
//#define DEBUG 1
//#define DEBUG_CTRL_CHARS

#ifdef DEBUG
#define debugPrint printf
#else
void debugPrint(...){}
#endif

/**
 * Constructor for the SSH driver.  Accepts a host name or IP
 * address.  The class will attempt to resolve the name to an
 * IP address before connecting.  Initializes internal variables.
 *
 * @param host - Host name/IP to attempt a connection with.
 */
SSHDriver::SSHDriver(const char *host)
{
  static const char *functionName = "SSHDriver::SSHDriver";
  debugPrint("%s : Method called\n", functionName);

  // Initialize internal SSH parameters
  auth_pw_ = 0;
  got_ = 0;
  connected_ = 0;
  // Username and password currently set to empty strings
  strcpy(username_, "");
  strcpy(password_, "");
  // Store the host address
  strcpy(host_, host);
}

/**
 * Setup the username for the connection.  Obviously the
 * username must exist on the device running the SSH
 * server.
 *
 * @param username - Username for the SSH connection.
 * @return - Success or failure.
 */
SSHDriverStatus SSHDriver::setUsername(const char *username)
{
  static const char *functionName = "SSHDriver::setUsername";
  debugPrint("%s : Method called\n", functionName);

  // Store the username
  strcpy(username_, username);

  return SSHDriverSuccess;
}

/**
 * Setup the password for the username on this connection.
 * A password does not need to be entered.  If it is not then
 * key based authorization will be attempted.
 *
 * @param password - Password for the SSH connection.
 * @return - Success or failure.
 */
SSHDriverStatus SSHDriver::setPassword(const char *password)
{
  static const char *functionName = "SSHDriver::setPassword";
  debugPrint("%s : Method called\n", functionName);

  // Store the password
  strcpy(password_, password);

  // Set the password authentication to on
  auth_pw_ = 1;

  return SSHDriverSuccess;
}

/**
 * Attempt to create a connection and authorize the username
 * with the password (or by keys).  Once the connection has
 * been established a dumb terminal is created and an attempt
 * to read the initial welcome lines is made.
 *
 * @return - Success or failure.
 */
SSHDriverStatus SSHDriver::connectSSH()
{
  unsigned long hostaddr;
  int rc;
  int i;
  static const char *functionName = "SSHDriver::connect";
  debugPrint("%s : Method called\n", functionName);

#ifdef WIN32
  WSADATA wsadata;

  WSAStartup(MAKEWORD(2,0), &wsadata);
#endif

  in_addr inhost;
  if (hostToIPAddr(host_, &inhost) < 0){
    debugPrint("%s : libssh2 unknown host %s\n", functionName, host_);
    return SSHDriverError;
  }
  hostaddr = inhost.s_addr;
  debugPrint("%s : String host address (%s)\n", functionName, host_);
  debugPrint("%s : libssh2 host address (%ld)\n", functionName, hostaddr);

  rc = libssh2_init(0);
  if (rc != 0) {
    debugPrint("%s : libssh2 initialization failed, error code (%d)\n", functionName, rc);
    return SSHDriverError;
  }

  // Create the socket neccessary for the connection
  sock_ = socket(AF_INET, SOCK_STREAM, 0);

  sin_.sin_family = AF_INET;
  sin_.sin_port = htons(22);
  sin_.sin_addr.s_addr = hostaddr;
  if (connect(sock_, (struct sockaddr*)(&sin_), sizeof(struct sockaddr_in)) != 0){
    debugPrint("%s : socket failed to connect!\n", functionName);
    return SSHDriverError;
  }

  // Create a session instance
  session_ = libssh2_session_init();
  if(!session_){
    debugPrint("%s : libssh2 failed to create a session instance\n", functionName);
    return SSHDriverError;
  }

  // Start up the session. This will trade welcome banners, exchange keys,
  // and setup crypto, compression, and MAC layers
  rc = libssh2_session_handshake(session_, sock_);
  if(rc){
    debugPrint("%s : libssh2 failure establishing SSH session: %d\n", functionName, rc);
    return SSHDriverError;
  }

  // Here we now have a connection that will need to be closed
  connected_ = 1;

  // At this point the connection hasn't yet authenticated.  The first thing to do
  // is check the hostkey's fingerprint against the known hosts.
  const char *fingerprint = libssh2_hostkey_hash(session_, LIBSSH2_HOSTKEY_HASH_SHA1);
  debugPrint("%s : SSH fingerprint: ", functionName);
  for(i = 0; i < 20; i++) {
    debugPrint("%02X ", (unsigned char)fingerprint[i]);
  }
  debugPrint("\n");

  if (auth_pw_ == 1){
    // Authenticate via password
    if (libssh2_userauth_password(session_, username_, password_)) {
      debugPrint("%s : SSH authentication by password failed.\n", functionName);
      disconnectSSH();
      return SSHDriverError;
    } else {
      debugPrint("%s : SSH authentication by password worked.\n", functionName);
    }
  } else {
    /* Or by public key */
    char rsapubbuff[256];
    char rsabuff[256];
    sprintf(rsapubbuff, "/home/%s/.ssh/id_rsa.pub", username_);
    sprintf(rsabuff, "/home/%s/.ssh/id_rsa", username_);
    if (libssh2_userauth_publickey_fromfile(session_, username_, rsapubbuff, rsabuff, password_)){
      debugPrint("%s : SSH authentication by public key failed\n", functionName);
      disconnectSSH();
      return SSHDriverError;
    }
  }

  // Open the channel for read/write
  channel_ = libssh2_channel_open_session(session_);
  debugPrint("%s : SSH channel opened\n", functionName);

  // Request a terminal with 'dumb' terminal emulation
  // See /etc/termcap for more options
  if (libssh2_channel_request_pty(channel_, "dumb")){
    debugPrint("%s : Failed requesting dumb pty\n", functionName);
    disconnectSSH();
    return SSHDriverError;
  }

  // Open a SHELL on that pty
  if (libssh2_channel_shell(channel_)) {
    debugPrint("%s : Unable to request shell on allocated pty\n", functionName);
    disconnectSSH();
    return SSHDriverError;
  }

  setBlocking(0);

  // Here we should wait for the initial welcome line
  char buffer[1024];
  size_t bytes = 0;
  const char *ps1_last_txt = "!?%#";
  for (int i=0; i <strlen(ps1_last_txt); i++)
  {
    // Set the prompt and read it back
    sprintf(buffer, "PS1=%c\n", ps1_last_txt[i]);

    write(buffer, strlen(buffer), &bytes, 1000);
    read(buffer, 512, &bytes, ps1_last_txt[i], 3000);
  }
  /* Read the final '\n' */
  read(buffer, 512, &bytes, '\n', 1000);
  debugPrint("%s : Connection ready...\n", functionName);

  return SSHDriverSuccess;
}

/**
 * Set the connection to a blocking or non-blocking connection.
 *
 * @param blocking - 0 for non-blocking or 1 for blocking.
 * @return - Success or failure.
 */
SSHDriverStatus SSHDriver::setBlocking(int blocking)
{
  static const char *functionName = "SSHDriver::setBlocking";
  debugPrint("%s : Method called\n", functionName);

  // Make the channel blocking or non-blocking
  libssh2_channel_set_blocking(channel_, blocking);
  debugPrint("%s : Set blocking value to %d\n", functionName, blocking);
  return SSHDriverSuccess;
}

/**
 * Flush the connection as best as possible.
 *
 * @return - Success or failure.
 */
SSHDriverStatus SSHDriver::flush()
{
  char buff[2048];
  static const char *functionName = "SSHDriver::flush";
  debugPrint("%s : Method called\n", functionName);

  if (connected_ == 0){
    debugPrint("%s : Not connected\n", functionName);
    return SSHDriverError;
  }

  int rc = libssh2_channel_flush_ex(channel_, 0);
  rc |= libssh2_channel_flush_ex(channel_, 1);
  rc |= libssh2_channel_flush_ex(channel_, 2);
  rc = libssh2_channel_read(channel_, buff, 2048);
  if (rc < 0){
    return SSHDriverError;
  }
  return SSHDriverSuccess;
}

/**
 * Write data to the connected channel.  A timeout should be
 * specified in milliseconds.
 *
 * @param buffer - The string buffer to be written.
 * @param bufferSize - The number of bytes to write.
 * @param bytesWritten - The number of bytes that were written.
 * @param timeout - A timeout in ms for the write.
 * @return - Success or failure.
 */
SSHDriverStatus SSHDriver::write(const char *buffer, size_t bufferSize, size_t *bytesWritten, int timeout)
{
  char input[2048];
  static const char *functionName = "SSHDriver::write";
  debugPrint("%s : Method called\n", functionName);
  *bytesWritten = 0;

  if (connected_ == 0){
    debugPrint("%s : Not connected\n", functionName);
    return SSHDriverError;
  }

  timeval stime;
  timeval ctime;
  gettimeofday(&stime, NULL);
  long mtimeout = (stime.tv_usec / 1000) + timeout;
  long tnow = 0;

  strncpy(input, buffer, bufferSize);
  input[bufferSize] = 0;
  flush();
  debugPrint("%s : Writing => %s\n", functionName, input);

  int rc = libssh2_channel_write(channel_, buffer, bufferSize);
  if (rc > 0){
    debugPrint("%s : %d bytes written\n", functionName, rc);
    *bytesWritten = rc;
  } else {
    debugPrint("%s : No bytes were written, libssh2 error (%d)\n", functionName, rc);
    bytesWritten = 0;
    return SSHDriverError;
  }

  // Now we need to read back the same numer of bytes, to remove the written string from the buffer
  int bytesToRead = *bytesWritten;
  int bytes = 0;
  char buff[512];
  rc = 0;
  int crCount = 0;
  // Count the number of \n characters sent
  for (int index = 0; index < (int)*bytesWritten; index++){
    if (buffer[index] == '\n'){
      // CR, need to read back 1 extra character
      crCount++;
    }
  }
  bytesToRead += crCount;
  while ((bytesToRead > 0) && (tnow < mtimeout)){
    rc = libssh2_channel_read(channel_, &buff[bytes], bytesToRead);
    if (rc > 0){
      bytes+=rc;
      bytesToRead-=rc;
    }
    if (bytesToRead > 0){
      usleep(50);
      gettimeofday(&ctime, NULL);
      tnow = ((ctime.tv_sec - stime.tv_sec) * 1000) + (ctime.tv_usec / 1000);
    }
  }

  buff[bytes] = '\0';
#ifdef DEBUG_CTRL_CHARS
  debugPrint("%s : Bytes =>\"", functionName);
  for (int j = 0; j < bytes; j++){
    char ch =  buff[j];
    if (isprint(ch)) {
      debugPrint("%c", ch);
    } else if (ch == '\n') {
      debugPrint("\\n");
    } else if (ch == '\r') {
      debugPrint("\\r");
    } else {
      debugPrint("\\%03o", ch);
    }
  }
  debugPrint("\"\n");
#else
  debugPrint("%s : Bytes =>\n", functionName);
  for (int j = 0; j < bytes; j++){
    debugPrint("[%d] ", buff[j]);
  }
  debugPrint("\n");
#endif

  gettimeofday(&ctime, NULL);
  tnow = ((ctime.tv_sec - stime.tv_sec) * 1000) + (ctime.tv_usec / 1000);
  debugPrint("%s : Time taken for write => %ld ms\n", functionName, (tnow-(mtimeout-timeout)));
  if (tnow >= mtimeout){
    return SSHDriverError;
  }

  return SSHDriverSuccess;
}

/**
 * Read data from the connected channel.  A timeout should be
 * specified in milliseconds.  The read method will continue to
 * read data from the channel until either the specified 
 * terminator is read or the timeout is reached.
 *
 * @param buffer - A string buffer to hold the read data.
 * @param bufferSize - The maximum number of bytes to read.
 * @param bytesWritten - The number of bytes that have been read.
 * @param readTerm - A terminator to use as a check for EOM (End Of Message).
 * @param timeout - A timeout in ms for the read.
 * @return - Success or failure.
 */
SSHDriverStatus SSHDriver::read(char *buffer, size_t bufferSize, size_t *bytesRead, int readTerm, int timeout)
{
  static const char *functionName = "SSHDriver::read";
  debugPrint("%s : Method called\n", functionName);
  debugPrint("%s : Read terminator %d\n", functionName, readTerm);

  if (connected_ == 0){
    debugPrint("%s : Not connected\n", functionName);
    return SSHDriverError;
  }

  timeval stime;
  timeval ctime;
  gettimeofday(&stime, NULL);
  long mtimeout = (stime.tv_usec / 1000) + timeout;
  long tnow = 0;
  int rc = 0;
  int matched = 0;
  int matchedindex = 0;
  int lastCount = 0;
  *bytesRead = 0;
#ifdef DEBUG
  memset(buffer, 0, bufferSize);
#endif
  while ((matched == 0) && (tnow < mtimeout)){
    /* TODO: Make sure that this loop does not run over bufferSize when
     a match is not found and matched is never set to 1 */
    rc = libssh2_channel_read(channel_, &buffer[*bytesRead], (bufferSize-*bytesRead));
    if (rc > 0){
      *bytesRead+=rc;
    }
    for (int index = lastCount; index < (int)*bytesRead; index++){
      // Match against output terminator
      if (buffer[index] == readTerm){
        matched = 1;
        matchedindex = index;
      }
    }
    lastCount = *bytesRead;
    if (matched == 0){
      usleep(50);
      gettimeofday(&ctime, NULL);
      tnow = ((ctime.tv_sec - stime.tv_sec) * 1000) + (ctime.tv_usec / 1000);
    }
  }

  //buffer[matchedindex] = '\0';
#ifdef DEBUG_CTRL_CHARS
  debugPrint("%s : Bytes =>\"", functionName);
  for (int j = 0; j < lastCount; j++){
    char ch =  buffer[j];
    if (isprint(ch)) {
      debugPrint("%c", ch);
    } else if (ch == '\n') {
      debugPrint("\\n");
    } else if (ch == '\r') {
      debugPrint("\\r");
    } else {
      debugPrint("\\%03o", ch);
    }
  }
  debugPrint("\"\n");
#else
  debugPrint("%s : Bytes =>\n", functionName);
  for (int j = 0; j < lastCount; j++){
    debugPrint("[%d] ", buffer[j]);
  }
  debugPrint("\n");
#endif
  debugPrint("%s : Matched %d\n", functionName, matched);

  if (matched == 1){
    lastCount = matchedindex+1;
  }
  *bytesRead = lastCount;
  debugPrint("%s : Line => %s", functionName, buffer);

  gettimeofday(&ctime, NULL);
  tnow = ((ctime.tv_sec - stime.tv_sec) * 1000) + (ctime.tv_usec / 1000);
  debugPrint("%s : Time taken for read => %ld ms\n", functionName, (tnow-(mtimeout-timeout)));

  if ((timeout > 0) && (tnow >= mtimeout)){
    return SSHDriverError;
  }

  return SSHDriverSuccess;
}

/**
 * Close the connection.
 *
 * @return - Success or failure.
 */
SSHDriverStatus SSHDriver::disconnectSSH()
{
  static const char *functionName = "SSHDriver::disconnect";
  debugPrint("%s : Method called\n", functionName);

  if (connected_ == 1){
    connected_ = 0;
    libssh2_session_disconnect(session_, "Normal Shutdown");
    libssh2_session_free(session_);

#ifdef WIN32
    closesocket(sock_);
#else
    close(sock_);
#endif
    debugPrint("%s : Completed disconnect\n", functionName);

    libssh2_exit();
  
  } else {
    debugPrint("%s : Connection was never established\n", functionName);
  }
  return SSHDriverSuccess;
}

/**
 * Destructor, cleanup.
 */
SSHDriver::~SSHDriver()
{
  static const char *functionName = "SSHDriver::~SSHDriver";
  debugPrint("%s : Method called\n", functionName);
}

