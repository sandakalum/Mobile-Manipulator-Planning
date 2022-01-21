// Definition of the ClientSocket class

#ifndef ClientSocket_class
#define ClientSocket_class

#include "Socket.h"

class ClientSocket : private Socket {
 public:
  ClientSocket(std::string host, int port);
  virtual ~ClientSocket() {};
  void init(std::string host, int port);

  const ClientSocket& operator<<(const std::string&) const;
  const ClientSocket& operator>>(std::string&) const;

 private:
  std::string hostIP_s;
  int port;
};

#endif
