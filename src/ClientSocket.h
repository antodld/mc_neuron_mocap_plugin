// Definition of the ClientSocket class

#ifndef ClientSocket_class
#define ClientSocket_class

#include "Socket.h"

class ClientSocket : private Socket
{
public:
  bool connected() const noexcept
  {
    return connected_;
  }
  bool created()
  {
    return created_;
  }
  std::string host()
  {
    return host_;
  }
  void host(const std::string & host)
  {
    host_ = host;
  }
  int port()
  {
    return port_;
  }
  void port(const int port)
  {
    port_ = port;
  }

  bool connect();

  bool create();

  ClientSocket() = default;
  ClientSocket(std::string host, int port);
  virtual ~ClientSocket() = default;

  const ClientSocket & operator<<(const std::string &) const;
  const ClientSocket & operator>>(std::string &) const;

private:
  bool connected_ = false;
  bool created_ = false;
  int port_;
  std::string host_;
};

#endif