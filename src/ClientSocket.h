// Definition of the ClientSocket class

#ifndef ClientSocket_class
#define ClientSocket_class

#include "Socket.h"


class ClientSocket : private Socket
{
 public:

  bool connected() 
  {
    return connected_;
  }
  bool created()
  {
    return created_;
  }

  ClientSocket() = default;
  ClientSocket ( std::string host, int port );
  virtual ~ClientSocket(){};

  const ClientSocket& operator << ( const std::string& ) const;
  const ClientSocket& operator >> ( std::string& ) const;

  private :

    connected_ = false;
    created_ = false;

};


#endif