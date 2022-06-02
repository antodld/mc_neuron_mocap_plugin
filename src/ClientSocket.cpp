// Implementation of the ClientSocket class

#include "ClientSocket.h"
#include "SocketException.h"


ClientSocket::ClientSocket ( std::string host, int port )
{
  host_ = host;
  port_ = port;
  // if ( ! Socket::create() )
  //   {
  //     throw SocketException ( "Could not create client socket." );
  //   }

  // if ( ! Socket::connect ( host, port ) )
  //   {
  //     throw SocketException ( "Could not bind to port." );
  //   }

  created_ = Socket::create();
  if (!created_){std::cout << "[ClientSocket] Warning : Could not create client socket " << std::endl;}
  else
  {
    connected_ = Socket::connect ( host_, port_ );
    if(!connected_){std::cout << "[ClientSocket] Warning : Could not bind to port " << std::endl;}
  }

}

bool ClientSocket::connect()
{
  if (!created_)
  {
    created_ = Socket::create();
    if (!created_)
    {
      std::cout << "[ClientSocket] Warning : Could not create client socket " << std::endl;
      return false;
    }
  }
  connected_ = Socket::connect ( host_, port_ );
  return connected_;

}


const ClientSocket& ClientSocket::operator << ( const std::string& s ) const
{
  if ( ! Socket::send ( s ) )
    {
      throw SocketException ( "Could not write to socket." );
    }

  return *this;

}


const ClientSocket& ClientSocket::operator >> ( std::string& s ) const
{
  if ( ! Socket::recv ( s ) )
    {
      throw SocketException ( "Could not read from socket." );
    }

  return *this;
}