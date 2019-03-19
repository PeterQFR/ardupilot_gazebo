/*
 * test.cpp
 *
 *  Created on: 10 Feb. 2019
 *      Author: peter
 */


#include <ros/ros.h>
#include <signal.h>
#include <ros/xmlrpc_manager.h>

#include <functional>
#include <fcntl.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>

sig_atomic_t volatile g_request_shutdown = 0;
/// \brief A servo packet.
struct ServoPacket
{
  /// \brief Motor speed data.
  float motorSpeed[8];
};

/// \brief Flight Dynamics Model packet that is sent back to the ArduCopter
struct fdmPacket
{
  /// \brief packet timestamp
  double timestamp;

  /// \brief IMU angular velocity
  double imuAngularVelocityRPY[3];

  /// \brief IMU linear acceleration
  double imuLinearAccelerationXYZ[3];

  /// \brief IMU quaternion orientation
  double imuOrientationQuat[4];

  /// \brief Model velocity in NED frame
  double velocityXYZ[3];

  /// \brief Model position in NED frame
  double positionXYZ[3];
};

// Private data class
class ArduCopterSocketPrivate
{
  /// \brief constructor
  public: ArduCopterSocketPrivate()
  {
    // initialize socket udp socket
    fd = socket(AF_INET, SOCK_DGRAM, 0);
    fcntl(fd, F_SETFD, FD_CLOEXEC);
  }

  /// \brief destructor
  public: ~ArduCopterSocketPrivate()
  {
    if (fd != -1)
    {
      ::close(fd);
      fd = -1;
    }
  }

  /// \brief Bind to an adress and port
  /// \param[in] _address Address to bind to.
  /// \param[in] _port Port to bind to.
  /// \return True on success.
  public: bool Bind(const char *_address, const uint16_t _port)
  {
    struct sockaddr_in sockaddr;
    this->MakeSockAddr(_address, _port, sockaddr);

    if (bind(this->fd, (struct sockaddr *)&sockaddr, sizeof(sockaddr)) != 0)
    {
      shutdown(this->fd, 0);
      close(this->fd);
      return false;
    }
    int one = 1;
    setsockopt(this->fd, SOL_SOCKET, SO_REUSEADDR,
          &one, sizeof(one));

    fcntl(this->fd, F_SETFL,
    fcntl(this->fd, F_GETFL, 0) | O_NONBLOCK);
    return true;
  }

  /// \brief Connect to an adress and port
  /// \param[in] _address Address to connect to.
  /// \param[in] _port Port to connect to.
  /// \return True on success.
  public : bool Connect(const char *_address, const uint16_t _port)
  {
    struct sockaddr_in sockaddr;
    this->MakeSockAddr(_address, _port, sockaddr);

    if (connect(this->fd, (struct sockaddr *)&sockaddr, sizeof(sockaddr)) != 0)
    {
      shutdown(this->fd, 0);
      close(this->fd);
      return false;
    }
    int one = 1;
    setsockopt(this->fd, SOL_SOCKET, SO_REUSEADDR,
          &one, sizeof(one));

    fcntl(this->fd, F_SETFL,
    fcntl(this->fd, F_GETFL, 0) | O_NONBLOCK);
    return true;
  }

  /// \brief Make a socket
  /// \param[in] _address Socket address.
  /// \param[in] _port Socket port
  /// \param[out] _sockaddr New socket address structure.
  public: void MakeSockAddr(const char *_address, const uint16_t _port,
    struct sockaddr_in &_sockaddr)
  {
    memset(&_sockaddr, 0, sizeof(_sockaddr));

    #ifdef HAVE_SOCK_SIN_LEN
      _sockaddr.sin_len = sizeof(_sockaddr);
    #endif

    _sockaddr.sin_port = htons(_port);
    _sockaddr.sin_family = AF_INET;
    _sockaddr.sin_addr.s_addr = inet_addr(_address);
  }

  public: ssize_t Send(const void *_buf, size_t _size)
  {
    return send(this->fd, _buf, _size, 0);
  }

  /// \brief Receive data
  /// \param[out] _buf Buffer that receives the data.
  /// \param[in] _size Size of the buffer.
  /// \param[in] _timeoutMS Milliseconds to wait for data.
  public: ssize_t Recv(void *_buf, const size_t _size, uint32_t _timeoutMs)
  {
    fd_set fds;
    struct timeval tv;

    FD_ZERO(&fds);
    FD_SET(this->fd, &fds);

    tv.tv_sec = _timeoutMs / 1000;
    tv.tv_usec = (_timeoutMs % 1000) * 1000UL;

    if (select(this->fd+1, &fds, NULL, NULL, &tv) != 1)
    {
        return -1;
    }

    return recv(this->fd, _buf, _size, 0);
  }

  /// \brief Socket handle
  private: int fd;
};



// Replacement SIGINT handler
void mySigIntHandler(int sig)
{
  g_request_shutdown = 1;
}

// Replacement "shutdown" XMLRPC callback
void shutdownCallback(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result)
{
  int num_params = 0;
  if (params.getType() == XmlRpc::XmlRpcValue::TypeArray)
    num_params = params.size();
  if (num_params > 1)
  {
    std::string reason = params[1];
    ROS_WARN("Shutdown request received. Reason: [%s]", reason.c_str());
    g_request_shutdown = 1; // Set flag
  }

  result = ros::xmlrpc::responseInt(1, "", 0);
}

fdmPacket getPacket(double starttime)
{
	fdmPacket pkt;
	pkt.imuAngularVelocityRPY[0]=0.0;
	pkt.imuAngularVelocityRPY[1]=0.0;
	pkt.imuAngularVelocityRPY[2]=0.0;

	pkt.imuLinearAccelerationXYZ[0]=0;
	pkt.imuLinearAccelerationXYZ[1]=0;
	pkt.imuLinearAccelerationXYZ[2]=9.8;

	pkt.imuOrientationQuat[0] = 1.0;
	pkt.imuOrientationQuat[1] = 0.0;
	pkt.imuOrientationQuat[2] = 0.0;
	pkt.imuOrientationQuat[3] = 0.0;

	pkt.velocityXYZ[0] =0.0;
	pkt.velocityXYZ[1] =0.0;
	pkt.velocityXYZ[2] =0.0;

	pkt.positionXYZ[0] =0.0;
	pkt.positionXYZ[1] =0.0;
	pkt.positionXYZ[2] =0.0;

	pkt.timestamp = ros::Time::now().toSec()-starttime;
	return pkt;
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "orbslam2", ros::init_options::NoSigintHandler);
	ros::start();
	signal(SIGINT, mySigIntHandler);

	// Override XMLRPC shutdown
	ros::XMLRPCManager::instance()->unbind("shutdown");
	ros::XMLRPCManager::instance()->bind("shutdown", shutdownCallback);
	ArduCopterSocketPrivate outsocket, insocket;

	double starttime = ros::Time::now().toSec();
	if (!outsocket.Connect("127.0.0.1", 9003))
		ROS_ERROR("Failed to bind to SITL instance");
	if (!insocket.Connect("127.0.0.1", 9002))
			ROS_ERROR("Failed to bind to Gazebo instance");

	char buffer[1024];
	fdmPacket pkt;

	ros::Rate r(100);
	while (!g_request_shutdown)
	{
		pkt = getPacket(starttime);
		outsocket.Send((void*) &pkt, sizeof(fdmPacket));
		ROS_INFO("Sending Packet");
		int rec =insocket.Recv((void*) buffer, 16*sizeof(float), 10);
		if(rec>0)
			ROS_INFO("Buffer %s %d", buffer, rec);

		r.sleep();
	}

return 0;

}
