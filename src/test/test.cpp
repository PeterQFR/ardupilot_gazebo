/*
 * test.cpp
 *
 *  Created on: 10 Feb. 2019
 *      Author: peter
 */


#include <ros/ros.h>
#include <signal.h>
#include <ros/xmlrpc_manager.h>
#include <ArduCopterPlugin.hh>

sig_atomic_t volatile g_request_shutdown = 0;

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




int main(int argc, char **argv)
{
	ros::init(argc, argv, "orbslam2", ros::init_options::NoSigintHandler);
	ros::start();
	signal(SIGINT, mySigIntHandler);

	// Override XMLRPC shutdown
	ros::XMLRPCManager::instance()->unbind("shutdown");
	ros::XMLRPCManager::instance()->bind("shutdown", shutdownCallback);
	gazebo::ArduCopterSocketPrivate socket;
	if (!socket.Bind("127.0.0.1", 5670))
		ROS_ERROR("Failed to bind to SITL instance");

	uchar buffer[1024];

	ros::Rate r(50);
	while (!g_request_shutdown)
	{
		if(socket.Recv((void*) buf, 1024, 20)>0)
			ROS_INFO("Buffer %s", buffer);

		r.sleep();
	}

return 0;

}
