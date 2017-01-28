
#include "PhysicsClientUDP_C_API.h"
#include "PhysicsClientUDP.h"
#include "PhysicsDirect.h"
#include <stdio.h>

//think more about naming. The b3ConnectPhysicsLoopback
b3PhysicsClientHandle b3ConnectPhysicsUDP(const char* hostName, int port)
{

	UdpNetworkedPhysicsProcessor* udp = new UdpNetworkedPhysicsProcessor(hostName, port);

	PhysicsDirect* direct = new PhysicsDirect(udp, true);

	bool connected;
	connected = direct->connect();
	if (connected)
	{
		printf("b3ConnectPhysicsUDP connected successfully.\n");
	}
	else
	{
		printf("b3ConnectPhysicsUDP NOT connected.\n");

	}
	return (b3PhysicsClientHandle)direct;
}



