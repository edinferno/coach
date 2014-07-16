#include "game.hpp"

#include <cstring>
#include <iostream>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>


#include "SPLCoachMessage.hpp"

bool Game::BroadcastStrategy(GameStrategy strategy)
{
    const uint8_t TEAM_NUMBER = 9;

	//Initialize broadcast socket parameters
    static const int broadcast_enable = 1;

    //Initialize UDP broadcast server info
    struct sockaddr_in broadcast_servaddr;
	bzero(&broadcast_servaddr, sizeof(broadcast_servaddr));
	broadcast_servaddr.sin_family = AF_INET;
	broadcast_servaddr.sin_addr.s_addr = inet_addr("255.255.255.255");
	broadcast_servaddr.sin_port = htons(SPL_COACH_MESSAGE_PORT);

    //Open the broadcast socket
   	int broadcast_socket = socket(AF_INET, SOCK_DGRAM, 0);
    if (broadcast_socket < 0)
    {
    	std::cerr << "ERROR opening broadcast socket" << std::endl;
    	return false;
    }

    //Set the socket to broadcast mode
    setsockopt(broadcast_socket, SOL_SOCKET, SO_BROADCAST, &broadcast_enable, sizeof(broadcast_enable));

    SPLCoachMessage coach_msg;

  	coach_msg.team = TEAM_NUMBER;

    //Send strategy
    memset (coach_msg.message, 0, SPL_COACH_MESSAGE_SIZE);
    switch(strategy)
    {
    	case Defence:
    		strcpy((char*)coach_msg.message, "Defence");
    		break;

    	case Attack:
    	    strcpy((char*)coach_msg.message, "Attack");
    		break;

    	case KeepCalmAndCarryOn:
    		strcpy((char*)coach_msg.message, "Keep calm and carry on");
    		break;
    }
    int n = sendto(broadcast_socket,
                   &coach_msg,
                   sizeof(SPLCoachMessage),
                   0, // No flags
                   (struct sockaddr*)&broadcast_servaddr,
                   sizeof(broadcast_servaddr));

    if(n != sizeof(SPLCoachMessage))
    	return false;

    close(broadcast_socket);

    return true;
}
