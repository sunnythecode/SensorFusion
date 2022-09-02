#include<PiModule.h>

// Server side implementation of UDP client-server model
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>

//IKD
#include <frc/smartdashboard/SmartDashboard.h>

	
#define PORT	 8080
#define MAXLINE 1024
	
PiModule::PiModule() {}

// Driver code
void PiModule::start_server() {
    
	int sockfd;
	char buffer[MAXLINE];
	const char *hello = "Hello from server";
	struct sockaddr_in servaddr, cliaddr;
		
	// Creating socket file descriptor
	if ( (sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0 ) {
		perror("socket creation failed");
		exit(EXIT_FAILURE);
	}
		
	memset(&servaddr, 0, sizeof(servaddr));
	memset(&cliaddr, 0, sizeof(cliaddr));
		
	// Filling server information
	servaddr.sin_family = AF_INET; // IPv4
	servaddr.sin_addr.s_addr = INADDR_ANY;
	servaddr.sin_port = htons(PORT);
		
	// Bind the socket with the server address
	if ( bind(sockfd, (const struct sockaddr *)&servaddr,
			sizeof(servaddr)) < 0 )
	{
		perror("bind failed");
		exit(EXIT_FAILURE);
	}
		
	int n;
    socklen_t len;
	
	sockfd_g = sockfd;
	buffer_g = buffer;
	cliaddr_g = cliaddr;

	len = sizeof(cliaddr); //len is value/result
	
}

double PiModule::update_pi() {
	int n;
	socklen_t len;
	const char *hello = "Hello from server";

	n = recvfrom(sockfd_g, (char *)buffer_g, MAXLINE,
				MSG_WAITALL, ( struct sockaddr *) &cliaddr_g,
				&len);
	buffer_g[n] = '\0';
	printf("Client : %s\n", buffer_g);
	//sendto(sockfd_g, (const char *)hello, strlen(hello), 0, (const struct sockaddr *) &cliaddr_g, len);
    current_val = std::atof(buffer_g);
	frc::SmartDashboard::PutNumber(buffer_g, 0);
    }

double PiModule::get_distance() { return current_val; }
