// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
/*
#include "SensorModule.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <math.h>

#include <stdio.h>
#include <iostream>
#include <strings.h>
#include <sys/types.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include<netinet/in.h>
#define PORT 8080
#define MAXLINE 1024


SensorModule::SensorModule() {
	rev::SparkMaxRelativeEncoder* lEncoder, rev::SparkMaxRelativeEncoder* rEncoder): lEncoder{lEncoder}, rEncoder{rEncoder} {}
};

int SensorModule::receiveInt() {
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
	
	len = sizeof(cliaddr); //len is value/result
	
	n = recvfrom(sockfd, (char *)buffer, MAXLINE,
				MSG_WAITALL, ( struct sockaddr *) &cliaddr,
				&len);
	buffer[n] = '\0';
	printf("Client : %s\n", buffer);
	sendto(sockfd, (const char *)hello, strlen(hello),
		0, (const struct sockaddr *) &cliaddr,
			len);
	printf("Hello message sent.\n");
		
	return 0;
}

}

void SensorModule::update() {
	imu->GetAngle()


}

*/