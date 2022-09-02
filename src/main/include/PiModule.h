// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#define PI 3.141592
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>

class PiModule {
    public:
    
    PiModule();

    void start_server();
    double current_val = -1;
    double get_distance();
    double update_pi();

    int sockfd_g;
    char* buffer_g;
    struct sockaddr_in servaddr, cliaddr_g;



};
