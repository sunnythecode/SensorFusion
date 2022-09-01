// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#define PI 3.141592


class PiModule {
    public:
    
    PiModule();

    void start_server();
    double current_val = -1;
    double get_distance();


};
