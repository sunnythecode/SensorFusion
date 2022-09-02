// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#define PI 3.14159265

#include <vector>
#include <string>
#include <math.h> 
#include <wpi/numbers>

#include <rev/CANSparkMax.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/ADIS16448_IMU.h>

class SensorFusion {
 public:
    //Constructor: Pass motors, gyro, and more...)
    SensorFusion(rev::SparkMaxRelativeEncoder* lEncoder, rev::SparkMaxRelativeEncoder* rEncoder);

    //Wraparound code from 0 - 360
    double driftAlgorithm(double short_sensor, double last_short_sensor, double long_sensor, double error, double corrected, double long_bound, double max_drift);
    double range360(double inp);
    double wraparound_to_change(double last_gyro, double gyro);

    //Encoders to coord is really just encoders to theta(position code is not tested)
    void encoders_to_coord(double left, double right);

    //Sensor functions but they return between (0, 360)
    double get_magnometer();
    double get_gyro();

    //Sensor Drift Algorithm that works with module variables
    void gyro_drift();
    void enc_drift();

    // Actual Functions used outside of class: 
    void initializeSensors();
    void updateSensors(bool print);
    double getTrueGyro();
    double getTrueEnc();


    //Sensor Drift
    double bound_num = 0.02; // Changeable: Bound for drift decrease
    double maginit;
    double max_drift = 0.3; // Changeable: Maximum drift value outside bound

    // Gyro Drift Variables
    double mag_corrected; // This is really magnetic heading corrected
    double last_gyro = 0.0;
    

    //Encoder Drift Variables
    double enc_theta = 0.0;
    double mag_corrected_enc; // Magnetic heading - encoder
    double last_enc = 0.0;

    double dbaseline = 26.0; //robot baseline(width between right and left wheels inches)
    double wrap_bound = 350;
    double corrected_bound = 354;



    //Position
    std::vector<double> position;


    //Gyro Stuff
    frc::ADIS16448_IMU* gyro_imu = new frc::ADIS16448_IMU();
    

    //Encoders
    rev::SparkMaxRelativeEncoder* lEncoder;
    rev::SparkMaxRelativeEncoder* rEncoder;

 private:
};

