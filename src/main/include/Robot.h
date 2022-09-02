// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
#include <math.h> 
#pragma once
#define PI 3.14159265
#include <vector>
#include <string>

#include <wpi/numbers>
#include <frc/Joystick.h>
#include <frc/XboxController.h>
#include <frc/TimedRobot.h>
#include <rev/CANSparkMax.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/SmartDashboard.h>


#include "PiModule.h"
#include "SensorFusion.h"
#include "SFDrive.h"

#include <frc/drive/DifferentialDrive.h>


class Robot : public frc::TimedRobot {
 public:
  //Fix these:
  int rMotorLeaderID = 9;
  int rMotorFollowerID = 3;
  int lMotorLeaderID = 15;
  int lMotorFollowerID = 16;
  

  //Functions
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void TestInit() override;
  void TestPeriodic() override;

  //Drive constants:
  int driveMotorCurrentLimit = 30;

  //Pi Sensor
  PiModule* pi_us = new PiModule();
  double pi_val = 0;

  //Gyro Stuff
  

  //Controller
  frc::XboxController* ctr = new frc::XboxController{0};

  //Motors
  rev::CANSparkMax* m_RL = new rev::CANSparkMax(rMotorLeaderID, rev::CANSparkMax::MotorType::kBrushless);
  rev::CANSparkMax* m_RF = new rev::CANSparkMax(rMotorFollowerID, rev::CANSparkMax::MotorType::kBrushless);
  rev::CANSparkMax* m_LL = new rev::CANSparkMax(lMotorLeaderID, rev::CANSparkMax::MotorType::kBrushless);
  rev::CANSparkMax* m_LF = new rev::CANSparkMax(lMotorFollowerID, rev::CANSparkMax::MotorType::kBrushless);

  //Drive Base:
  SFDrive* m_drive = new SFDrive(m_LL, m_RL);

  
  //Encoders
  rev::SparkMaxRelativeEncoder lEncoder = m_LL->GetEncoder();
  rev::SparkMaxRelativeEncoder rEncoder = m_RL->GetEncoder();

  //Sensor Fusion
  rev::SparkMaxRelativeEncoder* lEnc = &lEncoder;
  rev::SparkMaxRelativeEncoder* rEnc = &rEncoder;
  SensorFusion* sensors = new SensorFusion(lEnc, rEnc);





 private:
};