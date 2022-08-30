// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <fmt/core.h>

#include <frc/smartdashboard/SmartDashboard.h>

void Robot::RobotInit() {
  maginit = get_magnometer();
  gyro_imu->Reset();
  gyro_corrected = 0.0;


  m_LL->RestoreFactoryDefaults();
  m_LF->RestoreFactoryDefaults();
  m_RL->RestoreFactoryDefaults();
  m_RF->RestoreFactoryDefaults();


  
  m_LL->SetInverted(true);
  m_LF->Follow(*m_LL, false);
  m_RL->SetInverted(false);
  m_RF->Follow(*m_RL, false);

  m_LL->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  m_LF->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  m_RL->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  m_RF->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

  lEncoder.SetPosition(0);
  rEncoder.SetPosition(0);
  lEncoder.SetPositionConversionFactor(1.96);
  rEncoder.SetPositionConversionFactor(1.96);
  position.push_back(0.0);
  position.push_back(0.0);
  position.push_back(0.0);
}

void Robot::RobotPeriodic() {
  frc::SmartDashboard::PutNumber("lEncoder", lEncoder.GetPosition());
  frc::SmartDashboard::PutNumber("rEncoder", rEncoder.GetPosition());

  encoders_to_coord(lEncoder.GetPosition(), rEncoder.GetPosition());
  //lEncoder.SetPosition(0);
  //rEncoder.SetPosition(0);
  gyro_edit();



  //frc::SmartDashboard::PutNumber("Position X", position[0]);
  //frc::SmartDashboard::PutNumber("Position Y", position[1]);
  frc::SmartDashboard::PutNumber("Position theta", position[2]);
  frc::SmartDashboard::PutNumber("Magnometer", get_magnometer());
  //frc::SmartDashboard::PutNumber("GyroEdited", gyromag);

  frc::SmartDashboard::PutNumber("Gyro", get_gyro());
  frc::SmartDashboard::PutNumber("Gyro Corrected", gyro_corrected);
}

void Robot::AutonomousInit() {}

void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic() {
  m_drive->ArcadeDrive(ctr->GetLeftY(), ctr->GetRightX());
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
