// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <fmt/core.h>

#include <frc/smartdashboard/SmartDashboard.h>

void Robot::RobotInit() {
  sensors->initializeSensors();

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
}

void Robot::RobotPeriodic() {
  sensors->updateSensors(false); // false to not print sensor stuff
  frc::SmartDashboard::PutNumber("True Gyro", sensors->getTrueGyro());
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
