// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <fmt/core.h>

#include <frc/smartdashboard/SmartDashboard.h>

void Robot::RobotInit() {
  m_LL->RestoreFactoryDefaults();
  m_LF->RestoreFactoryDefaults();
  m_RL->RestoreFactoryDefaults();
  m_RF->RestoreFactoryDefaults();
  
  m_LL->SetInverted(true);
  m_LF->Follow(*m_LL, true);
  m_RL->SetInverted(false);
  m_RF->Follow(*m_RL, false);
  
  
  

  
}

void Robot::RobotPeriodic() {
  frc::SmartDashboard::PutNumber("lEncoder", lEncoder.GetPosition());
  frc::SmartDashboard::PutNumber("rEncoder", rEncoder.GetPosition());
  frc::SmartDashboard::PutNumber("", m->vals[0]);
}

void Robot::AutonomousInit() {}

void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic() {
  m_drive->ArcadeDrive(controller->GetRightY(), controller->GetLeftX());
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
