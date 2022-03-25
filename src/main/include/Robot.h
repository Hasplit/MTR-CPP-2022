// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/TimedRobot.h>
#include <frc/XboxController.h>
#include "Constants.h"
#include <frc/drive/DifferentialDrive.h>
#include <frc/motorcontrol/PWMSparkMax.h>
#include <frc/DigitalOutput.h>
#include <frc/Relay.h>
#include <cameraserver/CameraServer.h>
#include <frc2/command/Command.h>

#include "RobotContainer.h"

class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestPeriodic() override;
  void SimulationInit() override;
  void SimulationPeriodic() override;

 private:
  // Have it null by default so that if testing teleop it
  // doesn't have undefined behavior and potentially crash.
  frc2::Command* m_autonomousCommand = nullptr;

  // Class Variables
  uint8_t currentSpeedLevel = 0;

  // Motors on robot using spark motor controller profiles
  frc::PWMSparkMax m_left{8};
  frc::PWMSparkMax m_right{9};
  frc::DifferentialDrive m_driveController{m_left, m_right};

  frc::CameraServer _CameraServer{};
  frc::XboxController pilotController{0};

  RobotContainer m_container;
};
