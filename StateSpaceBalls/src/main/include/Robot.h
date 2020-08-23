/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <string>


#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <string>
#include <iostream>
#include <frc/Encoder.h>
#include <frc/Joystick.h>
#include <frc/TimedRobot.h>
#include <frc/AnalogInput.h>
#include <frc/PWMVictorSPX.h>
#include <frc/MedianFilter.h>
#include <frc/XboxController.h>
#include <frc/controller/PIDController.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <wpi/math>
#include <frc/DriverStation.h>
#include <ctre/Phoenix.h>
#include <frc/Solenoid.h>
#include "AHRS.h"
#include <frc/util/color.h>
#include <frc/SpeedControllerGroup.h>
#include <frc/Filesystem.h>
#include <frc/trajectory/TrajectoryUtil.h>
#include <wpi/SmallString.h>
#include <wpi/Path.h>
#include <frc/DriverStation.h>
#include <frc/SpeedControllerGroup.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/XboxController.h>
#include <frc/DigitalOutput.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <frc/DigitalInput.h>
#include <frc/Compressor.h>
#include <frc/Timer.h>

class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestPeriodic() override;
  void DisabledPeriodic() override;
  void DisabledInit() override;

 private:
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;
};
