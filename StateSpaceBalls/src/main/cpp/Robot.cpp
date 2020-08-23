/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"

//////////////////////////////////////////////
/// Drive
/*
drive sproket = 12
driven sproket = 28
*/
WPI_TalonSRX T1{1};
WPI_TalonSRX T2{2};
WPI_TalonSRX T3{15};
WPI_TalonSRX T4{16};
frc::Joystick Joy1(0);
frc::Joystick Joy2(1);
frc::XboxController Xb1(2);
frc::SpeedControllerGroup SpeedControl(T1, T2, T3, T4);
frc::SpeedControllerGroup S1(T1, T2);
frc::SpeedControllerGroup S2(T3, T4);
//frc::DifferentialDrive Drive(S1, S2);
frc::AnalogInput UltraS{0};
//frc::DriverStation Driver;
double LeftJoy, RightJoy, LeftSpeed, RightSpeed, Wheel, Joy;

#define MAX_POWER 0.5

//////////////////////////////////////////////////////////
//Auto
double PIDControl;
static constexpr double kP = .033;       // original .033X.65
static constexpr double kI = 0.00003128; //.000
static constexpr double kD = 0.007;      // .00
double Ultramann;
static constexpr int kHoldDistance = 12;
static constexpr double kValuetoInches = 0.125;
#define Auto1
//#define Auto2
int Robot_State, Prev_Robot, RobotTimer;
double LO, RO;
//Encoders
frc::Encoder FLe{0, 1};
frc::Encoder FRe{2, 3};

//ultrasonic
frc::AnalogInput Ultraman(0);

frc::MedianFilter<double> filter{5};

frc2::PIDController PIDL{kP, kI, kD};
frc2::PIDController PIDR{kP, kI, kD};

///////////////////////////////
//solenoids
frc::Solenoid Shifter{0};
frc::Solenoid ManpState{1};
frc::Solenoid ClimbLock{3};
frc::Solenoid Shift4{2};
frc::Compressor Compress{0};
double EncoderGroup;
////////////////////////////////////
//shooter
WPI_TalonSRX Shooter{9};
WPI_TalonSRX Feeder{10};

bool enabled = Compress.Enabled();
bool pressureSwitch = Compress.GetPressureSwitchValue();
double current = Compress.GetCompressorCurrent();
//////////////////////////////////////
// Manip
WPI_TalonSRX Manp1{5};
WPI_TalonSRX Manp2{6};
frc::DigitalInput Ball{9};
#define True 1
#define False 0
#define Idle 1
#define Trasfer 2
#define Index 3
bool Ballsense, ManipStarted, ManpiReversed;
int BallCount;
int BallTimer;
int IndexState;
/////////////////////////////////////////////
//Climb
WPI_TalonSRX CL1{3};
WPI_TalonSRX CL2{4};
WPI_TalonSRX CR3{13};
WPI_TalonSRX CR4{14};
frc::SpeedControllerGroup LeftClimb(CL1, CL2);
frc::SpeedControllerGroup RightClimb(CR3, CR4);
frc::DigitalInput LeftLimit{9};
frc::DigitalInput RightLimit{8};

bool LeftMax, RightMax;
double LCS, RCS, LY, RY;

void Robot::RobotInit()
{
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
  IndexState = Idle;
}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic()
{
  Compress.Start();
  frc::SmartDashboard::PutNumber("BallTimer", BallTimer);
  frc::SmartDashboard::PutNumber("RobotTimer", RobotTimer);
  frc::SmartDashboard::PutBoolean("BallSensed", Ballsense);
}

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */
void Robot::AutonomousInit()
{
  m_autoSelected = m_chooser.GetSelected();
  // m_autoSelected = SmartDashboard::GetString("Auto Selector",
  //     kAutoNameDefault);
  std::cout << "Auto selected: " << m_autoSelected << std::endl;

  if (m_autoSelected == kAutoNameCustom)
  {
    // Custom Auto goes here
  }
  else
  {
    // Default Auto goes here
    RobotTimer = 0;
  }
}

void Robot::AutonomousPeriodic()
{
  if (m_autoSelected == kAutoNameCustom)
  {
    // Custom Auto goes here
  }
  else
  {
    // Default Auto goes here
    RobotTimer++;
    BallTimer++;
    Shooter.Set(ControlMode::PercentOutput, -1);
#ifdef Auto1
    ManpState.Set(True);
    if (RobotTimer < 95)
    {
      T1.Set(ControlMode::PercentOutput, 0.3);
      T2.Set(ControlMode::PercentOutput, 0.3);
      T3.Set(ControlMode::PercentOutput, -0.3);
      T4.Set(ControlMode::PercentOutput, -0.3);
      Feeder.Set(ControlMode::PercentOutput, 0);
    }

    if (201 < RobotTimer < 400)
    {
      Feeder.Set(ControlMode::PercentOutput, -1);
    }

    /*
    if (RobotTimer < 400)
    {
      T1.Set(ControlMode::PercentOutput, 0.5);
      T2.Set(ControlMode::PercentOutput, 0.5);
      T3.Set(ControlMode::PercentOutput, 0.5);
      T4.Set(ControlMode::PercentOutput, 0.5);
      Manp1.Set(ControlMode::PercentOutput, -0.5);
      Manp2.Set(ControlMode::PercentOutput, -0.5);

      if (Ballsense == 1)
      {
        BallTimer = 0;
      }
      else
      {
        Feeder.Set(ControlMode::PercentOutput, -Xb1.GetTriggerAxis(frc::GenericHID::kLeftHand));
      }

      if (BallTimer < 18)
      {
        Feeder.Set(ControlMode::PercentOutput, -0.5);
      }
    }
  }

  if (RobotTimer < 600)
  {
    T1.Set(ControlMode::PercentOutput, -0.5);
    T2.Set(ControlMode::PercentOutput, -0.5);
    T3.Set(ControlMode::PercentOutput, -0.5);
    T4.Set(ControlMode::PercentOutput, -0.5);
  }
  if(RobotTimer < 800){
    Feeder.Set(ControlMode::PercentOutput, -0.5);
  }
  */
#endif

#ifdef Auto2
    ManpState.Set(False);
    if (RobotTimer < 95)
    {
      T1.Set(ControlMode::PercentOutput, 0.3);
      T2.Set(ControlMode::PercentOutput, 0.3);
      T3.Set(ControlMode::PercentOutput, -0.3);
      T4.Set(ControlMode::PercentOutput, -0.3);
      Feeder.Set(ControlMode::PercentOutput, 0);
    }
#endif
  }
}

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic()
{

  T1.SetNeutralMode(Coast);
  T2.SetNeutralMode(Coast);
  T3.SetNeutralMode(Coast);
  T4.SetNeutralMode(Coast);
  LeftMax = LeftLimit.Get();
  RightMax = RightLimit.Get();
  LeftJoy = -Joy1.GetY();
  RightJoy = -Joy2.GetY();
  //Joy = -Joy2.GetY();
  //RightJoy = Joy2.GetY();
  //Wheel = Joy1.GetX();

  //Drive.SetMaxOutput(1);

  LeftJoy = LeftSpeed;
  RightJoy = RightSpeed;
  if (Joy1.GetY() < 0.5)
  {
    T1.Set(ControlMode::PercentOutput, 0);
    T2.Set(ControlMode::PercentOutput, 0);
  }
  if (Joy1.GetY() > -0.5)
  {
    T1.Set(ControlMode::PercentOutput, 0);
    T2.Set(ControlMode::PercentOutput, 0);
  }
  if (Joy2.GetY() < 0.5)
  {
    T1.Set(ControlMode::PercentOutput, 0);
    T2.Set(ControlMode::PercentOutput, 0);
  }
  if (Joy2.GetY() > -0.5)
  {
    T1.Set(ControlMode::PercentOutput, 0);
    T2.Set(ControlMode::PercentOutput, 0);
  }

  T1.Set(ControlMode::PercentOutput, -Joy1.GetY());
  T2.Set(ControlMode::PercentOutput, -Joy1.GetY());
  T3.Set(ControlMode::PercentOutput, Joy2.GetY());
  T4.Set(ControlMode::PercentOutput, Joy2.GetY());

  /*
if(UltraS.GetValue()  >= 10){
}
*/
  /*
  Drive.CurvatureDrive(Joy, Wheel, Joy1.GetRawButton(1));
  if (Xb1.GetYButton())
  {
    T1.Set(ControlMode::PercentOutput, 0.5);
    T2.Set(ControlMode::PercentOutput, 0.5);
    T2.Set(ControlMode::PercentOutput, -0.5);
    T4.Set(ControlMode::PercentOutput, -0.5);
  }
  if (Xb1.GetXButton())
  {
    T1.Set(ControlMode::PercentOutput, -0.5);
    T2.Set(ControlMode::PercentOutput, -0.5);
    T2.Set(ControlMode::PercentOutput, 0.5);
    T4.Set(ControlMode::PercentOutput, 0.5);
  }
*/
  if (Joy2.GetRawButton(3))
  {
    Shifter.Set(true);
  }
  if (Joy2.GetRawButton(2))
  {
    Shifter.Set(false);
  }

  //////////////////////////////////////////////////////
  //shooter
  Ballsense = Ball.Get();
  BallTimer++;
  //Feeder.SetNeutralMode(Coast);
  if (0 < Joy2.GetZ() < -0.25)
  {
    Shooter.Set(ControlMode::PercentOutput, 0);
  }

  if (0 < Joy2.GetZ() > 0.25)
  {
    Shooter.Set(ControlMode::PercentOutput, 0);
  }
  Shooter.Set(ControlMode::PercentOutput, -0.3);

  if (Xb1.GetBumper(frc::GenericHID::kRightHand))
  {
    ManpState.Set(true);
  }
  if (Xb1.GetBumper(frc::GenericHID::kLeftHand))
  {
    ManpState.Set(false);
  }
  /*
  if (Ballsense == 1)
  {
    Manp1.Set(ControlMode::PercentOutput, -0.2);
    Manp2.Set(ControlMode::PercentOutput, -0.2);
    Feeder.Set(ControlMode::PercentOutput, -1.0);
  }
  else
  {
    Manp1.Set(ControlMode::PercentOutput, -Xb1.GetTriggerAxis(frc::GenericHID::kRightHand));
    Manp2.Set(ControlMode::PercentOutput, -Xb1.GetTriggerAxis(frc::GenericHID::kRightHand));
    Feeder.Set(ControlMode::PercentOutput, -Xb1.GetTriggerAxis(frc::GenericHID::kLeftHand));
  }
  */

  /*
  if (IndexState == Idle)
  {
    if (Ballsense == 1)
    {
      BallTimer = 0;
      Manp1.Set(ControlMode::PercentOutput, -0.5);
      Manp2.Set(ControlMode::PercentOutput, -0.5);
      Feeder.Set(ControlMode::PercentOutput, -0.5);
      IndexState = Trasfer;
    }
    else
    {
      Feeder.Set(ControlMode::PercentOutput, -Xb1.GetTriggerAxis(frc::GenericHID::kLeftHand));
      Manp1.Set(ControlMode::PercentOutput, -Xb1.GetTriggerAxis(frc::GenericHID::kRightHand));
      Manp2.Set(ControlMode::PercentOutput, -Xb1.GetTriggerAxis(frc::GenericHID::kRightHand));
    }
  }

  if (IndexState == Trasfer)
  {
    if (BallTimer > 9)
    {
      Manp1.Set(ControlMode::PercentOutput, 0);
      Manp2.Set(ControlMode::PercentOutput, 0);
      IndexState = Index;
    }
  }

  if (IndexState == Index)
  {
    if (BallTimer > 18)
    {
      Feeder.Set(ControlMode::PercentOutput, 0);
    }
    if (BallTimer > 27)
    {
      IndexState = Idle;
    }
  }
  */

  if (Ballsense == 1)
  {
    BallTimer = 0;
  }
  else
  {
    Feeder.Set(ControlMode::PercentOutput, -Xb1.GetTriggerAxis(frc::GenericHID::kLeftHand));
    Manp1.Set(ControlMode::PercentOutput, -Xb1.GetTriggerAxis(frc::GenericHID::kRightHand));
    Manp2.Set(ControlMode::PercentOutput, -Xb1.GetTriggerAxis(frc::GenericHID::kRightHand));
  }

  if (BallTimer < 20)
  {
    Feeder.Set(ControlMode::PercentOutput, -0.5);
  }

  if (BallTimer > 10000)
  {
    BallTimer = 500;
  }

  ///////////////////////////////////////////////////
  //Climb

  //if (Driver.GetMatchTime() < 30.0)
  if (1 == 1)
  {

    if (Xb1.GetYButton()) //Enable Climb
    {
      ClimbLock.Set(True);

      LeftMax = LeftLimit.Get();
      RightMax = RightLimit.Get();
      LY = -Xb1.GetY(frc::GenericHID::kRightHand);
      RY = Xb1.GetY(frc::GenericHID::kLeftHand);

      if (RY > .6)
      {
        RY = .6;
      }
      if (RY < -.6)
      {
        RY = -.6;
      }
      if (LY > .6)
      {
        LY = .6;
      }
      if (LY < -.6)
      {
        LY = -.6;
      }

      RightClimb.Set(LY);
      LeftClimb.Set(RY);
    }
    else
    {
      ClimbLock.Set(False);
      RightClimb.Set(0);
      LeftClimb.Set(0);
    }
  }
  else
  {
    ClimbLock.Set(False);
    RightClimb.Set(0);
    LeftClimb.Set(0);
  }
}

void Robot::TestPeriodic() {}

void Robot::DisabledInit() {}
void Robot::DisabledPeriodic()
{
  //std::cout << "I am Running " << std::endl;
  //LeftClimb.Set(-0.50);
  //RightClimb.Set(0.50);
  ClimbLock.Set(True);
}

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif
