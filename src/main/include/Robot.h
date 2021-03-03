/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

/*
move: left Joystick -> Up and down
rotate: right Joystick -> Right and left
extend grabber: A Button -> Toggle
collect: X Button -> Hold
eject: start or back button -> Toggle
shooter: right trigger -> Hold
climb: second controller left trigger -> Hold for 5 seconds to extend climb arm
  2nd controller's left joystick, up and down will control climbing up and down
move on bar: left Joystick -> Right and left
color wheel:  
    Turn 3 times: Y Button -> Press
    Turn to color: B Button -> Hold
    Set color: Select corresponding color of button on second controller
*/
#pragma once

#include <string>
#include <rev/CanSparkMax.h>
#include <frc/TimedRobot.h>
#include <frc/Joystick.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/drive/DifferentialDrive.h>
#include <rev/CanSparkMax.h>
#include <rev/ColorSensorV3.h>
#include <rev/ColorMatch.h>
#include <frc/Timer.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <frc/AnalogInput.h>
#include <frc/AnalogOutput.h>
#include <frc/DigitalInput.h>
#include <frc/DigitalOutput.h>
#include <frc/DriverStation.h>


class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
//  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestPeriodic() override;
  void InitializePIDControllers();
  bool LimelightTracking();
  void hopperMovement(double);
  // void resetHopper();
  double calculateRPM(double);


 private:
  //ids
  static const int leftLeadDeviceID = 11, rightLeadDeviceID = 8, leftFollowDeviceID = 9, rightFollowDeviceID = 14;
  static const int leadShooterID = 5, followerShooterID = 1, colorWheelID =7, colorWheelPivotID = 12;
  static const int turretID = 15, intakeID = 2, intakePivotID = 4, hopperID = 3;
  static const int drumID = 10, climbArmID = 13, feederID = 6;
  
  int shootcount = 0;

  int hopper4check = 0;
  int hopper4passed= 0;

  bool alreadyShooting = false;
  
  int POVCheck = 0;

  bool manualControl = false;

  // PID coefficient structure
  struct pidCoeff {
    double kP;
    double kI;
    double kD;
    double kIz;
    double kFF;
    double kMinOutput;
    double kMaxOutput;
  };
    // DETERMINE THESE EXPERIMENTALLY!!!!!!!
//pidCoeff ----------------{kP, kI, kD, kIz, kFF, kMinOutput, kMaxOutput}
  pidCoeff m_intakePivotCoeff {0.7, 0.0, 0.1, 0.0, 0.0, -1.0, 1.0};
  pidCoeff m_climbArmCoeff {0.015, 0.0, 0.0, 0.0, 0.0, -1.0, 1.0};
  pidCoeff m_colorWheelCoeff {0.1, 0.0, 0.25, 0.0, 0.0, -1.0, 1.0};
  pidCoeff m_shooterPIDCoeff {0.0005, 0.0, 0.009, 0.0, 0.000199, -1.0, 1.0};
  pidCoeff m_hopperPIDCoeff {1.5, 0.0, 10.0, 0.0, 0.0, -1.0, 1.0};
  pidCoeff m_turretPIDCoeff {0.03, 0.0, 0.0, 0.0, 0.0, -1.0, 1.0};
  pidCoeff m_intakePIDCoeff {0.03, 0.0, 0.0, 0.0, 0.0, -1.0, 1.0};
  pidCoeff m_feederPIDCoeff {0.05, 0.0, 0.0, 0.0, 0.0, -1.0, 1.0};
  


  //Drive Base
  rev::CANSparkMax m_leftLeadMotor{leftLeadDeviceID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_rightLeadMotor{rightLeadDeviceID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_leftFollowMotor{leftFollowDeviceID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_rightFollowMotor{rightFollowDeviceID, rev::CANSparkMax::MotorType::kBrushless};
  frc::DifferentialDrive m_robotDrive{m_leftLeadMotor, m_rightLeadMotor};

  //Auto
  frc::Timer autoTimer;
    int test2 = 0;
  double climbHeight = 0;
  bool fiveBallShooting = false;
  bool regularshooting = false;

  //Joystick
  frc::Joystick m_stick{0};
  frc::Joystick m_stick1{1};
  int test = 0;
  //Color Sensor
  rev::CANSparkMax drumFollow{colorWheelPivotID, rev::CANSparkMax::MotorType::kBrushless};

  int colorWheelToggle;
  int colorToggle = 0;
  int colorWheelStop = 0;
  std::string oldColor;
  int colorCount = 1;
  int buttonCheck = 0;

  // static constexpr auto i2cPort = frc::I2C::Port::kOnboard;

  // rev::ColorSensorV3 m_colorSensor{i2cPort};
  // rev::ColorMatch m_colorMatcher;

  
  static constexpr frc::Color kBlueTarget = frc::Color(0.143, 0.427, 0.429);
  static constexpr frc::Color kGreenTarget = frc::Color(0.197, 0.561, 0.240);
  static constexpr frc::Color kRedTarget = frc::Color(0.561, 0.232, 0.114);
  static constexpr frc::Color kYellowTarget = frc::Color(0.361, 0.524, 0.113);
  // Yellow
  // .31359 red
  // .572 green
  // .114 blue
  // Red
  // .5089
  // .3599
  // .131
  // fake yellow
  // .429
  // .3939
  // .17688
  //Timer
  frc::Timer colorTimer;

  //Limelight
  std::shared_ptr<NetworkTable> m_table = nt::NetworkTableInstance::GetDefault().GetTable("limelight-scorpio");
  double tx_OFFSET = 3.0;
  int shoot;

  //Turret
  rev::CANSparkMax m_turretMotor{turretID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANEncoder m_turretEncoder = m_turretMotor.GetEncoder();
  rev::CANPIDController m_turretPID = m_turretMotor.GetPIDController();


  //Collection System
  int switch0Toggle = 0;
  int switch2Toggle = 0;
  int shooterToggle = 0;
  int positionHold1;
  int positionHold2;
  int positionHold3;
  int positionHold4;
  bool full = false;
  bool fiveBallCheck = false;
  int vibecheck;
  double hopperPosition = 0.0;
  int intakeState = 0;
  bool checking = true;

  int hopper [6] = {0, 0, 0, 0, 0, 0};


  double move;
  double rotate;

  rev::CANSparkMax m_intakeMotor{intakeID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_intakePivotMotor{intakePivotID, rev::CANSparkMax::MotorType::kBrushless};
  //Pivot motor has alt. encoder
  rev::CANEncoder m_intakePivotEncoder = m_intakePivotMotor.GetAlternateEncoder(rev::CANEncoder::AlternateEncoderType::kQuadrature, 8192);
// rev::CANEncoder m_intakePivotEncoder = m_intakePivotMotor.GetEncoder();

double m_intakeAngles[3] {0.0, -0.44, -.3};
bool intakeshoottoggle = 0;

  rev::CANPIDController m_intakePivotPID = m_intakePivotMotor.GetPIDController();
  rev::CANSparkMax m_hopperMotor{hopperID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANPIDController m_hopperPID = m_hopperMotor.GetPIDController();
  rev::CANEncoder m_hopperEncoder = m_hopperMotor.GetAlternateEncoder(rev::CANEncoder::AlternateEncoderType::kQuadrature, 8192);
  frc::DigitalInput switch1{2};
  frc::DigitalInput switch2{1}; 
  frc::Timer intakeTimer;

  double motorSpeed = 3500/*distanceOffFromMinimum / minimumDistance;*/;


  //Climb
  rev::CANSparkMax m_drumMotor{drumID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANEncoder m_drumMotorEncoder = m_drumMotor.GetEncoder();

  rev::CANSparkMax m_climbArmMotor{climbArmID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANPIDController m_climbArmPID = m_climbArmMotor.GetPIDController();
  rev::CANEncoder m_climbArmEncoder = m_climbArmMotor.GetEncoder();


  //Shooter
  rev::CANSparkMax m_shooterLead{leadShooterID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_shooterFollow{followerShooterID, rev::CANSparkMax::MotorType::kBrushless};  
  rev::CANEncoder m_shooterEncoder = m_shooterLead.GetEncoder();
  rev::CANPIDController m_shooterPID = m_shooterLead.GetPIDController();
  rev::CANEncoder m_shooterFollowEncoder = m_shooterFollow.GetEncoder();
  rev::CANPIDController m_shooterFollowPID = m_shooterFollow.GetPIDController();
  int shot = 0;
  frc::Timer shooterTimer;
  frc::Timer fiveBallTimer;


  //Feeder
  rev::CANSparkMax m_feederMotor{feederID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANPIDController m_feederPID = m_feederMotor.GetPIDController();
  double feederDirection = 1.0;


  
  double constantLimelightAngle = 22;
  double heightLimelight = 21.275;
  double heightOfTarget = 89.75;
  double distanceBaseline = 10.0;
  double distanceOffFromMinimum;
  double baselineShooterRPM = 3000;
  double maximumDistance = 360;
  double minimumDistance = 30;

  // Targeting state
  bool m_IsTargeting = false;
  
  //Driver Station
  std::string gameData;


  

  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;
};
