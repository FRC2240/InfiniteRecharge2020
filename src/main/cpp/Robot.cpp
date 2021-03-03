/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"
#include "log.h"
#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>


double clamp(double in, double minval, double maxval)
{
  if (in > maxval) return maxval;
  if (in < minval) return minval;
  return in;
}

double Robot::calculateRPM(double d){
  double rpm; 
  if(d < 125.0){
//    rpm = (-95.671 * d) + 14322.0;
    rpm = 1.19 * ((-45.671 * d) + 14322.0);
  }else{
    rpm = 1.19 * (0.028052 * d * d - 8.5977 * d + 2946.0);
  }
  return rpm;
}

bool Robot::LimelightTracking()
{

  bool shoot = false;

  // Proportional Steering Constant:
  // If your robot doesn't turn fast enough toward the target, make this number bigger
  // If your robot oscillates (swings back and forth past the target) make this smaller
  const double STEER_K = 0.03 ;

  const double MAX_STEER = 0.2;

  double tx = m_table->GetNumber("tx",0.0);
  double tv = m_table->GetNumber("tv",0.0);

  double limelightTurnCmd = 0.0;

  if (tv > 0.0)
  {
        // Proportional steering
        LOGGER(INFO) << "tx: " << tx;
        limelightTurnCmd = (tx + tx_OFFSET)*STEER_K;
        limelightTurnCmd = clamp(limelightTurnCmd,-MAX_STEER, MAX_STEER);
        if (tx < .25) {
          shoot = true;
        }
  }
  // LOGGER(INFO) << "limelightTurnCmd value: " << limelightTurnCmd;
  m_turretMotor.Set(limelightTurnCmd);
  return shoot;
}

void Robot::hopperMovement(double rotations) {
    hopperPosition = hopperPosition - (rotations * .25);
    LOGGER(INFO) << "Hopper position: " << hopperPosition;
    m_hopperPID.SetReference(hopperPosition, rev::ControlType::kPosition);
    positionHold1 = hopper[1];
    positionHold2 = hopper[2];
    positionHold3 = hopper[3];
    positionHold4 = hopper[4];
    hopper[1] = positionHold4;
    hopper[2] = positionHold1;
    hopper[3] = positionHold2;
    hopper[4] = positionHold3;
}

// void Robot::resetHopper() {
//   int rotation = m_hopperEncoder.GetPosition() / 168;
//   double offsetInRotation = m_hopperEncoder.GetPosition() - rotation;
//   m_hopperPID.SetReference((m_hopperEncoder.GetPosition() - offsetInRotation), rev::ControlType::kPosition);
// }

void Robot::RobotInit() {
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

  // m_leftLeadMotor.RestoreFactoryDefaults();
  // m_rightLeadMotor.RestoreFactoryDefaults();
  // m_leftFollowMotor.RestoreFactoryDefaults();
  // m_rightFollowMotor.RestoreFactoryDefaults();

  m_leftFollowMotor.Follow(m_leftLeadMotor);
  m_rightFollowMotor.Follow(m_rightLeadMotor);

  // m_feederMotor.SetSmartCurrentLimit(20);

  drumFollow.Follow(m_drumMotor, true);

  m_shooterFollow.Follow(m_shooterLead, true);

  InitializePIDControllers();
  
  m_intakePivotEncoder.SetPosition(0.0);
  m_turretEncoder.SetPosition(0.0);
  m_drumMotorEncoder.SetPosition(0.0);
  m_climbArmEncoder.SetPosition(0.0);
  m_hopperEncoder.SetPosition(0.0);

  //Turret soft stops
  m_turretMotor.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, true);
  m_turretMotor.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, true);
  m_turretMotor.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, -15.0);
  m_turretMotor.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, 15.0);

  m_hopperPID.SetFeedbackDevice(m_hopperEncoder);
  m_hopperEncoder.SetInverted(true);
  m_intakePivotPID.SetFeedbackDevice(m_intakePivotEncoder);
  m_intakePivotEncoder.SetInverted(true);

  frc::SmartDashboard::PutNumber("shooter speed", 3500);

  frc::SmartDashboard::PutNumber("P Gain", m_shooterPIDCoeff.kP);
  frc::SmartDashboard::PutNumber("I Gain", m_shooterPIDCoeff.kI);
  frc::SmartDashboard::PutNumber("D Gain", m_shooterPIDCoeff.kD);
  frc::SmartDashboard::PutNumber("I Zone", m_shooterPIDCoeff.kIz);
  frc::SmartDashboard::PutNumber("Feed Forward", m_shooterPIDCoeff.kFF);
  frc::SmartDashboard::PutNumber("Max Output", m_shooterPIDCoeff.kMaxOutput);
  frc::SmartDashboard::PutNumber("Min Output", m_shooterPIDCoeff.kMinOutput);

  frc::SmartDashboard::PutNumber("tx offset", tx_OFFSET);

}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
//void Robot::RobotPeriodic() {}

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
void Robot::AutonomousInit() {
  m_autoSelected = m_chooser.GetSelected();
  // m_autoSelected = SmartDashboard::GetString("Auto Selector",
  //     kAutoNameDefault);
  std::cout << "Auto selected: " << m_autoSelected << std::endl;

  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

void Robot::AutonomousPeriodic() {
  autoTimer.Start();
  if (autoTimer.Get() <= 0.25) {
    m_robotDrive.ArcadeDrive(-.5, 0);
  }
  if (autoTimer.Get() >= 2 && autoTimer.Get() <= 10) {
    m_table->PutNumber("pipeline", 0);
    double ty = m_table->GetNumber("ty",0.0);
    double distance = ((heightOfTarget-heightLimelight) / tan((constantLimelightAngle+ty) * (3.141592653 / 180 )));
    m_intakePivotPID.SetReference(m_intakeAngles[1], rev::ControlType::kPosition);
    m_shooterPID.SetReference(calculateRPM(distance), rev::ControlType::kVelocity);
      if (LimelightTracking()){
      LOGGER(INFO) << "SHOOTHER DOWN";
      if ((m_shooterEncoder.GetVelocity() >= (calculateRPM(distance) * 0.995))/* && (shooterTimer.Get() < .75)*//* && (hopper[3] == 1)*/) {
        m_feederMotor.Set(.75);
        shooterTimer.Start();
        LOGGER(INFO) << "not here";
      }
      if (shooterTimer.Get() >= .75){
        hopperMovement(1);
        shooterTimer.Reset();
      }
      
      // if ((hopper[3] == 0)) {
      //   hopperMovement(1.0);
      // }
    }
  } else {
      m_table->PutNumber("pipeline", 1);
      m_shooterLead.Set(0);
      m_feederMotor.Set(0);
      m_turretPID.SetReference(0, rev::ControlType::kPosition);
      shooterTimer.Stop();
      shooterTimer.Reset();
  } 
}



void Robot::TeleopInit() {
  m_table->PutNumber("pipeline", 1);
}

void Robot::TeleopPeriodic() {
  double kMotorSpeed = frc::SmartDashboard::GetNumber("shooter speed", 3500);

  frc::SmartDashboard::GetNumber("P Gain", m_shooterPIDCoeff.kP);
  frc::SmartDashboard::GetNumber("I Gain", m_shooterPIDCoeff.kI);
  frc::SmartDashboard::GetNumber("D Gain", m_shooterPIDCoeff.kD);
  frc::SmartDashboard::GetNumber("I Zone", m_shooterPIDCoeff.kIz);
  frc::SmartDashboard::GetNumber("Feed Forward", m_shooterPIDCoeff.kFF);
  frc::SmartDashboard::GetNumber("Max Output", m_shooterPIDCoeff.kMaxOutput);
  frc::SmartDashboard::GetNumber("Min Output", m_shooterPIDCoeff.kMinOutput);

  frc::SmartDashboard::GetNumber("tx offset", tx_OFFSET);

  if((kMotorSpeed != motorSpeed)) { motorSpeed = kMotorSpeed; }

  // LOGGER(INFO) << "hopper" << m_hopperEncoder.GetPosition();
  int shooterOn = 0;
  int collectionSwitch = switch1.Get();
  int hopperSwitch = switch2.Get();
  // LOGGER(INFO) << "switch" << hopperSwitch;
  //Drive
  if (m_stick1.GetRawButton(5)) {
  move = 0.5 * m_stick.GetRawAxis(1);
  rotate = 0.5 * 0.7 * m_stick.GetRawAxis(4);
  } else {
  move = m_stick.GetRawAxis(1);
  rotate = 0.7 * m_stick.GetRawAxis(4);
  }
  // move = m_stick.GetRawAxis(1);
  // rotate = 0.7 * m_stick.GetRawAxis(4);
  m_robotDrive.ArcadeDrive(-move, rotate);

  //Intake: hopper array {0, 1, 2, 3, 4, 5}
  /*
  ____________
  |          |
  |          |
  |    5*    |  <----------Shooter
  |          |
  |__________|
  |\        /|
  | \  3*  / |
  |  \    /  |
  |   \  /   |
  |    \/    |
  | 4  /\  2 |  <-------Hopper
  |   /  \   |
  |  /    \  |      * = limit switch location
  | /  1*  \ |
  |/________\|   
    |       |
    |   0*  |   <----- intake
    |       |    
    |_______|
  */
//  if (m_stick1.GetRawButton(8)) {
//    resetHopper();
//  }
  

//Checks if button has been held down for longer than one loop. If so, checking is false, and will not change state of intake.
if((intakeState == 0) && (m_stick.GetRawButton(3)) && (checking == false)){
    checking = true;
    intakeState = 1;
}else if ((intakeState == 1) && (m_stick.GetRawButton(3)) && (checking == false)){
    checking = true;
    intakeState = 0;
}else if (m_stick.GetRawButton(3) == false){
    checking = false;
}
if (m_stick.GetRawButton(2)) {
  m_intakeMotor.Set(-1);
}
if ((m_stick1.GetPOV(0) != -1) && (POVCheck == 0)) {
  POVCheck = 1;
  switch (m_stick1.GetPOV(0)) {
    case 0 :
      hopperMovement(1);
      break;
    case 90 :
      hopperMovement(2);
      break;
    case 180 :
      hopperMovement(-1);
      break;
    case 270 :
      hopperMovement(-2);
      break;
    default :
      break;
  }
} else if ((POVCheck == 1) && (m_stick1.GetPOV(0) == -1)) {
  POVCheck = 0;
}
  int passed = 0;
  // Switch# == 0 means it's on
  //intake position
  if (collectionSwitch == 0) {
    hopper[0] = 1;
    vibecheck = 1;
  } else if ((vibecheck == 1) && (collectionSwitch == 1)) {
    hopper[0] = 0;
    passed = 1;
    vibecheck = 0;
  }
  else {  
    hopper[0] = 0;
  }
  if (hopperSwitch == 0) {
    hopper[4] = 1;
  } else {
    hopper[4] = 0;
  }


  //position 3
  if (hopperSwitch == 0 && hopper4check == 0 && hopper4passed == 0) {
    hopper4check = 1;
  } else if (hopper4check == 1 && hopperSwitch == 1) {
    hopper4passed = 1;
    hopper4check = 0;
  }
  // LOGGER(INFO) << "passed" << hopper4passed;
  // LOGGER(INFO) << "ooops";

  // if ((hopper[4] == 1) && fiveBallCheck == false) {
  //   fiveBallCheck = true;
  // }
  if (/*fiveBallCheck == true &&*/ hopper[4] == 1 && hopper4passed == 1) {
    full = true;
  } else {
    full = false;
  }
  LOGGER(INFO) << "full" << full;
//  LOGGER(INFO) << "full" << full << "\n";
  LOGGER(INFO) << hopper[0] << hopper[4];

//Sets intake to rest or working
if((intakeState == 1) && full == false){
  m_intakePivotPID.SetReference(m_intakeAngles[1], rev::ControlType::kPosition);
//  LOGGER(INFO) << "DOWN";
  if (passed == 1){
    intakeTimer.Start();
    passed = 0;
  }
  if (intakeTimer.Get() >= .5) {
    hopperMovement(1);
    intakeTimer.Stop();
    intakeTimer.Reset();
  }
  if((m_intakePivotEncoder.GetPosition() >= -.5) && (m_intakePivotEncoder.GetPosition() <= -.4)){
    m_intakeMotor.Set(intakeState);
  }
} else if ((intakeState == 0) && (shoot != 1) && (manualControl == false)) {
    m_intakePivotPID.SetReference(m_intakeAngles[0], rev::ControlType::kPosition);
    m_intakeMotor.Set(0);
} else if ((intakeState == 1) && (full == true) && (collectionSwitch == 1)) {
    m_intakePivotPID.SetReference(m_intakeAngles[1], rev::ControlType::kPosition);
    if((m_intakePivotEncoder.GetPosition() >= -.5) && (m_intakePivotEncoder.GetPosition() <= -.4)){
      m_intakeMotor.Set(intakeState);
    }
} else if ((intakeState == 1) && (full == true) && (collectionSwitch == 0)) {
    m_intakePivotPID.SetReference(m_intakeAngles[1], rev::ControlType::kPosition);
    m_intakeMotor.Set(0);
}


  
  //Shooter
  // 2/29 change: Changed lines 408 and 411 from "motorSpeed" to "calculateRPM(distance)"

  frc::SmartDashboard::PutNumber("Current RPM", m_shooterEncoder.GetVelocity());
  if (m_stick.GetRawAxis(3) < .75) {
      m_table->PutNumber("pipeline", 1);
      m_shooterLead.Set(0);
      m_feederMotor.Set(0);
      m_turretPID.SetReference(0, rev::ControlType::kPosition);
      shooterTimer.Stop();
      shooterTimer.Reset();
      shoot = 0;
      shootcount = 0;
      fiveBallTimer.Stop();
      fiveBallTimer.Reset();
  }
  if ((fabs(m_stick.GetRawAxis(3)) > 0.75) && ((hopper[0] == 0 && !fiveBallShooting) || (hopper[0] == 0 && regularshooting))) {
    LOGGER(INFO) << "4 ball";    
    regularshooting = true;
    hopper4passed = 0;
    alreadyShooting = true;
    m_table->PutNumber("pipeline", 0);
    double ty = m_table->GetNumber("ty",0.0);
    double distance = ((heightOfTarget-heightLimelight) / tan((constantLimelightAngle+ty) * (3.141592653 / 180 )));
    frc::SmartDashboard::PutNumber("Current Distance", distance);
    LOGGER(INFO) << "distance" << distance;
    // LOGGER(INFO) << "ty" << ty;
    shoot = 1;
    m_intakePivotPID.SetReference(m_intakeAngles[1], rev::ControlType::kPosition);
    LOGGER(INFO) << "RPM" << calculateRPM(distance);
    m_shooterPID.SetReference(calculateRPM(distance), rev::ControlType::kVelocity);
    if (LimelightTracking()){
      LOGGER(INFO) << "motor speed" << m_shooterEncoder.GetVelocity();
      if ((m_shooterEncoder.GetVelocity() >= (calculateRPM(distance) * 0.995))/* && (shooterTimer.Get() < .75)*//* && (hopper[3] == 1)*/) {
        m_feederMotor.Set(.75);
        shooterTimer.Start();
      }
      if (shooterTimer.Get() >= .5){
        hopperMovement(1);
        shootcount++;
        shooterTimer.Reset();
      }
    }
  } else if ((fabs(m_stick.GetRawAxis(3)) > 0.75) && ((hopper[0] == 1 && !regularshooting) || (hopper[0] == 0 && fiveBallShooting))) {
    LOGGER(INFO) << "5 ball";
    fiveBallShooting = true;
    hopper4passed = 0;
    alreadyShooting = true;
    m_table->PutNumber("pipeline", 0);
    double ty = m_table->GetNumber("ty",0.0);
    double distance = ((heightOfTarget-heightLimelight) / tan((constantLimelightAngle+ty) * (3.141592653 / 180 )));
    frc::SmartDashboard::PutNumber("Current Distance", distance);
    LOGGER(INFO) << "distance" << distance;
    // LOGGER(INFO) << "ty" << ty;
    shoot = 1;
    if (intakeshoottoggle == 0) {
    m_intakePivotPID.SetReference(m_intakeAngles[2], rev::ControlType::kPosition);
    intakeshoottoggle = 1;
    }
    LOGGER(INFO) << "RPM" << calculateRPM(distance);
    m_shooterPID.SetReference(calculateRPM(distance), rev::ControlType::kVelocity);
    if (LimelightTracking()){
      LOGGER(INFO) << "motor speed" << m_shooterEncoder.GetVelocity();
      if ((m_shooterEncoder.GetVelocity() >= (calculateRPM(distance) * 0.995))/* && (shooterTimer.Get() < .75)*//* && (hopper[3] == 1)*/) {
        m_feederMotor.Set(.75);
        shooterTimer.Start();
      }
      if (shooterTimer.Get() >= .5 && shootcount <= 1){
        hopperMovement(1);
        shootcount++;
        shooterTimer.Reset();
      } else if (shootcount > 1 && shooterTimer.Get() >= .5) {
        shootcount++;
        shooterTimer.Reset();
      }
      if (shootcount == 2) {
        m_intakePivotPID.SetReference(m_intakeAngles[1], rev::ControlType::kPosition);
      } else if (shootcount == 5) {
        fiveBallTimer.Start();
        m_intakeMotor.Set(1);
      }
      if (fiveBallTimer.Get() >= 2) {
        hopperMovement(1);
        fiveBallTimer.Reset();
      }
    }
  } else {
    intakeshoottoggle = 0;
    regularshooting = false;
    fiveBallShooting = false;
  }
    

  // Climbing code
  if ((m_stick1.GetRawAxis(1) > .5) && climbHeight >= 0) {
    m_climbArmPID.SetReference(climbHeight, rev::ControlType::kPosition);
    climbHeight = climbHeight -1;
  } else if ((m_stick1.GetRawAxis(1) < -.5) && climbHeight <= 160) {
    m_climbArmPID.SetReference(climbHeight, rev::ControlType::kPosition);
    climbHeight++;
  }
    // LOGGER(INFO) << "climbHeight" << climbHeight;

  // m_climbArmPID.SetReference(145*(m_stick1.GetRawAxis(3)), rev::ControlType::kPosition);
  m_drumMotor.Set(-m_stick1.GetRawAxis(2));

  // LOGGER(INFO) << "Intake Pivot Position: " << m_intakePivotEncoder.GetPosition();
  if (m_stick1.GetRawButton(9)) {
    m_intakePivotPID.SetReference(m_intakeAngles[1], rev::ControlType::kPosition);
    manualControl = true;
  } else if (m_stick1.GetRawButton(10)) {
    m_intakePivotPID.SetReference(m_intakeAngles[0], rev::ControlType::kPosition);
    manualControl = true;
  } else {
    manualControl = false;
  }
}

 
void Robot::TestPeriodic() {
  if (m_stick1.GetRawButton(1) && m_stick1.GetRawButton(9)) {
    m_drumMotor.Set(m_stick1.GetRawAxis(2));
  } else {
    m_drumMotor.Set(0);
  }
  LOGGER(INFO) << "Position: " << m_intakePivotEncoder.GetPosition();
if ((m_stick1.GetPOV(0) != -1) && (POVCheck == 0)) {
  POVCheck = 1;
  switch (m_stick1.GetPOV(0)) {
    case 0 :
      hopperMovement(1);
      break;
    case 90 :
      hopperMovement(2);
      break;
    case 180 :
      hopperMovement(-1);
      break;
    case 270 :
      hopperMovement(-2);
      break;
    default :
      break;
  }
} else if ((POVCheck == 1) && (m_stick1.GetPOV(0) == -1)) {
  POVCheck = 0;
}

  if (m_stick1.GetRawButton(2)) {
    m_intakePivotPID.SetReference(m_intakeAngles[2], rev::ControlType::kPosition);
  }
  // double move;
  // double rotate;
  // double m_intakeAngles[2] {0.0, -0.44};

  // LOGGER(INFO) << "Intake Pivot Position: " << m_intakePivotEncoder.GetPosition();

  // if(m_stick.GetRawButton(4)){
  //   m_intakePivotPID.SetReference(m_intakeAngles[1], rev::ControlType::kPosition);
  // }else if(m_stick.GetRawButton(1)){
  //   m_intakePivotPID.SetReference(m_intakeAngles[0], rev::ControlType::kPosition);
  // }else{
  //   m_intakePivotMotor.Set(0);
  // }
  // move = m_stick.GetRawAxis(1);
  // rotate = m_stick.GetRawAxis(4);
  // LOGGER(INFO) << "Move: " << move;
  // m_robotDrive.ArcadeDrive(-move, rotate);
  // double m_intakeAngles[2] {0.0, -34.0};
  // double move = m_stick.GetRawAxis(1);
  // double rotate = m_stick.GetRawAxis(4);
  // m_robotDrive.ArcadeDrive(-move, rotate);
  // m_climbArmPID.SetReference(145*(m_stick1.GetRawAxis(3)), rev::ControlType::kPosition);
  // m_drumMotor.Set(-m_stick1.GetRawAxis(2));
  // if (m_stick1.GetRawButton(9)) {
  //   m_intakePivotPID.SetReference(m_intakeAngles[1], rev::ControlType::kPosition);
  // } else if (m_stick1.GetRawButton(10)) {
  //   m_intakePivotPID.SetReference(m_intakeAngles[0], rev::ControlType::kPosition);
  // }

}

void Robot::InitializePIDControllers() {

  m_climbArmPID.SetP(m_climbArmCoeff.kP);
  m_climbArmPID.SetI(m_climbArmCoeff.kI);
  m_climbArmPID.SetD(m_climbArmCoeff.kD);
  m_climbArmPID.SetIZone(m_climbArmCoeff.kIz);
  m_climbArmPID.SetFF(m_climbArmCoeff.kFF);
  m_climbArmPID.SetOutputRange(m_climbArmCoeff.kMinOutput, m_climbArmCoeff.kMaxOutput);

  m_intakePivotPID.SetP(m_intakePivotCoeff.kP);
  m_intakePivotPID.SetI(m_intakePivotCoeff.kI);
  m_intakePivotPID.SetD(m_intakePivotCoeff.kD);
  m_intakePivotPID.SetIZone(m_intakePivotCoeff.kIz);
  m_intakePivotPID.SetFF(m_intakePivotCoeff.kFF);
  m_intakePivotPID.SetOutputRange(m_intakePivotCoeff.kMinOutput, m_intakePivotCoeff.kMaxOutput);

  m_shooterPID.SetP(m_shooterPIDCoeff.kP);
  m_shooterPID.SetI(m_shooterPIDCoeff.kI);
  m_shooterPID.SetD(m_shooterPIDCoeff.kD);
  m_shooterPID.SetIZone(m_shooterPIDCoeff.kIz);
  m_shooterPID.SetFF(m_shooterPIDCoeff.kFF);
  m_shooterPID.SetOutputRange(m_shooterPIDCoeff.kMinOutput, m_shooterPIDCoeff.kMaxOutput);

  m_hopperPID.SetP(m_hopperPIDCoeff.kP);
  m_hopperPID.SetI(m_hopperPIDCoeff.kI);
  m_hopperPID.SetD(m_hopperPIDCoeff.kD);
  m_hopperPID.SetIZone(m_hopperPIDCoeff.kIz);
  m_hopperPID.SetFF(m_hopperPIDCoeff.kFF);
  m_hopperPID.SetOutputRange(m_hopperPIDCoeff.kMinOutput, m_hopperPIDCoeff.kMaxOutput);

  m_turretPID.SetP(m_turretPIDCoeff.kP);
  m_turretPID.SetI(m_turretPIDCoeff.kI);
  m_turretPID.SetD(m_turretPIDCoeff.kD);
  m_turretPID.SetIZone(m_turretPIDCoeff.kIz);
  m_turretPID.SetFF(m_turretPIDCoeff.kFF);
  m_turretPID.SetOutputRange(m_turretPIDCoeff.kMinOutput, m_turretPIDCoeff.kMaxOutput);
  
  m_feederPID.SetP(m_feederPIDCoeff.kP);
  m_feederPID.SetI(m_feederPIDCoeff.kI);
  m_feederPID.SetD(m_feederPIDCoeff.kD);
  m_feederPID.SetIZone(m_feederPIDCoeff.kIz);
  m_feederPID.SetFF(m_feederPIDCoeff.kFF);
  m_feederPID.SetOutputRange(m_feederPIDCoeff.kMinOutput, m_feederPIDCoeff.kMaxOutput);
}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif