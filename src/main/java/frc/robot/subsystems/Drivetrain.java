// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */

    private WPI_TalonFX leftFront = new WPI_TalonFX(Constants.LEFT_FRONT);
    public WPI_TalonFX rightFront = new WPI_TalonFX(Constants.RIGHT_FRONT);
    private WPI_TalonFX leftBack = new WPI_TalonFX(Constants.LEFT_BACK);
    private WPI_TalonFX rightBack = new WPI_TalonFX(Constants.RIGHT_BACK);

    private DifferentialDrive drivetrain = new DifferentialDrive(leftFront, rightFront);

    private Pigeon2 pigeon = new Pigeon2(Constants.PIGEON_CANID);
    private double snail = 1.0;
    private boolean isSlow;
    private int drivetrian_flip = 1;

    public Drivetrain() {
      configMotors();
     }
  

public void configMotors(){
  leftFront.configFactoryDefault();
  rightFront.configFactoryDefault();
  leftBack.configFactoryDefault();
  rightBack.configFactoryDefault();

  leftFront.setNeutralMode(NeutralMode.Brake);
  rightFront.setNeutralMode(NeutralMode.Brake);
  leftBack.setNeutralMode(NeutralMode.Brake);
  rightBack.setNeutralMode(NeutralMode.Brake);

  leftBack.follow(leftFront);
  rightBack.follow(rightFront);

  leftBack.setInverted(false);
  leftFront.setInverted(false);
  rightBack.setInverted(true);
  rightFront.setInverted(true);

  leftFront.configOpenloopRamp(0.5);
  rightFront.configOpenloopRamp(0.5);
  leftBack.configOpenloopRamp(0.5);
  rightBack.configOpenloopRamp(0.5);

  leftFront.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 30);
  rightFront.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 30);
  leftFront.selectProfileSlot(0, 0);
  rightFront.selectProfileSlot(0, 0);

  leftFront.config_kF(0, 0.045);
  leftFront.config_kP(0, 0.049);
  leftFront.config_kI(0, 0);
  leftFront.config_kD(0, 0);
  rightFront.config_kF(0, 0.045);
  rightFront.config_kP(0, 0.049);
  rightFront.config_kI(0, 0);
  rightFront.config_kD(0, 0);

  rightFront.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, 30);
  rightFront.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, 30);
  leftFront.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, 30);
  leftFront.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, 30);

  rightFront.configNominalOutputForward(0);
  leftFront.configNominalOutputForward(0);
  rightFront.configNominalOutputReverse(0);
  rightFront.configNominalOutputReverse(0);

  rightFront.configPeakOutputForward(1);
  leftFront.configPeakOutputForward(1);
  rightFront.configPeakOutputReverse(-1);
  leftFront.configPeakOutputReverse(-1);

  leftFront.setSelectedSensorPosition(0);
  rightFront.setSelectedSensorPosition(0);
  leftBack.setSelectedSensorPosition(0);
  rightBack.setSelectedSensorPosition(0);
  leftFront.setSensorPhase(false);

  rightFront.configMotionCruiseVelocity(Constants.DRIVE_CRUISE_VELOCITY, Constants.DRIVE_PID_TIMEOUT);
  leftFront.configMotionCruiseVelocity(Constants.DRIVE_CRUISE_VELOCITY, Constants.DRIVE_PID_TIMEOUT);

  rightFront.configMotionAcceleration(Constants.DRIVE_ACCELERATION, Constants.DRIVE_PID_TIMEOUT);
  leftFront.configMotionAcceleration(Constants.DRIVE_ACCELERATION, Constants.DRIVE_PID_TIMEOUT);

}

public void arcadeDrive(double throttle, double rotation) {
    drivetrain.arcadeDrive(snail * drivetrian_flip * -throttle, snail * rotation);

}

public void tankDrive(double leftSpeed, double rightSpeed) {
    drivetrain.tankDrive(snail * -leftSpeed, snail * -rightSpeed);
}

public void driveForward(double speed) {  
  drivetrain.tankDrive(-speed, -speed);
}

public void stop() {
    drivetrain.stopMotor();
}

public void zeroPigeon(double reset) {
  pigeon.setYaw(reset);
}

public double getYaw() {
      return pigeon.getYaw();
}

public double getPitch() {
  return pigeon.getPitch();
}

public double getRoll() {
  return pigeon.getRoll();
}

public void invert_drivetrain() {
  drivetrian_flip *= -1;
}

public void snailSpeed() {
  snail = 0.6;
  isSlow = true;
}
public void normalSpeed() {
  snail = 1;
  isSlow = false;
}
public void toggleSnailSpeed() {
  if (isSlow) {
    normalSpeed();
  } 
  else {
    snailSpeed();
  }
}

public void setDriveMotionMagic(double distance, double maxvelocity, double maxAcceleration) {
    drivetrain.setSafetyEnabled(false);

    // leftFront.getSensorCollection().setIntegratedSensorPosition(0, 30);
    // rightFront.getSensorCollection().setIntegratedSensorPosition(0, 30);

    // rightFront.configMotionCruiseVelocity(Constants.DRIVE_CRUISE_VELOCITY, Constants.DRIVE_PID_TIMEOUT);
    // leftFront.configMotionCruiseVelocity(Constants.DRIVE_CRUISE_VELOCITY, Constants.DRIVE_PID_TIMEOUT);

    // rightFront.configMotionAcceleration(Constants.DRIVE_ACCELERATION, Constants.DRIVE_PID_TIMEOUT);
    // leftFront.configMotionAcceleration(Constants.DRIVE_ACCELERATION, Constants.DRIVE_PID_TIMEOUT);

    rightFront.set(ControlMode.MotionMagic, distance);  //why is this negative
    leftFront.set(ControlMode.MotionMagic, distance);
    
}

public void stopDriveMotionMagic() {
  leftFront.set(ControlMode.PercentOutput, 0);
  rightFront.set(ControlMode.PercentOutput, 0);

  drivetrain.setSafetyEnabled(true);
}

public void setTurnMotionMagic(double distance, double maxvelocity, double maxAcceleration) {
  drivetrain.setSafetyEnabled(false);

  // leftFront.getSensorCollection().setIntegratedSensorPosition(0, 30);
  // leftFront.getSensorCollection().setIntegratedSensorPosition(0, 30);

  // rightFront.configMotionCruiseVelocity(Constants.DRIVE_CRUISE_VELOCITY, Constants.DRIVE_PID_TIMEOUT);
  // leftFront.configMotionCruiseVelocity(Constants.DRIVE_CRUISE_VELOCITY, Constants.DRIVE_PID_TIMEOUT);

  // rightFront.configMotionAcceleration(Constants.DRIVE_ACCELERATION, Constants.DRIVE_PID_TIMEOUT);
  // leftFront.configMotionAcceleration(Constants.DRIVE_ACCELERATION, Constants.DRIVE_PID_TIMEOUT);

  rightFront.set(ControlMode.MotionMagic, distance);  //opposite signs to cause rotation
  leftFront.set(ControlMode.MotionMagic, -distance);
  
}

public void stopTurnMotionMagic() {
  leftFront.set(ControlMode.PercentOutput, 0);
  rightFront.set(ControlMode.PercentOutput, 0);

  drivetrain.setSafetyEnabled(true);
}

@Override
public void periodic() {

}

public boolean isDriveMagicMotionDone(double distanceTicks) {
  double sensorDistance = rightFront.getSelectedSensorPosition(0);
  double percentError = 100*(-distanceTicks - sensorDistance) / -distanceTicks;
  return (distanceTicks <14000 && percentError <5) || percentError <1;  //where did the 14000 come from?
}


public double getLeadRightSensorPosition() {
  return rightFront.getSelectedSensorPosition(0);
}


public void zeroDrivetrainEncoders() {
  leftFront.setSelectedSensorPosition(0);
  leftBack.setSelectedSensorPosition(0);
  rightFront.setSelectedSensorPosition(0);
  rightBack.setSelectedSensorPosition(0);
}

  
}




 

