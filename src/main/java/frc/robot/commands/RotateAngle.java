// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class RotateAngle extends CommandBase {
  public static final double motorTicksPerFoot = 13994.16;  //adjust this using wheelbase
  private final Drivetrain drivetrain;
  private final double distance;
  private double turnStartTime;
  /** Creates a new RotateAngle. */
  public RotateAngle(Drivetrain drivetrain, double turnAngle) {
    // Use addRequirements() here to declare subsystem dependencies.
    //distance = ((turnAngle*22*Math.PI)/360*12) * motorTicksPerFoot;  
    distance = (((turnAngle/360)*Math.PI*22) / 12) * motorTicksPerFoot;
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);

  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.zeroDrivetrainEncoders();  //try this next meeting ON BLOCKS
    turnStartTime = Timer.getFPGATimestamp();
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.setTurnMotionMagic(distance, 10000*0.5, 0.5*4000);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.stopTurnMotionMagic();
  }

  private boolean isTurnMotionMagicDone() {
    double sensorDistance = drivetrain.getLeadRightSensorPosition();
    double error = sensorDistance - distance;
    //System.out.println(" Right Front Drive Sensor ="+ sensorDistance + ", arcLength Ticks= " + distance);
    double percentErr = Math.abs(error)/Math.abs(distance);
    if(percentErr < 0.01)  {
      return true;
    }
    double timepassed = Timer.getFPGATimestamp() - turnStartTime;
    if (timepassed > 3) {
      return true;
    }

    //System.out.println(" Rotate percent error = "+percentErr +", turn time passed = "+timepassed);

    return false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean isturnMotionDone = isTurnMotionMagicDone();
    return isturnMotionDone;
  }
}