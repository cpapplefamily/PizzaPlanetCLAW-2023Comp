// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class DriveForwardDistance extends CommandBase {
  public static final double motorTicksPerFoot = 13994.16;
  private final Drivetrain drivetrain;
  private final double distance;
  private double driveStartTime;
  /** Creates a new DriveForwardDistance. */
  public DriveForwardDistance(Drivetrain drivetrain, double distanceFeet) {
    // Use addRequirements() here to declare subsystem dependencies.
    distance = distanceFeet * motorTicksPerFoot;
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveStartTime = Timer.getFPGATimestamp();
    // drivetrain.setDriveMotionMagic(distance, 10000, 4000);
    drivetrain.zeroDrivetrainEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.setDriveMotionMagic(distance, 6000, 2000);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.stopDriveMotionMagic();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Timer.getFPGATimestamp() > driveStartTime + 5 || drivetrain.isDriveMagicMotionDone(distance);
  }
}
