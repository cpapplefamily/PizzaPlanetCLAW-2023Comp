// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;

public class TurnToTrackedTarget extends CommandBase {
Drivetrain drivetrain;
Vision vision;
double visionTargetAngle;
double visionKP;
double visionError;
  /** Creates a new TurnToTrackedTarget. */

  public TurnToTrackedTarget(Drivetrain drivetrain, Vision vision) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    this.vision = vision;
    visionKP = Constants.TRACK_TAG_ROTATION_KP;
    addRequirements(drivetrain, vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (vision.getHasTarget())  {
      visionError = vision.getBestTarget().getYaw();
      double value = -Math.min(visionError*visionKP, 1);  //calculate motor percentage value
      drivetrain.arcadeDrive(-value, value);
    }
    else {
      drivetrain.stop();
    }
    }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //System.out.println("TARGET TRACKING ENDED");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
    //return Math.abs(visionError) < 3;
  }
}
