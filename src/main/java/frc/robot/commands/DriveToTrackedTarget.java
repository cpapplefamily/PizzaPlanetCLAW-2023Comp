// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;

public class DriveToTrackedTarget extends CommandBase {
  private Drivetrain drivetrain;
    private Vision vision;

    double angleToTarget;
    int targetTagID;
    double desiredDistanceToTarget;
    double targetArea;
    boolean usingArea;
    double translationalError;
  /** Creates a new DriveToTrackedTarget. */

  //Rotates and drives to best (nearest) target
  public DriveToTrackedTarget(Drivetrain drivetrain, Vision vision, double distanceToTarget) {
    this.drivetrain = drivetrain;
    this.vision = vision;
    desiredDistanceToTarget = distanceToTarget;
    addRequirements(drivetrain, vision);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  //Rotates and drives to specific April tag
  public DriveToTrackedTarget(double distanceToTarget, int targetTagID)  {
    //this.distanceToTarget = distanceToTarget;
    this.targetTagID = targetTagID;
  }

  //Rotates and drives to best (nearest) target using area instead of distance
  public DriveToTrackedTarget(double targetArea, boolean usingArea)  {
    //this(targetArea);
    this.usingArea = usingArea;
  }

  //Rotates and drives to specific April tag using area instead of distance
  public DriveToTrackedTarget(double targetArea, int targetTagID, boolean usingArea)  {
    this(targetArea, targetTagID);
    this.usingArea = usingArea;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (vision.getHasTarget())  {
      PhotonTrackedTarget trackedTarget;
      if (targetTagID == 0)  {
        trackedTarget = vision.getBestTarget();
      }  else  {
        trackedTarget = vision.getTargetWithID(targetTagID);
      }
      if (trackedTarget != null)  {
        double rotationalError = trackedTarget.getYaw();
        double translationValue;
        if (usingArea)  {
          translationalError = desiredDistanceToTarget - trackedTarget.getArea();
          translationValue = translationalError * Constants.TRACKED_TAG_AREA_KP;
        }  else  {
          translationalError = -desiredDistanceToTarget + PhotonUtils.calculateDistanceToTargetMeters(
            Constants.CAMERA_HEIGHT_METERS,
            Constants.TARGET_HEIGHT_METERS,
            Constants.CAMERA_PITCH_RADIANS,
            trackedTarget.getPitch()
          );
          translationValue = translationalError * Constants.TRACKED_TAG_DISTANCE_DRIVE_KP*3;
        }
      double rotationValue = -rotationalError * Constants.TRACK_TAG_ROTATION_KP;
      double leftPower = translationValue - rotationValue;  //NEGATIVE
      double rightPower = translationValue + rotationValue;  //POSITIVE
      double leftDriveRate;
      double rightDriveRate;
      if (leftPower > Constants.APRILTAG_POWER_CAP  || rightPower > Constants.APRILTAG_POWER_CAP)  {
        double max = Math.max(Math.abs(leftPower), Math.abs(rightPower));
        leftDriveRate = Math.copySign(leftPower/max, leftPower);
        rightDriveRate = Math.copySign(rightPower/max, rightPower);
      }  else if (leftPower < -Constants.APRILTAG_POWER_CAP  || rightPower < -Constants.APRILTAG_POWER_CAP)  {
          double min = Math.max(Math.abs(leftPower), Math.abs(rightPower));
          leftDriveRate = Math.copySign(leftPower/min, leftPower);
          rightDriveRate = Math.copySign(rightPower/min, rightPower);
      } else  {
        leftDriveRate = leftPower;
        rightDriveRate = rightPower;
      }
      drivetrain.tankDrive(leftDriveRate, rightDriveRate);

      // System.out.println("TargetArea: " + desiredDistanceToTarget);
      // System.out.println("CurrentArea: " + translationValue);
      // System.out.println("Distance:  " + desiredDistanceToTarget);
      // System.out.println("RotationalError:  " + rotationalError);
      // System.out.println("TranslationalError:  " + translationalError);
      // System.out.println("RotationalValue:  " + rotationValue);
      // System.out.println("TranslationalValye:  " + translationValue);
      // System.out.println("LeftDriveRate:  " + leftDriveRate);
      // System.out.println("RightDriveRate:  " + rightDriveRate);
    }
    }  else {
      drivetrain.stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (usingArea)  {
      return desiredDistanceToTarget <= 0.5;
    }  else {
      return translationalError <= 0.2;
    }
  }
}




