// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Vision extends SubsystemBase {
  /** Creates a new Vision. */
  public PhotonCamera camera = new PhotonCamera(Constants.USB_CAMERA_NAME);
  boolean hasTarget;  //True means target detected; false means no target detected
  PhotonPipelineResult result;  //sores all collected Photonvision data

  public Vision() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    PhotonPipelineResult result = camera.getLatestResult();
    hasTarget = result.hasTargets();
    if (hasTarget)  {
      this.result = result;
    }
  }

public PhotonTrackedTarget getTargetWithID(int id) {  //Returns the April tage ID (if it exists)
  List<PhotonTrackedTarget> targets = result.getTargets();
  for (PhotonTrackedTarget i : targets)  {
    if (i.getFiducialId() == id)  {
      return i;
    }
  }
  return null;  //No target found
}

public PhotonTrackedTarget getBestTarget() {
  if  (hasTarget) {
    return result.getBestTarget();
  }
  else {
    return null;
  }
}

public boolean getHasTarget() {
  return hasTarget;
}

}
