// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PhotonVisionConstants;

public class PhotonVisionSubsytem extends SubsystemBase {
  /** Creates a new PhotonVisionSubsytem. */
  public PhotonVisionSubsytem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

    /** VISION */
    public double getRangeToTag(PhotonPipelineResult result){
      double range =
        PhotonUtils.calculateDistanceToTargetMeters(
                PhotonVisionConstants.CAMERA_HEIGHT_METERS,
                PhotonVisionConstants.TARGET_HEIGHT_METERS,
                PhotonVisionConstants.CAMERA_PITCH_RADIANS,
                Units.degreesToRadians(result.getBestTarget().getPitch()));
      return range;
    }

    /** Calculate strafe speed */
    public double getVisionStrafeSpeed(DriveSubsystem m_DriveSubsystem, PhotonPipelineResult result){
      double strafeSpeed;
      if (result.hasTargets()) {
        strafeSpeed = -m_DriveSubsystem.controller.calculate(result.getBestTarget().getYaw(), 0);
      } else {
        strafeSpeed = 0;
      }
      return strafeSpeed;
    }
    
    /** Calculate forward speed */
    public double getVisionForwardSpeed(DriveSubsystem m_DriveSubsystem, PhotonPipelineResult result){
      double forwardSpeed;
      if (result.hasTargets()) {
        forwardSpeed = -m_DriveSubsystem.controller.calculate(getRangeToTag(result), PhotonVisionConstants.GOAL_RANGE_METERS);
      } else {
        forwardSpeed = 0;
      }
      return forwardSpeed;
    }
}
