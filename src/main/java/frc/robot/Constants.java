// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
  public static class DriveConstants {
    // Constants for @params  
    public static final int k_FrontLeftMotorPort = 3;
    public static final int k_RearLeftMotorPort = 4;
    public static final int k_FrontRightMotorPort = 1;
    public static final int k_RearRightMotorPort = 2;
    public static final int k_TestMotor = 6;

    public static final int[] k_FrontLeftEncoderPorts = new int[] {0, 1};
    public static final int[] k_RearLeftEncoderPorts = new int[] {2, 3};
    public static final int[] k_FrontRightEncoderPorts = new int[] {4, 5};
    public static final int[] k_RearRightEncoderPorts = new int[] {6, 7};

    public static final boolean k_FrontLeftEncoderReversed = false;
    public static final boolean k_RearLeftEncoderReversed = true;
    public static final boolean k_FrontRightEncoderReversed = false;
    public static final boolean k_RearRightEncoderReversed = true;

    public static final double k_TrackWidth = 0.5;
    // Distance between centers of right and left wheels on robot
    public static final double k_WheelBase = 0.7;
    // Distance between centers of front and back wheels on robot

    public static final MecanumDriveKinematics k_DriveKinematics =
        new MecanumDriveKinematics(
            new Translation2d(k_WheelBase / 2, k_TrackWidth / 2),
            new Translation2d(k_WheelBase / 2, -k_TrackWidth / 2),
            new Translation2d(-k_WheelBase / 2, k_TrackWidth / 2),
            new Translation2d(-k_WheelBase / 2, -k_TrackWidth / 2));

    public static final int k_EncoderCPR = 1024;
    public static final double k_WheelDiameterMeters = 0.15;
    public static final double k_EncoderDistancePerPulse =
        // Assumes the encoders are directly mounted on the wheel shafts
        (k_WheelDiameterMeters * Math.PI) / (double) k_EncoderCPR;

    // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
    // These characterization values MUST be determined either experimentally or theoretically
    // for *your* robot's drive.
    // The SysId tool provides a convenient method for obtaining these values for your robot.
    public static final SimpleMotorFeedforward k_Feedforward =
        new SimpleMotorFeedforward(1, 0.8, 0.15);

    // Example value only - as above, this must be tuned for your drive!
    public static final double k_PFrontLeftVel = 0.5;
    public static final double k_PRearLeftVel = 0.5;
    public static final double k_PFrontRightVel = 0.5;
    public static final double k_PRearRightVel = 0.5;
  }

  public static class PhotonVisionConstants {
    final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(24);
    final double TARGET_HEIGHT_METERS = Units.feetToMeters(5);
    // Angle between horizontal and the camera.
    final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(0);

    // How far from the target we want to be
    final double GOAL_RANGE_METERS = Units.feetToMeters(3);

  }
}
