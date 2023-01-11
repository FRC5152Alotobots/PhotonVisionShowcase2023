// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants.DriveConstants;

public class PidDriveSubsystem extends PIDSubsystem {

  /** Motors  */
  private final WPI_VictorSPX m_frontLeftMotor = new WPI_VictorSPX(DriveConstants.k_FrontLeftMotorPort);
  private final WPI_VictorSPX m_rearLeftMotor = new WPI_VictorSPX(DriveConstants.k_RearLeftMotorPort);
  private final WPI_VictorSPX m_frontRightMotor = new WPI_VictorSPX(DriveConstants.k_FrontRightMotorPort);
  private final WPI_VictorSPX m_rearRightMotor = new WPI_VictorSPX(DriveConstants.k_RearRightMotorPort);
  
/** Encoders */
private final Encoder m_frontLeftEncoder =
new Encoder(
    DriveConstants.k_FrontLeftEncoderPorts[0],
    DriveConstants.k_FrontLeftEncoderPorts[1],
    DriveConstants.k_FrontLeftEncoderReversed);

// The rear-left-side drive encoder
private final Encoder m_rearLeftEncoder =
new Encoder(
    DriveConstants.k_RearLeftEncoderPorts[0],
    DriveConstants.k_RearLeftEncoderPorts[1],
    DriveConstants.k_RearLeftEncoderReversed);

// The front-right--side drive encoder
private final Encoder m_frontRightEncoder =
new Encoder(
    DriveConstants.k_FrontRightEncoderPorts[0],
    DriveConstants.k_FrontRightEncoderPorts[1],
    DriveConstants.k_FrontRightEncoderReversed);

// The rear-right-side drive encoder
private final Encoder m_rearRightEncoder =
new Encoder(
    DriveConstants.k_RearRightEncoderPorts[0],
    DriveConstants.k_RearRightEncoderPorts[1],
    DriveConstants.k_RearRightEncoderReversed);


  /** Creates a new PidDriveSubsystem. */
  public PidDriveSubsystem() {
    super(
        // The PIDController used by the subsystem
        new PIDController(DriveConstants.k_driveLinKp, DriveConstants.k_driveLinKi, DriveConstants.k_driveLinKd));
        // TODO: MOVE APRILTAG PID's HERE
  }

  @Override
  public void useOutput(double output, double setpoint) {
    // Use the output here
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return 0;
  }
}
