// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Groups.Group_Drive;

import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.kinematics.MecanumDriveMotorVoltages;
// import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
// import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
// import edu.wpi.first.wpilibj.ADXRS450_Gyro;
// import edu.wpi.first.wpilibj.Encoder;
// import edu.wpi.first.wpilibj.drive.MecanumDrive;
import frc.robot.Groups.Group_Drive.DriveConstants;
// import edu.wpi.first.wpilibj.interfaces.Gyro;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
public class Subsys_Drive extends SubsystemBase {
  /** Creates a new Subsys_MecanumDrive. */
  private final WPI_VictorSPX m_frontLeftMotor = new WPI_VictorSPX(DriveConstants.k_FrontLeftMotorPort);
  private final WPI_VictorSPX m_rearLeftMotor = new WPI_VictorSPX(DriveConstants.k_RearLeftMotorPort);
  private final WPI_VictorSPX m_frontRightMotor = new WPI_VictorSPX(DriveConstants.k_FrontRightMotorPort);
  private final WPI_VictorSPX m_rearRightMotor = new WPI_VictorSPX(DriveConstants.k_RearRightMotorPort);
  
  private final MecanumDrive m_MecanumDrive = new MecanumDrive(
    m_frontLeftMotor, 
    m_rearLeftMotor,
    m_frontRightMotor, 
    m_rearRightMotor
  );

  public Subsys_Drive() {
    m_frontLeftMotor.setInverted(false);
    m_rearLeftMotor.setInverted(false);
    m_frontRightMotor.setInverted(false);
    m_rearRightMotor.setInverted(false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void MecanumDrive(double xSpeed, double ySpeed, double rot) {
      m_MecanumDrive.driveCartesian(ySpeed, xSpeed, rot);
  }
}
