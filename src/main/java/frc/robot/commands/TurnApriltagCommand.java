// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.PhotonVisionConstants;
import frc.robot.subsystems.DriveSubsystem;

public class TurnApriltagCommand extends CommandBase {  
  //Declare Variables 
  private final DriveSubsystem m_DriveSubsystem;
  private DoubleSupplier xControllerStick;
  private DoubleSupplier yControllerStick;
  private DoubleSupplier zControllerStick;
  // Change this to match the name of your camera

  PhotonCamera camera = new PhotonCamera("OV5647");
  PIDController controller = new PIDController(DriveConstants.k_driveLinKp, DriveConstants.k_driveLinKi, DriveConstants.k_driveLinKd);
  PIDController turnController = new PIDController(DriveConstants.k_driveAngKp, DriveConstants.k_driveAngKi, DriveConstants.k_driveAngKd);
  
  /**Constructor 
   * @param TestFalconSubsystem
   * @param Joystick_DS 
   */
  public TurnApriltagCommand( DriveSubsystem DriveSys, DoubleSupplier xControllerStick, DoubleSupplier yControllerStick, DoubleSupplier zControllerStick) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_DriveSubsystem = DriveSys;
    addRequirements(m_DriveSubsystem);
    this.xControllerStick = xControllerStick;
    this.yControllerStick = yControllerStick;
    this.zControllerStick = zControllerStick;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_DriveSubsystem.initShuffleboard();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double forwardSpeed;
    double rotationSpeed;

    var result = camera.getLatestResult();
    if (result.hasTargets()){
      double range =
      PhotonUtils.calculateDistanceToTargetMeters(
              PhotonVisionConstants.CAMERA_HEIGHT_METERS,
              PhotonVisionConstants.TARGET_HEIGHT_METERS,
              PhotonVisionConstants.CAMERA_PITCH_RADIANS,
              Units.degreesToRadians(result.getBestTarget().getPitch()));

              forwardSpeed = -controller.calculate(range, PhotonVisionConstants.GOAL_RANGE_METERS);
              rotationSpeed = -turnController.calculate(result.getBestTarget().getYaw(), 0);
    } else {
      rotationSpeed = 0;
      forwardSpeed = 0;
    }
    //Use values to drive robot
    m_DriveSubsystem.MecanumDrive(0, forwardSpeed, rotationSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}