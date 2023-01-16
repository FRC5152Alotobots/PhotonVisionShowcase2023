// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

  PhotonCamera cameraFront = new PhotonCamera("OV5647");
  PhotonCamera cameraRear = new PhotonCamera("OV5647");
  
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
    //m_DriveSubsystem.initShuffleboard();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double forwardSpeed;
    double rotationSpeed;
    double strafeSpeed;

    /* FRONT CAMERA */
    var resultFront = cameraFront.getLatestResult();
    if (resultFront.hasTargets()){
      //Get range to target
      var range = m_DriveSubsystem.getRangeToTag(resultFront);
      forwardSpeed = -m_DriveSubsystem.controller.calculate(range, PhotonVisionConstants.GOAL_RANGE_METERS);
      //rotationSpeed = m_DriveSubsystem.turnController.calculate(result.getBestTarget().getYaw(), 0);
      strafeSpeed = -m_DriveSubsystem.controller.calculate(resultFront.getBestTarget().getYaw(), 0);
    } else {
      //rotationSpeed = 0;
      forwardSpeed = yControllerStick.getAsDouble();
      strafeSpeed = xControllerStick.getAsDouble();
    }
    //Use values to drive robot
    m_DriveSubsystem.MecanumDrive(strafeSpeed*0.5, forwardSpeed*0.4, 0);
    //SmartDashboard.putNumber("Rotation", rotationSpeed);
    SmartDashboard.putNumber("Controller Z POS", zControllerStick.getAsDouble());
  
    /* REAR CAMERA */
  var resultRear = cameraFront.getLatestResult();
  if (resultFront.hasTargets()){
    //Get range to target
    var range = m_DriveSubsystem.getRangeToTag(resultRear);
    forwardSpeed = -m_DriveSubsystem.controller.calculate(range, PhotonVisionConstants.GOAL_RANGE_METERS);
    //rotationSpeed = m_DriveSubsystem.turnController.calculate(result.getBestTarget().getYaw(), 0);
    strafeSpeed = -m_DriveSubsystem.controller.calculate(resultRear.getBestTarget().getYaw(), 0);
  } else {
    //rotationSpeed = 0;
    forwardSpeed = yControllerStick.getAsDouble();
    strafeSpeed = xControllerStick.getAsDouble();
  }
  //Use values to drive robot
  m_DriveSubsystem.MecanumDrive(strafeSpeed*0.5, forwardSpeed*0.4, 0);
  //SmartDashboard.putNumber("Rotation", rotationSpeed);
  SmartDashboard.putNumber("Controller Z POS", zControllerStick.getAsDouble());
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