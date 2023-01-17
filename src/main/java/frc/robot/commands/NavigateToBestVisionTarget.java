// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.photonvision.PhotonCamera;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PhotonVisionSubsytem;

public class NavigateToBestVisionTarget extends CommandBase {  
  //Declare Variables 
  private final DriveSubsystem m_DriveSubsystem;
  private final PhotonVisionSubsytem m_PhotonVisionSubsytem;
  // Change this to match the name of your camera

  PhotonCamera cameraFront = new PhotonCamera("OV5647");
  PhotonCamera cameraRear = new PhotonCamera("OV5647");
  
  /**Constructor 
   * @param TestFalconSubsystem
   * @param Joystick_DS 
   */
  public NavigateToBestVisionTarget( DriveSubsystem DriveSys, PhotonVisionSubsytem PhotonVisionSys) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_DriveSubsystem = DriveSys;
    m_PhotonVisionSubsytem = PhotonVisionSys;
    addRequirements(m_DriveSubsystem, m_PhotonVisionSubsytem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //m_DriveSubsystem.initShuffleboard();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {


    /* FRONT CAMERA */
    var resultFront = cameraFront.getLatestResult();
    /*
    if (resultFront.hasTargets()){
      //Get range to target
      var range = m_PhotonVisionSubsytem.getRangeToTag(resultFront);
      forwardSpeed = -m_DriveSubsystem.controller.calculate(range, PhotonVisionConstants.GOAL_RANGE_METERS);
      //rotationSpeed = m_DriveSubsystem.turnController.calculate(result.getBestTarget().getYaw(), 0);
      strafeSpeed = -m_DriveSubsystem.controller.calculate(resultFront.getBestTarget().getYaw(), 0);
    } else {
      //rotationSpeed = 0;
      forwardSpeed = yControllerStick.getAsDouble();
      strafeSpeed = xControllerStick.getAsDouble();
    }
    */
    
    //Use values to drive robot
    m_DriveSubsystem.MecanumDrive(m_PhotonVisionSubsytem.getVisionStrafeSpeed(m_DriveSubsystem, resultFront)*0.5, m_PhotonVisionSubsytem.getVisionForwardSpeed(m_DriveSubsystem, resultFront)*0.4, 0);
    //SmartDashboard.putNumber("Rotation", rotationSpeed);
    //SmartDashboard.putNumber("Controller Z POS", zControllerStick.getAsDouble());

  }
    /* REAR CAMERA */ /* 
  var resultRear = cameraFront.getLatestResult();
  if (resultFront.hasTargets()){
    //Get range to target
    var range = m_PhotonVisionSubsytem.getRangeToTag(resultRear);
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
*/

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.  
  @Override
  public boolean isFinished() {
    return false;
  }
}