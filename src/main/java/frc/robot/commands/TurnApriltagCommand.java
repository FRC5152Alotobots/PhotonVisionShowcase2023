// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class TurnApriltagCommand extends CommandBase {  
  //Declare Variables 
  private final DriveSubsystem m_DriveSubsystem;
  private DoubleSupplier xControllerStick;
  private DoubleSupplier yControllerStick;
  private DoubleSupplier zControllerStick;
  // Change this to match the name of your camera

  PhotonCamera camera = new PhotonCamera("OV5647");

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
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var result = camera.getLatestResult();
    if (result.hasTargets()){
    m_DriveSubsystem.MecanumDrive(xControllerStick.getAsDouble(), yControllerStick.getAsDouble(), -result.getBestTarget().getYaw());
    } else {
      m_DriveSubsystem.MecanumDrive(xControllerStick.getAsDouble(), yControllerStick.getAsDouble(), zControllerStick.getAsDouble());
    }
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