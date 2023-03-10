// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class TeleopCommand extends CommandBase {  
  //Declare Variables 
  private final DriveSubsystem m_DriveSubsystem;
  private DoubleSupplier xControllerStick;
  private DoubleSupplier yControllerStick;
  private DoubleSupplier zControllerStick;
  
  
  /**Constructor 
   * @param TestFalconSubsystem
   * @param Joystick_DS 
   */
  public TeleopCommand( DriveSubsystem DriveSys, DoubleSupplier xControllerStick, DoubleSupplier yControllerStick, DoubleSupplier zControllerStick) {
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
    m_DriveSubsystem.MecanumDrive(-xControllerStick.getAsDouble(), -yControllerStick.getAsDouble(), -zControllerStick.getAsDouble());
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