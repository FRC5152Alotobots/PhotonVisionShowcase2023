// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class TurnApriltagCommand extends CommandBase {  
  //Declare Variables 
  private final DriveSubsystem m_DriveSubsystem;
  private DoubleSupplier joyValue;

  /**Constructor 
   * @param TestFalconSubsystem
   * @param Joystick_DS 
   */
  public TurnApriltagCommand( DriveSubsystem DriveSys, DoubleSupplier joyValue) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_DriveSubsystem = DriveSys;
    addRequirements(m_DriveSubsystem);
    this.joyValue = joyValue;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_.setPrecentOutput(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_FalconSys.setPrecentOutput(joyValue.getAsDouble());
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