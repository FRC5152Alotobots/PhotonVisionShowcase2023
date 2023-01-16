// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Groups.Group_Drive.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class Cmd_PhotonDrive extends CommandBase {
  /** Creates a new Cmd_PhotonDrive. */
  private final Subsys_Drive Subsys_Drive;
  public Cmd_PhotonDrive(Subsys_Drive Subsys_Drive) {
      this.Subsys_Drive = Subsys_Drive;
      addRequirements(Subsys_Drive);
    // Use addRequirements() here to decare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
