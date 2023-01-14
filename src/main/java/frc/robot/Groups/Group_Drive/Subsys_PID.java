// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Groups.Group_Drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Groups.Group_Drive.DriveConstants;

public class Subsys_PID extends PIDSubsystem {
  /** Creates a new Subsys_PID. */
  public Subsys_PID() {
    super(
        // The PIDController used by the subsystem
        new PIDController(DriveConstants.k_PID_Kp, DriveConstants.k_PID_Ki, DriveConstants.k_PID_Kd)
    );
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
