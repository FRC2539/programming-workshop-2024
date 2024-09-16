// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class DriveSubsystem extends SubsystemBase {
  /** Creates a command which drives the left side based on the left joystick and the right side based on the right joystick. */
  public abstract Command tankDriveCommand(DoubleSupplier leftAxis, DoubleSupplier rightAxis);

  /** Creates a command which drives based on the drive axis with added turn based on the turn axis. */
  public abstract Command arcadeDriveCommand(DoubleSupplier driveAxis, DoubleSupplier turnAxis);

  /** Directly set motor speeds */
  public abstract void setDriveSpeeds(double leftPercent, double rightPercent);
}
