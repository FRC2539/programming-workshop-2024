// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  /** Creates a new SparkMax brushless motor */
  CANSparkMax motor = new CANSparkMax(0, CANSparkMax.MotorType.kBrushless);
  
  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {}

  /** Run motor at half speed during command */
  public Command runMotorCommand() {
    return runEnd(
        () -> {
          motor.set(0.5);
        }, () -> {
          motor.set(0);
        });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
