// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystemSparkMAX extends SubsystemBase {
  /** Creates a new SparkMax brushless motor */
  private CANSparkMax m_leftLeaderMotor;
  private CANSparkMax m_leftFollowerMotor;
  private CANSparkMax m_rightLeaderMotor;
  private CANSparkMax m_rightFollowerMotor;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystemSparkMAX() {
    // FIXME - If the motor is brushless, use `MotorType.kBrushless`. If the motor is brushed, use `MotorType.kBrushed`
    MotorType motorType = MotorType.kBrushless;

    m_leftLeaderMotor = new CANSparkMax(0, motorType);
    m_leftFollowerMotor = new CANSparkMax(1, motorType);
    m_rightLeaderMotor = new CANSparkMax(2, motorType);
    m_rightFollowerMotor = new CANSparkMax(3, motorType);

    // FIXME - Set the followers to follow the leader
    m_leftFollowerMotor.follow(m_leftLeaderMotor, false);
    m_rightFollowerMotor.follow(m_rightLeaderMotor, false);

    // FIXME - Invert the motors as necessary
    m_leftLeaderMotor.setInverted(true);
    m_rightLeaderMotor.setInverted(true);
  }

  /** Creates a command which drives the left side based on the left joystick and the right side based on the right joystick. */
  public Command tankDriveCommand(DoubleSupplier leftAxis, DoubleSupplier rightAxis) {
    return run(() -> {
      setDriveSpeeds(leftAxis.getAsDouble(), rightAxis.getAsDouble());
    });
  }

  /** Creates a command which drives based on the drive axis with added turn based on the turn axis. */
  public Command arcadeDriveCommand(DoubleSupplier driveAxis, DoubleSupplier turnAxis) {
    return run(() -> {
      double leftPercent = driveAxis.getAsDouble() + turnAxis.getAsDouble();
      double rightPercent = driveAxis.getAsDouble() - turnAxis.getAsDouble();
      setDriveSpeeds(leftPercent, rightPercent);
    });
  }

  /** Directly set motor speeds */
  public void setDriveSpeeds(double leftPercent, double rightPercent) {
    m_leftLeaderMotor.set(leftPercent);
    m_rightLeaderMotor.set(rightPercent);
  } 

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
