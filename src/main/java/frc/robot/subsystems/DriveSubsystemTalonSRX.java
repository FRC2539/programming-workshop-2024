// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.Command;

public class DriveSubsystemTalonSRX extends DriveSubsystem {
  /** Creates a new SparkMax brushless motor */
  private TalonSRX m_leftLeaderMotor;
  private TalonSRX m_leftFollowerMotor;
  private TalonSRX m_rightLeaderMotor;
  private TalonSRX m_rightFollowerMotor;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystemTalonSRX() {
    m_leftLeaderMotor = new TalonSRX(0);
    m_leftFollowerMotor = new TalonSRX(1);
    m_rightLeaderMotor = new TalonSRX(2);
    m_rightFollowerMotor = new TalonSRX(3);

    // Set the followers to follow the leader
    m_leftFollowerMotor.follow(m_leftLeaderMotor);
    m_rightFollowerMotor.follow(m_rightLeaderMotor);

    // FIXME - Invert the motors as necessary
    m_leftLeaderMotor.setInverted(true);
    m_rightLeaderMotor.setInverted(true);
    m_leftFollowerMotor.setInverted(true);
    m_rightFollowerMotor.setInverted(true);
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
    m_leftLeaderMotor.set(TalonSRXControlMode.PercentOutput, leftPercent);
    m_rightLeaderMotor.set(TalonSRXControlMode.PercentOutput, rightPercent);
  } 

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
