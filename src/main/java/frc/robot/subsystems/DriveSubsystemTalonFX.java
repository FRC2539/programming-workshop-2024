// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;

public class DriveSubsystemTalonFX extends DriveSubsystem {
  /** Creates a new SparkMax brushless motor */
  private TalonFX m_leftLeaderMotor;
  private TalonFX m_leftFollowerMotor;
  private TalonFX m_rightLeaderMotor;
  private TalonFX m_rightFollowerMotor;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystemTalonFX() {
    // FIXME - Change to the name of your canbus if your motors are on the canbus
    String canbusName = "rio";

    m_leftLeaderMotor = new TalonFX(0, canbusName);
    m_leftFollowerMotor = new TalonFX(1, canbusName);
    m_rightLeaderMotor = new TalonFX(2, canbusName);
    m_rightFollowerMotor = new TalonFX(3, canbusName);

    // FIXME - Set the followers to follow the leader
    m_leftFollowerMotor.setControl(new Follower(0, false));
    m_rightFollowerMotor.setControl(new Follower(2, false));

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
