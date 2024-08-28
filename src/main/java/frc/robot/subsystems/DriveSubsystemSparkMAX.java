// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystemSparkMAX extends SubsystemBase {
  /** Creates a new Digital Sensor */
  DigitalInput m_proximitySensor;

  /** Creates a new SparkMax brushless motor */
  CANSparkMax m_motor;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystemSparkMAX() {
    m_proximitySensor = new DigitalInput(0);
    // FIXME - If the motor is brushless, use `MotorType.kBrushless`. If the motor is brushed, use `MotorType.kBrushed`
    MotorType motorType = MotorType.kBrushless;
    m_motor = new CANSparkMax(0, motorType);
  }

  /** Run motor at half speed during command */
  public Command runMotorCommand() {
    return runEnd(
      () -> m_motor.set(0.5), 
      () -> m_motor.stopMotor()
    );
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public boolean isSensorActive() {
    return m_proximitySensor.get();
  }

  public void setRawMotorSpeed(double speed) {
    m_motor.set(speed);
  }
}
