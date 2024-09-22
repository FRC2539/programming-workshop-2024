// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

// this would usually extend SubsystemBase, im just doing this so that the comands work with multiple subsystems
public class DriveSubsystemSparkMAX extends DriveSubsystem {
  /** Creates a new SparkMax brushless motor */
  private CANSparkMax m_leftLeaderMotor;
  private CANSparkMax m_leftFollowerMotor;
  private CANSparkMax m_rightLeaderMotor;
  private CANSparkMax m_rightFollowerMotor;

  // Motor Speeds
  private double m_leftSimSpeed = 0;
  private double m_rightSimSpeed = 0;

  private double m_leftSimPosition = 0;
  private double m_rightSimPosition = 0;

  private final DCMotor motor = DCMotor.getNEO(2);

  private final double drivebaseWidth = Units.inchesToMeters(24);

  private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(drivebaseWidth);

  private final DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(new Rotation2d(), 0, 0);

  private final AHRS m_gyro = new AHRS();

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

    m_gyro.reset();

    odometry.resetPosition(getGyro(), 0, 0, new Pose2d());
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
    if (Robot.isSimulation()) {
      m_leftSimSpeed = leftPercent;
      m_rightSimSpeed = rightPercent;
    }

    m_leftLeaderMotor.set(leftPercent);
    m_rightLeaderMotor.set(rightPercent);
  } 

  public void logInfo() {
    DogLog.log(
      "/DriveSubsystem/Wheels/LeftSpeedRPM", 
      getLeftSpeed()
    );

    DogLog.log(
      "/DriveSubsystem/Wheels/RightSpeedRPM", 
      getRightSpeed());
    
    DogLog.log(
      "/DriveSubsystem/Wheels/LeftPosition", 
      getLeftPosition());
    
    DogLog.log(
      "/DriveSubsystem/Wheels/RightPosition", 
      getRightPosition());

    DogLog.log(
      "/DriveSubsystem/Gyro", getGyro());
    
    DogLog.log(
      "/DriveSubsystem/Position",
      odometry.getPoseMeters()
    );
  }

  private double getLeftSpeed() {
    if (Robot.isSimulation()) {
      return m_leftSimSpeed;
    }
    return m_leftLeaderMotor.getEncoder().getVelocity();
  }

  private double getRightSpeed() {
    if (Robot.isSimulation()) {
      return m_rightSimSpeed;
    }
    return m_leftLeaderMotor.getEncoder().getVelocity();
  }

  private double getLeftPosition() {
    if (Robot.isSimulation()) {
      return m_leftSimPosition;
    }
    return m_leftLeaderMotor.getEncoder().getPosition();
  }

  private double getRightPosition() {
    if (Robot.isSimulation()) {
      return m_rightSimPosition;
    }
    return m_rightLeaderMotor.getEncoder().getPosition();
  }

  private Rotation2d getGyro() {
    if (Robot.isSimulation()) {
      return odometry.getPoseMeters().getRotation();
    }
    return m_gyro.getRotation2d();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    logInfo();
    odometry.update(getGyro(), getLeftPosition(), getRightPosition());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    m_leftSimPosition += Units.radiansToRotations(motor.getSpeed(0,m_leftSimSpeed) * 0.02);
    m_rightSimPosition += Units.radiansToRotations(motor.getSpeed(0, m_rightSimSpeed) * 0.02);

    double rotationAmount = kinematics.toChassisSpeeds(new DifferentialDriveWheelSpeeds(motor.getSpeed(0,m_leftSimSpeed) , motor.getSpeed(0, m_rightSimSpeed))).omegaRadiansPerSecond * 0.02;
    odometry.update(getGyro().plus(Rotation2d.fromRadians(rotationAmount)), getLeftPosition(), getRightPosition());
  }
}