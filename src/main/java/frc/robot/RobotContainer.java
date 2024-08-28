// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.DriveSubsystemSparkMAX;
import frc.robot.subsystems.DriveSubsystemTalonFX;
import frc.robot.subsystems.DriveSubsystemTalonSRX;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  // FIXME - Uncomment the correct class for your purposes
  private final DriveSubsystemSparkMAX m_driveSubsystem = new DriveSubsystemSparkMAX();
  // private final DriveSubsystemTalonFX m_driveSubsystem = new DriveSubsystemTalonFX();
  // private final DriveSubsystemTalonSRX m_driveSubsystem = new DriveSubsystemTalonSRX();

  // Creates the Joystick controller to drive the robot
  Joystick m_joystick = new Joystick(0);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /** Use this method to define your trigger->command mappings. */
  private void configureBindings() {
    // Put any trigger->command mappings here.

    // FIXME - comment out the examples you don't want

    // Run motor when button is pressed
    new JoystickButton(m_joystick, 1)
      .whileTrue(m_driveSubsystem.runMotorCommand());

    // Run motor when button is pressed and sensor is active
    new JoystickButton(m_joystick, 1)
      .and(() -> m_driveSubsystem.isSensorActive())
      .whileTrue(m_driveSubsystem.runMotorCommand());

    // Run a motor at the speed of the joystick
    m_driveSubsystem.setDefaultCommand(
      Commands.run(
        () -> m_driveSubsystem.setRawMotorSpeed(m_joystick.getRawAxis(0)),
        m_driveSubsystem
      )
    );
  }

  
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return new Command() {};
  }
}
