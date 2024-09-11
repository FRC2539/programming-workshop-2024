// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.DriveSubsystemSparkMAX;
import frc.robot.subsystems.DriveSubsystemTalonFX;
import frc.robot.subsystems.DriveSubsystemTalonSRX;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

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

    // FIXME - Uncomment the correct class for your purposes

    // Initialize left and right joysticks
    Joystick leftJoystick = new Joystick(0);
    Joystick rightJoystick = new Joystick(1);

    // Set up DoubleSuppliers for left and right joystick axes
    DoubleSupplier leftAxis = () -> leftJoystick.getRawAxis(0);
    DoubleSupplier rightAxis = () -> rightJoystick.getRawAxis(0);

    // Create a tank drive command based on the left and right axes
    Command tankDriveCommand = m_driveSubsystem.tankDriveCommand(leftAxis, rightAxis);

    // Set the tank drive command as the default command for the drive subsystem
    m_driveSubsystem.setDefaultCommand(tankDriveCommand);

    // Set up DoubleSuppliers for drive and turn axes
    DoubleSupplier driveAxis = () -> leftJoystick.getRawAxis(0);
    DoubleSupplier turnAxis = () -> -rightJoystick.getRawAxis(1);

    // Create an arcade drive command based on the drive and turn axes
    Command arcadeDriveCommand = m_driveSubsystem.arcadeDriveCommand(driveAxis, turnAxis);

    // Create a toggle switch to use Arcade Drive
    Trigger useArcadeDriveButton = new JoystickButton(leftJoystick, 1);
    useArcadeDriveButton.toggleOnTrue(arcadeDriveCommand);

    // Create a button to run a quickturn
    Trigger quickTurn = new JoystickButton(leftJoystick, 2);
    quickTurn.whileTrue(
      // Run the quickturn with a speed of 0.5 for the left and -0.5 on the right
      m_driveSubsystem.tankDriveCommand(() -> 0.5, () -> -0.5).withTimeout(1) // (but timeout after 1 second)
    );

  }

  
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return new Command() {};
  }
}
