package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.DriveSubsystem;

public class AimAtLimelightCommand extends Command {
    private DriveSubsystem m_driveSubsystem;
    private PIDController m_pidController;
    private final String limelightName = "limelight";

    public AimAtLimelightCommand(DriveSubsystem driveSubsystem) {
        m_driveSubsystem = driveSubsystem;

        m_pidController = new PIDController(0.1, 0, 0);
        m_pidController.setTolerance(2);
        addRequirements(m_driveSubsystem);
    }

    @Override
    public void initialize() {
        m_pidController.reset();
    }

    @Override
    public void execute() {
        
        // If we dont see the target, stop the motors.
        if (LimelightHelpers.getFiducialID(limelightName) == -1) {
            m_driveSubsystem.setDriveSpeeds(0, 0);
        } else {
            // Calculate the power to send to the motors.
            double response = m_pidController.calculate(LimelightHelpers.getTX(limelightName));
            m_driveSubsystem.setDriveSpeeds(response, -response);
        }
        
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the motors.
        m_driveSubsystem.setDriveSpeeds(0, 0);
    }

    @Override
    public boolean isFinished() {
        // Return true when the robot is within tolerance of the target.
        return m_pidController.atSetpoint();
    }
}
