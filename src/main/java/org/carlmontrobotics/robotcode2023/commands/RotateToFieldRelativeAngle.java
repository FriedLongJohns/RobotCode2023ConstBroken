package org.carlmontrobotics.robotcode2023.commands;

import org.carlmontrobotics.robotcode2023.subsystems.Drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj2.command.CommandBase;
import static org.carlmontrobotics.robotcode2023.Constants.Drivetrain.*;

public class RotateToFieldRelativeAngle extends CommandBase {

    public final TeleopDrive teleopDrive;
    public final Drivetrain drivetrain;

    public final PIDController rotationPID = new PIDController(thetaPIDController[0], thetaPIDController[1], thetaPIDController[2]);

    public RotateToFieldRelativeAngle(Rotation2d angle, Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        this.teleopDrive = (TeleopDrive) drivetrain.getDefaultCommand();

        rotationPID.enableContinuousInput(-180, 180);
        rotationPID.setSetpoint(angle.getDegrees());
        rotationPID.setTolerance(positionTolerance[2], velocityTolerance[2]);
        SendableRegistry.addChild(this, rotationPID);

        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        double[] driverRequestedSpeeds = teleopDrive.getRequestedSpeeds();
        drivetrain.drive(driverRequestedSpeeds[0], driverRequestedSpeeds[1], rotationPID.calculate(drivetrain.getHeading()));
    }

    @Override
    public boolean isFinished() {
        return rotationPID.atSetpoint();
    }
}
