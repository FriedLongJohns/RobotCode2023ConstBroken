package org.carlmontrobotics.robotcode2023.commands;

import org.carlmontrobotics.robotcode2023.subsystems.Drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj2.command.CommandBase;
import static org.carlmontrobotics.robotcode2023.Constants.Drivetrain.thetaPIDController;

public class RotateToFieldRelativeAngle extends CommandBase {

    public static final double THRESHOLD = 4; // degrees

    public final Rotation2d targetAngle;
    public final TeleopDrive teleopDrive;
    public final Drivetrain drivetrain;

    public static final PIDController rotationPID = new PIDController(thetaPIDController[0], thetaPIDController[1], thetaPIDController[2]);

    public RotateToFieldRelativeAngle(Rotation2d angle, Drivetrain drivetrain) {
        this.targetAngle = angle;
        this.drivetrain = drivetrain;
        this.teleopDrive = (TeleopDrive) drivetrain.getDefaultCommand();

        rotationPID.enableContinuousInput(-180, 180);
        rotationPID.setSetpoint(angle.getDegrees());
        rotationPID.setTolerance(THRESHOLD);
        SendableRegistry.addChild(this, rotationPID);

        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        drivetrain.drive(teleopDrive.currentForward, teleopDrive.currentStrafe, rotationPID.calculate(drivetrain.getHeading(), targetAngle.getDegrees()));
    }
    
    @Override
    public boolean isFinished() {
        return rotationPID.atSetpoint();
    }
}
