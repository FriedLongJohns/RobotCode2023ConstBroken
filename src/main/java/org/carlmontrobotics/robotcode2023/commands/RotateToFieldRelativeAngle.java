package org.carlmontrobotics.robotcode2023.commands;

import org.carlmontrobotics.robotcode2023.subsystems.Drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import static org.carlmontrobotics.robotcode2023.Constants.Drivetrain.thetaPIDController;

import org.carlmontrobotics.robotcode2023.RobotContainer;

public class RotateToFieldRelativeAngle extends CommandBase {

    public static final int THRESHOLD = 2;

    public final Rotation2d angle;
    public final TeleopDrive teleopDrive;
    public final Drivetrain drivetrain;
    public final PIDController rotationPID = new PIDController(SmartDashboard.getNumber("pp", 0), SmartDashboard.getNumber("ii", 0), SmartDashboard.getNumber("dd", 0));

    public RotateToFieldRelativeAngle(Rotation2d angle, Drivetrain drivetrain) {
        SmartDashboard.putNumber("pp", thetaPIDController[0]);
        SmartDashboard.putNumber("ii", thetaPIDController[1]);
        SmartDashboard.putNumber("dd", thetaPIDController[2]);
        SmartDashboard.putNumber("add", 0);


        this.angle = angle;
        this.drivetrain = drivetrain;
        this.teleopDrive = (TeleopDrive) drivetrain.getDefaultCommand();
        this.rotationPID.enableContinuousInput(-180, 180);

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        rotationPID.setP(SmartDashboard.getNumber("pp", 0));
    }

    @Override
    public void execute() {
        drivetrain.drive(teleopDrive.currentForward, teleopDrive.currentStrafe, rotationPID.calculate(drivetrain.getHeading(), angle.getDegrees()) + SmartDashboard.getNumber("add", 0));
    }
    
    @Override
    public boolean isFinished() {
        return Math.abs(angle.getDegrees() - drivetrain.getHeading()) < THRESHOLD;
    }

    @Override
    public void end(boolean interrupted) {
        
    }

}
