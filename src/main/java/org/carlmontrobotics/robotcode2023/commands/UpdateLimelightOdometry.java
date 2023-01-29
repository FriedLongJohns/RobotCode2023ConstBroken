package org.carlmontrobotics.robotcode2023.commands;

import static org.carlmontrobotics.robotcode2023.Constants.Drivetrain.minLimelightOdometryUpdateSpeed;

import org.carlmontrobotics.lib199.Limelight;
import org.carlmontrobotics.robotcode2023.subsystems.Drivetrain;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class UpdateLimelightOdometry extends ConditionalCommand{

    public UpdateLimelightOdometry(Drivetrain drivetrain, Limelight lime) {
        super(
            new InstantCommand(() -> drivetrain.updateOdometryFromLimelight(lime)),
            new InstantCommand(),
            () -> {
                ChassisSpeeds robotSpeeds = drivetrain.getSpeeds();
                double robotSpeed = Math.hypot(robotSpeeds.vxMetersPerSecond, robotSpeeds.vyMetersPerSecond);
                return robotSpeed < minLimelightOdometryUpdateSpeed &&
                    NetworkTableInstance.getDefault().getTable(lime.config.ntName).getEntry("tv").getDouble(0) == 1;
            }
        );
    }

}
