package org.carlmontrobotics.robotcode2023.commands;

import static org.carlmontrobotics.robotcode2023.Constants.Drivetrain.autoMaxAccelMps2;
import static org.carlmontrobotics.robotcode2023.Constants.Drivetrain.maxSpeed;

import java.util.HashMap;

import org.carlmontrobotics.lib199.Limelight;
import org.carlmontrobotics.lib199.path.PPRobotPath;
import org.carlmontrobotics.robotcode2023.subsystems.Drivetrain;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;

public class DriveToPoint extends ProxyCommand {

    private final Drivetrain drivetrain;

    public DriveToPoint(Pose2d targetPose, Drivetrain drivetrain) {
        super(() ->
            new PPRobotPath(
                PathPlanner.generatePath(
                    new PathConstraints(maxSpeed, autoMaxAccelMps2),
                    PathPoint.fromCurrentHolonomicState(
                        drivetrain.getOdometry().getPoseMeters(),
                        drivetrain.getSpeeds()
                    ),
                    PathPoint.fromCurrentHolonomicState(
                        targetPose,
                        null
                    )
                ),
                drivetrain,
                new HashMap<>()
            ).getPathCommand(true, true)
        );
        addRequirements(this.drivetrain = drivetrain);
    }

    public Command realignAt(Pose2d alignPoint, Limelight lime) {
        return new DriveToPoint(alignPoint, drivetrain)
            .andThen(new InstantCommand(() -> drivetrain.updateOdometryFromLimelight(lime)))
            .andThen(this);
    }
}
