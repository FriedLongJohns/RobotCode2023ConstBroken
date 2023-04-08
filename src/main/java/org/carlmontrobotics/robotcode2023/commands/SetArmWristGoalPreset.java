package org.carlmontrobotics.robotcode2023.commands;

import java.util.function.BooleanSupplier;

import org.carlmontrobotics.robotcode2023.Constants.GoalPos;
import org.carlmontrobotics.robotcode2023.subsystems.Arm;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.ProxyCommand;

public class SetArmWristGoalPreset extends ProxyCommand {

    public SetArmWristGoalPreset(GoalPos[][] preset, BooleanSupplier isCube, BooleanSupplier isFront, Arm arm) {
        super(() -> {
            GoalPos pos = preset[isFront.getAsBoolean() ? 0 : 1][isCube.getAsBoolean() ? 0 : 1];
            return DriverStation.isAutonomous() ? new SetArmWristPositionV4(pos.armPos, pos.wristPos, arm) : new SetArmWristPositionV3(pos.armPos, pos.wristPos, arm);
        });
    }
}
