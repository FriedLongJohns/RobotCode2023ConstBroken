package org.carlmontrobotics.robotcode2023.commands;

import org.carlmontrobotics.robotcode2023.subsystems.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class SetArmWristPosition extends ConditionalCommand {

    public SetArmWristPosition(double armPos, double wristPos, boolean moveArmFirst, Arm arm) {
        super(
            new SequentialCommandGroup(
                new InstantCommand(moveArmFirst ? () -> arm.setArmTarget(armPos) : () -> arm.setWristTarget(wristPos)),
                new WaitUntilCommand(moveArmFirst ? arm::armAtSetpoint : arm::wristAtSetpoint),
                new InstantCommand(moveArmFirst ? () -> arm.setWristTarget(wristPos) : () -> arm.setArmTarget(armPos)),
                new WaitUntilCommand(moveArmFirst ? arm::wristAtSetpoint : arm::armAtSetpoint)
            ),
            new SequentialCommandGroup(
                new InstantCommand(() -> arm.setWristTarget(wristStowPos)),
                new WaitUntilCommand(arm::wristAtSetpoint),
                new InstantCommand(() -> arm.setArmTarget(-Math.PI / 2 + armSideTransferOffsetFromVertical * (armPos > -Math.PI / 2 ? 1 : -1))),
                new WaitUntilCommand(arm::armAtSetpoint),
                // Use a proxy command for 2 reasons,
                // 1) nesting this command inside itself could likely break something
                // 2) if we just use a constructor, this command would allocate itself in its own constructor, leading to infinite memory usage (now it's only allocated when it's needed)
                new ProxyCommand(() -> new SetArmWristPosition(armPos, wristPos, moveArmFirst, arm))
            ),
            () -> Math.signum(arm.getArmPos() - Math.PI / 2) == Math.signum(armPos - (-Math.PI / 2)) // If the arm is already on the right side of the robot
        );
        addRequirements(arm);
    }

}
