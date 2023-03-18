package org.carlmontrobotics.robotcode2023.commands;

import static org.carlmontrobotics.robotcode2023.Constants.GoalPos;
import static org.carlmontrobotics.robotcode2023.Constants.Arm.wristStowPos;

import org.carlmontrobotics.robotcode2023.subsystems.Arm;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class SetArmWristPosition extends SequentialCommandGroup {

    public SetArmWristPosition(double armPos, double wristPos, Arm arm) {
        super();
        // TODO: Test it!
        //if arm is lower than MID when moving and if goal is lower than this by ANY AMOUNT, then fold the wrist first
        //need separate flipped case for front too since values are flipped when arm passes through bellypan
        if (
                ( arm.isFront && (arm.getArmPos() < arm.getArmGoal(GoalPos.MID)) && (arm.getArmPos() > armPos)) ||
                (!arm.isFront && (arm.getArmPos() > arm.getArmGoal(GoalPos.MID)) && (arm.getArmPos() < armPos))
             ){
            //but have special case where arm is in front and wrist would collide, so we have to move arm pos to mid before folding
            if (arm.isFront){//TODO MAKE SURE THIS LINE IS CORRECT
                super.addCommands(
                    new InstantCommand(() -> arm.setArmTarget(GoalPos.MID[0].armPos)),
                    new WaitUntilCommand(arm.armPID::atSetpoint)
                );
            }
            super.addCommands(
                new InstantCommand(() -> arm.setWristTarget(wristStowPos)),
                new WaitUntilCommand(arm.wristPID::atSetpoint),//always stow first, avoid collision
                new InstantCommand(() -> arm.setArmTarget(armPos)),
                new WaitUntilCommand(arm.armPID::atSetpoint),
                new InstantCommand(() -> arm.setWristTarget(wristPos)),//assume wristPos isn't gonna collide with drivetrain because that'd be a stupid thing for people to do and I don't wanna add more if statements
                new WaitUntilCommand(arm.wristPID::atSetpoint)
            );
        }else{
            super.addCommands(//no collision? move both at once for moar speed
                new InstantCommand(() -> arm.setArmTarget(armPos)),
                new InstantCommand(() -> arm.setWristTarget(wristPos)),
                new WaitUntilCommand(() -> {return (arm.wristPID.atSetpoint() && arm.armPID.atSetpoint());})
            );
        }
        addRequirements(arm);
    }

}
