package org.carlmontrobotics.robotcode2023.commands;

import org.carlmontrobotics.robotcode2023.subsystems.Roller;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class HoldRoller extends CommandBase {

    private final Roller roller;
    private double pos;

    public HoldRoller(Roller roller) {
        addRequirements(this.roller = roller);
    }

    @Override
    public void initialize() {
        pos = roller.getPosition();
    }

    @Override
    public void execute() {
        double currentPos = roller.getPosition();
        roller.setSpeed(Math.abs(currentPos - pos) > 1 ? Math.copySign(0.3, -(currentPos - pos)) : 0);
    }

}
