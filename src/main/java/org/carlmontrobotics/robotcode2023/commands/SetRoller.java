package org.carlmontrobotics.robotcode2023.commands;

import java.awt.Color;

import org.carlmontrobotics.robotcode2023.subsystems.Roller;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SetRoller extends CommandBase {

    private final Roller roller;
    private final double speed;
    private final Color ledColor;
    private final Timer timer = new Timer();

    public SetRoller(Roller roller, double speed, Color ledColor) {
        addRequirements(this.roller = roller);
        this.speed = speed;
        this.ledColor = ledColor;
    }

    @Override
    public void initialize() {
        roller.setSpeed(speed);
        roller.setLedColor(ledColor);
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        roller.setSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return roller.hasGamePiece() && timer.get() > .5;
    }

}
