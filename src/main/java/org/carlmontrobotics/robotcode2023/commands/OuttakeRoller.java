package org.carlmontrobotics.robotcode2023.commands;

import java.awt.Color;
import java.util.function.DoubleSupplier;

import org.carlmontrobotics.robotcode2023.subsystems.Roller;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class OuttakeRoller extends CommandBase {

    private final Roller roller;
    private DoubleSupplier speed;
    private final Color ledColor;
    private final Timer timer = new Timer();

    public OuttakeRoller(Roller roller, DoubleSupplier speed, Color ledColor) {
        addRequirements(this.roller = roller);
        this.speed = speed;
        this.ledColor = ledColor;
    }

    @Override
    public void initialize() {
        roller.setSpeed(speed.getAsDouble());
        roller.setLedColor(ledColor);
        timer.reset();
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
        double time = timer.get();
        //SmartDashboard.putNumber("Time Elapsed", time);
        //SmartDashboard.putNumber("Time Target", roller.getTime());

        if (!roller.hasGamePiece())
            timer.start();
        return !roller.hasGamePiece() && time > roller.getTime();
    }
}
