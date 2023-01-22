package org.carlmontrobotics.robotcode2023.commands;

import org.carlmontrobotics.robotcode2023.subsystems.Drivetrain;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AlignChargingStation extends CommandBase {

    private final Drivetrain drivetrain;
    private double tolerance = 2.5, speed = 0.3 / 13, time = 500, lastTime = -1, ff = 0.1;
    private boolean fwd, fieldOriented;

    public AlignChargingStation(Drivetrain drivetrain) {
        addRequirements(this.drivetrain = drivetrain);
    }

    @Override
    public void initialize() {
        fwd = Math.abs(getPitch()) > Math.abs(getRoll());
        fieldOriented = SmartDashboard.getBoolean("Field Oriented", true);
        SmartDashboard.putBoolean("Field Oriented", false);
    }

    @Override
    public void execute() {
        double pitch = getPitch();
        double roll = getRoll();
        double forward = fwd && Math.abs(pitch) > tolerance ? speed * pitch + ff : 0;
        double strafe = !fwd && Math.abs(roll) > tolerance ? speed * roll + ff : 0;
        drivetrain.drive(-forward, -strafe, 0);
    }

    @Override
    public boolean isFinished() {
        lastTime = (fwd ? Math.abs(getPitch()) < tolerance : Math.abs(getRoll()) < tolerance) ? lastTime == -1 ? System.currentTimeMillis() : lastTime : -1;
        return System.currentTimeMillis() - lastTime > time && lastTime != -1;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
        SmartDashboard.putBoolean("Field Oriented", fieldOriented);
    }

    private double getPitch() {
        return drivetrain.getPitch() - drivetrain.initPitch;
    }

    private double getRoll() {
        return drivetrain.getRoll() - drivetrain.initRoll;
    }

}
