package org.carlmontrobotics.robotcode2023.commands;

import static org.carlmontrobotics.robotcode2023.Constants.Drivetrain.*;

import org.carlmontrobotics.robotcode2023.subsystems.Drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class AlignChargingStation extends CommandBase {

    private final Drivetrain drivetrain;
    private double lastTime = -1;
    private boolean fwd, fieldOriented;

    public AlignChargingStation(Drivetrain drivetrain) {
        addRequirements(this.drivetrain = drivetrain);
    }

    @Override
    public void initialize() {
        fwd = Math.abs(getPitch()) > Math.abs(getRoll());
        fieldOriented = drivetrain.getFieldOriented();
        drivetrain.setFieldOriented(false);
    }

    @Override
    public void execute() {
        double pitch = getPitch();
        double roll = getRoll();
        double forward = fwd && Math.abs(pitch) > chargeStationAlignTolerance ? chargeStationAlignSpeed * pitch + chargeStationAlignFF : 0;
        double strafe = !fwd && Math.abs(roll) > chargeStationAlignTolerance ? chargeStationAlignSpeed * roll + chargeStationAlignFF : 0;
        drivetrain.drive(-forward, -strafe, 0);
    }

    @Override
    public boolean isFinished() {
        lastTime = (fwd ? Math.abs(getPitch()) < chargeStationAlignTolerance : Math.abs(getRoll()) < chargeStationAlignTolerance) ? lastTime == -1 ? System.currentTimeMillis() : lastTime : -1;
        return System.currentTimeMillis() - lastTime > chargeStationAlignTime && lastTime != -1;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
        drivetrain.setFieldOriented(fieldOriented);
    }

    private double getPitch() {
        return drivetrain.getPitch() - drivetrain.initPitch;
    }

    private double getRoll() {
        return drivetrain.getRoll() - drivetrain.initRoll;
    }

}
