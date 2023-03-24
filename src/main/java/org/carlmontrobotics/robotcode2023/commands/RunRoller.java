package org.carlmontrobotics.robotcode2023.commands;

import org.carlmontrobotics.robotcode2023.Constants.Roller.GameObject;
import org.carlmontrobotics.robotcode2023.Constants.Roller.RollerMode;
import org.carlmontrobotics.robotcode2023.subsystems.Roller;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class RunRoller extends CommandBase {

    private final Roller roller;
    private final Timer timer = new Timer();
    private final RollerMode mode;

    public RunRoller(Roller roller, RollerMode mode) {
        addRequirements(this.roller = roller);
        this.mode = mode;
    }

    @Override
    public void initialize() {
        if(roller.hasGamePiece() && mode.obj != GameObject.NONE) cancel();
        if(!roller.hasGamePiece() && mode.obj == GameObject.NONE) cancel();
        timer.reset();
        roller.setRollerMode(mode);
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {
        roller.setSpeed(0);
        timer.stop();
    }

    @Override
    public boolean isFinished() {
        
        double time = timer.get();

        // TODO: distance sensor detects belt when wrist is spinning (concern)
        if (roller.getGamePiece() == mode.obj) {
            timer.start();
        }
        SmartDashboard.putString("Target Piece", mode.obj.toString());
        SmartDashboard.putNumber("Time Target", mode.time);
        SmartDashboard.putNumber("SetRoller Time Elapsed (s)", time);

        return roller.getGamePiece() == mode.obj && time > mode.time;
    }
}
