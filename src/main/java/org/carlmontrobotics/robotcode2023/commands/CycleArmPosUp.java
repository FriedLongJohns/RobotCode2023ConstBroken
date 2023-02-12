// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.robotcode2023.commands;

import org.carlmontrobotics.robotcode2023.Constants;
import org.carlmontrobotics.robotcode2023.subsystems.Arm;

import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class CycleArmPosUp extends CommandBase {
  Arm arm;
  /** Creates a new ArmCycle. */
  public CycleArmPosUp(Arm arm) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    arm.cycleUp();
    //hmm maybe the arm subsystem does a little TOO MUCH stuff
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (interrupted){
      arm.goalPos = arm.motorLencoder.getPosition();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished(){
    //If the arm goes too far then it should try to go back and this will wait until then
    return (arm.closeSnappedArmPos().value == arm.snappedArmPos().value);
  }
}
