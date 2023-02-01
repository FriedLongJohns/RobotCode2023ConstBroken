// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.robotcode2023.commands;

import org.carlmontrobotics.robotcode2023.Constants;
import org.carlmontrobotics.robotcode2023.subsystems.Arm;

import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ArmCycle extends CommandBase {
  Arm arm;
  double goalPos;
  /** Creates a new ArmCycle. */
  public ArmCycle(Arm arm) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //if motor is on the lowest level it will go up and if its on the highest it go down
    arm.motorL.set(1);

    Double currentPos = arm.motorLencoder.getPosition();

    if(currentPos>0.4){
      arm.motorL.set(-1);
    }
    //The Motor will stop when reaching goal position
    String armState = arm.armPosition();
    if(armState.equals("Start")){
      goalPos = SmartDashboard.getNumber("EncoderMidPosition",currentPos);
    } else if(armState.equals("Mid")){
      goalPos = SmartDashboard.getNumber("EncoderHighPosition", currentPos);
    } else if(armState.equals("High")){
      goalPos = SmartDashboard.getNumber("EncoderStartPosition", currentPos);
    } else{
      currentPos = goalPos;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished(){
    //Checks for erros in positioning 
    Double currentPos = arm.motorLencoder.getPosition();
    
    if (currentPos<goalPos && goalPos<currentPos){
      return true;
    }
    return false;
    
  }
}
