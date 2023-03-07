// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.robotcode2023.subsystems;

import org.carlmontrobotics.lib199.MotorControllerFactory;
import org.carlmontrobotics.lib199.MotorErrors.TemperatureLimit;
import org.carlmontrobotics.robotcode2023.Constants;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

public class Wrist extends SubsystemBase {
  private CANSparkMax motorL = MotorControllerFactory.createSparkMax(Constants.wrist_motorL_port, TemperatureLimit.NEO);
  private CANSparkMax motorR = MotorControllerFactory.createSparkMax(Constants.wrist_motorR_port, TemperatureLimit.NEO);
  public RelativeEncoder motorLEncoder = motorL.getEncoder();
  public RelativeEncoder motorREncoder = motorR.getEncoder();
    
  public double encoderErrorTolerance = .05;
  public static double goalPos;
    
  public final double loClamp = .0;//TODO GET REAL VALUE
  public final double ecactb = .3;//TODO NICER VAR NAME
    //enable clamping when arm gets close to it's limits by this amount
    
  private static double kS = .2; //volts | base speed
  private static double kG = .1; //volts | gravity... something
  private static double kV = .2; //volts*secs/rad | extra velocity
  private static double kA = .3; //volts*secs^2/rad | vacceleration
  /// these are all units ^ , actual wrist speed is determined by values in .calculate
    
  private static double FFvelocity = 2;
  private static double FFaccel = 1;
  private ArmFeedforward wristFeed = new ArmFeedforward(kS,kG,kV,kA);
    
  private Arm arm;
    
  public enum WristPreset {
    INTAKE(0.31), MID(-1.74), HIGH(-1.83);
    
    public double value; //not static so SmartDashboard can touch [IMPORTANT TO KNOW!]
    WristPreset(double value) {
      this.value = value;
    }
    public WristPreset next() {
      switch (this) {
        case INTAKE: return MID;
        case MID: return HIGH;
        case HIGH: return INTAKE;
      }
      return null;
    }
    public WristPreset prev(){
      switch (this) {
        case INTAKE: return HIGH;
        case MID: return INTAKE;
        case HIGH: return MID;
      }
      return null;
    }
  }
  /** Creates a new Wrist. */
  public Wrist(Arm arm) {
    motorR.follow(motorL, true);
    motorLEncoder.setPositionConversionFactor(1/60);
    motorLEncoder.setPosition(0.0);
    SmartDashboard.putNumber("FF: Velocity", FFvelocity);
    SmartDashboard.putNumber("FF: Acceleration", FFaccel);
    SmartDashboard.putNumber("GoalPosition", goalPos);
    arm=arm;
  }

  @Override
  public void periodic() {
    FFvelocity = SmartDashboard.getNumber("FF: Velocity", FFvelocity);
    FFaccel = SmartDashboard.getNumber("FF: Acceleration", FFaccel);
    goalPos = SmartDashboard.getNumber("GoalPosition", goalPos);
    if (Math.abs(arm.encoder.getPosition() - arm.loClamp) < ecactb) {
        goalPos = MathUtil.clamp(goalPos, loClamp, 99999);//TODO USE ACTUAL VALUES
    }
    SmartDashboard.putNumber("ArmLEncoderPos", motorLEncoder.getPosition());

    double difference = goalPos - motorLEncoder.getPosition();
    if (difference > encoderErrorTolerance){//even PID needs an acceptable error sometimes
      //assuming calculate() is some sort of PID-esque thing
      motorL.set(wristFeed.calculate(difference, FFvelocity, FFaccel));
    }
  }
  public WristPreset snappedArmPos(){
    double encoderPos = motorLEncoder.getPosition();
    
    for(WristPreset check : WristPreset.values()){
      double lowdist = (check.value - check.prev().value) / 2;
      double hidist = (check.next().value - check.value) / 2; // get the halfway points between each position and it's neighbors
      if (check.value - lowdist < encoderPos && encoderPos < check.value + hidist){
        //seperate high and low instead of ABS because maybe difference isn't constant between each position of arm
          //and yes it still works for lowest and highest value
          return check;
      }
    }
    //help something went wrong
    return null;
  }

  public WristPreset closeSnappedArmPos() {//more precise snapping
    double encoderPos = motorLEncoder.getPosition();
    
    for(WristPreset check : WristPreset.values()){
        if (Math.abs(check.value - encoderPos) > encoderErrorTolerance) {//maybe will break if cone/cube values are close, but if they are close then lower error or only use one enum
            return check;
        }
    }
    //help something went wrong
    return null;
  }

  public void cycleUp(){ 
    // because most people won't remember/want to do this long function chain
    SmartDashboard.putNumber("GoalPosition", 
    closeSnappedArmPos() != null ? closeSnappedArmPos().next().value : snappedArmPos().next().value);
    // if closeSnappedArmPos is working, swap based on it - otherwise use less accurate snapping
  }
  
  public void cycleDown(){
    SmartDashboard.putNumber("GoalPosition", 
    closeSnappedArmPos() != null ? closeSnappedArmPos().prev().value : snappedArmPos().prev().value
    );
  }
  
  public void setPreset(WristPreset preset){
    SmartDashboard.putNumber("GoalPosition", preset.value);
  }
  }
  //testing something. Disregard this comment


