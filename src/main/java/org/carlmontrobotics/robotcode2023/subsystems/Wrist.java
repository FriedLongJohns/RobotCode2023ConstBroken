// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.robotcode2023.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.carlmontrobotics.lib199.MotorControllerFactory;
import org.carlmontrobotics.lib199.MotorErrors.TemperatureLimit;
import org.carlmontrobotics.robotcode2023.Constants;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.ArmFeedforward;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

public class Wrist extends SubsystemBase {
  private CANSparkMax motorL = MotorControllerFactory.createSparkMax(Constants.wrist_motorL_port, TemperatureLimit.NEO);
  private CANSparkMax motorR = MotorControllerFactory.createSparkMax(Constants.wrist_motorR_port, TemperatureLimit.NEO);
  public RelativeEncoder motorLEncoder = motorL.getEncoder();
  public RelativeEncoder motorREncoder = motorR.getEncoder();
  public double encoderErrorTolerance = .05;
  public static double goalPos;
  private static double kS = .2; //volts | base speed
  private static double kG = .1; //volts | gravity... something
  private static double kV = .2; //volts*secs/rad | extra velocity
  private static double kA = .3; //volts*secs^2/rad | vacceleration
  /// these are all units ^ , actual wrist speed is determined by values in .calculate
  private static double FFvelocity = 2;
  private static double FFaccel = 1;
  private ArmFeedforward wristFeed = new ArmFeedforward(kS,kG,kV,kA);
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
  public Wrist() {
    motorR.follow(motorL, true);
    motorLEncoder.setPositionConversionFactor(1/60);
    motorLEncoder.setPosition(0.0);
    SmartDashboard.putNumber("FF: Velocity", FFvelocity);
    SmartDashboard.putNumber("FF: Acceleration", FFaccel);
    SmartDashboard.putNumber("GoalPosition", goalPos);
  }

  @Override
  public void periodic() {
    FFvelocity = SmartDashboard.getNumber("FF: Velocity", FFvelocity);
    FFaccel = SmartDashboard.getNumber("FF: Acceleration", FFaccel);
    goalPos = SmartDashboard.getNumber("GoalPosition", goalPos);
    SmartDashboard.putNumber("ArmLEncoderPos", motorLEncoder.getPosition());

    double difference = goalPos - motorLEncoder.getPosition();
    if (difference > encoderErrorTolerance){//even PID needs an acceptable error sometimes
      //assuming calculate() is some sort of PID-esque thing
      motorL.set(wristFeed.calculate(difference, FFvelocity, FFaccel));
    }
  }
  }


