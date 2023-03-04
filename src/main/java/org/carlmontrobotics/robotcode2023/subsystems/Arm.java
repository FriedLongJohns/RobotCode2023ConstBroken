package org.carlmontrobotics.robotcode2023.subsystems;

import org.carlmontrobotics.lib199.MotorControllerFactory;
import org.carlmontrobotics.lib199.MotorErrors.TemperatureLimit;
import org.carlmontrobotics.robotcode2023.Constants;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;


public class Arm extends SubsystemBase {
  
  public CANSparkMax motor = MotorControllerFactory.createSparkMax(Constants.Arm.port, TemperatureLimit.NEO);
  public static CANSparkMax motorL = MotorControllerFactory.createSparkMax(Constants.Arm.portL, TemperatureLimit.NEO);
  public CANSparkMax motorR = MotorControllerFactory.createSparkMax(Constants.Arm.portR, TemperatureLimit.NEO);
  private final RelativeEncoder motorLencoder = motorL.getEncoder();
  private final RelativeEncoder motorRencoder = motorR.getEncoder();

  public double encoderErrorTolerance = .05;

  private double kS = .067766; //volts | base speed
  private double kG = .0075982; //volts | gravity... something
  private double kV = .019762; //volts*secs/rad | extra velocity
  private double kA = .00039212; //volts*secs^2/rad | vacceleration
  /// these are all units ^ , actual arm speed is determined by values in .calculate
  private double FFvelocity = .01;
  private double FFaccel = .01;
  private ArmFeedforward armFeed = new ArmFeedforward(kS, kG, kV, kA);

  public double goalPos;

  public enum ArmPreset {
    INTAKE(0.31), MID(-1.74), HIGH(-1.83);
    
    public double value; //not static so SmartDashboard can touch [IMPORTANT TO KNOW!]
    ArmPreset(double value) {
      this.value = value;
    }
    public ArmPreset next() {
      switch (this) {
        case INTAKE: return MID;

        case MID: return HIGH;
        case HIGH: return INTAKE;
      }
      return null;
    }
  }

  public Arm() {
    motorLencoder.setPositionConversionFactor(1/60);
    motorLencoder.setPosition(0.0);
    SmartDashboard.putNumber("FF: Velocity", FFvelocity);
    SmartDashboard.putNumber("FF: Acceleration", FFaccel);
    SmartDashboard.putNumber("GoalPosition", goalPos);
  }

  @Override
  public void periodic() {
    FFvelocity = SmartDashboard.getNumber("FF: Velocity", FFvelocity);
    FFaccel = SmartDashboard.getNumber("FF: Acceleration", FFaccel);
    goalPos = SmartDashboard.getNumber("GoalPosition", goalPos);
    SmartDashboard.putNumber("ArmLencoderPos", motorLencoder.getPosition());

    double difference = goalPos - motorLencoder.getPosition();
    if (difference > encoderErrorTolerance) {//even PID needs an acceptable error sometimes
      //assuming calculate() is some sort of PID-esque thing
      motor.set(armFeed.calculate(difference, FFvelocity, FFaccel));
    }
  }

  //Snaps raw encoder pos to one of our cycle positions
  public ArmPreset snappedArmPos() {
    double encoderPos = motorLencoder.getPosition();
    
    for(ArmPreset check : ArmPreset.values()) {
      double lowdist = (check.value - check.prev().value) / 2;
      double hidist = (check.next().value - check.value) / 2; // get the halfway points between each position and it's neighbors
      if (check.value - lowdist < encoderPos && encoderPos < check.value + hidist) {
        //seperate high and low instead of ABS because maybe difference isn't constant between each position of arm
          //and yes it still works for lowest and highest value
          return check;
      }
    }
    //help something went wrong
    return null;
  }

  public ArmPreset closeSnappedArmPos() {//more precise snapping
    double encoderPos = motorLencoder.getPosition();
    
    for(ArmPreset check : ArmPreset.values()) {
        if (Math.abs(check.value - encoderPos) > encoderErrorTolerance) {//maybe will break if cone/cube values are close, but if they are close then lower error or only use one enum
            return check;
        }
    }
    //help something went wrong
    return null;
  }

  public void cycleUp() { 
    // because most people won't remember/want to do this long function chain
    SmartDashboard.putNumber("GoalPosition", 
    closeSnappedArmPos() != null ? closeSnappedArmPos().next().value : snappedArmPos().next().value);
    // if closeSnappedArmPos is working, swap based on it - otherwise use less accurate snapping
  }
  
  public void cycleDown() {
    SmartDashboard.putNumber("GoalPosition", 
    closeSnappedArmPos() != null ? closeSnappedArmPos().prev().value : snappedArmPos().prev().value
    );
  }
  
  public void setPreset(ArmPreset preset) {
    SmartDashboard.putNumber("GoalPosition", preset.value);
  }
}
