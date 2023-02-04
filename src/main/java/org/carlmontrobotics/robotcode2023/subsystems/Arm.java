package org.carlmontrobotics.robotcode2023.subsystems;
import org.carlmontrobotics.lib199.MotorControllerFactory;
import org.carlmontrobotics.lib199.MotorErrors.TemperatureLimit;
import org.carlmontrobotics.robotcode2023.Constants;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;


public class Arm extends SubsystemBase
{
  public CANSparkMax motorL = MotorControllerFactory.createSparkMax(Constants.Arm.portL, TemperatureLimit.NEO);
  public CANSparkMax motorR = MotorControllerFactory.createSparkMax(Constants.Arm.portR, TemperatureLimit.NEO);
  public RelativeEncoder motorLencoder = motorL.getEncoder();
  public RelativeEncoder motorRencoder = motorR.getEncoder();

  public double encoderErrorTolerance = .05;

  private double kS = .2; //volts | base speed
  private double kG = .1; //volts | gravity... something
  private double kV = .2; //volts*secs/rad | extra velocity
  private double kA = .3; //volts*secs^2/rad | vacceleration
  /// these are all units ^ , actual arm speed is determined by values in .calculate
  private double FFvelocity = 2;
  private double FFaccel = 1;
  private ArmFeedforward armFeed = new ArmFeedforward(kS,kG,kV,kA);

  public double goalPos;



  //MotorL is the leader btw
  public Arm(){
    motorR.follow(motorL, true);
    //TODO: Assign the 60:1 gear ratio to the motors

    SmartDashboard.putNumber("EncoderLowPos", ArmPos.LOW.value);
    SmartDashboard.putNumber("EncoderMidPos", ArmPos.MID.value);
    SmartDashboard.putNumber("EncoderHighPos", ArmPos.HIGH.value);

    SmartDashboard.putNumber("FeedForward:kS", kS);
    SmartDashboard.putNumber("FeedForward:kG", kG);
    SmartDashboard.putNumber("FeedForward:kV", kV);
    SmartDashboard.putNumber("FeedForward:kA", kA);
    SmartDashboard.putNumber("FF: Velocity", FFvelocity);
    SmartDashboard.putNumber("FF: Acceleration", FFaccel);

    SmartDashboard.putNumber("GoalPosition", goalPos);
  }

  @Override
  public void periodic() {
    //you can do this because enum is just a class :)
    ArmPos.LOW.value = SmartDashboard.getNumber("EncoderLowPos", ArmPos.LOW.value);
    ArmPos.MID.value = SmartDashboard.getNumber("EncoderMidPos", ArmPos.MID.value);
    ArmPos.HIGH.value = SmartDashboard.getNumber("EncoderHighPos", ArmPos.HIGH.value);

    kS = SmartDashboard.getNumber("FeedForward: kS", kS);
    kG = SmartDashboard.getNumber("FeedForward: kG", kG);
    kV = SmartDashboard.getNumber("FeedForward: kV", kV);
    kA = SmartDashboard.getNumber("FeedForward: kA", kA);
    FFvelocity = SmartDashboard.getNumber("FF: Velocity", FFvelocity);
    FFaccel = SmartDashboard.getNumber("FF: Acceleration", FFaccel);

    goalPos = SmartDashboard.getNumber("GoalPosition", goalPos);//pls no touch while arm is cycle-ing


    double difference = goalPos-motorLencoder.getPosition();
    if (difference>encoderErrorTolerance){//even PID needs an acceptable error sometimes
      //assuming calculate() is some sort of PID-esque thing
      motorL.set(armFeed.calculate(difference, FFvelocity, FFaccel));
    }
  }

  //Snaps raw encoder pos to one of our cycle positions
  public ArmPos snappedArmPos(){
    double encoderPos = motorLencoder.getPosition();

    if(encoderPos>=ArmPos.MHH.value){//greater than halfway between mid and high
      return ArmPos.HIGH;
    } if(encoderPos<=ArmPos.LMH.value){//lass than halfway between low and mid
      return ArmPos.LOW;
    } else{//we are (essentially) at half point
      return ArmPos.MID;
    }
  }

  public ArmPos closeSnappedArmPos(){//more precise snapping
    double encoderPos = motorLencoder.getPosition();

    if (Math.abs(ArmPos.LOW.value-encoderPos)>encoderErrorTolerance){
      return ArmPos.LOW;
    } else if (Math.abs(ArmPos.MID.value-encoderPos)>encoderErrorTolerance){
      return ArmPos.MID;
    } else if (Math.abs(ArmPos.HIGH.value-encoderPos)>encoderErrorTolerance){
      return ArmPos.HIGH;
    } else {
      return ArmPos.ERROR;
    }
  }

  public void cyclePosition(){//because most people won't remember/want to do this long function chain
    goalPos = snappedArmPos().next().value;
  }

  public enum ArmPos{
    LOW(0),MID(.2),HIGH(.4),ERROR(-1),LMH(.1),MHH(.3);
    //Low-Mid Halfpoint and Mid-High Halfpoint are for easy position snapping math

    public double value;
    ArmPos(double value){
      this.value=value;
    }

    public ArmPos next(){
      switch (this){
        case LOW:
          return ArmPos.MID;
        case MID:
          return ArmPos.HIGH;
        case HIGH:
          return ArmPos.LOW;
        default:
          return this;
      }
    }
  }
}
