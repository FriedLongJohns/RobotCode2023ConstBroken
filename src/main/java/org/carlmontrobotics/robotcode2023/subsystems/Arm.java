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

  public enum ArmPreset {
    CONEINTAKE(0), CUBEINTAKE(0), CONELOWROW(.15), CUBELOWROW(.1), CONEMIDROW(.35), CUBEMIDROW(.3), CONEHIGHROW(.55), CUBEHIGHROW(.5), ERROR(-1);
    
    public double value;//not static so SmartDashboard can touch
    ArmPreset(double value){
      this.value=value;
    }
    public ArmPreset next(){//will stay Cone/Cube values even upon return
      switch (this){
        case CONEINTAKE:
          return ArmPreset.CONELOWROW;
        case CUBEINTAKE:
          return ArmPreset.CUBELOWROW;
        case CONELOWROW:
          return ArmPreset.CONEMIDROW;
        case CUBELOWROW:
          return ArmPreset.CUBEMIDROW;
        case CONEMIDROW:
          return ArmPreset.CONEHIGHROW;
        case CUBEMIDROW:
          return ArmPreset.CUBEHIGHROW;
        case CONEHIGHROW:
          return ArmPreset.CONEINTAKE;
        case CUBEHIGHROW:
          return ArmPreset.CUBEINTAKE;
        default:
          return this;
      }
    }
    public ArmPreset prev(){
      switch (this){
        case CONEINTAKE:
          return ArmPreset.CONEHIGHROW;
        case CUBEINTAKE:
          return ArmPreset.CUBEHIGHROW;
        case CONELOWROW:
          return ArmPreset.CONEINTAKE;
        case CUBELOWROW:
          return ArmPreset.CUBEINTAKE;
        case CONEMIDROW:
          return ArmPreset.CONELOWROW;
        case CUBEMIDROW:
          return ArmPreset.CUBELOWROW;
        case CONEHIGHROW:
          return ArmPreset.CONEMIDROW;
        case CUBEHIGHROW:
          return ArmPreset.CUBEMIDROW;
        default:
          return this;
      }
    }
    public ArmPreset swapType(){//change from cone to cube and vice versa
      switch (this){
        case CONEINTAKE:
          return ArmPreset.CUBEINTAKE;
        case CUBEINTAKE:
          return ArmPreset.CONEINTAKE;
        case CONELOWROW:
          return ArmPreset.CUBELOWROW;
        case CUBELOWROW:
          return ArmPreset.CONELOWROW;
        case CONEMIDROW:
          return ArmPreset.CUBEMIDROW;
        case CUBEMIDROW:
          return ArmPreset.CONEMIDROW;
        case CONEHIGHROW:
          return ArmPreset.CUBEHIGHROW;
        case CUBEHIGHROW:
          return ArmPreset.CONEHIGHROW;
        default://ERROR
          return this;
      }
    }
  }



  //MotorL is the leader btw
  public Arm(){
    motorR.follow(motorL, true);
    motorLencoder.setPositionConversionFactor(1/60);
    motorRencoder.setPositionConversionFactor(1/60);

    SmartDashboard.putNumber("ConeIntake", ArmPreset.CONEINTAKE.value);
    SmartDashboard.putNumber("ConeLOW", ArmPreset.CONELOWROW.value);
    SmartDashboard.putNumber("ConeMID", ArmPreset.CONEMIDROW.value);
    SmartDashboard.putNumber("ConeHIGH", ArmPreset.CONEHIGHROW.value);
    SmartDashboard.putNumber("CubeIntake", ArmPreset.CUBEINTAKE.value);
    SmartDashboard.putNumber("CubeLOW", ArmPreset.CUBELOWROW.value);
    SmartDashboard.putNumber("CubeMID", ArmPreset.CUBEMIDROW.value);
    SmartDashboard.putNumber("CubeHIGH", ArmPreset.CUBEHIGHROW.value);

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
    ArmPreset.CONEINTAKE.value = SmartDashboard.getNumber("ConeIntake", ArmPreset.CONEINTAKE.value);
    ArmPreset.CONELOWROW.value = SmartDashboard.getNumber("ConeLOW", ArmPreset.CONELOWROW.value);
    ArmPreset.CONEMIDROW.value = SmartDashboard.getNumber("ConeMID", ArmPreset.CONEMIDROW.value);
    ArmPreset.CONEHIGHROW.value = SmartDashboard.getNumber("ConeHIGH", ArmPreset.CONEHIGHROW.value);
    ArmPreset.CUBEINTAKE.value = SmartDashboard.getNumber("CubeIntake", ArmPreset.CUBEINTAKE.value);
    ArmPreset.CUBELOWROW.value = SmartDashboard.getNumber("CubeLOW", ArmPreset.CUBELOWROW.value);
    ArmPreset.CUBEMIDROW.value = SmartDashboard.getNumber("CubeMID", ArmPreset.CUBEMIDROW.value);
    ArmPreset.CUBEHIGHROW.value = SmartDashboard.getNumber("CubeHIGH", ArmPreset.CUBEHIGHROW.value);

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
  public ArmPreset snappedArmPos(){
    double encoderPos = motorLencoder.getPosition();
    
    for(ArmPreset check : ArmPreset.values()){
        double predist=(check.value-check.prev().value)/2;
        double hidist=(check.next().value-check.value)/2;//get the halfway points between each position and it's neighbors
        if (check.value-predist < encoderPos && encoderPos < check.value+predist){//seperate high and low instead of ABS because maybe difference isn't constant between each position of arm
            //and yes it still works for lowest and highest value
            return check;
        }
    }
    //help something went wrong
    return ArmPreset.ERROR;
  }

  public ArmPreset closeSnappedArmPos(){//more precise snapping
    double encoderPos = motorLencoder.getPosition();
    
    for(ArmPreset check : ArmPreset.values()){
        if (Math.abs(check.value-encoderPos)>encoderErrorTolerance){//maybe will break if cone/cube values are close, but if they are close then lower error or only use one enum
            return check;
        }
    }
    //help something went wrong
    return ArmPreset.ERROR;
  }

  public void cycleUp(){//because most people won't remember/want to do this long function chain
    goalPos = closeSnappedArmPos()!=ArmPreset.ERROR ? closeSnappedArmPos().next().value : snappedArmPos().next().value;
    //if closeSnappedArmPos is working, swap based on it - otherwise use less accurate snapping
  }
  
  public void cycleDown(){
    goalPos = closeSnappedArmPos()!=ArmPreset.ERROR ? closeSnappedArmPos().prev().value : snappedArmPos().prev().value;
  }
  
  public void swapType(){
    goalPos = closeSnappedArmPos()!=ArmPreset.ERROR ? closeSnappedArmPos().swapType().value : snappedArmPos().swapType().value;
  }

  public void setPreset(ArmPreset preset){
    goalPos=preset.value;
  }

}
