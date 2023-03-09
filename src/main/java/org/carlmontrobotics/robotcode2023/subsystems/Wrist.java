package org.carlmontrobotics.robotcode2023.subsystems;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import org.carlmontrobotics.lib199.MotorControllerFactory;
import org.carlmontrobotics.lib199.MotorErrors.TemperatureLimit;
import org.carlmontrobotics.robotcode2023.Constants;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.CANSparkMax;

public class Wrist extends SubsystemBase {
  
  private CANSparkMax motor = MotorControllerFactory.createSparkMax(Constants.Wrist.port, TemperatureLimit.NEO_550);
  private SparkMaxAbsoluteEncoder encoder = motor.getAbsoluteEncoder(Type.kDutyCycle);
  private double posTolerance = .05;
  private double velTolerance = 1;

  private double kS = .067766; //volts | base speed
  private double kG = .0075982; //volts | gravity... something
  private double kV = .019762; //volts*secs/rad | extra velocity
  private double kA = .00039212; //volts*secs^2/rad | vacceleration
  /// these are all units ^ , actual arm speed is determined by values in .calculate
  //FIXME DO WRIST sysid
  private double kP = 0;
  private double kI = 0; //FIXME add real values
  private double kD = 0;

  private double FFvelocity = .01;
  private double FFaccel = .01;
  private ArmFeedforward wristFeed = new ArmFeedforward(kS, kG, kV, kA);
  private PIDController pid = new PIDController(kP, kI, kD);
  
  public double goalPos = 0; // initial position
  
  //FIXME GET ACTUAL OFFSET (if needed)
  //TODO offset by arm encoderPos but also allow local clamping
    
  private double hiClamp = Math.PI*.5; //FIXME GET NUMBERS
  private double loClamp = -Math.PI*.5;
  public final DoubleConsumer setGoalPos = (pos) -> {goalPos = MathUtil.clamp(pos, loClamp, hiClamp);};
  public final DoubleSupplier getGoalPos = () -> {return goalPos;};

  private double gearRatio = 1/15;//FIX ME GET GEAR RATIO

  public enum WristPreset {
    //radians
    //aproximate values
    MOVEGAMEPIECE(0),
    
    CUBEGROUNDPICKUP(0.408511765),   CONEGROUNDPICKUP(0.408513963836), 
    CUBEOUTPUTLOW(-0.922567341767),  CONEOUTPUTLOW(-0.189970118087),
    CUBEOUTPUTMID(-1.5006099006),    CONEOUTPUTHIGH(-2.113082674302),
    CUBEOUTPUTHIGH(-1.886179015595), CONEOUTPUTMID(-1.864984784356);
    
    public double value; //not static so SmartDashboard can touch [IMPORTANT TO KNOW!]
    WristPreset(double value) {
      this.value = value;
    }
    public WristPreset swapType(){
      switch (this) {
        case CUBEGROUNDPICKUP: return CONEGROUNDPICKUP;
        case CONEGROUNDPICKUP: return CUBEGROUNDPICKUP;
        case CUBEOUTPUTLOW:    return CONEOUTPUTLOW;
        case CUBEOUTPUTMID:    return CONEOUTPUTMID;
        case CUBEOUTPUTHIGH:   return CONEOUTPUTHIGH;
        case CONEOUTPUTLOW:    return CUBEOUTPUTLOW;
        case CONEOUTPUTMID:    return CUBEOUTPUTMID;
        case CONEOUTPUTHIGH:   return CUBEOUTPUTHIGH;
        default://includes MOVEGAMEPIECE
          return this;
    
      }
    }
  }
  


  public Wrist() {
    encoder.setPositionConversionFactor(gearRatio * 2 * Math.PI);
    encoder.setZeroOffset(goalPos);
    pid.setTolerance(posTolerance, velTolerance);
    //SmartDashboard.putNumber("GoalPosition", goalPos);
  }

  @Override
  public void periodic() {
    pid.setP(kP);
    pid.setI(kI);
    pid.setD(kD);
    double currentPos = encoder.getPosition();
    //goalPos = SmartDashboard.getNumber("GoalPosition", goalPos);    
    goalPos = MathUtil.clamp(goalPos, loClamp, hiClamp);
      
    motor.setVoltage(wristFeed.calculate(currentPos, 0, 0)
         + pid.calculate(currentPos, goalPos));
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty("FF: Velocity", () -> FFvelocity, x -> this.FFvelocity = x);
    builder.addDoubleProperty("FF: Accel",    () -> FFaccel,    x -> this.FFaccel = x);
    builder.addDoubleProperty("goalPos",      () -> goalPos,    x -> this.goalPos = x);
    builder.addDoubleProperty("kP",           () -> kP,         x -> this.kP = x);
    builder.addDoubleProperty("kI",           () -> kI,         x -> this.kI = x);
    builder.addDoubleProperty("kD",           () -> kD,         x -> this.kD = x);
    builder.addDoubleProperty("kV",           () -> kV,         x -> this.kV = x);
    builder.addDoubleProperty("kG",           () -> kG,         x -> this.kG = x);
    builder.addDoubleProperty("kS",           () -> kS,         x -> this.kS = x);
    builder.addDoubleProperty("kA",           () -> kA,         x -> this.kA = x);
  }
  
  public void setPreset(WristPreset preset) {
    goalPos = preset.value;
  }
}
