package org.carlmontrobotics.robotcode2023.subsystems;

import org.carlmontrobotics.lib199.MotorControllerFactory;
import org.carlmontrobotics.lib199.MotorErrors.TemperatureLimit;
import org.carlmontrobotics.robotcode2023.Constants;
import org.carlmontrobotics.robotcode2023.subsystems.Wrist;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax;

import java.util.function.DoubleConsumer;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;



public class Arm extends SubsystemBase {

  //actual arm
//  private CANSparkMax motor = MotorControllerFactory.createSparkMax(Constants.Arm.port, TemperatureLimit.NEO);
//  private SparkMaxAbsoluteEncoder encoder = motor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
  private double gearRatio = 225;
  
  public CANSparkMax motor = MotorControllerFactory.createSparkMax(Constants.Arm.port, TemperatureLimit.NEO);
  public SparkMaxAbsoluteEncoder encoder = motor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
  public double encoderErrorTolerance = .05;

  private double kS = .067766; //volts | base speed
  private double kG = .0075982; //volts | gravity... something
  private double kV = .019762; //volts*secs/rad | extra velocity
  private double kA = .00039212; //volts*secs^2/rad | vacceleration
  /// these are all units ^ , actual arm speed is determined by values in .calculate
  private double kP = 1.1;
  private double kI = 1.1; //will add real values
  private double kD = 1.1;

  private double FFvelocity = .01;
  private double FFaccel = .01;
  private ArmFeedforward armFeed = new ArmFeedforward(kS, kG, kV, kA);
  private PIDController pid = new PIDController(kP, kI, kD);
  
  private ArmPreset goalEnum;
  
  private double hiClamp = -Math.PI*.5; //TODO GET NUMBERS
  private double loClamp = -Math.PI*1.4;
  
  private Wrist wrist;
  private double initalOffset = -Math.PI / 2;
  
  public final Consumer<ArmPreset> setGoalEnum = (nenum) -> {goalEnum.reset(); goalEnum = nenum;};
  public final DoubleConsumer setGoalPos =        (pos) -> {goalEnum.value = MathUtil.clamp(pos, loClamp, hiClamp);};
  public final DoubleSupplier getGoalPos =          () -> {return goalEnum.value;};
  
  private double EncoderPos = encoder.getZeroOffset();
  
  private static int itemType=0;//for use with enum but needs persistence
  public enum ArmPreset {
    //radians
    //approximate values
    //                   cube            cone
    GROUND(new double[] {1.71254781475, 2.29021388861}),
    LOW(new double[]    {1.02603845343, 4.10094074984}),
    MID(new double[]    {0.83753504023, 1.13715693466}),
    HIGH(new double[]   {0.76831247212, 1.46030263853}),
    HOLD(new double[] {0.f,0.f});
    
    public double value;
    private double[] origin;
      
    ArmPreset(double values[]){
      this.value = values[itemType];
      this.origin = values;
    }
    private void reset(){
      this.value = this.origin[itemType];//for when we manually set the goalEnum pos, when we cycle this will fix itself
    }
  }
     
   /* public double value; //not static so SmartDashboard can touch [IMPORTANT TO KNOW!]
    ArmPreset(double value) {
      this.value = value;
    } 
    */

  public Arm(Wrist wristo) {
    /*
    encoder.setPositionConversionFactor(1/gearRatio);
    encoder.setPosition(initialOffset);
    pid.setTolerance(2.5,10);
    */

    SmartDashboard.putNumber("Motor Voltage", 0);
    encoder.setPositionConversionFactor(1 / gearRatio * 2 * Math.PI);
    motor.setIdleMode(IdleMode.kBrake);
    
    wrist = wristo;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Encoder Pos", encoder.getPosition());
    double speed = SmartDashboard.getNumber("Motor Voltage", 0);
    motor.set(speed);
    /*
    pid.setP(kP);
    pid.setI(kI);
    pid.setD(kD);
     double currentPos = encoder.getZeroOffset();
    
    goalPos = MathUtil.clamp(goalEnum.value, loClamp, hiClamp);
    
    if (Math.Abs(goalPos) > Math.PI/6.0) || (Double.doubleToRawLongBits(pos) < 0 != Double.doubleToRawLongBits(goalEnum) < 0){
    //if their signs are different, they pass through 0 (bellypan)
    //but also if within 30deg of bellypan fold anyways
      wrist.fold=true;
    } else{
      wrist.fold=false;
    }
      
      motor.setVoltage(armFeed.calculate(currentPos, 0, 0)
         + pid.calculate(currentPos, goalPos));
         
         goalPos = SmartDashboard.getNumber("GoalPosition", goalPos);
         SmartDashboard.getNumber("EncoderPos", currentPos);
         */
  }
  /*
  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty("FF: Velocity", () -> FFvelocity,      x -> this.FFvelocity = x);
    builder.addDoubleProperty("FF: Accel",    () -> FFaccel,         x -> this.FFaccel = x);
    builder.addDoubleProperty("goalPos",      () -> goalEnum.value,  x -> this.goalEnum.value = x);
    builder.addDoubleProperty("EncoderPos",   () -> EncoderPos,      x -> this.EncoderPos = x);
    builder.addDoubleProperty("kP",           () -> kP,              x -> this.kP = x);
    builder.addDoubleProperty("kI",           () -> kI,              x -> this.kI = x);
    builder.addDoubleProperty("kD",           () -> kD,              x -> this.kD = x);
    builder.addDoubleProperty("kV",           () -> kV,              x -> this.kV = x);
    builder.addDoubleProperty("kG",           () -> kG,              x -> this.kG = x);
    builder.addDoubleProperty("kS",           () -> kS,              x -> this.kS = x);
    builder.addDoubleProperty("kA",           () -> kA,              x -> this.kA = x);
  }
  */
  public void swapItemType(){
    itemType = (itemType==0) ? 1 : 0;
  }
  
}