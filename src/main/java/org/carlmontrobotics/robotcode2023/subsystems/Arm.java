package org.carlmontrobotics.robotcode2023.subsystems;

import org.carlmontrobotics.MotorConfig;
import org.carlmontrobotics.lib199.MotorControllerFactory;
import org.carlmontrobotics.robotcode2023.Constants;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax;

import java.util.function.DoubleConsumer;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;



public class Arm extends SubsystemBase {

  //actual arm
  private CANSparkMax armMotor = MotorControllerFactory.createSparkMax(Constants.Arm.port, MotorConfig.NEO);
  private CANSparkMax wristMotor = MotorControllerFactory.createSparkMax(Constants.Wrist.port, MotorConfig.NEO_550);
  
  private SparkMaxAbsoluteEncoder armEncoder = armMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
  private SparkMaxAbsoluteEncoder wristEncoder = wristMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
  
  private double armGearRatio = 1/47.25;
  private double wristGearRatio = 1/15;
  //READ ME: Values are stored in the style of [arm,wrist]
  //0 is arm
  //1 is wrist
  
  //TODO GET NUMBERS
  private static double[] hiClamp = {-Math.PI*.5,   -Math.PI*.5}; 
  private static double[] loClamp = {-Math.PI*1.4,  -Math.PI*1.4};
  
  //FeedForward
  //FIXME WRIST NEEDS SYSID DONE
  private double[] kS = {.067766,   .05}; //volts | base speed
  private double[] kG = {.0075982,  .007}; //volts | gravity... something
  private double[] kV = {.019762,   .02}; //volts*secs/rad | extra velocity
  private double[] kA = {.00039212, .0004}; //volts*secs^2/rad | vacceleration

  private double[] FFvelocity = {.01,.01};
  private double[] FFaccel =    {.01,.01};
  
  private ArmFeedforward armFeed = new ArmFeedforward(kS[0], kG[0], kV[0], kA[0]);
  private ArmFeedforward wristFeed=new ArmFeedforward(kS[1], kG[1], kV[1], kA[1]);
  
  //PID
  //FIXME BOTH WRIST AND ARM NEED PID DONE
  private double[] kP = {.3,        .3};
  private double[] kI = {.3,        .3};
  private double[] kD = {.3,        .3}; 
  
  private double[] posTolerance = {.05, .05};
  private double[] velTolerance = {1,   1};
  
  private PIDController armpid = new PIDController(kP[0], kI[0], kD[0]);
  private PIDController wristpid=new PIDController(kP[1], kI[1], kD[1]);
  
  
  private ArmPreset goalEnum;
  
  private double initialArmOffset = -Math.PI / 2;
  private double wristFoldPos = 2.5307274154;
  
  public final Consumer<ArmPreset> setGoalEnum =      (nenum) -> {goalEnum.reset(); goalEnum = nenum;};
  public final DoubleConsumer setGoalArmPos =         (pos) -> {goalEnum.value[0] = MathUtil.clamp(pos, loClamp[0], hiClamp[0]);};
  public final DoubleConsumer setGoalWristPos =       (pos) -> {goalEnum.value[1] = MathUtil.clamp(pos, loClamp[1], hiClamp[1]);};
  public final DoubleSupplier getGoalArmPos =         () -> {return goalEnum.value[0];};
  public final DoubleSupplier getGoalWristPos =       () -> {return goalEnum.value[1];};
  
  private static int itemType=0;//for use with enum but needs persistence
  public enum ArmPreset {
    //radians
    //approximate values
    //                     Cube                                           Cone
    //                     Arm              Wrist                         Arm             Wrist
    GROUND(new double[]  { 1.71254781475,   0.4085117650}, new double[] { 2.29021388861,  0.408513963836}),
    LOW(new double[]     { 1.02603845343,  -0.9225673417}, new double[] { 4.10094074984, -0.18997011808}),
    MID(new double[]     { 0.83753504023,  -1.5006099006}, new double[] { 1.13715693466, -2.11308267430}),
    HIGH(new double[]    { 0.76831247212,  -1.8861790155}, new double[] { 1.46030263853, -1.86498478435}),

    HOLD(new double[] {0.f,0.f}, new double[] {0.f,0.f});
    
    public double[] value;
    private double[][] origin;
      
    ArmPreset(double[] cuvals, double[] covals){
      this.origin = new double[][] {cuvals, covals};
      this.value = this.origin[itemType];
    }
    private void reset(){
      this.value = this.origin[itemType];//for when we manually set the goalEnum pos, when we cycle this will fix itself
    }
    public double[] clampedValues(){
      return new double[] {
        MathUtil.clamp(this.value[0], loClamp[0], hiClamp[0]),
        MathUtil.clamp(this.value[1], loClamp[1], hiClamp[1])
      };
    }
  }

  public Arm() {
    armEncoder.setPositionConversionFactor(armGearRatio);
    wristEncoder.setPositionConversionFactor(wristGearRatio);
    
    armEncoder.setZeroOffset(initialArmOffset);
    
    armpid.setTolerance(posTolerance[0], velTolerance[0]);
    wristpid.setTolerance(posTolerance[1], velTolerance[1]);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Arm EncoderPos", armEncoder.getPosition());
    double speed = SmartDashboard.getNumber("Motor Voltage", 0);
    armMotor.set(speed);
    
//    double[] currentPos = new double[] {armEncoder.getPosition(),wristEncoder.getPosition()};
//    double[] goalPos = goalEnum.clampedValues();
//
//    //wrist bellypan folding
//    if ((Math.abs(currentPos[0]) < Math.PI/6.0) || (Double.doubleToRawLongBits(goalPos[0]) < 0 != Double.doubleToRawLongBits(currentPos[0]) < 0)) {
//      //if their signs are different, they pass through 0 (bellypan)
//      //but also if within 30deg of bellypan fold anyways
//      wristMotor.setVoltage(wristFeed.calculate(currentPos[1], 0, 0)
//         + wristpid.calculate(currentPos[1], wristFoldPos));
//    } else{
//      wristMotor.setVoltage(wristFeed.calculate(currentPos[1], 0, 0)
//         + wristpid.calculate(currentPos[1], goalPos[1]));
//    }
//    //arm
//    armMotor.setVoltage(armFeed.calculate(currentPos[0], 0, 0)
//    + armpid.calculate(currentPos[0], goalPos[0]));
  }
  
  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty("ArmFFVelocity",() -> FFvelocity[0],       x -> this.FFvelocity[0] = x);
    builder.addDoubleProperty("WisFFVelocity",() -> FFvelocity[0],       x -> this.FFvelocity[0] = x);
    builder.addDoubleProperty("ArmFFAccel",   () -> FFaccel[0],          x -> this.FFaccel[0] = x);
    builder.addDoubleProperty("WisFFAccel",   () -> FFaccel[0],          x -> this.FFaccel[0] = x);
    builder.addDoubleProperty("goalPos:Arm",  () -> goalEnum.value[0],x -> this.goalEnum.value[0] = x);
    builder.addDoubleProperty("goalPos:Wis",  () -> goalEnum.value[1],x -> this.goalEnum.value[1] = x);
    builder.addDoubleProperty("ArmPos",       () -> armEncoder.getPosition(),   null);
    builder.addDoubleProperty("WisPos",       () -> wristEncoder.getPosition(), null);
    builder.addDoubleProperty("kP: Arm",      () -> kP[0],            x -> this.kP[0] = x);
    builder.addDoubleProperty("kP: Wis",      () -> kP[1],            x -> this.kP[1] = x);
    builder.addDoubleProperty("kI: Arm",      () -> kI[0],            x -> this.kI[0] = x);
    builder.addDoubleProperty("kI: Wis",      () -> kI[1],            x -> this.kI[1] = x);
    builder.addDoubleProperty("kD: Arm",      () -> kD[0],            x -> this.kD[0] = x);
    builder.addDoubleProperty("kD: Wis",      () -> kD[1],            x -> this.kD[1] = x);
    builder.addDoubleProperty("kV: Arm",      () -> kV[0],            x -> this.kV[0] = x);
    builder.addDoubleProperty("kV: Wis",      () -> kV[1],            x -> this.kV[1] = x);
    builder.addDoubleProperty("kG: Arm",      () -> kG[0],            x -> this.kG[0] = x);
    builder.addDoubleProperty("kG: Wis",      () -> kG[1],            x -> this.kG[1] = x);
    builder.addDoubleProperty("kS: Arm",      () -> kS[0],            x -> this.kS[0] = x);
    builder.addDoubleProperty("kS: Wis",      () -> kS[1],            x -> this.kS[1] = x);
    builder.addDoubleProperty("kA: Arm",      () -> kA[0],            x -> this.kA[0] = x);
    builder.addDoubleProperty("kA: Wis",      () -> kA[1],            x -> this.kA[1] = x);
  } 
  
  public void swapItemType(){
    itemType = (itemType==0) ? 1 : 0;
  }
  
  public double getRobotWristAngle(){
    return armEncoder.getPosition()+wristEncoder.getPosition();
  }
  
}