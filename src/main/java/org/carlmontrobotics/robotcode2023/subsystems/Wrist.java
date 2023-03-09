package org.carlmontrobotics.robotcode2023.subsystems;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;
import java.util.function.Consumer;

import org.carlmontrobotics.lib199.MotorControllerFactory;
import org.carlmontrobotics.lib199.MotorErrors.TemperatureLimit;
import org.carlmontrobotics.robotcode2023.Constants;
import org.carlmontrobotics.robotcode2023.subsystems.Arm;

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
  
  public WristPreset goalEnum;
  private double gearRatio = 1/15;//FIX ME GET GEAR RATIO
  
  //FIXME GET ACTUAL OFFSET (if needed)
  //TODO offset by arm encoderPos but also allow local clamping
    
  private double hiClamp = Math.PI*.5; //FIXME GET NUMBERS
  private double loClamp = -Math.PI*.5;
  
  public boolean fold = false;//for when arm moves through bellypan
  private double foldPos = 2.5307274154;
    
  public final Consumer<WristPreset> setGoalEnum = (nenum) -> {goalEnum.reset(); goalEnum = nenum;};
  public final DoubleConsumer setGoalPos =        (pos) -> {goalEnum.value = MathUtil.clamp(pos, loClamp, hiClamp);};
  public final DoubleSupplier getGoalPos =          () -> {return goalEnum.value;};
  
  public Arm arm;
  
  private static int itemType=0;//for use with enum but needs persistence
  public enum WristPreset {
    //radians
    //aproximate values
    //                   cube            cone
    GROUND(new double[] {0.40851176500, 0.408513963836}),
    LOW(new double[]    {-0.9225673417, -0.18997011808}),
    MID(new double[]    {-1.5006099006, -2.11308267430}),
    HIGH(new double[]   {-1.8861790155, -1.86498478435}),
    HOLD(new double[] {0.f,0.f});
    
    public double value;
    private double[] origin;
    
    WristPreset(double values[]) {
      this.value = values[itemType];
      this.origin = values;
    }
    private void reset(){
      this.value = this.origin[itemType];//for when we manually set the goalEnum pos, when we cycle this will fix itself
    }
  }
  


  public Wrist() {
    encoder.setPositionConversionFactor(gearRatio * 2 * Math.PI);
    pid.setTolerance(posTolerance, velTolerance);
    //SmartDashboard.putNumber("GoalPosition", goalPos);
  }

  @Override
  public void periodic() {
    pid.setP(kP);
    pid.setI(kI);
    pid.setD(kD);
    
    double currentPos = encoder.getPosition();
    double goalPos = MathUtil.clamp(goalEnum.value, loClamp, hiClamp);
    
    if (fold==false){
      motor.setVoltage(wristFeed.calculate(currentPos, 0, 0)
         + pid.calculate(currentPos, goalPos));
    } else {
      motor.setVoltage(wristFeed.calculate(currentPos, 0, 0)
         + pid.calculate(currentPos, foldPos));
    }
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty("FF: Velocity", () -> FFvelocity,      x -> this.FFvelocity = x);
    builder.addDoubleProperty("FF: Accel",    () -> FFaccel,         x -> this.FFaccel = x);
    builder.addDoubleProperty("goalPos",      () -> goalEnum.value,  x -> this.goalEnum.value = x);
    builder.addBooleanProperty("fold?",       () -> fold,            x -> this.fold = x);
    builder.addDoubleProperty("kP",           () -> kP,              x -> this.kP = x);
    builder.addDoubleProperty("kI",           () -> kI,              x -> this.kI = x);
    builder.addDoubleProperty("kD",           () -> kD,              x -> this.kD = x);
    builder.addDoubleProperty("kV",           () -> kV,              x -> this.kV = x);
    builder.addDoubleProperty("kG",           () -> kG,              x -> this.kG = x);
    builder.addDoubleProperty("kS",           () -> kS,              x -> this.kS = x);
    builder.addDoubleProperty("kA",           () -> kA,              x -> this.kA = x);
  }
  
  public double getWorldAngle(){
    return arm.encoder.getPosition() + encoder.getPosition();
  }
  
  public void swapItemType(){
    itemType = (itemType==0) ? 1 : 0;
  }
}
