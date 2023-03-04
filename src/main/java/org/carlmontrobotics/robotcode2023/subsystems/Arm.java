package org.carlmontrobotics.robotcode2023.subsystems;

import org.carlmontrobotics.lib199.MotorControllerFactory;
import org.carlmontrobotics.lib199.MotorErrors.TemperatureLimit;
import org.carlmontrobotics.robotcode2023.Constants;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
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
  private PIDController pid = new PIDController(Kp, Ki, Kd);
  
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
    SmartDashboard.putNumber("KG", kG);
    SmartDashboard.putString("Debooog", "No.");
    SmartDashboard.putString("ArmENUM", 
    snappedArmPos().toString()
    );
    SmartDashboard.putNumber("KP", Kp);
    SmartDashboard.putNumber("KI", Ki);
    SmartDashboard.putNumber("KD", Kd);
    pid.setTolerance(2.5,10);
  }

  @Override
  public void periodic() {
    FFvelocity = SmartDashboard.getNumber("FF: Velocity", FFvelocity);
    FFaccel = SmartDashboard.getNumber("FF: Acceleration", FFaccel);
    goalPos = SmartDashboard.getNumber("GoalPosition", goalPos);
    SmartDashboard.putNumber("ArmLencoderPos", motorLencoder.getPosition());
    SmartDashboard.putString("ArmENUM", 
    (closeSnappedArmPos() != null) ? closeSnappedArmPos().toString() : snappedArmPos().toString()
    );
    Kp = SmartDashboard.getNumber("KP", Kp);
    Ki = SmartDashboard.getNumber("KI", Ki);
    Kd = SmartDashboard.getNumber("KD", Kd);
    pid.setP(Kp);
    pid.setI(Ki);
    pid.setD(Kd);
    double currentPos = motorLencoder.getPosition();
    //pid.atSetpoint();

    // motor.set(pid.calculate( motorLencoder.getPosition(), setpoint));
    //FIXME CLAMP LIMIT FOR PROTOTYPE ONLY
    goalPos = MathUtil.clamp(goalPos, -Math.PI*1.4, -Math.PI*.5);
    //                           -4.39,  -1.57
    
      motor.setVoltage(armFeed.calculate(currentPos, 0, 0)
         + pid.calculate(currentPos, goalPos));
         
     
    
    
  }
  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);

    //builder.addBooleanProperty("Magnetic Field Disturbance", gyro::isMagneticDisturbance, null);
    //builder.addDoubleProperty("Odometry X", () -> getPose().getX(), null);
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
    closeSnappedArmPos() != null ? closeSnappedArmPos().next().value : snappedArmPos().next().value
    );
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
