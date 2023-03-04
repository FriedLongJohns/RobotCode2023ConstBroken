package org.carlmontrobotics.robotcode2023.subsystems;

import org.carlmontrobotics.lib199.MotorControllerFactory;
import org.carlmontrobotics.lib199.MotorErrors.TemperatureLimit;
import org.carlmontrobotics.robotcode2023.Constants;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;


public class Arm extends SubsystemBase {
  
  public CANSparkMax motor = MotorControllerFactory.createSparkMax(Constants.Arm.port, TemperatureLimit.NEO);
  public AbsoluteEncoder encoder;

  public double encoderErrorTolerance = .05;

  private double kS = .067766; //volts | base speed
  private double kG = .0075982; //volts | gravity... something
  private double kV = .019762; //volts*secs/rad | extra velocity
  private double kA = .00039212; //volts*secs^2/rad | vacceleration
  /// these are all units ^ , actual arm speed is determined by values in .calculate
  private double kP = 1.1;
  private double kI = 1.1;
  private double kD = 1.1;

  private double FFvelocity = .01;
  private double FFaccel = .01;
  private ArmFeedforward armFeed = new ArmFeedforward(kS, kG, kV, kA);
  private PIDController pid = new PIDController(kA, kI, kD);
  
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
    public ArmPreset prev() {
      switch (this) {
        case MID: return INTAKE;

        case HIGH: return MID;
        case INTAKE: return HIGH;
      }
      return null;
      }  
    }
  

  public Arm() {
    encoder.setPositionConversionFactor(1/60);
    encoder.getZeroOffset();
      SmartDashboard.putNumber("FF: Velocity", FFvelocity);
      SmartDashboard.putNumber("FF: Acceleration", FFaccel);
      SmartDashboard.putNumber("GoalPosition", goalPos);
      SmartDashboard.putNumber("KG", kG);
      SmartDashboard.putString("Debooog", "No.");
      SmartDashboard.putString("ArmENUM", 
      snappedArmPos().toString()
    );
    SmartDashboard.putNumber("kP", kP);
    SmartDashboard.putNumber("kI", kI);
    SmartDashboard.putNumber("kD", kD);
    pid.setTolerance(2.5,10);
  }

  @Override
  public void periodic() {
    FFvelocity = SmartDashboard.getNumber("FF: Velocity", FFvelocity);
    FFaccel = SmartDashboard.getNumber("FF: Acceleration", FFaccel);
    goalPos = SmartDashboard.getNumber("GoalPosition", goalPos);
    SmartDashboard.putNumber("ArmLencoderPos", encoder.	getZeroOffset());
    SmartDashboard.putString("ArmENUM", 
      (closeSnappedArmPos() != null) ? closeSnappedArmPos().toString() : snappedArmPos().toString()
    );
    kP = SmartDashboard.getNumber("kP", kP);
    kI = SmartDashboard.getNumber("kI", kI);
    kD = SmartDashboard.getNumber("kD", kD);
    pid.setP(kP);
    pid.setI(kI);
    pid.setD(kD);
    double currentPos = encoder.getZeroOffset();
    //pid.atSetpoint();

    // motor.set(pid.calculate( motorLencoder.	getZeroOffset(), setpoint));
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
    double encoderPos = encoder.getZeroOffset();
    
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
    double encoderPos = encoder.getZeroOffset();
    
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
    // if closeSnappedArmPos is workIng, swap based on it - otherwise use less accurate snapping
  }
  
  public void cycleDown() {
    SmartDashboard.putNumber("GoalPosition", 
    closeSnappedArmPos() != null ? closeSnappedArmPos().prev().value : snappedArmPos().prev().value);
  }
  
  public void setPreset(ArmPreset preset) {
    SmartDashboard.putNumber("GoalPosition", preset.value);
  }
}
