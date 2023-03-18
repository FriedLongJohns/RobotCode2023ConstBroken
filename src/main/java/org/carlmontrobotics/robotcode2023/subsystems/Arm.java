package org.carlmontrobotics.robotcode2023.subsystems;

import static org.carlmontrobotics.robotcode2023.Constants.Arm.*;

import org.carlmontrobotics.MotorConfig;
import org.carlmontrobotics.lib199.MotorControllerFactory;
import org.carlmontrobotics.robotcode2023.Constants.GoalPos;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Arm angle is measured from horizontal on the intake side of the robot and bounded between -3π/2 and π/2
// Wrist angle is measured relative to the arm with 0 being parallel to the arm and bounded between -π and π (Center of Mass of Roller)
public class Arm extends SubsystemBase {

    private final CANSparkMax armMotor = MotorControllerFactory.createSparkMax(armMotorPort, MotorConfig.NEO);
    private final CANSparkMax wristMotor = MotorControllerFactory.createSparkMax(wristMotorPort, MotorConfig.NEO);
    private final SparkMaxAbsoluteEncoder armEncoder = armMotor
            .getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
    private final SparkMaxAbsoluteEncoder wristEncoder = wristMotor
            .getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);

    private final SimpleMotorFeedforward armFeed = new SimpleMotorFeedforward(kS[ARM], kV[ARM], kA[ARM]);
    private ArmFeedforward wristFeed = new ArmFeedforward(kS[WRIST], kG[WRIST], kV[WRIST], kA[WRIST]);

    public PIDController armPID = new PIDController(kP[ARM], kI[ARM], kD[ARM]);
    public PIDController wristPID = new PIDController(kP[WRIST], kI[WRIST], kD[WRIST]);

    private SendableBuilder senb;

    private int object = CUBE;
    public boolean isFront = true;

    public Arm() {
        armMotor.setInverted(inverted[ARM]);
        wristMotor.setInverted(inverted[WRIST]);

        armEncoder.setPositionConversionFactor(rotationToRad);
        wristEncoder.setPositionConversionFactor(rotationToRad);

        wristEncoder.setZeroOffset(offsetRad[ARM]);
        armEncoder.setZeroOffset(offsetRad[WRIST]);

        armPID.setTolerance(posToleranceRad[ARM], velToleranceRadPSec[ARM]);
        wristPID.setTolerance(posToleranceRad[WRIST], velToleranceRadPSec[WRIST]);

        SmartDashboard.putBoolean("Toggle", false);
        senb.update();
    }

    @Override
    public void periodic() {
        // TODO: REMOVE THIS WHEN PID CONSTANTS ARE DONE
        armPID.setP(kP[ARM]);
        armPID.setI(kI[ARM]);
        armPID.setD(kD[ARM]);
        wristPID.setP(kP[WRIST]);
        wristPID.setI(kI[WRIST]);
        wristPID.setD(kD[WRIST]);
        if (SmartDashboard.getBoolean("Toggle", false)) {
            senb.update();
        }
    }

    public void driveArm(double vel, double accel) {
        double armFeedVolts = getKg() * getCoM().getAngle().getCos() + armFeed.calculate(vel, accel);
        double armPIDVolts = armPID.calculate(armEncoder.getPosition(), goalPosRad[ARM]);
        if ((getArmPos() > ARM_UPPER_LIMIT_RAD && armFeedVolts > 0) || 
            (getArmPos() < ARM_LOWER_LIMIT_RAD && armFeedVolts < 0)) {
            armFeedVolts = getKg() * getCoM().getAngle().getCos() + armFeed.calculate(0, 0);
        }
        // TODO: REMOVE WHEN DONE WITH TESTING (ANY CODE REVIEWERS, PLEASE REJECT MERGES
        // TO MASTER IF THIS IS STILL HERE)
        SmartDashboard.putNumber("ArmFeedVolts", armFeedVolts);
        SmartDashboard.putNumber("ArmPIDVolts", armPIDVolts);
        double volts = armFeedVolts + armPIDVolts;
        SmartDashboard.putNumber("ArmTotalVolts", volts);
        armMotor.setVoltage(volts);
    }

    public void driveWrist(double vel, double accel) {
        double wristFeedVolts = wristFeed.calculate(getWristPosRelativeToGround(), vel, accel);
        double wristPIDVolts = wristPID.calculate(getWristPos(), goalPosRad[WRIST]);
        if ((getWristPos() > WRIST_UPPER_LIMIT_RAD && wristFeedVolts > 0) || 
            (getWristPos() < WRIST_LOWER_LIMIT_RAD && wristFeedVolts < 0)) {
            wristFeedVolts = wristFeed.calculate(getWristPosRelativeToGround(), 0, 0);
        }
        // TODO: REMOVE WHEN DONE WITH TESTING (ANY CODE REVIEWERS, PLEASE REJECT MERGES
        // TO MASTER IF THIS IS STILL HERE)
        SmartDashboard.putNumber("WristFeedVolts", wristFeedVolts);
        SmartDashboard.putNumber("WristPIDVolts", wristPIDVolts);
        double volts = wristFeedVolts + wristPIDVolts;
        SmartDashboard.putNumber("WristTotalVolts", volts);
        wristMotor.setVoltage(volts);
    }

    public void driveArm(TrapezoidProfile.State profile) {
        double armFeedVolts = getKg() * getCoM().getAngle().getCos() + armFeed.calculate(profile.velocity, 0);
        double armPIDVolts = armPID.calculate(armEncoder.getPosition(), profile.position);
        if (getArmPos() > ARM_UPPER_LIMIT_RAD || getArmPos() < ARM_LOWER_LIMIT_RAD) {
            armFeedVolts = getKg() * getCoM().getAngle().getCos() + armFeed.calculate(0, 0);
        }
        // TODO: REMOVE WHEN DONE WITH TESTING (ANY CODE REVIEWERS, PLEASE REJECT MERGES
        // TO MASTER IF THIS IS STILL HERE)
        SmartDashboard.putNumber("ArmFeedVolts", armFeedVolts);
        SmartDashboard.putNumber("ArmPIDVolts", armPIDVolts);
        double volts = armFeedVolts + armPIDVolts;
        SmartDashboard.putNumber("ArmTotalVolts", volts);
        armMotor.setVoltage(volts);
    }

    public void driveWrist(TrapezoidProfile.State profile) {
        double wristFeedVolts = getKg() * getCoM().getAngle().getCos() + wristFeed.calculate(profile.velocity, 0);
        double wristPIDVolts = wristPID.calculate(wristEncoder.getPosition(), profile.position);
        if (getWristPos() > WRIST_UPPER_LIMIT_RAD || getWristPos() < WRIST_LOWER_LIMIT_RAD) {
            wristFeedVolts = getKg() * getCoM().getAngle().getCos() + wristFeed.calculate(0, 0);
        }
        // TODO: REMOVE WHEN DONE WITH TESTING (ANY CODE REVIEWERS, PLEASE REJECT MERGES
        // TO MASTER IF THIS IS STILL HERE)
        SmartDashboard.putNumber("WristFeedVolts", wristFeedVolts);
        SmartDashboard.putNumber("WristPIDVolts", wristPIDVolts);
        double volts = wristFeedVolts + wristPIDVolts;
        SmartDashboard.putNumber("WristTotalVolts", volts);
        wristMotor.setVoltage(volts);
    }

    public double getArmClampedGoal(double goal) {
        return MathUtil.clamp(MathUtil.inputModulus(goal, ARM_DISCONTINUITY_RAD, ARM_DISCONTINUITY_RAD + 2 * Math.PI), ARM_LOWER_LIMIT_RAD, ARM_UPPER_LIMIT_RAD);
    }
    
    public double getWristClampedGoal(double goal) {
        return MathUtil.clamp(MathUtil.inputModulus(goal, WRIST_DISCONTINUITY_RAD, WRIST_DISCONTINUITY_RAD + 2 * Math.PI), WRIST_LOWER_LIMIT_RAD, WRIST_UPPER_LIMIT_RAD);
    }

    // distance from center of mass relative to joint holding arm
    public Translation2d getCoM() {
        Translation2d comOfArm = new Translation2d(COM_ARM_LENGTH_METERS, Rotation2d.fromRadians(getArmPos()))
                .times(ARM_MASS_KG);
        Translation2d comOfRoller = new Translation2d(ARM_LENGTH_METERS, Rotation2d.fromRadians(getArmPos()))
                .plus(new Translation2d(COM_ROLLER_LENGTH_METERS,
                        Rotation2d.fromRadians(getWristPosRelativeToGround())))
                .times(ROLLER_MASS_KG);
        return comOfArm.plus(comOfRoller).div(ARM_MASS_KG + ROLLER_MASS_KG);
    }

    public double maxHoldingTorqueNM() {
        return (ARM_MASS_KG + ROLLER_MASS_KG) * g * getCoM().getNorm();
    }

    public double getKg() {
        return V_PER_NM * maxHoldingTorqueNM();
    }

    public double getArmPos() {
        return MathUtil.clamp(armEncoder.getPosition(), ARM_DISCONTINUITY_RAD,
                ARM_DISCONTINUITY_RAD + 2 * Math.PI);
    }

    public double getWristPos() {
        return MathUtil.clamp(wristEncoder.getPosition(), ARM_DISCONTINUITY_RAD,
                ARM_DISCONTINUITY_RAD + 2 * Math.PI);
    }

    // Unbounded wrist position relative to ground
    public double getWristPosRelativeToGround() {
        return getArmPos() + wristEncoder.getPosition();
    }

    public void setArmTarget(double target) {
        goalPosRad[ARM] = getArmClampedGoal(target);
    }

    public void setWristTarget(double target) {
        goalPosRad[WRIST] = getWristClampedGoal(target);
    }

    public void setArmWristTarget(double targetArm, double targetWrist) {
        setArmTarget(targetArm);
        setWristTarget(targetWrist);
    }

    public void toggleObjectType() {
        object++;
        object %= 2;
    }

    public void toggleFront() {
        isFront = !isFront;
    }

    public double getArmGoal(GoalPos[] pos) {
        if (isFront) {
            return pos[object].armPos;
        } else {
            return Rotation2d.fromRadians(-Math.PI).minus(Rotation2d.fromRadians(pos[object].armPos)).plus(Rotation2d.fromDegrees(10)).getRadians();
        }
    }

    public double getWristGoal(GoalPos[] pos) {
        if (isFront) {
            return pos[object].wristPos;
        } else {
            return Rotation2d.fromRadians(pos[object].wristPos).unaryMinus().plus(Rotation2d.fromDegrees(10)).getRadians();
        }
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        senb=builder;
        builder.addDoubleProperty("WristGoalPos", () -> goalPosRad[WRIST], goal -> goalPosRad[WRIST] = goal);
        builder.addDoubleProperty("ArmGoalPos",   () -> goalPosRad[ARM], goal -> goalPosRad[ARM] = goal);
        builder.addDoubleProperty("ArmPos",       () -> getArmPos(), null);
        builder.addDoubleProperty("WisPos",       () -> getWristPos(), null);
        builder.addDoubleProperty("RobotWisPos",  () -> getWristPosRelativeToGround(), null);
        builder.addDoubleProperty("kP: Arm", () -> kP[ARM], x -> kP[ARM] = x);
        builder.addDoubleProperty("kP: Wis", () -> kP[WRIST], x -> kP[WRIST] = x);
        builder.addDoubleProperty("kI: Arm", () -> kI[ARM], x -> kI[ARM] = x);
        builder.addDoubleProperty("kI: Wis", () -> kI[WRIST], x -> kI[WRIST] = x);
        builder.addDoubleProperty("kD: Arm", () -> kD[ARM], x -> kD[ARM] = x);
        builder.addDoubleProperty("kD: Wis", () -> kD[WRIST], x -> kD[WRIST] = x);
        //arm control
        builder.addDoubleProperty("Max FF Vel Arm", () -> MAX_FF_VEL[ARM],     x -> MAX_FF_VEL[ARM] = x);
        builder.addDoubleProperty("Max FF Acc Arm", () -> MAX_FF_ACCEL[ARM],   x -> MAX_FF_ACCEL[ARM] = x);
        builder.addDoubleProperty("Max FF Vel Wis", () -> MAX_FF_VEL[WRIST],   x -> MAX_FF_VEL[WRIST] = x);
        builder.addDoubleProperty("Max FF Acc Wis", () -> MAX_FF_ACCEL[WRIST], x -> MAX_FF_ACCEL[WRIST] = x);
        //arm positions
        builder.addDoubleProperty("Arm.High.Cone",  () -> GoalPos.HIGH[CONE].armPos,       x -> GoalPos.HIGH[CONE].armPos = x);
        builder.addDoubleProperty("Arm.Mid.Cone",   () -> GoalPos.MID[CONE].armPos,        x -> GoalPos.MID[CONE].armPos = x);
        builder.addDoubleProperty("Arm.Low.Cone",   () -> GoalPos.LOW[CONE].armPos,        x -> GoalPos.LOW[CONE].armPos = x);
        builder.addDoubleProperty("Arm.Store.Cone", () -> GoalPos.STORED[CONE].armPos,     x -> GoalPos.STORED[CONE].armPos = x);
        builder.addDoubleProperty("Arm.Shelf.Cone", () -> GoalPos.SHELF[CONE].armPos,      x -> GoalPos.SHELF[CONE].armPos = x);
        builder.addDoubleProperty("Arm.Subst.Cone", () -> GoalPos.SUBSTATION[CONE].armPos, x -> GoalPos.SUBSTATION[CONE].armPos = x);
        builder.addDoubleProperty("Arm.Intak.Cone", () -> GoalPos.INTAKE[CONE].armPos,     x -> GoalPos.INTAKE[CONE].armPos = x);
        builder.addDoubleProperty("Arm.High.Cube",  () -> GoalPos.HIGH[CUBE].armPos,       x -> GoalPos.HIGH[CUBE].armPos = x);
        builder.addDoubleProperty("Arm.Mid.Cube",   () -> GoalPos.MID[CUBE].armPos,        x -> GoalPos.MID[CUBE].armPos = x);
        builder.addDoubleProperty("Arm.Low.Cube",   () -> GoalPos.LOW[CUBE].armPos,        x -> GoalPos.LOW[CUBE].armPos = x);
        builder.addDoubleProperty("Arm.Store.Cube", () -> GoalPos.STORED[CUBE].armPos,     x -> GoalPos.STORED[CUBE].armPos = x);
        builder.addDoubleProperty("Arm.Shelf.Cube", () -> GoalPos.SHELF[CUBE].armPos,      x -> GoalPos.SHELF[CUBE].armPos = x);
        builder.addDoubleProperty("Arm.Subst.Cube", () -> GoalPos.SUBSTATION[CUBE].armPos, x -> GoalPos.SUBSTATION[CUBE].armPos = x);
        builder.addDoubleProperty("Arm.Intak.Cube", () -> GoalPos.INTAKE[CUBE].armPos,     x -> GoalPos.INTAKE[CUBE].armPos = x);
        //wrist positions
        builder.addDoubleProperty("Wis.High.Cone",  () -> GoalPos.HIGH[CONE].wristPos,       x -> GoalPos.HIGH[CONE].wristPos = x);
        builder.addDoubleProperty("Wis.Mid.Cone",   () -> GoalPos.MID[CONE].wristPos,        x -> GoalPos.MID[CONE].wristPos = x);
        builder.addDoubleProperty("Wis.Low.Cone",   () -> GoalPos.LOW[CONE].wristPos,        x -> GoalPos.LOW[CONE].wristPos = x);
        builder.addDoubleProperty("Wis.Store.Cone", () -> GoalPos.STORED[CONE].wristPos,     x -> GoalPos.STORED[CONE].wristPos = x);
        builder.addDoubleProperty("Wis.Shelf.Cone", () -> GoalPos.SHELF[CONE].wristPos,      x -> GoalPos.SHELF[CONE].wristPos = x);
        builder.addDoubleProperty("Wis.Subst.Cone", () -> GoalPos.SUBSTATION[CONE].wristPos, x -> GoalPos.SUBSTATION[CONE].wristPos = x);
        builder.addDoubleProperty("Wis.Intak.Cone", () -> GoalPos.INTAKE[CONE].wristPos,     x -> GoalPos.INTAKE[CONE].wristPos = x);
        builder.addDoubleProperty("Wis.High.Cube",  () -> GoalPos.HIGH[CUBE].wristPos,       x -> GoalPos.HIGH[CUBE].wristPos = x);
        builder.addDoubleProperty("Wis.Mid.Cube",   () -> GoalPos.MID[CUBE].wristPos,        x -> GoalPos.MID[CUBE].wristPos = x);
        builder.addDoubleProperty("Wis.Low.Cube",   () -> GoalPos.LOW[CUBE].wristPos,        x -> GoalPos.LOW[CUBE].wristPos = x);
        builder.addDoubleProperty("Wis.Store.Cube", () -> GoalPos.STORED[CUBE].wristPos,     x -> GoalPos.STORED[CUBE].wristPos = x);
        builder.addDoubleProperty("Wis.Shelf.Cube", () -> GoalPos.SHELF[CUBE].wristPos,      x -> GoalPos.SHELF[CUBE].wristPos = x);
        builder.addDoubleProperty("Wis.Subst.Cube", () -> GoalPos.SUBSTATION[CUBE].wristPos, x -> GoalPos.SUBSTATION[CUBE].wristPos = x);
        builder.addDoubleProperty("Wis.Intak.Cube", () -> GoalPos.INTAKE[CUBE].wristPos,     x -> GoalPos.INTAKE[CUBE].wristPos = x);
    }


}