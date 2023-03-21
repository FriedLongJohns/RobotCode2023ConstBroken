package org.carlmontrobotics.robotcode2023.subsystems;

import static org.carlmontrobotics.robotcode2023.Constants.Arm.*;

import org.carlmontrobotics.MotorConfig;
import org.carlmontrobotics.lib199.MotorControllerFactory;
import org.carlmontrobotics.robotcode2023.Constants.GoalPos;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Arm angle is measured from horizontal on the intake side of the robot and bounded between -3π/2 and π/2
// Wrist angle is measured relative to the arm with 0 being parallel to the arm and bounded between -π and π (Center of Mass of Roller)
public class Arm extends SubsystemBase {

    private final CANSparkMax armMotor = MotorControllerFactory.createSparkMax(armMotorPort, MotorConfig.NEO);
    private final CANSparkMax wristMotor = MotorControllerFactory.createSparkMax(wristMotorPort, MotorConfig.NEO);
    private final RelativeEncoder armRelEncoder = armMotor.getEncoder();
    private final SparkMaxAbsoluteEncoder armEncoder = armMotor
            .getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
    private final SparkMaxAbsoluteEncoder wristEncoder = wristMotor
            .getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);

    private final SimpleMotorFeedforward armFeed = new SimpleMotorFeedforward(kS[ARM], kV[ARM], kA[ARM]);
    private ArmFeedforward wristFeed = new ArmFeedforward(kS[WRIST], kG[WRIST], kV[WRIST], kA[WRIST]);

    private final PIDController armPID = new PIDController(kP[ARM], kI[ARM], kD[ARM]);
    private final PIDController wristPID = new PIDController(kP[WRIST], kI[WRIST], kD[WRIST]);

    private TrapezoidProfile armProfile = new TrapezoidProfile(armConstraints, goalState[ARM], getCurrentArmState());
    private TrapezoidProfile wristProfile = new TrapezoidProfile(wristConstraints, goalState[WRIST], getCurrentWristState());
    private Timer armProfileTimer = new Timer();
    private Timer wristProfileTimer = new Timer();

    // rad, rad/s
    public static TrapezoidProfile.State[] goalState = { new TrapezoidProfile.State(-Math.PI / 2, 0), new TrapezoidProfile.State(0, 0) };

    public Arm() {
        armMotor.setInverted(inverted[ARM]);
        wristMotor.setInverted(inverted[WRIST]);

        armEncoder.setPositionConversionFactor(rotationToRad);
        wristEncoder.setPositionConversionFactor(rotationToRad);
        armEncoder.setVelocityConversionFactor(rotationToRad);
        wristEncoder.setVelocityConversionFactor(rotationToRad);

        armEncoder.setZeroOffset(offsetRad[ARM]);
        wristEncoder.setZeroOffset(offsetRad[WRIST]);

        armPID.setTolerance(posToleranceRad[ARM], velToleranceRadPSec[ARM]);
        wristPID.setTolerance(posToleranceRad[WRIST], velToleranceRadPSec[WRIST]);

        SmartDashboard.putData("Arm", this);

        armProfileTimer.start();
        wristProfileTimer.start();

        setArmTarget(goalState[ARM].position, 0);
        setWristTarget(goalState[WRIST].position, 0);
        wristMotor.setSmartCurrentLimit(WRIST_CURRENT_LIMIT);
    }

    @Override
    public void periodic() {

        // TODO: REMOVE THIS WHEN PID CONSTANTS ARE DONE
        wristConstraints = new TrapezoidProfile.Constraints(MAX_FF_VEL[WRIST], MAX_FF_ACCEL[WRIST]);
        armConstraints = new TrapezoidProfile.Constraints(MAX_FF_VEL[ARM], MAX_FF_ACCEL[ARM]);
        armPID.setP(kP[ARM]);
        armPID.setI(kI[ARM]);
        armPID.setD(kD[ARM]);
        wristPID.setP(kP[WRIST]);
        wristPID.setI(kI[WRIST]);
        wristPID.setD(kD[WRIST]);

        SmartDashboard.putNumber("MaxHoldingTorque", maxHoldingTorqueNM());
        SmartDashboard.putNumber("V_PER_NM", getV_PER_NM());
        SmartDashboard.putNumber("COMDistance", getCoM().getNorm());
        SmartDashboard.putNumber("InternalArmVelocity", armRelEncoder.getVelocity());
        SmartDashboard.putNumber("Arm Current", armMotor.getOutputCurrent());
        SmartDashboard.putNumber("Wrist Current", wristMotor.getOutputCurrent());

        driveArm(armProfile.calculate(armProfileTimer.get()));
        driveWrist(wristProfile.calculate(wristProfileTimer.get()));
    }

    //#region Drive Methods

    private void driveArm(TrapezoidProfile.State state) {
        double kgv = getKg();
        double armFeedVolts = kgv * getCoM().getAngle().getCos() + armFeed.calculate(state.velocity, 0);
        double armPIDVolts = armPID.calculate(getArmPos(), state.position);
        if ((getArmPos() > ARM_UPPER_LIMIT_RAD && state.velocity > 0) || 
            (getArmPos() < ARM_LOWER_LIMIT_RAD && state.velocity < 0)) {
            armFeedVolts = kgv * getCoM().getAngle().getCos() + armFeed.calculate(0, 0);
        }
        // TODO: REMOVE WHEN DONE WITH TESTING (ANY CODE REVIEWERS, PLEASE REJECT MERGES
        // TO MASTER IF THIS IS STILL HERE)
        SmartDashboard.putNumber("ArmFeedVolts", armFeedVolts);
        SmartDashboard.putNumber("ArmPIDVolts", armPIDVolts);
        double volts = armFeedVolts + armPIDVolts;
        SmartDashboard.putNumber("ArmTotalVolts", volts);
        armMotor.setVoltage(volts);
    }

    private void driveWrist(TrapezoidProfile.State state) {
        double kgv = wristFeed.calculate(getWristPosRelativeToGround(), state.velocity, 0);
        double wristFeedVolts = wristFeed.calculate(getWristPosRelativeToGround(), state.velocity, 0);
        double wristPIDVolts = wristPID.calculate(getWristPos(), state.position);
        if ((getWristPos() > WRIST_UPPER_LIMIT_RAD && state.velocity > 0) || 
            (getWristPos() < WRIST_LOWER_LIMIT_RAD && state.velocity < 0)) {
            wristFeedVolts = kgv;
        }
        // TODO: REMOVE WHEN DONE WITH TESTING (ANY CODE REVIEWERS, PLEASE REJECT MERGES
        // TO MASTER IF THIS IS STILL HERE)
        SmartDashboard.putNumber("WristFeedVolts", wristFeedVolts);
        SmartDashboard.putNumber("WristPIDVolts", wristPIDVolts);
        double volts = wristFeedVolts + wristPIDVolts;
        SmartDashboard.putNumber("WristTotalVolts", volts);
        wristMotor.setVoltage(volts);
    }

    public void setArmTarget(double targetPos, double targetVel) {
        armProfile = new TrapezoidProfile(armConstraints, new TrapezoidProfile.State(targetPos, targetVel), armProfile.calculate(armProfileTimer.get()));
        armProfileTimer.reset();

        goalState[ARM].position = getArmClampedGoal(targetPos);
        goalState[ARM].velocity = targetVel;
    }

    public void setWristTarget(double targetPos, double targetVel) {
        wristProfile = new TrapezoidProfile(wristConstraints, new TrapezoidProfile.State(targetPos, targetVel), wristProfile.calculate(wristProfileTimer.get()));
        wristProfileTimer.reset();

        goalState[WRIST].position = getWristClampedGoal(targetPos);
        goalState[WRIST].velocity = targetVel;
    }

    public void resetGoal() {
        setArmTarget(getArmPos(), 0);
        setWristTarget(getWristPos(), 0);
        armProfile = new TrapezoidProfile(armConstraints, new TrapezoidProfile.State(getArmPos(), 0), new TrapezoidProfile.State(getArmPos(), 0));
        wristProfile = new TrapezoidProfile(wristConstraints, new TrapezoidProfile.State(getWristPos(), 0), new TrapezoidProfile.State(getWristPos(), 0));
    }

    //#endregion

    //#region Getters

    public double getArmPos() {
        return MathUtil.inputModulus(armEncoder.getPosition(), ARM_DISCONTINUITY_RAD,
                ARM_DISCONTINUITY_RAD + 2 * Math.PI);
    }

    public double getArmVel() {
        return armEncoder.getVelocity();
    }

    public double getWristPos() {
        return MathUtil.inputModulus(wristEncoder.getPosition(), WRIST_DISCONTINUITY_RAD,
                WRIST_DISCONTINUITY_RAD + 2 * Math.PI);
    }

    // Unbounded wrist position relative to ground
    public double getWristPosRelativeToGround() {
        return getArmPos() + wristEncoder.getPosition();
    }

    public double getWristVel() {
        return wristEncoder.getVelocity();
    }

    public TrapezoidProfile.State getCurrentArmState() {
        return new TrapezoidProfile.State(getArmPos(), getArmVel());
    }

    public TrapezoidProfile.State getCurrentWristState() {
        return new TrapezoidProfile.State(getWristPos(), getWristVel());
    }

    public TrapezoidProfile.State getCurrentArmGoal() {
        return goalState[ARM];
    }

    public TrapezoidProfile.State getCurrentWristGoal() {
        return goalState[WRIST];
    }

    public boolean armAtSetpoint() {
        return armPID.atSetpoint();
    }

    public boolean wristAtSetpoint() {
        return wristPID.atSetpoint();
    }

    //#endregion

    //#region Util Methods

    public double getArmClampedGoal(double goal) {
        return MathUtil.clamp(MathUtil.inputModulus(goal, ARM_DISCONTINUITY_RAD, ARM_DISCONTINUITY_RAD + 2 * Math.PI), ARM_LOWER_LIMIT_RAD, ARM_UPPER_LIMIT_RAD);
    }

    public double getWristClampedGoal(double goal) {
        return MathUtil.clamp(MathUtil.inputModulus(goal, WRIST_DISCONTINUITY_RAD, WRIST_DISCONTINUITY_RAD + 2 * Math.PI), WRIST_LOWER_LIMIT_RAD, WRIST_UPPER_LIMIT_RAD);
    }

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

    public double getV_PER_NM() {
        double kg = kG[ARM];
        double phi = 2.638;
        double Ma = ARM_MASS_KG;
        double Mr = ROLLER_MASS_KG;
        double Ra = ARM_LENGTH_METERS;
        double Rr = COM_ROLLER_LENGTH_METERS;
        double PaRa = COM_ARM_LENGTH_METERS;
        double g = 9.80;

        double c = (kg) / (g * Math.sqrt(Math.pow(Ma * PaRa + Mr * Ra, 2) + Math.pow(Mr * Rr, 2) + 2 * (Ma * PaRa + Mr * Ra) * (Mr * Rr) * Math.cos(phi)));
        return c;
    }

    public double getKg() {
        return getV_PER_NM() * maxHoldingTorqueNM();
    }

    //#endregion

    //#region SmartDashboard Methods

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("WristGoalPos", () -> goalState[WRIST].position, goal -> goalState[WRIST].position = goal);
        builder.addDoubleProperty("ArmGoalPos",   () -> goalState[ARM].position, goal -> goalState[ARM].position = goal);
        builder.addDoubleProperty("ArmPos",       () -> getArmPos(), null);
        builder.addDoubleProperty("WristPos",       () -> getWristPos(), null);
        builder.addDoubleProperty("RobotWristPos",  () -> getWristPosRelativeToGround(), null);
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
        builder.addDoubleProperty("Arm.Back.High.Cone",  () -> GoalPos.HIGH[BACK][CONE].armPos,       x -> GoalPos.HIGH[BACK][CONE].armPos = x);
        builder.addDoubleProperty("Arm.Back.Mid.Cone",   () -> GoalPos.MID[BACK][CONE].armPos,        x -> GoalPos.MID[BACK][CONE].armPos = x);
        builder.addDoubleProperty("Arm.Back.Low.Cone",   () -> GoalPos.LOW[BACK][CONE].armPos,        x -> GoalPos.LOW[BACK][CONE].armPos = x);
        builder.addDoubleProperty("Arm.Back.Store.Cone", () -> GoalPos.STORED[BACK][CONE].armPos,     x -> GoalPos.STORED[BACK][CONE].armPos = x);
        builder.addDoubleProperty("Arm.Back.Shelf.Cone", () -> GoalPos.SHELF[BACK][CONE].armPos,      x -> GoalPos.SHELF[BACK][CONE].armPos = x);
        builder.addDoubleProperty("Arm.Back.Subst.Cone", () -> GoalPos.SUBSTATION[BACK][CONE].armPos, x -> GoalPos.SUBSTATION[BACK][CONE].armPos = x);
        builder.addDoubleProperty("Arm.Back.Intak.Cone", () -> GoalPos.INTAKE[BACK][CONE].armPos,     x -> GoalPos.INTAKE[BACK][CONE].armPos = x);
        builder.addDoubleProperty("Arm.Back.High.Cube",  () -> GoalPos.HIGH[BACK][CUBE].armPos,       x -> GoalPos.HIGH[BACK][CUBE].armPos = x);
        builder.addDoubleProperty("Arm.Back.Mid.Cube",   () -> GoalPos.MID[BACK][CUBE].armPos,        x -> GoalPos.MID[BACK][CUBE].armPos = x);
        builder.addDoubleProperty("Arm.Back.Low.Cube",   () -> GoalPos.LOW[BACK][CUBE].armPos,        x -> GoalPos.LOW[BACK][CUBE].armPos = x);
        builder.addDoubleProperty("Arm.Back.Store.Cube", () -> GoalPos.STORED[BACK][CUBE].armPos,     x -> GoalPos.STORED[BACK][CUBE].armPos = x);
        builder.addDoubleProperty("Arm.Back.Shelf.Cube", () -> GoalPos.SHELF[BACK][CUBE].armPos,      x -> GoalPos.SHELF[BACK][CUBE].armPos = x);
        builder.addDoubleProperty("Arm.Back.Subst.Cube", () -> GoalPos.SUBSTATION[BACK][CUBE].armPos, x -> GoalPos.SUBSTATION[BACK][CUBE].armPos = x);
        builder.addDoubleProperty("Arm.Back.Intak.Cube", () -> GoalPos.INTAKE[BACK][CUBE].armPos,     x -> GoalPos.INTAKE[BACK][CUBE].armPos = x);
        //wrist positions
        builder.addDoubleProperty("Wrist.Back.High.Cone",  () -> GoalPos.HIGH[BACK][CONE].wristPos,       x -> GoalPos.HIGH[BACK][CONE].wristPos = x);
        builder.addDoubleProperty("Wrist.Back.Mid.Cone",   () -> GoalPos.MID[BACK][CONE].wristPos,        x -> GoalPos.MID[BACK][CONE].wristPos = x);
        builder.addDoubleProperty("Wrist.Back.Low.Cone",   () -> GoalPos.LOW[BACK][CONE].wristPos,        x -> GoalPos.LOW[BACK][CONE].wristPos = x);
        builder.addDoubleProperty("Wrist.Back.Store.Cone", () -> GoalPos.STORED[BACK][CONE].wristPos,     x -> GoalPos.STORED[BACK][CONE].wristPos = x);
        builder.addDoubleProperty("Wrist.Back.Shelf.Cone", () -> GoalPos.SHELF[BACK][CONE].wristPos,      x -> GoalPos.SHELF[BACK][CONE].wristPos = x);
        builder.addDoubleProperty("Wrist.Back.Substation.Cone", () -> GoalPos.SUBSTATION[BACK][CONE].wristPos, x -> GoalPos.SUBSTATION[BACK][CONE].wristPos = x);
        builder.addDoubleProperty("Wrist.Back.Intake.Cone", () -> GoalPos.INTAKE[BACK][CONE].wristPos,     x -> GoalPos.INTAKE[BACK][CONE].wristPos = x);
        builder.addDoubleProperty("Wrist.Back.High.Cube",  () -> GoalPos.HIGH[BACK][CUBE].wristPos,       x -> GoalPos.HIGH[BACK][CUBE].wristPos = x);
        builder.addDoubleProperty("Wrist.Back.Mid.Cube",   () -> GoalPos.MID[BACK][CUBE].wristPos,        x -> GoalPos.MID[BACK][CUBE].wristPos = x);
        builder.addDoubleProperty("Wrist.Back.Low.Cube",   () -> GoalPos.LOW[BACK][CUBE].wristPos,        x -> GoalPos.LOW[BACK][CUBE].wristPos = x);
        builder.addDoubleProperty("Wrist.Back.Store.Cube", () -> GoalPos.STORED[BACK][CUBE].wristPos,     x -> GoalPos.STORED[BACK][CUBE].wristPos = x);
        builder.addDoubleProperty("Wrist.Back.Shelf.Cube", () -> GoalPos.SHELF[BACK][CUBE].wristPos,      x -> GoalPos.SHELF[BACK][CUBE].wristPos = x);
        builder.addDoubleProperty("Wrist.Back.Substation.Cube", () -> GoalPos.SUBSTATION[BACK][CUBE].wristPos, x -> GoalPos.SUBSTATION[BACK][CUBE].wristPos = x);
        builder.addDoubleProperty("Wrist.Back.Intake.Cube", () -> GoalPos.INTAKE[BACK][CUBE].wristPos,     x -> GoalPos.INTAKE[BACK][CUBE].wristPos = x);
    }

    // In the scenario that initSendable method does not work like last time
    public void putPositionsOnSmartDashboard() {
        SmartDashboard.putNumber("LowBackArmCube", GoalPos.LOW[BACK][ARM].armPos);
        SmartDashboard.putNumber("LowBackWristCube", GoalPos.LOW[BACK][ARM].wristPos);
        SmartDashboard.putNumber("MidBackArmCube", GoalPos.MID[BACK][ARM].armPos);
        SmartDashboard.putNumber("MidBackWristCube", GoalPos.MID[BACK][ARM].wristPos);
        SmartDashboard.putNumber("HighBackArmCube", GoalPos.HIGH[BACK][ARM].armPos);
        SmartDashboard.putNumber("HighBackWristCube", GoalPos.HIGH[BACK][ARM].wristPos);
        SmartDashboard.putNumber("StoredBackArmCube", GoalPos.STORED[BACK][ARM].armPos);
        SmartDashboard.putNumber("StoredBackWristCube", GoalPos.STORED[BACK][ARM].wristPos);
        SmartDashboard.putNumber("ShelfBackArmCube", GoalPos.SHELF[BACK][ARM].armPos);
        SmartDashboard.putNumber("ShelfBackWristCube", GoalPos.SHELF[BACK][ARM].wristPos);
        SmartDashboard.putNumber("SubstationBackArmCube", GoalPos.SUBSTATION[BACK][ARM].armPos);
        SmartDashboard.putNumber("SubstationBackWristCube", GoalPos.SUBSTATION[BACK][ARM].wristPos);
        SmartDashboard.putNumber("IntakeBackArmCube", GoalPos.INTAKE[BACK][ARM].armPos);
        SmartDashboard.putNumber("IntakeBackWristCube", GoalPos.INTAKE[BACK][ARM].wristPos);
        SmartDashboard.putNumber("LowBackArmCone", GoalPos.LOW[BACK][WRIST].armPos);
        SmartDashboard.putNumber("LowBackWristCone", GoalPos.LOW[BACK][WRIST].wristPos);
        SmartDashboard.putNumber("MidBackArmCone", GoalPos.MID[BACK][WRIST].armPos);
        SmartDashboard.putNumber("MidBackWristCone", GoalPos.MID[BACK][WRIST].wristPos);
        SmartDashboard.putNumber("HighBackArmCone", GoalPos.HIGH[BACK][WRIST].armPos);
        SmartDashboard.putNumber("HighBackWristCone", GoalPos.HIGH[BACK][WRIST].wristPos);
        SmartDashboard.putNumber("StoredBackArmCone", GoalPos.STORED[BACK][WRIST].armPos);
        SmartDashboard.putNumber("StoredBackWristCone", GoalPos.STORED[BACK][WRIST].wristPos);
        SmartDashboard.putNumber("ShelfBackArmCone", GoalPos.SHELF[BACK][WRIST].armPos);
        SmartDashboard.putNumber("ShelfBackWristCone", GoalPos.SHELF[BACK][WRIST].wristPos);
        SmartDashboard.putNumber("SubstationBackArmCone", GoalPos.SUBSTATION[BACK][WRIST].armPos);
        SmartDashboard.putNumber("SubstationBackWristCone", GoalPos.SUBSTATION[BACK][WRIST].wristPos);
        SmartDashboard.putNumber("IntakeBackArmCone", GoalPos.INTAKE[BACK][WRIST].armPos);
        SmartDashboard.putNumber("IntakeBackWristCone", GoalPos.INTAKE[BACK][WRIST].wristPos);
    }

    public void getPositionsOnSmartDashboard() {
        GoalPos.LOW[BACK][CUBE].armPos = SmartDashboard.getNumber("LowArmCube", GoalPos.LOW[BACK][CUBE].armPos);
        GoalPos.LOW[BACK][CUBE].wristPos = SmartDashboard.getNumber("LowWristCube", GoalPos.LOW[BACK][CUBE].wristPos);
        GoalPos.MID[BACK][CUBE].armPos = SmartDashboard.getNumber("MidArmCube", GoalPos.MID[BACK][CUBE].armPos);
        GoalPos.MID[BACK][CUBE].wristPos = SmartDashboard.getNumber("MidWristCube", GoalPos.MID[BACK][CUBE].wristPos);
        GoalPos.HIGH[BACK][CUBE].armPos = SmartDashboard.getNumber("HighArmCube", GoalPos.HIGH[BACK][CUBE].armPos);
        GoalPos.HIGH[BACK][CUBE].wristPos = SmartDashboard.getNumber("HighWristCube", GoalPos.HIGH[BACK][CUBE].wristPos);
        GoalPos.STORED[BACK][CUBE].armPos = SmartDashboard.getNumber("StoredArmCube", GoalPos.STORED[BACK][CUBE].armPos);
        GoalPos.STORED[BACK][CUBE].wristPos = SmartDashboard.getNumber("StoredWristCube", GoalPos.STORED[BACK][CUBE].wristPos);
        GoalPos.SHELF[BACK][CUBE].armPos = SmartDashboard.getNumber("ShelfArmCube", GoalPos.SHELF[BACK][CUBE].armPos);
        GoalPos.SHELF[BACK][CUBE].wristPos = SmartDashboard.getNumber("ShelfWristCube", GoalPos.SHELF[BACK][CUBE].wristPos);
        GoalPos.SUBSTATION[BACK][CUBE].armPos = SmartDashboard.getNumber("SubstationArmCube",
                GoalPos.SUBSTATION[BACK][CUBE].armPos);
        GoalPos.SUBSTATION[BACK][CUBE].wristPos = SmartDashboard.getNumber("SubstationWristCube",
                GoalPos.SUBSTATION[BACK][CUBE].wristPos);
        GoalPos.INTAKE[BACK][CUBE].armPos = SmartDashboard.getNumber("IntakeArmCube", GoalPos.INTAKE[BACK][CUBE].armPos);
        GoalPos.INTAKE[BACK][CUBE].wristPos = SmartDashboard.getNumber("IntakeWristCube", GoalPos.INTAKE[BACK][CUBE].wristPos);
        GoalPos.LOW[BACK][CONE].armPos = SmartDashboard.getNumber("LowArmCone", GoalPos.LOW[BACK][CONE].armPos);
        GoalPos.LOW[BACK][CONE].wristPos = SmartDashboard.getNumber("LowWristCone", GoalPos.LOW[BACK][CONE].wristPos);
        GoalPos.MID[BACK][CONE].armPos = SmartDashboard.getNumber("MidArmCone", GoalPos.MID[BACK][CONE].armPos);
        GoalPos.MID[BACK][CONE].wristPos = SmartDashboard.getNumber("MidWristCone", GoalPos.MID[BACK][CONE].wristPos);
        GoalPos.HIGH[BACK][CONE].armPos = SmartDashboard.getNumber("HighArmCone", GoalPos.HIGH[BACK][CONE].armPos);
        GoalPos.HIGH[BACK][CONE].wristPos = SmartDashboard.getNumber("HighWristCone", GoalPos.HIGH[BACK][CONE].wristPos);
        GoalPos.STORED[BACK][CONE].armPos = SmartDashboard.getNumber("StoredArmCone", GoalPos.STORED[BACK][CONE].armPos);
        GoalPos.STORED[BACK][CONE].wristPos = SmartDashboard.getNumber("StoredWristCone", GoalPos.STORED[BACK][CONE].wristPos);
        GoalPos.SHELF[BACK][CONE].armPos = SmartDashboard.getNumber("ShelfArmCone", GoalPos.SHELF[BACK][CONE].armPos);
        GoalPos.SHELF[BACK][CONE].wristPos = SmartDashboard.getNumber("ShelfWristCone", GoalPos.SHELF[BACK][CONE].wristPos);
        GoalPos.SUBSTATION[BACK][CONE].armPos = SmartDashboard.getNumber("SubstationArmCone",
                GoalPos.SUBSTATION[BACK][CONE].armPos);
        GoalPos.SUBSTATION[BACK][CONE].wristPos = SmartDashboard.getNumber("SubstationWristCone",
                GoalPos.SUBSTATION[BACK][CONE].wristPos);
        GoalPos.INTAKE[BACK][CONE].armPos = SmartDashboard.getNumber("IntakeArmCone", GoalPos.INTAKE[BACK][CONE].armPos);
        GoalPos.INTAKE[BACK][CONE].wristPos = SmartDashboard.getNumber("IntakeWristCone", GoalPos.INTAKE[BACK][CONE].wristPos);
    }

    public void putArmControlOnSmartDashboard() {
        SmartDashboard.putNumber("kP: Wis", kP[WRIST]);
        SmartDashboard.putNumber("kI: Wis", kI[WRIST]);
        SmartDashboard.putNumber("kD: Wis", kD[WRIST]);
        SmartDashboard.putNumber("kP: Arm", kP[ARM]);
        SmartDashboard.putNumber("kI: Arm", kI[ARM]);
        SmartDashboard.putNumber("kD: Arm", kD[ARM]);
        SmartDashboard.putNumber("WristGoalDeg", Units.radiansToDegrees(goalState[WRIST].position));
        SmartDashboard.putNumber("ArmGoalDeg", Units.radiansToDegrees(goalState[ARM].position));
        SmartDashboard.putNumber("Max FF Vel Arm", MAX_FF_VEL[ARM]);
        SmartDashboard.putNumber("Max FF Accel Arm", MAX_FF_ACCEL[ARM]);
    }

    public void getArmControlOnSmartDashboard() {
        goalState[WRIST].position = Units.degreesToRadians(SmartDashboard.getNumber("WristGoalDeg", Units.radiansToDegrees(goalState[WRIST].position)));
        goalState[ARM].position = Units.degreesToRadians(SmartDashboard.getNumber("ArmGoalDeg", Units.radiansToDegrees(goalState[ARM].position)));
        MAX_FF_VEL[ARM] = SmartDashboard.getNumber("Max FF Vel Arm", MAX_FF_VEL[ARM]);
        MAX_FF_ACCEL[ARM] = SmartDashboard.getNumber("Max FF Accel Arm", MAX_FF_ACCEL[ARM]);
        kP[WRIST] = SmartDashboard.getNumber("kP: Wis", kP[WRIST]);
        kI[WRIST] = SmartDashboard.getNumber("kI: Wis", kI[WRIST]);
        kD[WRIST] = SmartDashboard.getNumber("kD: Wis", kD[WRIST]);
        kP[ARM] = SmartDashboard.getNumber("kP: Arm", kP[ARM]);
        kI[ARM] = SmartDashboard.getNumber("kI: Arm", kI[ARM]);
        kD[ARM] = SmartDashboard.getNumber("kD: Arm", kD[ARM]);
    }

    //#endregion

}
