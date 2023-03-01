package org.carlmontrobotics.robotcode2023.subsystems;

import static org.carlmontrobotics.robotcode2023.Constants.Drivetrain.*;

import java.util.Arrays;
import java.util.function.Supplier;

import org.carlmontrobotics.lib199.Limelight;
import org.carlmontrobotics.lib199.MotorControllerFactory;
import org.carlmontrobotics.lib199.MotorErrors.TemperatureLimit;
import org.carlmontrobotics.lib199.path.SwerveDriveInterface;
import org.carlmontrobotics.lib199.swerve.SwerveModule;
import org.carlmontrobotics.robotcode2023.commands.TeleopDrive;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase implements SwerveDriveInterface {
    private final AHRS gyro = new AHRS(SerialPort.Port.kMXP); // Also try kUSB and kUSB2

    private SwerveDriveKinematics kinematics = null;
    private SwerveDrivePoseEstimator odometry = null;
    private SwerveModule modules[];
    private Limelight lime;
    private boolean fieldOriented = true;
    private double fieldOffset = 0;

    public final float initPitch;
    public final float initRoll;

    private double prevAngle = 0;
    private double prevTime = 0;

    public Drivetrain(Limelight lime) {
        this.lime = lime;

        // Calibrate Gyro
        {
            gyro.calibrate();
            double initTimestamp = Timer.getFPGATimestamp();
            double currentTimestamp = initTimestamp;
            while (gyro.isCalibrating() && currentTimestamp - initTimestamp < 10) {
                currentTimestamp = Timer.getFPGATimestamp();
                try {
                    Thread.sleep(1000);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                    break;
                }
                System.out.println("Calibrating the gyro...");
            }
            gyro.reset();
            System.out.println("NavX-MXP firmware version: " + gyro.getFirmwareVersion());
            System.out.println("Magnetometer is calibrated: " + gyro.isMagnetometerCalibrated());
        }

        // Setup Kinematics
        {
            // Define the corners of the robot relative to the center of the robot using
            // Translation2d objects.
            // Positive x-values represent moving toward the front of the robot whereas
            // positive y-values represent moving toward the left of the robot.
            Translation2d locationFL = new Translation2d(wheelBase / 2, trackWidth / 2);
            Translation2d locationFR = new Translation2d(wheelBase / 2, -trackWidth / 2);
            Translation2d locationBL = new Translation2d(-wheelBase / 2, trackWidth / 2);
            Translation2d locationBR = new Translation2d(-wheelBase / 2, -trackWidth / 2);

            kinematics = new SwerveDriveKinematics(locationFL, locationFR, locationBL, locationBR);
        }

        // Initialize modules
        {
            initPitch = 0;
            initRoll = 0;
            Supplier<Float> pitchSupplier = () -> initPitch;
            Supplier<Float> rollSupplier = () -> initRoll;
            // initPitch = gyro.getPitch();
            // initRoll = gyro.getRoll();
            // Supplier<Float> pitchSupplier = () -> gyro.getPitch();
            // Supplier<Float> rollSupplier = () -> gyro.getRoll();

            SwerveModule moduleFL = new SwerveModule(swerveConfig, SwerveModule.ModuleType.FL,
                    MotorControllerFactory.createSparkMax(driveFrontLeftPort, TemperatureLimit.NEO),
                    MotorControllerFactory.createSparkMax(turnFrontLeftPort, TemperatureLimit.NEO),
                    MotorControllerFactory.createCANCoder(canCoderPortFL), 0,
                    pitchSupplier, rollSupplier);
            // Forward-Right
            SwerveModule moduleFR = new SwerveModule(swerveConfig, SwerveModule.ModuleType.FR,
                    MotorControllerFactory.createSparkMax(driveFrontRightPort, TemperatureLimit.NEO),
                    MotorControllerFactory.createSparkMax(turnFrontRightPort, TemperatureLimit.NEO),
                    MotorControllerFactory.createCANCoder(canCoderPortFR), 1,
                    pitchSupplier, rollSupplier);
            // Backward-Left
            SwerveModule moduleBL = new SwerveModule(swerveConfig, SwerveModule.ModuleType.BL,
                    MotorControllerFactory.createSparkMax(driveBackLeftPort, TemperatureLimit.NEO),
                    MotorControllerFactory.createSparkMax(turnBackLeftPort, TemperatureLimit.NEO),
                    MotorControllerFactory.createCANCoder(canCoderPortBL), 2,
                    pitchSupplier, rollSupplier);
            // Backward-Right
            SwerveModule moduleBR = new SwerveModule(swerveConfig, SwerveModule.ModuleType.BR,
                    MotorControllerFactory.createSparkMax(driveBackRightPort, TemperatureLimit.NEO),
                    MotorControllerFactory.createSparkMax(turnBackRightPort, TemperatureLimit.NEO),
                    MotorControllerFactory.createCANCoder(canCoderPortBR), 3,
                    pitchSupplier, rollSupplier);
            modules = new SwerveModule[] { moduleFL, moduleFR, moduleBL, moduleBR };
        }
        SmartDashboard.putNumber("kpTheta", thetaPIDController[0]);
        odometry = new SwerveDrivePoseEstimator(kinematics, Rotation2d.fromDegrees(getHeading()), getModulePositions(), new Pose2d());
        prevTime = Timer.getFPGATimestamp();
    }

    @Override
    public void periodic() {
        for (SwerveModule module : modules) module.periodic();

        // Update the odometry with current heading and encoder position
        odometry.update(Rotation2d.fromDegrees(getHeading()), getModulePositions());

        // Update the odometry with limelight
        if(lime != null && lime.hasTarget()) {
            odometry.addVisionMeasurement(lime.getTransform(Limelight.Transform.BOTPOSE).toPose2d(), WPIUtilJNI.now() * 1.0e-6 /* From the odometry.update implementation */);
        }

        // SmartDashboard.putNumber("Odometry X",
        // odometry.getPoseMeters().getTranslation().getX());
        // SmartDashboard.putNumber("Odometry Y",
        // odometry.getPoseMeters().getTranslation().getY());;
        //SmartDashboard.putNumber("Pitch", gyro.getPitch());
        //SmartDashboard.putNumber("Roll", gyro.getRoll());
       // SmartDashboard.putNumber("Raw gyro angle", gyro.getAngle());
        SmartDashboard.putNumber("Robot Heading", getHeading());
        fieldOriented = SmartDashboard.getBoolean("Field Oriented", true);
        // SmartDashboard.putNumber("Gyro Compass Heading", gyro.getCompassHeading());
        // SmartDashboard.putNumber("Compass Offset", compassOffset);
        // SmartDashboard.putBoolean("Current Magnetic Field Disturbance",
        // gyro.isMagneticDisturbance());

        // drive to point testing

        // PID testing for rotations
        thetaPIDController[0] = SmartDashboard.getNumber("kpTheta", thetaPIDController[0]);
        double temp = gyro.getAngle();
        double tempTime = Timer.getFPGATimestamp();
        SmartDashboard.putNumber("Angular Vel", Units.degreesToRadians(temp - prevAngle) / (tempTime - prevTime));
        prevAngle = temp;
        prevTime = tempTime;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);

        for(SwerveModule module : modules) SendableRegistry.addChild(this, module);

        builder.addBooleanProperty("Magnetic Field Disturbance", gyro::isMagneticDisturbance, null);
        builder.addBooleanProperty("Gyro Calibrating", gyro::isCalibrating, null);
        builder.addBooleanProperty("Field Oriented", () -> fieldOriented, fieldOriented -> this.fieldOriented = fieldOriented);
        builder.addDoubleProperty("Odometry X", () -> getPose().getX(), null);
        builder.addDoubleProperty("Odometry Y", () -> getPose().getY(), null);
        builder.addDoubleProperty("Odometry Heading", () -> getPose().getRotation().getDegrees(), null);
        builder.addDoubleProperty("Robot Heading", () -> getHeading(), null);
        builder.addDoubleProperty("Raw Gyro Angle", gyro::getAngle, null);
        builder.addDoubleProperty("Pitch", gyro::getPitch, null);
        builder.addDoubleProperty("Roll", gyro::getRoll, null);
        builder.addDoubleProperty("Field Offset", () -> fieldOffset, fieldOffset -> this.fieldOffset = fieldOffset);
    }

    //#region Drive Methods

    /**
     * Drives the robot using the given x, y, and rotation speed
     * 
     * @param forward  The desired forward speed, in m/s. Forward is positive.
     * @param strafe   The desired strafe speed, in m/s. Left is positive.
     * @param rotation The desired rotation speed, in rad/s. Counter clockwise is positive
     */
    public void drive(double forward, double strafe, double rotation) {
        drive(getSwerveStates(forward, strafe, rotation));
    }

    @Override
    public void drive(SwerveModuleState[] moduleStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, maxSpeed);

        // Move the modules based on desired (normalized) speed, desired angle, max
        // speed, drive modifier, and whether or not to reverse turning.
        for (int i = 0; i < 4; i++) {
            moduleStates[i] = SwerveModuleState.optimize(moduleStates[i],
                    Rotation2d.fromDegrees(modules[i].getModuleAngle()));
            modules[i].move(moduleStates[i].speedMetersPerSecond, moduleStates[i].angle.getDegrees());
        }
    }

    @Override
    public void stop() {
        for(SwerveModule module: modules) module.move(0, 0);
    }

    /**
     * Constructs and returns a ChassisSpeeds objects using forward, strafe, and
     * rotation values.
     * 
     * @param forward  The desired forward speed, in m/s. Forward is positive.
     * @param strafe   The desired strafe speed, in m/s. Left is positive.
     * @param rotation The desired rotation speed, in rad/s. Counter clockwise is positive.
     * @return A ChassisSpeeds object.
     */
    private ChassisSpeeds getChassisSpeeds(double forward, double strafe, double rotation) {
        ChassisSpeeds speeds;
        if (fieldOriented) { //TODO: field oriented
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(forward, strafe, rotation, Rotation2d.fromDegrees(getHeading()));
        } else {
            speeds = new ChassisSpeeds(forward, strafe, rotation);
        }
        return speeds;
    }

    /**
     * Constructs and returns four SwerveModuleState objects, one for each side,
     * using forward, strafe, and rotation values.
     * 
     * @param forward  The desired forward speed, in m/s. Forward is positive.
     * @param strafe   The desired strafe speed, in m/s. Left is positive.
     * @param rotation The desired rotation speed, in rad/s. Counter clockwise is positive.
     * @return A SwerveModuleState array, one for each side of the drivetrain (FL,
     *         FR, etc.).
     */
    private SwerveModuleState[] getSwerveStates(double forward, double strafe, double rotation) {
        return kinematics.toSwerveModuleStates(getChassisSpeeds(forward, -strafe, rotation));
    }

    //#endregion

    //#region Getters and Setters

    public double getHeading() {
        double x = gyro.getAngle();
        if (fieldOriented) x -= fieldOffset;
        return Math.IEEEremainder(x * (isGyroReversed ? -1.0 : 1.0), 360);
    }

    @Override
    public double getHeadingDeg() {
        return getHeading();
    }

    @Override
    public SwerveModulePosition[] getModulePositions() {
        return Arrays.stream(modules).map(SwerveModule::getCurrentPosition).toArray(SwerveModulePosition[]::new);
    }

    @Override
    public Pose2d getPose() {
        return odometry.getEstimatedPosition();
    }

    @Override
    public void setPose(Pose2d initialPose) {
        odometry.resetPosition(Rotation2d.fromDegrees(getHeading()), getModulePositions(), initialPose);
    }

    // Resets the gyro, so that the direction the robotic currently faces is
    // considered "forward"
    public void resetHeading() {
        gyro.reset();
    }

    public double getPitch() {
        return gyro.getPitch();
    }

    public double getRoll() {
        return gyro.getRoll();
    }

    public boolean getFieldOriented() {
        return fieldOriented;
    }

    public void setFieldOriented(boolean fieldOriented) {
        this.fieldOriented = fieldOriented;
    }

    public void resetFieldOrientation() {
        fieldOffset = gyro.getAngle();
    }

    public void resetOdometry() {
        odometry.resetPosition(new Rotation2d(), getModulePositions(), new Pose2d());
        gyro.reset();
    }

    @Override
    public SwerveDriveKinematics getKinematics() {
        return kinematics;
    }

    public ChassisSpeeds getSpeeds() {
        return kinematics.toChassisSpeeds(Arrays.stream(modules).map(SwerveModule::getCurrentState)
            .toArray(SwerveModuleState[]::new));
    }

    public void toggleMode() {
        for (SwerveModule module: modules)
            module.toggleMode();
    }

    public void brake() {
        for (SwerveModule module: modules)
            module.brake();
    }

    public void coast() {
        for (SwerveModule module: modules)
            module.coast();
    }

    @Override
    public double getMaxAccelMps2() {
        return autoMaxAccelMps2;
    }

    @Override
    public double getMaxSpeedMps() {
        return autoMaxSpeedMps;
    }

    @Override
    public double[][] getPIDConstants() {
        return new double[][] {
            xPIDController,
            yPIDController,
            thetaPIDController
        };
    }

    //#endregion

}
