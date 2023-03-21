// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.robotcode2023;

import java.awt.Color;

import org.carlmontrobotics.lib199.Limelight;
import org.carlmontrobotics.lib199.Limelight.Transform;
import org.carlmontrobotics.lib199.swerve.SwerveConfig;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.XboxController.Button;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final double g = 9.81; //meters per second squared

    public static final class Drivetrain {

        //#region Subsystem Constants

        public static final double wheelBase = Units.inchesToMeters(19.75);
        public static final double trackWidth = Units.inchesToMeters(28.75);
        // "swerveRadius" is the distance from the center of the robot to one of the modules
        public static final double swerveRadius = Math.sqrt(Math.pow(wheelBase / 2, 2) + Math.pow(trackWidth / 2, 2));
        // The gearing reduction from the drive motor controller to the wheels
        // Gearing for the Swerve Modules is 6.75 : 1
        public static final double driveGearing = 6.75;

        public static final double driveModifier = 1;
        public static final double wheelDiameterMeters = Units.inchesToMeters(4.0) * 7.36/7.65 /* empirical correction */;
        public static final double mu = 0.5; /* 70/83.2;  */

        public static final double NEOFreeSpeed = 5676 * (2 * Math.PI) / 60;    // radians/s
        // Angular speed to translational speed --> v = omega * r / gearing
        public static final double maxSpeed = NEOFreeSpeed * (wheelDiameterMeters / 2.0) / driveGearing;
        public static final double maxForward = maxSpeed;
        public static final double maxStrafe = maxSpeed;
        // maxRCW is the angular velocity of the robot.
        // Calculated by looking at one of the motors and treating it as a point mass moving around in a circle.
        // Tangential speed of this point mass is maxSpeed and the radius of the circle is sqrt((wheelBase/2)^2 + (trackWidth/2)^2)
        // Angular velocity = Tangential speed / radius
        public static final double maxRCW = maxSpeed / swerveRadius;

        public static final boolean[] reversed = {false, false, false, false};
        // public static final boolean[] reversed = {true, true, true, true};
        // Determine correct turnZero constants (FL, FR, BL, BR)
        public static final double[] turnZero = RobotBase.isSimulation() ?
            new double[] {0, 0, 0, 0} :
            new double[] {85.7812, 85.0782 , -96.9433, -162.9492};

        // kP, kI, and kD constants for turn motor controllers in the order of front-left, front-right, back-left, back-right.
        // Determine correct turn PID constants
        public static final double[] turnkP = {0.00374, 0.00374, 0.00374, 0.00374};
        public static final double[] turnkI = {0, 0, 0, 0};
        public static final double[] turnkD = {0, 0, 0, 0};
        public static final double[] turnkS = {0.2, 0.2, 0.2, 0.2};
        // V = kS + kV * v + kA * a
        // 12 = 0.2 + 0.00463 * v
        // v = (12 - 0.2) / 0.00463 = 2548.596 degrees/s
        public static final double[] turnkV = {0.00463, 0.00463, 0.00463, 0.00463};
        public static final double[] turnkA = {0.000115, 0.000115, 0.000115, 0.000115};

        // kP is an average of the forward and backward kP values
        // Forward: 1.72, 1.71, 1.92, 1.94
        // Backward: 1.92, 1.92, 2.11, 1.89
        public static final double[] drivekP = {1.82, 1.815, 2.015, 1.915};
        public static final double[] drivekI = {0, 0, 0, 0};
        public static final double[] drivekD = {0, 0, 0, 0};
        public static final boolean[] driveInversion = {false, false, false, false};
        public static final boolean[] turnInversion = {true, true, true, true};

        public static final double[] kForwardVolts = {0.129, 0.108, 0.14, 0.125};
        public static final double[] kBackwardVolts = {0.115, 0.169, 0.13, 0.148};

        public static final double[] kForwardVels = {2.910/1.1, 2.970/1.1, 2.890/1.1, 2.930/1.1};
        public static final double[] kBackwardVels = {2.890/1.1, 2.800/1.1, 2.850/1.1, 2.820/1.1};
        public static final double[] kForwardAccels = {0.145, 0.149, 0.192, 0.198};
        public static final double[] kBackwardAccels = {0.192, 0.187, 0.264, 0.176};

        public static final double autoMaxSpeedMps = 0.35 * 4.4;  // Meters / second
        public static final double autoMaxAccelMps2 = mu * g;  // Meters / seconds^2
        public static final double autoMaxVolt = 10.0;   // For Drivetrain voltage constraint in RobotPath.java
        // The maximum acceleration the robot can achieve is equal to the coefficient of static friction times the gravitational acceleration
        // a = mu * 9.8 m/s^2
        public static final double autoCentripetalAccel = mu * g * 2;

        public static final boolean isGyroReversed = true;

        // PID values are listed in the order kP, kI, and kD
        public static final double[] xPIDController = {4, 0.0, 0.0};
        public static final double[] yPIDController = {4, 0.0, 0.0};
        public static final double[] thetaPIDController = {0.10, 0.0, 0.001};

        public static final SwerveConfig swerveConfig = new SwerveConfig(wheelDiameterMeters, driveGearing, mu, autoCentripetalAccel, kForwardVolts, kForwardVels, kForwardAccels, kBackwardVolts, kBackwardVels, kBackwardAccels, drivekP, drivekI, drivekD, turnkP, turnkI, turnkD, turnkS, turnkV, turnkA, turnZero, driveInversion, reversed, driveModifier, turnInversion);

        public static final Limelight.Transform limelightTransformForPoseEstimation = Transform.BOTPOSE_WPIBLUE;

        //#endregion

        //#region Ports

        public static final int driveFrontLeftPort = 8;
        public static final int driveFrontRightPort = 13;
        public static final int driveBackLeftPort = 5;
        public static final int driveBackRightPort = 11;

        public static final int turnFrontLeftPort = 7;
        public static final int turnFrontRightPort = 14;
        public static final int turnBackLeftPort = 6;
        public static final int turnBackRightPort = 12;

        public static final int canCoderPortFL = 1;
        public static final int canCoderPortFR = 2;
        public static final int canCoderPortBL = 3;
        public static final int canCoderPortBR = 4;

        //#endregion

        //#region Command Constants

        public static final double kNormalDriveSpeed = 1; // Percent Multiplier
        public static final double kNormalDriveRotation = 0.55; // Percent Multiplier
        public static final double kSlowDriveSpeed = 0.25; // Percent Multiplier
        public static final double kSlowDriveRotation = 0.30; // Percent Multiplier
        public static final double kAlignMultiplier = 1D/3D;
        public static final double kAlignForward = 0.6;

        public static final double chargeStationAlignToleranceDeg = 2.5;
        public static final double chargeStationAlignSpeedMpSPerDeg = 0.3 / 20;
        public static final double chargeStationAlignTime = 500;
        public static final double chargeStationAlignFFMpS = 0;
        public static final double wheelTurnDriveSpeed = 0.0001; // Meters / Second ; A non-zero speed just used to orient the wheels to the correct angle. This should be very small to avoid actually moving the robot.

        public static final double[] positionTolerance = {Units.inchesToMeters(.5), Units.inchesToMeters(.5), 5}; // Meters, Meters, Degrees
        public static final double[] velocityTolerance = {Units.inchesToMeters(1), Units.inchesToMeters(1), 25}; // Meters, Meters, Degrees/Second

        //#endregion
    }

    public static final class Arm {

        //#region Subsystem Constants

        // Array Indexes (Just to make things easier to read)
        public static final int ARM = 0;
        public static final int WRIST = 1;
        public static final int CUBE = 0;
        public static final int CONE = 1;
        public static final int FRONT = 0;
        public static final int BACK = 1;

        // Feedforward
        // Arm, Wrist
        public static final double[] kS = {0.20642, .074798};
        public static final double[] kG = {0.6697, 0.36214};
        public static final double[] kV = {4.3735, 1.6743};
        public static final double[] kA = {0.24914, 0.032177};
        public static final double kG_WRIST = .36214; // (V)

        // PID
        // FIXME BOTH WRIST AND ARM NEED TO TEST PID (Wrist PID never tested)
        // Arm, Wrist
        public static double[] kP = {4.2736, 6}; // 4.2736 for arm from sysid was tested and it worked fine (V / rad)
        public static double[] kI = {0, 0}; // (V / (rad * s) )
        public static double[] kD = {0, 0}; // 0 for arm from sysid was tested and it worked fine (V / (rad / s) )

        // Arm, Wrist
        public static double[] posToleranceRad = { .05, .05 }; // rad
        public static double[] velToleranceRadPSec = { 0.5, 0.5 }; // rad/s

        public static double[] offsetRad = { 4.02, -0.7 + Math.PI / 2 }; // rad

        // needed to calculate feedforward values dynamically
        public static final double ARM_MASS_KG = Units.lbsToKilograms(8.1);
        public static final double ARM_LENGTH_METERS = Units.inchesToMeters(38.25);

        // Distance from the arm motor to the center of mass of the  arm
        public static final double COM_ARM_LENGTH_METERS = Units.inchesToMeters(13.23);
        public static final double ROLLER_MASS_KG = Units.lbsToKilograms(14.04);

        // distance of center of mass of roller to the WRIST motor
        public static final double COM_ROLLER_LENGTH_METERS = Units.inchesToMeters(9.47);
        public static final double ROLLER_LENGTH_METERS = Units.inchesToMeters(19.14);
        public static final double g = 9.81;

        public static final double V_PER_NM = 0.01423;

        public static final double DT_TOTAL_WIDTH = 0.7874;
        public static final double SAFE_HEIGHT = 0.585;
        public static final double ARM_JOINT_TOTAL_HEIGHT = Units.inchesToMeters(46.725);

        // TODO: Replace these values with Design's actual values
        public static final double MARGIN_OF_ERROR = Math.toRadians(10);
        public static final double ARM_LOWER_LIMIT_RAD = -3.569 + MARGIN_OF_ERROR;
        public static final double ARM_UPPER_LIMIT_RAD = .36 - MARGIN_OF_ERROR;
        public static final double ARM_DISCONTINUITY_RAD = (ARM_LOWER_LIMIT_RAD + ARM_UPPER_LIMIT_RAD) / 2 - Math.PI;
        public static final double WRIST_LOWER_LIMIT_RAD = -2.933 + MARGIN_OF_ERROR;
        public static final double WRIST_UPPER_LIMIT_RAD = 2.605 - MARGIN_OF_ERROR;
        public static final double WRIST_DISCONTINUITY_RAD = (WRIST_LOWER_LIMIT_RAD + WRIST_UPPER_LIMIT_RAD) / 2 - Math.PI;

        // TODO: Determine actual max vel/accel
        public static double[] MAX_FF_VEL = {1, 1}; // rad / s
        public static double[] MAX_FF_ACCEL = {1, 1}; // rad / s^2
        public static TrapezoidProfile.Constraints armConstraints = new TrapezoidProfile.Constraints(MAX_FF_VEL[ARM], MAX_FF_ACCEL[ARM]);
        public static TrapezoidProfile.Constraints wristConstraints = new TrapezoidProfile.Constraints(MAX_FF_VEL[WRIST], MAX_FF_ACCEL[WRIST]);

        //#endregion

        //#region Ports

        public static final boolean[] inverted = { true, false };
        public static final double rotationToRad = 2 * Math.PI;

        public static final int armMotorPort = 17;
        public static final int wristMotorPort = 19;

        //#endregion

        //#region Command Constants

        public static final double WRIST_STOW_POS = WRIST_UPPER_LIMIT_RAD;
        public static final double WRIST_NEG_STOW_POS = WRIST_LOWER_LIMIT_RAD;
        public static final double ARM_VERTICAL_POS = -Math.PI / 2;
        public static final double MIN_WRIST_FOLD_POS = Math.PI / 4;
        public static final int WRIST_CURRENT_LIMIT = 15; // amps
        public static final double ROLLER_COM_CORRECTION = 14.48;

        //#endregion

    }

    public static final class GoalPos {

        //#region Goal Positions
        // if intake/outtake on back, the "negative" pos will be used
        // 0 = "Front", 1 = "Back" (i.e. front refers to the side with the battery)
        // 0 = CUBE, 1 = CONE
        // GoalPos(arm, wrist)

        // TODO : Get angles for front
        public static GoalPos[][] LOW = {
            { // front
                new GoalPos(Units.degreesToRadians(-44.46), Units.degreesToRadians(117.17)), // cube
                new GoalPos(Units.degreesToRadians(0), Units.degreesToRadians(-100.78))             // cone
            },
            { // back
                new GoalPos(Units.degreesToRadians(-80.2), Units.degreesToRadians(61.37)), 
                new GoalPos(Units.degreesToRadians(-90), Units.degreesToRadians(133.24))
            }
        };
        public static GoalPos[][] MID = {
            {
                new GoalPos(Units.degreesToRadians(-85.98), Units.degreesToRadians(132.01)), 
                new GoalPos(Units.degreesToRadians(-106.85), Units.degreesToRadians(65.47)) 
            },
            {
                new GoalPos(Units.degreesToRadians(15.24), Units.degreesToRadians(-109.64)), 
                new GoalPos(Units.degreesToRadians(-47.63), Units.degreesToRadians(141.7)) 
            }
        };
        public static GoalPos[][] HIGH = {
            {
                new GoalPos(Units.degreesToRadians(-102.02), Units.degreesToRadians(-149.079)), 
                new GoalPos(Units.degreesToRadians(-104.18), Units.degreesToRadians(54.24)) 
            },
            {
                new GoalPos(Units.degreesToRadians(20.76), Units.degreesToRadians(-52.06)), 
                new GoalPos(Units.degreesToRadians(-14.07), Units.degreesToRadians(58.12)) 
            }
        };
        // TODO: Get positions for STORED, SHELF, and SUBSTATION
        public static GoalPos[][] STORED = {
            {
                new GoalPos(Units.degreesToRadians(-90), 0), 
                new GoalPos(40, -1.8861790155)
            },
            {
                new GoalPos(Units.degreesToRadians(-90), 0), 
                new GoalPos(40, -1.8861790155)
            }
        };
        public static GoalPos[][] SHELF = {
            {
                new GoalPos(Units.degreesToRadians(-90), 0), 
                new GoalPos(0, 0)
            },
            {
                new GoalPos(Units.degreesToRadians(-90), 0), 
                new GoalPos(0, 0)
            }
        };
        public static GoalPos[][] SUBSTATION = {
            {
                new GoalPos(Units.degreesToRadians(-90), 0), 
                new GoalPos(0, 0)
            },
            {
                new GoalPos(Units.degreesToRadians(-90), 0), 
                new GoalPos(0, 0)
            }
        };
        public static GoalPos[][] INTAKE = {
            {
                new GoalPos(Units.degreesToRadians(-72.5), Units.degreesToRadians(73.37)),
                new GoalPos(Units.degreesToRadians(-66.6), Units.degreesToRadians(29.94)) 
            },
            {
                new GoalPos(Units.degreesToRadians(-72.5), Units.degreesToRadians(73.37)),
                new GoalPos(Units.degreesToRadians(-66.6), Units.degreesToRadians(29.94)) 
            }
        };
        public double armPos, wristPos;

        public GoalPos(double armPos, double wristPos) {
            this.armPos = armPos;
            this.wristPos = wristPos;
        }
        //#endregion

    }

    public static final class Roller {

        //#region Subsystem Constants

        public static final int ledLength = 85;
        public static final double ledDefaultColorRestoreTime = 3; // The time in seconds after picking up a game piece to restore the LED color to defaultColor
        public static final Color defaultColor = new Color(0, 0, 200);
        public static final Color pickupSuccessColor = new Color(0, 200, 0);
        public static final Color conePickupColor = new Color(150, 150, 0);
        public static final Color cubePickupColor = new Color(50, 0, 200);

        public static final double distSensorDepthMM = 16;
        public static final double gamePieceDetectDistanceIn = 21;

        //#endregion

        //#region Ports

        public static final int rollerPort = 18;
        public static final int ledPort = 0;

        //#endregion

        //#region Command Constants
        // TODO: Determine actual speeds/timings for roller
        public static class RollerMode {
            public static RollerMode INTAKE_CONE = new RollerMode(-0.5, .5, true);
            public static RollerMode INTAKE_CUBE = new RollerMode(0.3, .25, true);
            public static RollerMode OUTTAKE_CONE = new RollerMode(0.5, .5, false);
            public static RollerMode OUTTAKE_CUBE = new RollerMode(-0.5, .5, false);
            public double speed, time;
            public boolean intake;
    
            /**
             * @param speed  A number between -1 and 1
             * @param time   Amount of time in seconds to keep the motor running after
             *               distance sensor has detected an object
             * @param intake Whether the roller is outtaking or intaking
             */
            public RollerMode(double speed, double time, boolean intake) {
                this.speed = speed;
                this.time = time;
                this.intake = intake;
            }
        }

        //#endregion
    }

    public static final class OI {

        public static final double JOY_THRESH = 0.01;
        public static final double MIN_AXIS_TRIGGER_VALUE = 0.25;

        public static final class Driver {
            public static final int port = 0;

            public static final int slowDriveButton = Button.kLeftBumper.value;
            public static final int chargeStationAlignButton = Button.kBack.value;
            public static final int resetFieldOrientationButton = Button.kRightBumper.value;
            public static final int toggleFieldOrientedButton = Button.kStart.value;

            public static final int rotateToFieldRelativeAngle0Deg = Button.kY.value;
            public static final int rotateToFieldRelativeAngle90Deg = Button.kB.value;
            public static final int rotateToFieldRelativeAngle180Deg = Button.kA.value;
            public static final int rotateToFieldRelativeAngle270Deg = Button.kX.value;
        }

        public static final class Manipulator {
            public static final int port = 1;

            public static final int toggleCubeButton = Button.kLeftBumper.value;
            public static final int toggleFrontButton = Button.kRightBumper.value;

            public static final int storePosButton = Button.kA.value;
            public static final int lowPosButton = Button.kX.value;
            public static final int midPosButton = Button.kY.value;
            public static final int highPosButton = Button.kB.value;

            public static final int shelfPickupPOV = 0;
            public static final int intakeConePOV = 90;
            public static final int substationPickupPOV = 180;
            public static final int intakeCubePOV = 270;

            public static final int stopRollerButton = Button.kBack.value;

            public static final Axis rollerIntakeCubeButton = Axis.kRightTrigger;
            public static final Axis rollerIntakeConeButton = Axis.kLeftTrigger;
        }
    }
}
