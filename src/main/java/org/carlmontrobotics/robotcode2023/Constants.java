// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.robotcode2023;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
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

    public static final int grabber_motor_port = 0;
    public static final int wrist_motorL_port = 100;
    public static final int wrist_motorR_port = 101;

    public static final double grabber_open_position = 2.8;
    public static final double grabber_closed_position = -6.9;
    public static final class Arm {

        //#region Subsystem Constants

        // Feedforward
        // FIXME WRIST NEEDS SYSID DONE
        // Arm, Wrist
        public static final double[] kS = { .067766, .074798 }; // (V)
        public static final double[] kV = { .019762, 1.6743 }; // (V / rad/s)
        public static final double[] kA = { .00039212, 0.032177 }; // (V / rad/s^2)
        public static final double kG_WRIST = .36214; // (V)

        // PID
        // FIXME BOTH WRIST AND ARM NEED PID DONE
        // Arm, Wrist
        public static double[] kP = { 0, 0 }; // (V / rad)
        public static double[] kI = { 0, 0 }; // (V / (rad * s) )
        public static double[] kD = { 0, 0 }; // (V / (rad / s) )

        // Arm, Wrist
        public static double[] posToleranceRad = { .05, .05 }; // rad
        public static double[] velToleranceRadPSec = { 0.5, 0.5 }; // rad/s

        public static double[] goalPosRad = { -Math.PI / 2, 0 }; // rad
        public static double[] offsetRad = { 2.08, 4.02 }; // rad

        // needed to calculate feedforward values dynamically
        public static final double ARM_MASS_KG = Units.lbsToKilograms(6.57);
        public static final double ARM_LENGTH_METERS = Units.inchesToMeters(38.25);

        // Distance from the arm motor to the center of mass of the  arm
        public static final double COM_ARM_LENGTH_METERS = Units.inchesToMeters(13.23);
        public static final double ROLLER_MASS_KG = Units.lbsToKilograms(10.91);

        // distance of center of mass of roller to the WRIST motor
        public static final double COM_ROLLER_LENGTH_METERS = Units.inchesToMeters(9.47);
        public static final double g = 9.81;

        public static final double V_PER_NM = 0;

        //#endregion

        //#region Ports

        public static final int armMotorPort = 17;
        public static final int wristMotorPort = 19;

        //#endregion

    }
    public static final class Wrist {
    }

    public static final class OI {
        public static final class Driver {
            public static final int port = 0;
        }
        public static final class Manipulator {
            public static final int port = 1;
            public static final int cone = Button.kRightBumper.value;
            public static final int cube = Button.kLeftBumper.value;
            public static final int intake = Button.kA.value;
            
            public static final int cycleUp = 1;//FIXME use correct buttons
            public static final int cycleDown = 2;
        }
        public static final class Controller {
          public static final int port = 2;

          public static Joystick controller = new Joystick(port);

          public static int X;
          public static int A;
          public static int B;
          public static int Y;
          public static int LB;
          public static int RB;
          public static int LT;
          public static int RT;
          public static int BACK;
          public static int START;

          //TODO: mode button setting to teletop init
          static {
              if (controller.getName().equals("Logitech Dual Action")) {
                  // Buttons and triggers
                  X = 1;
                  A = 2;
                  B = 3;
                  Y = 4;
                  LB = 5;
                  RB = 6;
                  LT = 7;
                  RT = 8;
                  BACK = 9;
                  START = 10;
              } else {
                  // Buttons and triggers for xbox controller
                  X = 3;
                  A = 1;
                  B = 2;
                  Y = 4;
                  LB = 5;
                  RB = 6;
                  BACK = 7;
                  START = 8;
                  //todo get arrow buttons
              }
           }
        }
    }
}
