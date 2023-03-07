// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.robotcode2023;

import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.Joystick;

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
    public static final class Arm{
        public static final int port = 2;
      //17 for real robot
    }
    public static final class Wrist {
        public static final int port = 5;
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
