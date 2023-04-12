package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.controls.controllers.ControllerType;

public class Config {

    public static final class Settings {

        // Joysticks
        public static final boolean EXPONENTIAL_JOYSTICKS = true;
        public static final double JOYSTICKS_EXPONENT = 2;

        // Controllers
        public static final ControllerType PRIMARY_CONTROLLER_TYPE = ControllerType.JOYSTICK;
        public static final ControllerType SECONDARY_CONTROLLER_TYPE = ControllerType.JOYSTICK;
        public static final ControllerType TERTIARY_CONTROLLER_TYPE = ControllerType.JOYSTICK;
        public static final ControllerType QUATERNARY_CONTROLLER_TYPE = ControllerType.JOYSTICK;

        // Dead zones
        public static final boolean PRIMARY_DEADZONE_ENABLED = true;
        public static final boolean SECONDARY_DEADZONE_ENABLED = true;

        // CPU period (seconds)
        public static final double CPU_PERIOD = 0.02;
    }

    public static final class Tolerances {
        // Primary controller deadzone size
        public static final double PRIMARY_CONTROLLER_DEADZONE_SIZE = 0.125;

        // Secondary controller deadzone size
        public static final double SECONDARY_CONTROLLER_DEADZONE_SIZE = 0.175;

        // Tertiary controller deadzone size
        public static final double TERTIARY_CONTROLLER_DEADZONE_SIZE = 0.09;

        // Quaternary controller deadzone size
        public static final double QUATERNARY_CONTROLLER_DEADZONE_SIZE = 0.09;
    }

    public static final class Ports {

        // Controllers
        public static final int PRIMARY_CONTROLLER = 0;
        public static final int SECONDARY_CONTROLLER = 1;
        public static final int TERTIARY_CONTROLLER = 2;
        public static final int QUATERNARY_CONTROLLER = 3;

        // main control system components
        // public static final int RoboRio = 0
        // public static final int PDP = 1
        // public static final int PH = 2

        // Spark tank motor controller ports
        public static final class SwerveDrive {
            // Pneumatics Control Hub CAN Address
            public static final int PH = 2;

            // Drivebase CAN bus Addresses
            public static final int FRONT_LEFT_DRIVE = 1;
            public static final int FRONT_LEFT_TURN = 2;
            public static final int FRONT_RIGHT_DRIVE = 3;
            public static final int FRONT_RIGHT_TURN = 4;
            public static final int BACK_LEFT_DRIVE = 5;
            public static final int BACK_LEFT_TURN = 6;
            public static final int BACK_RIGHT_DRIVE = 7;
            public static final int BACK_RIGHT_TURN = 8;

            // TODO: Measure positions of modules
            public static final Translation2d FRONT_LEFT_POS = new Translation2d(0, 0);
            public static final Translation2d FRONT_RIGHT_POS = new Translation2d(0, 0);
            public static final Translation2d BACK_LEFT_POS = new Translation2d(0, 0);
            public static final Translation2d BACK_RIGHT_POS = new Translation2d(0, 0);

            public static final int FRONT_LEFT_ENCODER = 1;
            public static final int FRONT_RIGHT_ENCODER = 2;
            public static final int BACK_LEFT_ENCODER = 3;
            public static final int BACK_RIGHT_ENCODER = 4;

            // Pneumatic Hub Ports
            // Drivebase PH Ports
            public static final int LOW_GEAR = 1; // shifter cylinder out = high gear
        }
    }

}
