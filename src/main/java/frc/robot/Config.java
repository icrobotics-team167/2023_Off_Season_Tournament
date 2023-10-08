package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.controls.controllers.ControllerType;

/**
 * Configuration settings file.
 */
public class Config {

    /**
     * General settings, such as exponential joysticks and controller types.
     */
    public static final class Settings {

        // Joysticks
        public static final boolean EXPONENTIAL_JOYSTICKS = true;
        public static final double JOYSTICKS_EXPONENT = 2;

        // Controllers
        public static final ControllerType PRIMARY_CONTROLLER_TYPE = ControllerType.JOYSTICK;
        public static final ControllerType SECONDARY_CONTROLLER_TYPE = ControllerType.JOYSTICK;
        public static final ControllerType TERTIARY_CONTROLLER_TYPE = ControllerType.JOYSTICK;
        public static final ControllerType QUATERNARY_CONTROLLER_TYPE = ControllerType.JOYSTICK;

        // CPU period (seconds)
        public static final double CPU_PERIOD = 0.02;

        /**
         * Drivebase speed limits.
         */
        public static final class SwerveDrive {
            // TODO: Figure out max movement speeds
            public static final double MAX_MOVE_SPEED = 1.0; // m/s;
            public static final double MAX_TURN_SPEED = Units.rotationsToRadians(1); // rotations/s
            public static final double MAX_TURN_ACCEL = 1; // Radians/s^2

            public static final double SLOWMODE_MULT = 0.4; // 40%
        }
    }

    /**
     * Tolerances for things that have a margin of error.
     */
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

    /**
     * Ports IDs.
     */
    public static final class Ports {

        // Controllers
        public static final int PRIMARY_CONTROLLER = 0;
        public static final int SECONDARY_CONTROLLER = 1;
        public static final int TERTIARY_CONTROLLER = 2;
        public static final int QUATERNARY_CONTROLLER = 3;

        // main control system components
        // public static final int RoboRio = 0
        // public static final int PDP = 1

        public static final int PIGEON = 10;

        /**
         * Drivebase ports.
         */
        public static final class SwerveDrive {
            // Drivebase CAN bus Addresses
            public static final int FRONT_LEFT_DRIVE = 2;
            public static final int FRONT_LEFT_TURN = 3;
            public static final int FRONT_RIGHT_DRIVE = 4;
            public static final int FRONT_RIGHT_TURN = 5;
            public static final int BACK_LEFT_DRIVE = 9;
            public static final int BACK_LEFT_TURN = 8;
            public static final int BACK_RIGHT_DRIVE = 6;
            public static final int BACK_RIGHT_TURN = 7;

            // TODO: Measure actual positions of modules
            // Measured in meters
            public static final double robotLength = Units.inchesToMeters(23 - 2 * 3.25); // 16.5 in
            public static final double robotWidth = Units.inchesToMeters(23 - 2 * 3.25); // 16.5 in
            public static final Translation2d FRONT_LEFT_POS = new Translation2d(-robotLength / 2.0, -robotWidth / 2.0);
            public static final Translation2d FRONT_RIGHT_POS = new Translation2d(-robotLength / 2.0, robotWidth / 2.0);
            public static final Translation2d BACK_LEFT_POS = new Translation2d(robotLength / 2.0, -robotWidth / 2.0);
            public static final Translation2d BACK_RIGHT_POS = new Translation2d(robotLength / 2.0, robotWidth / 2.0);

            // Drivebase Analog Encoder Ports
            public static final int FRONT_LEFT_ENCODER = 0;
            public static final int FRONT_RIGHT_ENCODER = 1;
            public static final int BACK_LEFT_ENCODER = 2;
            public static final int BACK_RIGHT_ENCODER = 3;

            // Drivebase encoder offsets
            // Dear build team, what the fuck.
            public static final double FRONT_LEFT_OFFSET = -95;
            public static final double FRONT_RIGHT_OFFSET = -53;
            public static final double BACK_LEFT_OFFSET = -42;
            public static final double BACK_RIGHT_OFFSET = 32;
        }

        public static final class Arm {
            // Arm CAN bus Addresses
            // TEMPORARY VALUES
            public static final int EXTEND_RETRACT = 12;
            public static final int PIVOT_1 = 10;
            public static final int PIVOT_2 = 11;
            public static final int CLAW = 13;

            // Digital Limit Switches
            public static final int EXTEND_RETRACT_SWITCH = 0;
        }
    }

}
