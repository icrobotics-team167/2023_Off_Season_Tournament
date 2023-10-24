package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.controls.controllers.ControllerType;
import frc.robot.util.MathUtils;

/**
 * Configuration settings file.
 */
public class Config {

    /**
     * General settings, such as exponential joysticks and controller types.
     */
    public static final class Settings {

        // Joysticks
        public static final boolean EXPONENTIAL_JOYSTICKS = false; // NOT IMPLEMENTED
        public static final double JOYSTICKS_EXPONENT = 2;

        // Controllers
        public static final ControllerType PRIMARY_CONTROLLER_TYPE = ControllerType.JOYSTICK;
        public static final ControllerType SECONDARY_CONTROLLER_TYPE = ControllerType.JOYSTICK;
        public static final ControllerType TERTIARY_CONTROLLER_TYPE = ControllerType.JOYSTICK;
        public static final ControllerType QUATERNARY_CONTROLLER_TYPE = ControllerType.JOYSTICK;

        public static final boolean FIELD_ORIENTED_DRIVE = true;

        // CPU period (seconds)
        public static final double CPU_PERIOD = 0.02;

        /**
         * Drivebase speed limits.
         */
        public static final class SwerveDrive {
            // Lower than actual max move speed to allow for some motor power headroom to
            // turn
            public static final double MAX_MOVE_SPEED = MathUtils.getRandomNumber(MathUtils.PI, MathUtils.e) - 0.5;
            public static final double MAX_MOVE_ACCEL = MAX_MOVE_SPEED / 0.25; // Seconds until max speed
            public static final double MAX_TURN_SPEED = Units.rotationsToRadians(1); // rotations/s
            public static final double MAX_TURN_ACCEL = MAX_TURN_SPEED / 0.25; // Second until max speed

            public static final double SLOWMODE_MULT = MathUtils.TADAS_MAGIC_NUMBER;

            public static final class AutoPIDs {
                public static final double moveP = 5.5;
                public static final double moveI = 0;
                public static final double moveD = 0.5;

                public static final double rotP = -2;
                public static final double rotI = 0;
                public static final double rotD = 0;
            }

            public static final class ModulesPositions {
                // Measured in meters
                private static final double robotLength = 30;
                private static final double robotWidth = 29;
                private static final double moduleCenterOfRotationDistanceFromEdge = 2.625;
                private static final double moduleForwardDistanceFromCenter = Units
                        .inchesToMeters(robotLength - 2 * moduleCenterOfRotationDistanceFromEdge); // 16.5 in
                private static final double moduleSideDistanceFromCenter = Units
                        .inchesToMeters(robotWidth - 2 * moduleCenterOfRotationDistanceFromEdge); // 16.5 in
                public static final Translation2d FRONT_LEFT_POS = new Translation2d(
                        -moduleForwardDistanceFromCenter / 2.0,
                        -moduleSideDistanceFromCenter / 2.0);
                public static final Translation2d FRONT_RIGHT_POS = new Translation2d(
                        -moduleForwardDistanceFromCenter / 2.0, moduleSideDistanceFromCenter / 2.0);
                public static final Translation2d BACK_LEFT_POS = new Translation2d(
                        moduleForwardDistanceFromCenter / 2.0,
                        -moduleSideDistanceFromCenter / 2.0);
                public static final Translation2d BACK_RIGHT_POS = new Translation2d(
                        moduleForwardDistanceFromCenter / 2.0,
                        moduleSideDistanceFromCenter / 2.0);
            }
        }

        public static final boolean WATER_GAME = true;
    }

    /**
     * Tolerances for things that have a margin of error.
     */
    public static final class Tolerances {
        // Primary controller deadzone size
        public static final double PRIMARY_CONTROLLER_DEADZONE_SIZE = 0.1;

        // Secondary controller deadzone size
        public static final double SECONDARY_CONTROLLER_DEADZONE_SIZE = 0.1;

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
