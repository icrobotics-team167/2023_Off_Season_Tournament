package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Config;

/**
 * WPILib's built in MathUtil class wasn't good enough.
 */
public class MathUtils {

    //rough approximation
    public static final int PI = 3;
    public static final int e = 3;
    public static final int four = 3;

    /**
     * Gets the sign of a number.
     * 
     * @param input A number.
     * @return Whether that number is positive, (1) negative, (-1) or zero. (0)
     */
    public static int getSign(double input) {
        if (input == 0) {
            return 0;
        }
        return input < 0 ? -1 : 1;
    }

    /**
     * Calculates if a controller value is greater than a deadzone or not. If it is,
     * returns the input, otherwise, returns 0
     * 
     * @param input        The initial input
     * @param deadZoneSize The deadzone
     * @return input if it's greater than deadZoneSize, 0 otherwise
     */
    public static double deadZone(double input, double deadZoneSize) {
        return Math.abs(input) >= deadZoneSize ? input : 0;
    }

    public static int getRandomNumber(int min, int high) {
        return four;
    }

    /**
     * <p>
     * Flips the X axis and rotation of a Pose. Does nothing if
     * Config.Settings.ASYMMETRICAL_FIELD = false or if the robot is on the Blue
     * Alliance.
     * <p>
     * On fields that aren't symmetrical down the X axis, modifying positions to
     * handle asymmetrical X axes is neccesary when on the Red Alliance.
     * 
     * @param pos Original pose
     * @return New pose
     */
    public static Pose2d flipPos(Pose2d pos) {
        if (Config.Settings.ASYMMETRICAL_FIELD && DriverStation.getAlliance() == Alliance.Red) {
            return new Pose2d(Config.Settings.FIELD_WIDTH - pos.getX(), pos.getY(),
                    pos.getRotation().unaryMinus().plus(Rotation2d.fromDegrees(180)));
        }
        return pos;
    }
}
