package frc.robot.util;

/**
 * WPILib's built in MathUtil class wasn't good enough.
 */
public class MathUtils {

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
}
