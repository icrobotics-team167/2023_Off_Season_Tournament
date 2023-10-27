package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Config;

/**
 * <p>
 * The robot's gyroscope, used to detect its orientation.
 * 
 * <p>
 * Is put in its own
 * class so that it's easier to replace code if we need to switch what type of
 * gyro we use.
 */
public class Gyro {
    private static Gyro instance;

    private Pigeon2 pigeon;

    private double yawOffset = 0;
    private double pitchOffset = 0;
    private double rollOffset = 0;

    /**
     * Allows only one instance of Gyro to exist at once.
     * 
     * @return An instance of Gyro. Will create a new one if it doesn't exist
     *         already
     */
    public static Gyro getInstance() {
        if (instance == null) {
            instance = new Gyro(Config.Ports.PIGEON);
        }
        return instance;
    }

    private Gyro(int CAN_ID) {
        pigeon = new Pigeon2(CAN_ID);
        // pigeon.configMountPose(-90,0,0);
    }

    /**
     * <p>
     * Resets the gyro.
     * 
     * <p>
     * Equivalent to running all 3 reset methods.
     */
    public void reset() {
        resetPitch();
        resetRoll();
        resetYaw();
    }

    /**
     * Gets the current yaw, as a Rotation2d object.
     * 
     * @return The current yaw, as a Rotation2d object. Positive is CCW
     */
    public Rotation2d getYaw() {
        return Rotation2d.fromDegrees(pigeon.getYaw() - yawOffset);
    }

    /**
     * <p>
     * Gets the current yaw, in degrees.
     * 
     * <p>
     * Equivalent to getYaw().getDegrees()
     * 
     * @return The current yaw, in degrees.
     */
    public double getYawDegrees() {
        return getYaw().getDegrees();
    }

    /**
     * Sets the current yaw.
     * 
     * @param yaw The new yaw, in degrees.
     */
    public void setYaw(double yaw) {
        yawOffset = pigeon.getYaw() - yaw;
    }

    /**
     * <p>
     * Resets the current yaw to 0.
     * 
     * <p>
     * Equivalent to setYaw(0);
     */
    public void resetYaw() {
        setYaw(0);
    }

    /**
     * Gets the current pitch, as a Rotation2d object.
     * 
     * @return The current pitch, as a Rotation2d object. Positive is tilted up.
     */
    public Rotation2d getPitch() {
        return Rotation2d.fromDegrees(-pigeon.getPitch() - pitchOffset);
    }

    /**
     * <p>
     * Gets the current pitch, in degrees.
     * 
     * <p>
     * Equivalent to getPitch().getDegrees()
     * 
     * @return The current pitch, in degrees.
     */
    public double getPitchDegrees() {
        return getPitch().getDegrees();
    }

    /**
     * Sets the current pitch.
     * 
     * @param pitch The new pitch.
     */
    public void setPitch(double pitch) {
        pitchOffset = -pigeon.getPitch() - pitch;
    }

    /**
     * <p>
     * Resets the current pitch to 0.
     * 
     * <p>
     * Equivalent to setPitch(0);
     */
    public void resetPitch() {
        setPitch(0);
    }

    /**
     * Gets the current roll, as a Rotation2d object.
     * 
     * @return The current roll, as a Rotation2d object. Positive is CCW.
     */
    public Rotation2d getRoll() {
        return Rotation2d.fromDegrees(-pigeon.getRoll() - rollOffset);
    }

    /**
     * <p>
     * Gets the current roll, in degrees.
     * 
     * <p>
     * Equivalent to getRoll().getDegrees()
     * 
     * @return The current roll, in degrees.
     */
    public double getRollDegrees() {
        return getRoll().getDegrees();
    }

    /**
     * Sets the current roll.
     * 
     * @param roll The new roll.
     */
    public void setRoll(double roll) {
        rollOffset = -pigeon.getRoll() - roll;
    }

    /**
     * <p>
     * Resets the current roll to 0.
     * 
     * <p>
     * Equivalent to setRoll(0);
     */
    public void resetRoll() {
        setRoll(0);
    }
}
