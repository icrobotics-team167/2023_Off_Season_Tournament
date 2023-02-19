package frc.robot.subsystems.turret;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Config;

/**
 * Turns the turret on the robot, for up to MAX_TURN_ANGLE degrees on both sides.
 * 
 * 
 */
public class Swivel {

    private CANSparkMax swivelMotor;
    private RelativeEncoder swivelEncoder;

    private double initialEncoderPosition;

    private final double MAX_TURN_ANGLE = 60;
    private final double MAX_TURN_SPEED = 0.2;

    // Singleton
    public static Swivel instance;

    /**
     * Allows only one instance of Swivel to exist at once.
     * 
     * @return An instance of Swivel. Creates a new one if it doesn't exist already.
     */
    public static Swivel getInstance() {
        if (instance == null) {
            instance = new Swivel();
        }
        return instance;
    }

    /**
     * Constructs a new swivel joint for the turret.
     */
    private Swivel() {
        // Set up motor
        swivelMotor = new CANSparkMax(Config.Ports.Arm.SWIVEL, CANSparkMaxLowLevel.MotorType.kBrushless);
        swivelMotor.restoreFactoryDefaults();
        swivelMotor.setInverted(false);
        swivelMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
        swivelMotor.setSmartCurrentLimit(60);
        swivelMotor.setSecondaryCurrentLimit(80);

        // Set up encoder
        swivelEncoder = swivelMotor.getEncoder();
        SmartDashboard.putNumber("Swivel.PositionScaleFactor", swivelEncoder.getPositionConversionFactor());

        // Set up positon (Assuming it's centered when powered on)
        initialEncoderPosition = swivelEncoder.getPosition();
    }

    /**
     * Swivels the turret.
     * Stops the motor if speed tries to move it more than 180 degrees to prevent
     * twisting wires.
     * 
     * @param speed How fast it should move. Positive speed values swivels
     *              clockwise,
     *              negative values swivels counterclockwise.
     */
    public void move(double speed) {
        SmartDashboard.putNumber("Swivel.degrees", getPositionDegrees());
        SmartDashboard.putNumber("Swivel.speed", speed);
        double motorOutput = MAX_TURN_SPEED * Math.abs(speed);
        if (speed > 0 && !tooFarRight()) {
            swivelMotor.set(-motorOutput);
        } else if (speed < 0 && !tooFarLeft()) {
            swivelMotor.set(motorOutput);
        } else {
            swivelMotor.stopMotor();
        }
    }

    /**
     * Gets the positon of the swivel joint in degrees.
     * 
     * @return The position of the joint. 0 degrees is position of the joint on code
     *         boot. (Not neccesarily absolutely centered.) Positive values means
     *         the arm is clockwise of the start position.
     */
    public double getPositionDegrees() {
        double scalar = 0.782;
        return (swivelEncoder.getPosition() - initialEncoderPosition) * -scalar;
    }

    /**
     * Gets if the joint is too far counterclockwise
     * 
     * @return If the joint is more than MAX_TURN_ANGLE degrees counterclockwise
     */
    private boolean tooFarLeft() {
        return getPositionDegrees() < -MAX_TURN_ANGLE;
    }

    /**
     * Gets if the joint is too far clockwise
     * 
     * @return If the joint is more than MAX_TURN_ANGLE degrees clockwise
     */
    private boolean tooFarRight() {
        return getPositionDegrees() > MAX_TURN_ANGLE;
    }
}
