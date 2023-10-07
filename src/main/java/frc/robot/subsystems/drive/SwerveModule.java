package frc.robot.subsystems.drive;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.controller.*;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.MovingAverage;

/**
 * A module in a swerve drive.
 */
public class SwerveModule {
    private final CANSparkMax driveMotor;
    private final CANSparkMax turnMotor;
    private final RelativeEncoder driveEncoder;
    private final AnalogEncoder turnEncoder;
    private final String moduleName;

    // TODO: Tune PID values
    private SparkMaxPIDController drivePID;
    private final double DRIVE_P = 0.25;
    private final double DRIVE_I = 0;
    private final double DRIVE_D = 0;
    private final double DRIVE_FF = 0.025;

    private PIDController turnPID;
    // private final double TURN_P = (1.0 / 180.0) / 5.0;
    private final double TURN_P = (1.0 / 180.0);
    private final double TURN_I = 0;
    private final double TURN_D = 0;

    private final double WHEEL_DIAMETER = 4;
    // Swerve Drive Specialties MK4i L2
    private final double DRIVE_GEAR_RATIO = 6.75;

    private double angleOffset;

    // Filter angles since the analog encoders are a bit noisy, prolly could get
    // away with a smaller moving average size since the noise is on the smaller
    // decimal points
    // private MovingAverage angleFilter = new MovingAverage(10, true);
    // Unused

    /**
     * Constructs a new Swerve module.
     * 
     * @param driveMotorID The CAN ID of the drive motor.
     * @param turnMotorID  The CAN ID of the turning motor.
     * @param encoderID    The analog port ID of the turn encoder. Also doubles as a
     *                     module ID.
     * @param angleOffset  The angle offset of the encoder, since the encoders
     *                     aren't facing forwards.
     */
    public SwerveModule(int driveMotorID, int turnMotorID, int encoderID, double angleOffset) {
        // Set up motors
        this.driveMotor = new CANSparkMax(driveMotorID, CANSparkMaxLowLevel.MotorType.kBrushless);
        this.turnMotor = new CANSparkMax(turnMotorID, CANSparkMaxLowLevel.MotorType.kBrushless);
        // Motor current draw limits to prevent the NEOs from burning
        driveMotor.setSmartCurrentLimit(60);
        driveMotor.setSecondaryCurrentLimit(80);
        turnMotor.setSmartCurrentLimit(40);
        turnMotor.setSecondaryCurrentLimit(60);

        // Set up drive motor encoder
        this.driveEncoder = driveMotor.getEncoder();
        // Convert from "rotations of motor" to "meters driven by wheel"
        this.driveEncoder.setPositionConversionFactor(WHEEL_DIAMETER * Math.PI / DRIVE_GEAR_RATIO);
        // Convert from "rotations per minute of motor" to "meters per second of wheel"
        this.driveEncoder
                .setVelocityConversionFactor(Units.inchesToMeters(WHEEL_DIAMETER * Math.PI / DRIVE_GEAR_RATIO / 60.0));

        // Set up the PID controller for the drive motor
        this.drivePID = this.driveMotor.getPIDController();
        this.drivePID.setP(DRIVE_P);
        this.drivePID.setI(DRIVE_I);
        this.drivePID.setD(DRIVE_D);
        this.drivePID.setFF(DRIVE_FF);

        // Set up turn encoder and angle filter
        this.turnEncoder = new AnalogEncoder(encoderID);
        // angleFilter.clear();

        // Set up the PID controller for the turning motor
        turnPID = new PIDController(TURN_P, TURN_I, TURN_D);
        // turnPID.setTolerance(5);
        turnPID.enableContinuousInput(-180, 180);

        this.angleOffset = angleOffset;

        // Little switch statement for cleaner-looking SmartDashboard outputs
        switch (encoderID) {
            case 0:
                moduleName = "Front Left";
                break;
            case 1:
                moduleName = "Front Right";
                break;
            case 2:
                moduleName = "Back Left";
                break;
            case 3:
                moduleName = "Back Right";
                break;
            default: // Unless something goes really wack this should never happen
                moduleName = String.valueOf(encoderID);
                DriverStation.reportWarning("Something went wrong, expected 4 swerve modules and instead got "
                        + String.valueOf(encoderID + 1), false);
        }
    }

    /**
     * Constructs a new Swerve module.
     * 
     * @param driveMotorID The CAN ID of the drive motor.
     * @param turnMotorID  The CAN ID of the turning motor.
     * @param encoderID    The analog port ID of the turn encoder. Also doubles as a
     *                     module ID.
     */
    public SwerveModule(int driveMotorID, int turnMotorID, int encoderID) {
        this(driveMotorID, turnMotorID, encoderID, 0.0);
    }

    /**
     * Moves the swerve module.
     * 
     * @param state The desired SwerveModuleState
     */
    public void move(SwerveModuleState state) {
        // Optimize the module state
        state = SwerveModuleState.optimize(state, Rotation2d.fromDegrees(getAngle()));
        // Calculate a PID for the turn motor, clamp the pid to -1/1, and set the motor
        // power to that
        // if (turnPID.atSetpoint()) {
        // return;
        // }
        var pidOutput = turnPID.calculate(getAngle(), state.angle.getDegrees());
        double turnPIDOutput = MathUtil.clamp(pidOutput, -1, 1);
        // if (turnPIDOutput < 0.01) {
        //     turnPIDOutput = 0;
        // }
        // turnMotor.set(turnPIDOutput);
        SmartDashboard.putNumber("Module " + moduleName + " turnMotor power", turnPIDOutput);
        // Give the drive motor's PID controller a target velocity and let it calculate
        // motor power from that
        drivePID.setReference(state.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);

        // Debug statements
        SmartDashboard.putNumber("Module " + moduleName + " current angle (degrees)", getAngle());
        SmartDashboard.putNumber("Module " + moduleName + " desired angle (degrees)", state.angle.getDegrees());
    }

    /**
     * Stops the module.
     */
    public void stop() {
        turnMotor.stopMotor();
        driveMotor.stopMotor();
    }

    /**
     * Gets the distance driven by the module in meters and the current rotation of
     * the module.
     * 
     * @return A SwerveModulePosition that represents distance driven and current
     *         angle.
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDistanceDriven(), Rotation2d.fromDegrees(getAngle()));
    }

    /**
     * Resets the distance driven by the module.
     */
    public void resetPosition() {
        driveEncoder.setPosition(0);
    }

    /**
     * Gets the current angle of the module. Filtered to smooth out noise.
     * 
     * @return Current angle in degrees, from -180 to 180.
     */
    public double getAngle() {
        // return turnEncoder.getAbsolutePosition() * 360 - 180;
        double rawEncoder = turnEncoder.getAbsolutePosition();
        double rawAngle = rawEncoder * 360 - 180;
        return Rotation2d.fromDegrees(rawAngle).rotateBy(Rotation2d.fromDegrees(angleOffset)).getDegrees();
    }

    /**
     * Gets the distance driven by the module.
     * 
     * @return Distance driven, in meters.
     */
    public double getDistanceDriven() {
        return driveEncoder.getPosition();
    }
}
