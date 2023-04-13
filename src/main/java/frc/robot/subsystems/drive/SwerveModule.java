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
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModule {
    private final CANSparkMax driveMotor;
    private final CANSparkMax turnMotor;
    private final RelativeEncoder driveEncoder;
    private final DutyCycleEncoder turnEncoder;
    private final int turnEncoderPort;

    // TODO: Tune PID values
    private SparkMaxPIDController drivePID;
    private final double DRIVE_P = 0.25;
    private final double DRIVE_I = 0;
    private final double DRIVE_D = 0;
    private final double DRIVE_FF = 0.025;

    private PIDController turnPID;
    private final double TURN_P = 1.0 / 90.0;
    private final double TURN_I = 0;
    private final double TURN_D = 0.5;

    private final double WHEEL_DIAMETER = 4;
    private final double GEAR_RATIO = 1; // TODO: Figure out module gear ratio

    private double distanceDrivenMeters;

    public SwerveModule(int driveMotorID, int turnMotorID, int turnEncoderDIOPort) {
        this.driveMotor = new CANSparkMax(driveMotorID, CANSparkMaxLowLevel.MotorType.kBrushless);
        this.turnMotor = new CANSparkMax(turnMotorID, CANSparkMaxLowLevel.MotorType.kBrushless);

        this.driveEncoder = driveMotor.getEncoder();
        // Convert from "rotations of motor" to "meters driven by wheel"
        this.driveEncoder.setPositionConversionFactor(WHEEL_DIAMETER * Math.PI / GEAR_RATIO);
        // Convert from "rotations per minute of motor" to "meters per second of wheel"
        this.driveEncoder.setVelocityConversionFactor(Units.inchesToMeters(WHEEL_DIAMETER * Math.PI / GEAR_RATIO / 60));
        this.drivePID = this.driveMotor.getPIDController();
        this.drivePID.setP(DRIVE_P);
        this.drivePID.setI(DRIVE_I);
        this.drivePID.setD(DRIVE_D);
        this.drivePID.setFF(DRIVE_FF);

        this.turnEncoder = new DutyCycleEncoder(turnEncoderDIOPort); // Assumes we're using a absolute encoder
        this.turnEncoderPort = turnEncoderDIOPort;

        driveMotor.setSmartCurrentLimit(60);
        driveMotor.setSecondaryCurrentLimit(80);
        turnMotor.setSmartCurrentLimit(40);
        turnMotor.setSecondaryCurrentLimit(60);

        turnPID = new PIDController(TURN_P, TURN_I, TURN_D);

        distanceDrivenMeters = 0;
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
        turnMotor.set(MathUtil.clamp(turnPID.calculate(getAngle(), state.angle.getDegrees()), -1, 1));
        // Calculate rive motor power from a speed in m/s, and set the motor power to
        // that
        // driveMotor.set(speedMetersPerSecondToMotorPower(state.speedMetersPerSecond));
        drivePID.setReference(state.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);

        // Calculate distance driven by the module for odometry/kinematics
        distanceDrivenMeters = getDistanceDriven();

        SmartDashboard.putNumber("Module " + turnEncoderPort + " current angle (degrees)", getAngle());
        SmartDashboard.putNumber("Module " + turnEncoderPort + " desired angle (degrees)", state.angle.getDegrees());
        SmartDashboard.putNumber("Module " + turnEncoderPort + " current velocity (m/s)", driveEncoder.getVelocity());
        SmartDashboard.putNumber("Module " + turnEncoderPort + " desired velocity (m/s)", state.speedMetersPerSecond);
    }

    public void stop() {
        turnMotor.stopMotor();
        driveMotor.stopMotor();
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(distanceDrivenMeters, Rotation2d.fromDegrees(getAngle()));
    }

    public void resetPosition() {
        distanceDrivenMeters = 0;
    }

    public double getAngle() {
        /*
         * TODO: Find out if getAbsolutePosition is from 0 to 360 or -180 to 180
         * If it's the latter no need to do anything, if it's the former we have to
         * subtract by 180
         */
        return turnEncoder.getAbsolutePosition();
    }

    public double getDistanceDriven() {
        return driveEncoder.getPosition();
    }
}
