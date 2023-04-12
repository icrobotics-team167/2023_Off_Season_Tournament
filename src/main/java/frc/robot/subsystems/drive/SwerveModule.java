package frc.robot.subsystems.drive;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.*;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.PeriodicTimer;

public class SwerveModule {
    private final CANSparkMax driveMotor;
    private final CANSparkMax turnMotor;
    private final RelativeEncoder driveEncoder;
    private final DutyCycleEncoder turnEncoder;
    private final int turnEncoderPort;

    private PIDController turnPID;
    private final double TURN_P = 1.0 / 90.0;
    private final double TURN_I = 0;
    private final double TURN_D = 0.5;
    private PeriodicTimer pidTimer;

    private double distanceDrivenMeters;

    public SwerveModule(int driveMotorID, int turnMotorID, int turnEncoderDIOPort) {
        this.driveMotor = new CANSparkMax(driveMotorID, CANSparkMaxLowLevel.MotorType.kBrushless);
        this.turnMotor = new CANSparkMax(turnMotorID, CANSparkMaxLowLevel.MotorType.kBrushless);

        this.driveEncoder = driveMotor.getEncoder();

        this.turnEncoder = new DutyCycleEncoder(turnEncoderDIOPort); // Assumes we're using a absolute encoder
        this.turnEncoderPort = turnEncoderDIOPort;

        driveMotor.setSmartCurrentLimit(60);
        driveMotor.setSecondaryCurrentLimit(80);
        turnMotor.setSmartCurrentLimit(40);
        turnMotor.setSecondaryCurrentLimit(60);

        pidTimer = new PeriodicTimer();
        pidTimer.reset();
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
        driveMotor.set(speedMetersPerSecondToMotorPower(state.speedMetersPerSecond));

        // Calculate distance driven by the module for odometry/kinematics
        distanceDrivenMeters = motorEncoderToMeters(driveEncoder.getPosition());

        SmartDashboard.putNumber("Module " + turnEncoderPort + " current angle", getAngle());
        SmartDashboard.putNumber("Module " + turnEncoderPort + " desired angle", state.angle.getDegrees());
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

    private double speedMetersPerSecondToMotorPower(double speedMetersPerSecond) {
        // TODO: Figure out right scale factor for speedMetersPerSecondToMotorPower
        final double SCALE_FACTOR = 1;
        return MathUtil.clamp(speedMetersPerSecond * SCALE_FACTOR, -1, 1);
    }

    private double motorEncoderToMeters(double encoderValue) {
        // TODO: Figure out right scale factor for motorEncoderToMeters
        // Might be redundant if we set setPositionScaleFactor() right
        final double SCALE_FACTOR = 1;
        return encoderValue * SCALE_FACTOR;
    }
}
