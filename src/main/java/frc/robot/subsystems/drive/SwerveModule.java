package frc.robot.subsystems.drive;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.MathUtils;
import frc.robot.util.PID;
import frc.robot.util.PeriodicTimer;

public class SwerveModule {
    private final CANSparkMax driveMotor;
    private final CANSparkMax turnMotor;
    private final RelativeEncoder driveEncoder;
    private final DutyCycleEncoder turnEncoder;
    private final int turnEncoderPort;

    private PID turnPID;
    private final double TURN_P = 1.0 / 90.0;
    private final double TURN_I = 0;
    private final double TURN_D = 0.5;
    private final double TURN_D_POWER = 15;
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
        turnPID = new PID(TURN_P, TURN_I, TURN_D, TURN_D_POWER, 180, pidTimer.get(), 0);

        distanceDrivenMeters = 0;
    }

    /**
     * Moves the swerve module.
     * 
     * @param state The desired SwerveModuleState
     */
    public void move(SwerveModuleState state) {
        state = SwerveModuleState.optimize(state, Rotation2d.fromDegrees(getAngle()));
        turnPID.setTarget(state.angle.getDegrees());
        turnMotor.set(turnPID.compute(getAngle(), pidTimer.get()));
        driveMotor.set(speedMetersPerSecondToMotorPower(state.speedMetersPerSecond));

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
