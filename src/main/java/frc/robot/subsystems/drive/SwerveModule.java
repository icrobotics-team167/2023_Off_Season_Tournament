package frc.robot.subsystems.drive;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.MathUtils;
import frc.robot.util.PID;
import frc.robot.util.PeriodicTimer;

public class SwerveModule {
    private final Translation2d position;
    private final CANSparkMax driveMotor;
    private final CANSparkMax turnMotor;
    private final DutyCycleEncoder turnEncoder;
    private final int turnEncoderPort;

    public PID turnPID;
    public final double turnP = 1.0 / 90.0;
    public final double turnI = 0;
    public final double turnD = 0.5;
    public final double turnDPower = 15;
    public PeriodicTimer pidTimer;

    public SwerveModule(Translation2d position, int driveMotorID, int turnMotorID, int turnEncoderDIOPort) {
        this.position = position;
        this.driveMotor = new CANSparkMax(driveMotorID, CANSparkMaxLowLevel.MotorType.kBrushless);
        this.turnMotor = new CANSparkMax(turnMotorID, CANSparkMaxLowLevel.MotorType.kBrushless);

        this.turnEncoder = new DutyCycleEncoder(turnEncoderDIOPort); // Assumes we're using a absolute encoder
        this.turnEncoderPort = turnEncoderDIOPort;

        driveMotor.setSmartCurrentLimit(60);
        driveMotor.setSecondaryCurrentLimit(80);
        turnMotor.setSmartCurrentLimit(40);
        turnMotor.setSecondaryCurrentLimit(60);

        pidTimer = new PeriodicTimer();
        pidTimer.reset();
        turnPID = new PID(turnP, turnI, turnD, turnDPower, 180, pidTimer.get(), 0);
    }

    /**
     * Moves the swerve module.
     * 
     * @param state The desired SwerveModuleState
     */
    public void move(SwerveModuleState state) {
        // TODO: Find out if turnEncoder.getAbsolutePosition is from 0 to 360 or -180 to
        // 180
        state = SwerveModuleState.optimize(state, new Rotation2d(getAngle()));
        turnPID.setTarget(state.angle.getDegrees());
        turnMotor.set(turnPID.compute(turnEncoder.getAbsolutePosition(), pidTimer.get()));
        driveMotor.set(speedMetersPerSecondToMotorPower(state.speedMetersPerSecond));

        SmartDashboard.putNumber("Module " + turnEncoderPort + " current angle", getAngle());
        SmartDashboard.putNumber("Module " + turnEncoderPort + " desired angle", state.angle.getDegrees());
    }

    public void stop() {
        turnMotor.stopMotor();
        driveMotor.stopMotor();
    }

    public Translation2d getPosition() {
        return position;
    }

    public double getAngle() {
        return turnEncoder.getAbsolutePosition();
    }

    public SwerveModuleState getState() {
        // TODO: Implement getState()
        throw new UnsupportedOperationException("Unimplemented method \"getState\"");
    }

    private double speedMetersPerSecondToMotorPower(double speedMetersPerSecond) {
        // TODO: Implement speedMetersPerSecondToMotorPower()
        throw new UnsupportedOperationException("Unimplemented method \"speedMetersPerSecondToMotorPower\"");
    }
}
