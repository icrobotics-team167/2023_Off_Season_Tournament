package frc.robot.subsystems.drive;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.util.MathUtils;
import frc.robot.util.PID;

public class SwerveModule {
    private final Translation2d position;
    private final CANSparkMax driveMotor;
    private final CANSparkMax turnMotor;
    private final DutyCycleEncoder turnEncoder;

    public PID turnPID;
    public final double turnP = 1.0 / 180.0;
    public final double turnI = 0;
    public final double turnD = 0.5;
    public final double turnDPower = 15;

    public SwerveModule(Translation2d position, int driveMotorID, int turnMotorID, int turnEncoderDIOPort) {
        this.position = position;
        this.driveMotor = new CANSparkMax(driveMotorID, CANSparkMaxLowLevel.MotorType.kBrushless);
        this.turnMotor = new CANSparkMax(turnMotorID, CANSparkMaxLowLevel.MotorType.kBrushless);

        this.turnEncoder = new DutyCycleEncoder(turnEncoderDIOPort); // Assumes we're using a absolute encoder

        driveMotor.setSmartCurrentLimit(60);
        driveMotor.setSecondaryCurrentLimit(80);
        turnMotor.setSmartCurrentLimit(40);
        turnMotor.setSecondaryCurrentLimit(60);

        turnPID = new PID(turnP, turnI, turnD, 15, 180, 0, 0);
    }

    public void move(SwerveModuleState state) {
        state = SwerveModuleState.optimize(state, new Rotation2d(turnEncoder.getAbsolutePosition()));
    }
}
