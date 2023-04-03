package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Translation2d;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

public class SwerveModule {
    private final Translation2d position;

    private final CANSparkMax driveMotor;
    private final CANSparkMax turnMotor;

    public SwerveModule(Translation2d position, int driveMotorID, int turnMotorID) {
        this.position = position;
        this.driveMotor = new CANSparkMax(driveMotorID, CANSparkMaxLowLevel.MotorType.kBrushless);
        this.turnMotor = new CANSparkMax(turnMotorID, CANSparkMaxLowLevel.MotorType.kBrushless);
    }
}
