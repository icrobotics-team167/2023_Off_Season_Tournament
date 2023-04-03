package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Translation2d;

public class SwerveModule {
    private final Translation2d position;

    private final int driveMotorID;
    private final int turnMotorID;

    public SwerveModule(Translation2d position, int driveMotorID, int turnMotorID) {
        this.position = position;
        this.driveMotorID = driveMotorID;
        this.turnMotorID = turnMotorID;
    }
}
