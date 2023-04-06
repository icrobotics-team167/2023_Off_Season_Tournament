package frc.robot.subsystems.drive;

import java.util.List;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.Config;

public class SwerveDriveBase {
    private static SwerveDriveBase instance;

    private SwerveModule[] modules;
    private SwerveDriveKinematics kinematics;
    private SwerveDriveOdometry odometry;

    public static SwerveDriveBase getInstance() {
        if (instance == null) {
            instance = new SwerveDriveBase();
        }
        return instance;
    }

    private SwerveDriveBase() {
        modules = new SwerveModule[] {
                new SwerveModule(Config.Ports.SwerveDrive.FRONT_LEFT_POS, Config.Ports.SwerveDrive.FRONT_LEFT_DRIVE,
                        Config.Ports.SwerveDrive.FRONT_LEFT_TURN, Config.Ports.SwerveDrive.FRONT_LEFT_ENCODER_PORT),
                new SwerveModule(Config.Ports.SwerveDrive.FRONT_RIGHT_POS, Config.Ports.SwerveDrive.FRONT_RIGHT_DRIVE,
                        Config.Ports.SwerveDrive.FRONT_RIGHT_TURN, Config.Ports.SwerveDrive.FRONT_RIGHT_ENCODER_PORT),
                new SwerveModule(Config.Ports.SwerveDrive.BACK_LEFT_POS, Config.Ports.SwerveDrive.BACK_LEFT_DRIVE,
                        Config.Ports.SwerveDrive.BACK_LEFT_TURN, Config.Ports.SwerveDrive.BACK_LEFT_ENCODER_PORT),
                new SwerveModule(Config.Ports.SwerveDrive.BACK_RIGHT_POS, Config.Ports.SwerveDrive.BACK_RIGHT_DRIVE,
                        Config.Ports.SwerveDrive.BACK_RIGHT_TURN, Config.Ports.SwerveDrive.BACK_RIGHT_ENCODER_PORT),
        };
        SwerveModulePosition[] modulePositions = new SwerveModulePosition[] {
            // TODO: Figure out odemetry
        };
        kinematics = new SwerveDriveKinematics(
                Config.Ports.SwerveDrive.FRONT_LEFT_POS,
                Config.Ports.SwerveDrive.FRONT_RIGHT_POS,
                Config.Ports.SwerveDrive.BACK_LEFT_POS,
                Config.Ports.SwerveDrive.BACK_RIGHT_POS);
        odometry = new SwerveDriveOdometry(kinematics, new Rotation2d(0), modulePositions);
    }

    public void drive(double xSpeed, double ySpeed, double rotationSpeed) {
        // TODO: Implement drive()
        throw new UnsupportedOperationException("Unimplemented method \"drive\"");
    }

    public void stop() {
        drive(0, 0, 0);
    }

    public void setLowGear() {
        // TODO: Implement setLowGear()
        throw new UnsupportedOperationException("Unimplemented method \"setLowGear\"");
    }

    public void setHighGear() {
        // TODO: Implement setHighGear()
        throw new UnsupportedOperationException("Unimplemented method \"setHighGear\"");
    }

    public void resetPosition() {
        // TODO: Implement resetPosition()
        throw new UnsupportedOperationException("Unimplemented method \"resetPosition\"");
    }
}
