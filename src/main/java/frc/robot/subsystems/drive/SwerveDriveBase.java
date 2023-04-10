package frc.robot.subsystems.drive;

import java.util.List;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.Config;
import frc.robot.subsystems.Subsystems;

public class SwerveDriveBase {
    private static SwerveDriveBase instance;

    private SwerveModule[] modules;
    private SwerveDriveKinematics kinematics;
    private SwerveModulePosition[] modulePositions;
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
        modulePositions = new SwerveModulePosition[] {
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition(),
        };
        kinematics = new SwerveDriveKinematics(
                Config.Ports.SwerveDrive.FRONT_LEFT_POS,
                Config.Ports.SwerveDrive.FRONT_RIGHT_POS,
                Config.Ports.SwerveDrive.BACK_LEFT_POS,
                Config.Ports.SwerveDrive.BACK_RIGHT_POS);
        odometry = new SwerveDriveOdometry(kinematics, new Rotation2d(0), modulePositions);
    }

    /**
     * Drives the robot.
     * 
     * @param xSpeed        Forwards/backwards motion, in meters/second. Positive is
     *                      forwards, negative is backwards.
     * @param ySpeed        Left/right motion, in meters/second. Positive is right,
     *                      negative is left.
     * @param rotationSpeed Rotation, in in radians/second. Positive is clockwise,
     *                      negative is counterclockwise.
     */
    public void drive(double xSpeed, double ySpeed, double rotationSpeed) {
        // TODO: Implement drive()
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, rotationSpeed);
        SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(chassisSpeeds);

        for (int i = 0; i < modules.length; i++) {
            modules[i].move(moduleStates[i]);
        }

        odometry.update(new Rotation2d(Units.degreesToRadians(Subsystems.navx.getAngle())), modulePositions);
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
