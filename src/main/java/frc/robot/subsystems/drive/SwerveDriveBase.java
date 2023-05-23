package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
                new SwerveModule(Config.Ports.SwerveDrive.FRONT_LEFT_DRIVE, Config.Ports.SwerveDrive.FRONT_LEFT_TURN,
                        0),
                new SwerveModule(Config.Ports.SwerveDrive.FRONT_RIGHT_DRIVE, Config.Ports.SwerveDrive.FRONT_RIGHT_TURN,
                        1),
                new SwerveModule(Config.Ports.SwerveDrive.BACK_LEFT_DRIVE, Config.Ports.SwerveDrive.BACK_LEFT_TURN,
                        2),
                new SwerveModule(Config.Ports.SwerveDrive.BACK_RIGHT_DRIVE, Config.Ports.SwerveDrive.BACK_RIGHT_TURN,
                        3),
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
        odometry = new SwerveDriveOdometry(kinematics, new Rotation2d(), modulePositions);
    }

    /**
     * Drives the robot.
     * 
     * @param xSpeed        Forwards/backwards velocity, in meters/second. Positive
     *                      is forwards, negative is backwards.
     * @param ySpeed        Left/right velocity, in meters/second. Positive is left,
     *                      negative is right.
     * @param rotationSpeed Rotation velocity, in in radians/second. Positive is
     *                      counterclockwise, negative is clockwise.
     */
    public void drive(double xSpeed, double ySpeed, double rotationSpeed) {
        // Exists so that I don't have to do ChassisSpeed stuff every time
        drive(new ChassisSpeeds(xSpeed, ySpeed, rotationSpeed));
    }

    /**
     * Drives the robot.
     * 
     * @param chassisSpeeds The desired motion of the robot, represented as a set of
     *                      velocities
     */
    public void drive(ChassisSpeeds chassisSpeeds) {
        // Generates desired states for the modules. May not be optimized.
        SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(chassisSpeeds);

        // Loop through every module
        for (int i = 0; i < modules.length; i++) {
            // Move them to the desired state
            modules[i].move(moduleStates[i]);
            // And update their odometry
            modulePositions[i] = modules[i].getPosition();
        }

        // Update the whole chassis's odometry using the individual module's odometry
        odometry.update(Rotation2d.fromDegrees(Subsystems.navx.getAngle()), modulePositions);

        // DEBUG
        SmartDashboard.putNumber("SwerveDriveBase.xPosMeters", odometry.getPoseMeters().getX());
        SmartDashboard.putNumber("SwerveDriveBase.yPosMeters", odometry.getPoseMeters().getY());
    }

    /**
     * Stops the robot.
     * Equivalent to running drive(0, 0, 0).
     */
    public void stop() {
        drive(0, 0, 0);
    }

    /**
     * Resets the odometry of the robot.
     */
    public void resetPosition() {
        for (int i = 0; i < modules.length; i++) {
            modules[i].resetPosition();
            modulePositions[i] = modules[i].getPosition();
        }
        odometry.update(Rotation2d.fromDegrees(Subsystems.navx.getAngle()), modulePositions);
    }
}
