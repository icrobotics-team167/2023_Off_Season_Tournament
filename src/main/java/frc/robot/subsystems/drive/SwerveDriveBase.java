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

/**
 * A swerve drive.
 */
public class SwerveDriveBase {
    private static SwerveDriveBase instance;

    private SwerveModule[] modules;
    private SwerveDriveKinematics kinematics;
    private SwerveModulePosition[] moduleOdometry;
    private SwerveDriveOdometry odometry;

    /**
     * Only allows one instance of SwerveDriveBase to exist at once.
     * 
     * @return The instance. Will make a new instance if one doesn't exist already.
     */
    public static SwerveDriveBase getInstance() {
        if (instance == null) {
            instance = new SwerveDriveBase();
        }
        return instance;
    }

    private SwerveDriveBase() {
        // Set up swerve modules
        modules = new SwerveModule[] {
                new SwerveModule(Config.Ports.SwerveDrive.FRONT_LEFT_DRIVE, Config.Ports.SwerveDrive.FRONT_LEFT_TURN,
                        Config.Ports.SwerveDrive.FRONT_LEFT_ENCODER, Config.Ports.SwerveDrive.FRONT_LEFT_OFFSET),
                new SwerveModule(Config.Ports.SwerveDrive.FRONT_RIGHT_DRIVE, Config.Ports.SwerveDrive.FRONT_RIGHT_TURN,
                        Config.Ports.SwerveDrive.FRONT_RIGHT_ENCODER, Config.Ports.SwerveDrive.FRONT_RIGHT_OFFSET),
                new SwerveModule(Config.Ports.SwerveDrive.BACK_LEFT_DRIVE, Config.Ports.SwerveDrive.BACK_LEFT_TURN,
                        Config.Ports.SwerveDrive.BACK_LEFT_ENCODER, Config.Ports.SwerveDrive.BACK_LEFT_OFFSET),
                new SwerveModule(Config.Ports.SwerveDrive.BACK_RIGHT_DRIVE, Config.Ports.SwerveDrive.BACK_RIGHT_TURN,
                        Config.Ports.SwerveDrive.BACK_RIGHT_ENCODER, Config.Ports.SwerveDrive.BACK_RIGHT_OFFSET),
        };
        // Initialize odometry and kinematics
        moduleOdometry = new SwerveModulePosition[] {
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
        odometry = new SwerveDriveOdometry(kinematics, new Rotation2d(), moduleOdometry);
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
            moduleOdometry[i] = modules[i].getPosition();
        }

        // Update the whole chassis's odometry using the individual module's odometry
        odometry.update(Subsystems.gyro.getYaw(), moduleOdometry);
        // odometry.update(Rotation2d.fromDegrees(0), modulePositions);

        // DEBUG
        SmartDashboard.putNumber("SwerveDriveBase.xPosMeters", odometry.getPoseMeters().getX());
        SmartDashboard.putNumber("SwerveDriveBase.yPosMeters", odometry.getPoseMeters().getY());
    }

    /**
     * Drives the robot relative to the driver station.
     * 
     * @param vx    Forwards/backwards velocity in m/s. Positive is away from the
     *              station, negative is towards the station.
     * @param vy    Left/right velocity in m/s. Positive is left relative to the
     *              station, negative is right relative to the station.
     * @param vr    Turning velocity in radians/s. Positive is counterclockwise,
     *              negative is clockwise.
     * @param angle The current robot angle, as measured by a gyroscope. 0 degrees
     *              is facing away from the driver station. Positive is
     *              counterclockwise, negative is clockwise.
     */
    public void fieldOrientedDrive(double vx, double vy, double vr, Rotation2d angle) {
        drive(ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, vr, angle));
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
            moduleOdometry[i] = modules[i].getPosition();
        }
        odometry.update(Subsystems.gyro.getYaw(), moduleOdometry);
        // odometry.update(Rotation2d.fromDegrees(0), modulePositions);
    }

    /**
     * Sends module telemetry to SmartDashboard.
     */
    public void sendTelemetry() {
        for (SwerveModule module : modules) {
            module.sendTelemetry();
        }
    }

    /**
     * Get a specific module for testing.
     * 
     * @param id The ID of the module. 0 is front left, 1 is front right, 2 is back
     *           left, 3 is back right.
     * @return The specified module.
     */
    public SwerveModule getModule(int id) {
        return modules[id];
    }
}
