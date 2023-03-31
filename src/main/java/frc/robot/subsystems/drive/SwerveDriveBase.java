package frc.robot.subsystems.drive;

public class SwerveDriveBase {
    private static SwerveDriveBase instance;

    public static SwerveDriveBase getInstance() {
        if (instance == null) {
            instance = new SwerveDriveBase();
        }
        return instance;
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
