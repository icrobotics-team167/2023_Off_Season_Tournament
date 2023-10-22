package frc.robot.subsystems.turret;

public record TurretPosition(double pivotAngle, double extensionPosition) {
    public TurretPosition withPivot(double pivot) {
        return new TurretPosition(pivot, this.extensionPosition);
    }

    public TurretPosition withExtension(double extension) {
        return new TurretPosition(this.pivotAngle, extension);
    }

    // commonly used positions
    public static final TurretPosition INITIAL = new TurretPosition(60, 17);
    // public static final TurretPosition MID_GOAL = new TurretPosition(36, 0, 20.1);
    // public static final TurretPosition HIGH_GOAL_CENTER = new TurretPosition(35, 0, 39.6);
    public static final TurretPosition INTAKE = new TurretPosition(-33, 17);
    public static final TurretPosition PLAYER_STATION = new TurretPosition(29.75, 33.4);

    //TODO: Actually find this value
    public static final TurretPosition CONE_HIGH = new TurretPosition(35.2, 52.0);

    //TODO: Actually find this value
    public static final TurretPosition CONE_MID = new TurretPosition(27.0, 35.3);

    public static final TurretPosition CUBE_MID = new TurretPosition(12.49, 27.2);

    public static final TurretPosition CUBE_HIGH = new TurretPosition(26.0,49.0);

}
