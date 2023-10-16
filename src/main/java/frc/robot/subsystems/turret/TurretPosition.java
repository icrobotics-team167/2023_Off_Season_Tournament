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

    // public static final TurretPosition HIGH_RIGHT = new TurretPosition(33, 15.19, 56);
    // public static final TurretPosition HIGH_LEFT = HIGH_RIGHT.withSwivel(-18);
    // public static final TurretPosition HIGH_MID = new TurretPosition(33, 0, 51.3);
    // public static final TurretPosition MID_RIGHT = new TurretPosition(25, 21.2, 35);
    // public static final TurretPosition MID_LEFT = new TurretPosition(28.7, -28, 38.5);
    // public static final TurretPosition MID_MID = new TurretPosition(17.3, 0,30.2);

    // // max extension can reach 2 more mid scoring positions PLEASE TEST THESE TOO
    // // (LOW PRIORITY)
    // public static final TurretPosition FAR_MID_RIGHT = new TurretPosition(35, 46, 42);
    // public static final TurretPosition FAR_MID_LEFT = FAR_MID_RIGHT.flipSwivelSign();

    // // positions for auto only
    // public static final TurretPosition HIGH_GOAL_CUBE_BLUE = new TurretPosition(34, -4.97, 52); // positioning for auto
    // public static final TurretPosition HIGH_GOAL_CUBE_RED = HIGH_GOAL_CUBE_BLUE.flipSwivelSign(); // positioning for
    //                                                                                               // auto
    // public static final TurretPosition HIGH_GOAL_CONE_BLUE = new TurretPosition(34, 4.97, 51); // positioning for auto
    // public static final TurretPosition HIGH_GOAL_CONE_RED = HIGH_GOAL_CONE_BLUE.flipSwivelSign(); // positioning for
                                                                                                  // auto

}
