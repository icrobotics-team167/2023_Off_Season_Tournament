package frc.robot.controls.controlschemes;

/**
 * <p>
 * An abstraction layer for control schemes. See the other control scheme
 * classes for implementation details.
 * 
 * <p>
 * Implemented by:
 * {@link frc.robot.controls.controlschemes.DeltaJoystickController}
 * {@link frc.robot.controls.controlschemes.NullController}
 */
public abstract class ControlScheme {

    // Driving (Primary driver's controller)
    public abstract double getSwerveX();

    public abstract double getSwerveY();

    public abstract double getSwerveTurn();

    public abstract boolean doSlowMode();

    public abstract boolean toggleLimelight();

    // Claw
    public abstract boolean intake();

    public abstract boolean outtake();

    // Arm controls
    public abstract double getArmPivot();

    public abstract double getArmExtend();

    public abstract boolean doLimitOverride();

    public abstract boolean doResetTurret();

    public abstract boolean doAutoHigh();

    public abstract boolean doAutoMid();

    public abstract boolean doAutoPickup();

    public abstract boolean doPlayerStation();
}
