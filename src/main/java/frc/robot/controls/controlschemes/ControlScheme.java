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
    public abstract double getSwerveSide();

    public abstract double getSwerveFW();

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

    public abstract boolean doConeHigh();

    public abstract boolean doConeMid();

    public abstract boolean doCubeMid();

    public abstract boolean doCubeHigh();

    public abstract boolean doAutoPickup();

    public abstract boolean doPlayerStation();

    // Misc
    public abstract boolean getRawButton(int controller, int buttonId);

    public abstract boolean corbinLock();

    public abstract boolean spencerLock();

    public abstract boolean fixForward();

    public abstract boolean doRobotRelative();
}
