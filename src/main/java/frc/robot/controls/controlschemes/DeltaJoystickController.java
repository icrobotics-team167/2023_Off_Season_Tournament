package frc.robot.controls.controlschemes;

import frc.robot.Config;
import frc.robot.controls.controllers.Controller;
import frc.robot.util.MathUtils;

/**
 * <p>
 * A control scheme that uses 4 flightsim joysticks. Default control scheme.
 * <p>
 * Delta is the fourth Greek letter and we have four joysticks.
 * Q.E.D.
 */
public class DeltaJoystickController extends ControlScheme {

    private Controller primary;
    private Controller secondary;
    private Controller tertiary;
    private Controller quaternary;

    /**
     * Constructs a new control scheme, with 4 flightsim joysticks.
     * 
     * @param primaryLeft    The primary driver's left stick.
     * @param primaryRight   The primary driver's right stick.
     * @param secondaryLeft  The secondary driver's left stick.
     * @param secondaryRight The secondary driver's right stick.
     */
    public DeltaJoystickController(Controller primaryLeft, Controller primaryRight, Controller secondaryLeft,
            Controller secondaryRight) {
        this.primary = primaryLeft;
        this.secondary = primaryRight;
        this.tertiary = secondaryLeft;
        this.quaternary = secondaryRight;
    }

    @Override
    public double getSwerveX() {
        return MathUtils.deadZone(primary.getLeftStickX(), Config.Tolerances.PRIMARY_CONTROLLER_DEADZONE_SIZE);
    }

    @Override
    public double getSwerveY() {
        return MathUtils.deadZone(primary.getLeftStickY(), Config.Tolerances.PRIMARY_CONTROLLER_DEADZONE_SIZE);
    }

    @Override
    public double getSwerveTurn() {
        return -MathUtils.deadZone(secondary.getLeftStickX(), Config.Tolerances.SECONDARY_CONTROLLER_DEADZONE_SIZE);
    }

    @Override
    public boolean doSlowMode() {
        return primary.getLeftTrigger();
    }

    @Override
    public boolean toggleLimelight() {
        return secondary.getButtonPressedById(2);
    }

    @Override
    public boolean intake() {
        return quaternary.getButtonById(3);
    }

    @Override
    public boolean outtake() {
        return quaternary.getButtonById(4);
    }

    @Override
    public double getArmPivot() {
        return (Math.abs(tertiary.getLeftStickY()) >= Config.Tolerances.TERTIARY_CONTROLLER_DEADZONE_SIZE
                ? tertiary.getLeftStickY()
                : 0);
    }

    @Override
    public double getArmExtend() {
        return (Math.abs(quaternary.getLeftStickY()) >= Config.Tolerances.QUATERNARY_CONTROLLER_DEADZONE_SIZE
                ? quaternary.getLeftStickY()
                : 0);
    }

    @Override
    public boolean doLimitOverride() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'doLimitOverride'");
    }

    @Override
    public boolean doResetTurret() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'doResetTurret'");
    }

    @Override
    public boolean doAutoHigh() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'doAutoHigh'");
    }

    @Override
    public boolean doAutoMid() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'doAutoMid'");
    }

    @Override
    public boolean doAutoPickup() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'doAutoPickup'");
    }

    @Override
    public boolean doPlayerStation() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'doPlayerStation'");
    }

}
