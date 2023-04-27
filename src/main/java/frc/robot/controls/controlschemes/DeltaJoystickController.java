package frc.robot.controls.controlschemes;

import frc.robot.Config;
import frc.robot.controls.controllers.Controller;

/**
 * <p>
 * A control scheme that uses 4 flightsim joysticks. Default control scheme.
 * <p>
 * Delta is the fourth Greek letter and we have four joysticks.
 * Q.E.D.
 */
public class DeltaJoystickController extends ControlScheme {

    private Controller primaryLeft;
    private Controller primaryRight;
    private Controller secondaryLeft;
    private Controller secondaryRight;

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
        this.primaryLeft = primaryLeft;
        this.primaryRight = primaryRight;
        this.secondaryLeft = secondaryLeft;
        this.secondaryRight = secondaryRight;
    }

    @Override
    public double getSwerveX() {
        return primaryLeft.getLeftStickX() >= Config.Tolerances.PRIMARY_CONTROLLER_DEADZONE_SIZE
                ? primaryLeft.getLeftStickX()
                : 0;
    }

    @Override
    public double getSwerveY() {
        return primaryLeft.getLeftStickY() >= Config.Tolerances.PRIMARY_CONTROLLER_DEADZONE_SIZE
                ? primaryLeft.getLeftStickY()
                : 0;
    }

    @Override
    public double getSwerveTurn() {
        return primaryRight.getLeftStickX() >= Config.Tolerances.PRIMARY_CONTROLLER_DEADZONE_SIZE
                ? primaryRight.getLeftStickX()
                : 0;
    }

    @Override
    public boolean doSlowMode() {

        return false;
    }

    @Override
    public boolean toggleLimelight() {
        return primaryRight.getButtonPressedById(2);
    }

}
