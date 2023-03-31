package frc.robot.controls.controlschemes;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Config;
import frc.robot.controls.controllers.Controller;

/**
 * Delta is the fourth Greek letter and we have four joysticks.
 * Q.E.D.
 */
public class DeltaJoystickController extends ControlScheme {

    private Controller primary;
    private Controller secondary;
    private Controller tertiary;
    private Controller quaternary;

    public DeltaJoystickController(Controller primary, Controller secondary, Controller tertiary,
            Controller quaternary) {
        this.primary = primary;
        this.secondary = secondary;
        this.tertiary = tertiary;
        this.quaternary = quaternary;
    }

    @Override
    public double getTankLeftSpeed() {

        return 0;
    }

    @Override
    public double getTankRightSpeed() {

        return 0;
    }

    @Override
    public double getArcadeThrottle() {
        return Math.abs(primary.getLeftStickY()) >= Config.Tolerances.PRIMARY_CONTROLLER_DEADZONE_SIZE
                ? primary.getLeftStickY()
                : 0;
    }

    @Override
    public double getArcadeWheel() {
        return -(Math.abs(secondary.getLeftStickX()) >= Config.Tolerances.SECONDARY_CONTROLLER_DEADZONE_SIZE
                ? secondary.getLeftStickX()
                : 0);
    }

    @Override
    public boolean doSwitchLowGear() {
        return primary.getLeftTrigger();
    }

    @Override
    public boolean doSwitchHighGear() {
        return secondary.getLeftTrigger();
    }

    @Override
    public boolean doSlowMode() {

        return false;
    }

    @Override
    public boolean toggleLimelight() {
        return secondary.getButtonPressedById(2);
    }

}
