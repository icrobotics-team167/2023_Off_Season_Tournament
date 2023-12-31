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
        return MathUtils.deadZone(tertiary.getLeftStickY(), Config.Tolerances.TERTIARY_CONTROLLER_DEADZONE_SIZE);
    }

    @Override
    public double getArmExtend() {
        return MathUtils.deadZone(quaternary.getLeftStickY(), Config.Tolerances.QUATERNARY_CONTROLLER_DEADZONE_SIZE);
    }

    @Override
    public boolean doLimitOverride() {
        return false;
    }

    @Override
    public boolean doResetTurret() {
        return quaternary.getButtonById(2);
    }
    
    @Override
    public boolean doConeHigh() {
        return quaternary.getButtonById(6);    
        
    }

    @Override
    public boolean doConeMid() {
        return quaternary.getButtonById(9);
        
    }

    @Override
    public boolean doAutoPickup() {
        return quaternary.getButtonById(8);    
    }

    @Override
    public boolean doPlayerStation() {
        return quaternary.getButtonById(7);   
    }

    @Override
    public boolean doCubeMid() {
        return quaternary.getButtonById(10);
    }

    @Override
    public boolean doCubeHigh() {
        return quaternary.getButtonById(5);
    }

    @Override
    public boolean getRawButton(int controller, int buttonId) {
        return false;
    }

    @Override
    public boolean corbinLock() {
        // TODO Auto-generated method stub
        return false;
    }

    @Override
    public boolean spencerLock() {
        // TODO Auto-generated method stub
        return false;
    }

    @Override
    public boolean resetYaw() {
        return secondary.getButtonPressedById(2);
    }
}
