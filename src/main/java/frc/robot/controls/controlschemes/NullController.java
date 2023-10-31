package frc.robot.controls.controlschemes;

/**
 * A control scheme where no controller is attached. Fallback.
 */
public class NullController extends ControlScheme {

    @Override
    public boolean doSlowMode() {

        return false;
    }

    @Override
    public boolean toggleLimelight() {
        return false;
    }

    @Override
    public double getSwerveX() {
        return 0;
    }

    @Override
    public double getSwerveY() {
        return 0;
    }

    @Override
    public double getSwerveTurn() {
        return 0;
    }

    @Override
    public boolean intake() {
        return false;
    }

    @Override
    public boolean outtake() {
        return false;
    }

    @Override
    public double getArmPivot() {
        return 0;
    }

    @Override
    public double getArmExtend() {
        return 0;
    }

    @Override
    public boolean doLimitOverride() {
        return false;
    }

    @Override
    public boolean doResetTurret() {
        return false;
    }

    @Override
    public boolean doConeHigh() {
        return false;
    }

    @Override
    public boolean doConeMid() {
        return false;
    }

    @Override
    public boolean doAutoPickup() {
        return false;
    }

    @Override
    public boolean doPlayerStation() {
        return false;
    }

    @Override
    public boolean getRawButton(int controller, int buttonId) {
        return false;
    }

    @Override
    public boolean doCubeMid() {
        // TODO Auto-generated method stub
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
    public boolean doCubeHigh() {
        // TODO Auto-generated method stub
        return false;
    }

    @Override
    public boolean resetYaw() {
        // TODO Auto-generated method stub
        return false;
    }
}
