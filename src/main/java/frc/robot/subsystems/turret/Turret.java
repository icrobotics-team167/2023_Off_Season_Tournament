package frc.robot.subsystems.turret;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Config;
import frc.robot.util.MathUtils;

public class Turret {
    private Pivot pivot;
    private ExtendRetract extendRetract;

    private static Turret instance;

    private final double PIVOT_SENSITIVITY_THRESHOLD = 2;
    private final double EXTENSION_SENSITIVITY_THRESHOLD = 1;

    public static Turret getInstance() {
        if (instance == null) {
            instance = new Turret();
        }
        return instance;
    }

    private Turret() {
        pivot = Pivot.getInstance();
        extendRetract = ExtendRetract.getInstance();
    }

    /**
     * Moves multiple parts of the arm at the same time.
     * 
     * @param pivotSpeed         The pivot speed
     * @param extendRetractSpeed The extension speed
     */
    public void move(double pivotSpeed, double extendRetractSpeed) {
        pivot.move(pivotSpeed);
        extendRetract.move(extendRetractSpeed);
    }

    /**
     * Moves multiple parts of the turret at the same time.
     * 
     * @param targetState The target turret state.
     * 
     * @return If it has finished moving (AKA when all the targets are met)
     */
    public boolean moveTo(TurretPosition targetState) {
        return moveTo(targetState, 1);
    }

    /**
     * Moves multiple parts of the turret at the same time.
     * 
     * @param targetState The target turret state.
     * @param speed       The turret movement speed.
     * 
     * @return If it has finished moving (AKA when all the targets are met)
     */
    public boolean moveTo(TurretPosition targetState, double speed) {
        boolean pivot = pivotToAngle(targetState.pivotAngle(), speed);
        boolean extend = extendToPosition(targetState.extensionPosition());
        return pivot && extend;
    }

    public void stop() {
        pivot.stop();
        extendRetract.stop();
    }

    public TurretPosition getPosition() {
        return new TurretPosition(pivot.getPositionDegrees(), extendRetract.getPositionInches());
    }

    public boolean pivotToAngle(double target) {
        return pivotToAngle(target, 1);
    }

    public boolean pivotToAngle(double target, double moveSpeed) {
        double speed;
        double error = target - pivot.getPositionDegrees();
        if (Math.abs(error) < PIVOT_SENSITIVITY_THRESHOLD) {
            speed = 0;
        } else {
            speed = MathUtils.getSign(error);
        }
        speed *= moveSpeed;
        pivot.move(speed);
        return speed == 0;
    }

    public boolean extendToPosition(double target) {
        return extendToPosition(target, 1);
    }

    public boolean extendToPosition(double target, double moveSpeed) {
        double speed;
        double error = target - extendRetract.getPositionInches();
        if (Math.abs(error) < EXTENSION_SENSITIVITY_THRESHOLD) {
            speed = 0;
        } else {
            speed = MathUtils.getSign(error);
        }
        speed *= moveSpeed;
        extendRetract.move(speed);
        return speed == 0;
    }

    public void setLimitOverride(boolean newVal) {
        pivot.setLimitOverride(newVal);
        extendRetract.setLimitOverride(newVal);
    }

    public void setSlowMode(boolean slow) {
        pivot.setSlowMode(slow);
    }
}
