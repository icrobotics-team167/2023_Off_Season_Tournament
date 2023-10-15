package frc.robot.routines;

import frc.robot.controls.controlschemes.ControlScheme;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.Subsystems;
import frc.robot.subsystems.drive.SwerveDriveBase;
import frc.robot.subsystems.turret.Claw;
import frc.robot.subsystems.turret.Turret;

public class Test {
    private ControlScheme controls;
    private SwerveDriveBase driveBase;

    private LimeLight limeLight = Subsystems.limeLight;
    private Turret turret = Subsystems.turret;
    private Claw claw = Subsystems.claw;

    public Test(ControlScheme controls) {
        this.controls = controls;
        driveBase = Subsystems.driveBase;
    }

    /**
     * Runs once at the start of test mode
     */
    public void init() {
        driveBase.resetPosition();
    }

    public void periodic() {
        // Drivebase motor testing
        // TODO: Implement
        throw new UnsupportedOperationException("Unimplemented method: periodic");
    }
}
