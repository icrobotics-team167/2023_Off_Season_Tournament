package frc.robot;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.controls.controllers.*;
import frc.robot.controls.controlschemes.*;
import frc.robot.routines.Action;
import frc.robot.routines.auto.*;
import frc.robot.routines.Teleop;
import frc.robot.routines.Test;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.Subsystems;

public class Robot extends TimedRobot {

    private SendableChooser<AutoRoutines> autoChooser = new SendableChooser<>();
    private ControlScheme controls;
    private Action auto;
    private Teleop teleop;
    private Test test;
    private LimeLight limeLight;

    public Robot() {
        super(Config.Settings.CPU_PERIOD);
    }

    @Override
    public void robotInit() {
        // Initialize the auto routine selector.
        AutoRoutines defaultRoutine = AutoRoutines.NOTHING; // Default routine is nothing to avoid accidents
        // Loop through the routines enum and add each of them to the routine selector.
        for (AutoRoutines routine : AutoRoutines.values()) {
            if (routine != defaultRoutine) {
                autoChooser.addOption(routine.name, routine);
            } else {
                autoChooser.setDefaultOption(defaultRoutine.name, defaultRoutine);
            }

        }
        SmartDashboard.putData("Autonomous Routines", autoChooser);

        try { // If all 4 controllers are properly initialized, create a control scheme with
              // those 4 controllers
            controls = new DeltaJoystickController(new ThrustMasterController(Config.Ports.PRIMARY_CONTROLLER),
                    new ThrustMasterController(Config.Ports.SECONDARY_CONTROLLER),
                    new ThrustMasterController(Config.Ports.TERTIARY_CONTROLLER),
                    new ThrustMasterController(Config.Ports.QUATERNARY_CONTROLLER));
        } catch (Exception e) { // Otherwise, make a null control scheme
            controls = new NullController();
        }

        limeLight = LimeLight.getInstance();

        Subsystems.setInitialStates();
        teleop = new Teleop(controls);
        test = new Test(controls);
    }

    @Override
    public void robotPeriodic() {
        // Telemetry
        SmartDashboard.putNumber("Robot.batteryVoltage", RobotController.getBatteryVoltage());
        SmartDashboard.putNumber("Robot.yaw", Subsystems.gyro.getYawDegrees());
        SmartDashboard.putNumber("Robot.pitch", Subsystems.gyro.getPitchDegrees());
        SmartDashboard.putNumber("Robot.roll", Subsystems.gyro.getRollDegrees());
        Subsystems.driveBase.sendTelemetry();
        
        // Update velocity
        Subsystems.driveBase.getVelocity();
    }

    @Override
    public void autonomousInit() {
        auto = autoChooser.getSelected().actions;
        limeLight.setVisionMode();
        auto.exec();
        // System.out.println("Auto selected: " + autoChooser.getSelected().name);
    }

    @Override
    public void autonomousPeriodic() {
        auto.exec();
    }

    @Override
    public void teleopInit() {
        teleop.init();
        limeLight.setCameraMode();
    }

    @Override
    public void teleopPeriodic() {
        teleop.periodic();
    }

    @Override
    public void testInit() {
        test.init();
    }

    @Override
    public void testPeriodic() {
        test.periodic();
    }

    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
    }

}
