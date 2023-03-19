package frc.robot.subsystems.turret;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import frc.robot.Config;

public class Claw {
    private CANSparkMax intakeMotor;

    private static final double INTAKE_SPEED = 0.5;
    private static final double OUTTAKE_SPEED = 0.2;

    private boolean overCurrent;
    private static final double CURRENT_LIMIT = 5; // TEMPORARY VALUE

    public static Claw instance;
    public static Claw getInstance() {
        if (instance == null) {
            instance = new Claw();
        }
        return instance;
    }

    private Claw() {
        intakeMotor = new CANSparkMax(Config.Ports.Arm.CLAW, CANSparkMaxLowLevel.MotorType.kBrushless);

        intakeMotor.restoreFactoryDefaults();
        intakeMotor.setInverted(false);
        intakeMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        intakeMotor.setSmartCurrentLimit(40);
        intakeMotor.setSecondaryCurrentLimit(60);

        overCurrent = false;
    }

    public void intake() {
        if (overCurrent) {
            intakeMotor.stopMotor();
            return;
        }
        overCurrent = intakeMotor.getBusVoltage() > CURRENT_LIMIT;
        intakeMotor.set(INTAKE_SPEED);
    }

    public void outtake() {
        overCurrent = false;
        intakeMotor.set(-OUTTAKE_SPEED);
    }

    public void stop() {
        overCurrent = false;
        intakeMotor.stopMotor();
    }
}