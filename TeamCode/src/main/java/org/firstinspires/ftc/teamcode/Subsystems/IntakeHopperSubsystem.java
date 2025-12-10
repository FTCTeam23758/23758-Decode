package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeHopperSubsystem {
    private DcMotorEx intakeMotor;
    private DcMotorEx hopperMotor;
    private boolean intakeRunning = false;

    public void init(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        hopperMotor = hardwareMap.get(DcMotorEx.class, "hopperMotor");
    }

    public void setIntakePower(double power) {
        intakeMotor.setPower(power);
        intakeRunning = power > 0;
    }

    public void setHopperPower(double power) {
        hopperMotor.setPower(power);
    }

    public boolean isIntakeRunning() {
        return intakeRunning;
    }

    public void stop() {
        intakeMotor.setPower(0);
        hopperMotor.setPower(0);
        intakeRunning = false;
    }
}