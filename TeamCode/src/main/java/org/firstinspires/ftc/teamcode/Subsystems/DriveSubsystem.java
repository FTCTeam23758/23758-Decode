package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class DriveSubsystem {
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private GoBildaPinpointDriver pinpoint;

    public void init(HardwareMap hardwareMap) {
        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft   = hardwareMap.get(DcMotor.class, "backLeft");
        backRight  = hardwareMap.get(DcMotor.class, "backRight");

        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
    }

    public void updatePinpoint() {
        pinpoint.update();
    }

    public void recalibrateIMU() {
        pinpoint.recalibrateIMU();
    }

    public double getHeading(AngleUnit unit) {
        return pinpoint.getHeading(unit);
    }

    public void driveFieldOriented(double forward, double strafe, double rotate, double speed) {
        double heading = pinpoint.getHeading(AngleUnit.RADIANS);

        double rotX = forward * Math.cos(-heading) - strafe * Math.sin(-heading);
        double rotY = forward * Math.sin(-heading) + strafe * Math.cos(-heading);

        double fl = (rotY + rotX + rotate) * speed;
        double bl = (rotY - rotX + rotate) * speed;
        double fr = (rotY - rotX - rotate) * speed;
        double br = (rotY + rotX - rotate) * speed;

        frontLeft.setPower(fl);
        backLeft.setPower(bl);
        frontRight.setPower(fr);
        backRight.setPower(br);
    }

    public void driveRobotOriented(double rotate) {
        frontLeft.setPower(rotate);
        backLeft.setPower(rotate);
        frontRight.setPower(-rotate);
        backRight.setPower(-rotate);
    }

    public void stop() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
}