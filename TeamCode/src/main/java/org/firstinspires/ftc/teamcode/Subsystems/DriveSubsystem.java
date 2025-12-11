package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.controller.PIDController;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class DriveSubsystem {
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private GoBildaPinpointDriver pinpoint;
    private LinearOpMode opMode;

    PIDController headingPID = new PIDController(0.04, 0, 0);
    PIDController forwardDistancePID = new PIDController(0.06, 0, 0); //kp 0.07
    PIDController strafeDistancePID = new PIDController(0.08, 0, 0); //kp 0.07

    public void init(HardwareMap hardwareMap, LinearOpMode opMode) {
        this.opMode = opMode;

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
        pinpoint.setOffsets(160.0, 0.0, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
        //pinpoint.resetPosAndIMU();

        pinpoint.setPosition(new Pose2D(DistanceUnit.MM, 0, 0, AngleUnit.DEGREES, 120));
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
        double fl = (double) 0 + (double) 0 + rotate;
        double bl = (double) 0 + rotate;
        double fr = (double) 0 - (double) 0 - rotate;
        double br = (double) 0 + (double) 0 - rotate;

        frontLeft.setPower(fl);
        backLeft.setPower(bl);
        frontRight.setPower(fr);
        backRight.setPower(br);
    }

    public void autoForward(double distance, double power){
        pinpoint.update();

        double targetHeading = getHeading(AngleUnit.DEGREES);

        strafeDistancePID.reset();

        double heading = 0;

        while(opMode.opModeIsActive()){
            pinpoint.update();
            Pose2D robotPose = pinpoint.getPosition();

            double currentDistance = robotPose.getX(DistanceUnit.CM);
            double distanceError = distance - currentDistance;
            heading = robotPose.getHeading(AngleUnit.DEGREES);

            double x = strafeDistancePID.calculate(currentDistance, distance);
            double rotation = headingPID.calculate(heading, targetHeading);

            x = Range.clip(x, -power, power);
            rotation = Range.clip(rotation, -power, power);

            opMode.telemetry.addData("Target Distance", distance);
            opMode.telemetry.addData("Current Distance", currentDistance);
            opMode.telemetry.addData("Distance Error", distanceError);
            opMode.telemetry.addData("Target Heading", targetHeading);
            opMode.telemetry.addData("Heading", heading);
            opMode.telemetry.addData("X Power", x);
            opMode.telemetry.addData("Rotation Power", rotation);
            opMode.telemetry.update();

            if(Math.abs(distanceError) < 0.4) break;

            frontLeft.setPower(x - rotation);
            backLeft.setPower(x - rotation);
            frontRight.setPower(x + rotation);
            backRight.setPower(x + rotation);
        }
        stop();
    }

    public void autoStrafe(double distance, double power){
        pinpoint.update();

        double targetHeading = getHeading(AngleUnit.DEGREES);

        strafeDistancePID.reset();

        double heading = 0;

        while(opMode.opModeIsActive()){
            pinpoint.update();
            Pose2D robotPose = pinpoint.getPosition();

            double currentDistance = robotPose.getY(DistanceUnit.CM);
            double distanceError = distance - currentDistance;
            heading = robotPose.getHeading(AngleUnit.DEGREES);

            double y = strafeDistancePID.calculate(currentDistance, distance);
            double rotation = headingPID.calculate(heading, targetHeading);

            y = Range.clip(y, -power, power);
            rotation = Range.clip(rotation, -power, power);

            opMode.telemetry.addData("Target Distance", distance);
            opMode.telemetry.addData("Current Distance", currentDistance);
            opMode.telemetry.addData("Distance Error", distanceError);
            opMode.telemetry.addData("Target Heading", targetHeading);
            opMode.telemetry.addData("Heading", heading);
            opMode.telemetry.addData("y Power", y);
            opMode.telemetry.addData("Rotation Power", rotation);
            opMode.telemetry.update();

            if(Math.abs(distanceError) < 0.7) break;

            frontLeft.setPower(-y - rotation);
            backLeft.setPower(y - rotation);
            frontRight.setPower(y + rotation);
            backRight.setPower(-y + rotation);
        }
        stop();
    }

    public void autoRotate(double heading, double power){
        while(opMode.opModeIsActive()){
            pinpoint.update();
            Pose2D robotPose = pinpoint.getPosition();

            double currentHeading = robotPose.getHeading(AngleUnit.DEGREES);

            double rotation = headingPID.calculate(currentHeading, heading);

            rotation = Range.clip(rotation, -power, power);

            if(Math.abs(currentHeading - heading) < 0.5) break;

            frontLeft.setPower(-rotation);
            backLeft.setPower(-rotation);
            frontRight.setPower(rotation);
            backRight.setPower(rotation);
        }
        stop();
    }

    public void stop() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
}