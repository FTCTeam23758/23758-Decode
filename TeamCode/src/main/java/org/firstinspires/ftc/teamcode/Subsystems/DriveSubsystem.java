package org.firstinspires.ftc.teamcode.Subsystems;

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

    PIDController headingPID = new PIDController(0.07, 0, 0.001);
    PIDController forwardDistancePID = new PIDController(0.06, 0, 0);
    PIDController strafeDistancePID = new PIDController(0.08, 0, 0);

    private boolean isBlueAlliance = false;

    public void init(HardwareMap hardwareMap, LinearOpMode opMode) {
        this.opMode = opMode;

        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

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
        pinpoint.resetPosAndIMU();
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

    public void setBlueAlliance(boolean isBlue) {
        this.isBlueAlliance = isBlue;
    }

    public void resetToZero() {
        pinpoint.resetPosAndIMU();
    }

    public void driveFieldOriented(double forward, double strafe, double rotate, double speed) {
        if (isBlueAlliance) {
            forward = -forward;
            strafe = -strafe;
        }

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
        double fl = rotate;
        double bl = rotate;
        double fr = -rotate;
        double br = -rotate;

        frontLeft.setPower(fl);
        backLeft.setPower(bl);
        frontRight.setPower(fr);
        backRight.setPower(br);
    }

    public void autoForwardRelative(double distanceCM, double power) {
        pinpoint.update();
        Pose2D startPose = pinpoint.getPosition();

        double startX = startPose.getX(DistanceUnit.CM);
        double startY = startPose.getY(DistanceUnit.CM);
        double targetHeading = startPose.getHeading(AngleUnit.DEGREES);

        forwardDistancePID.reset();
        headingPID.reset();

        while(opMode.opModeIsActive()) {
            pinpoint.update();
            Pose2D currentPose = pinpoint.getPosition();

            double currentX = currentPose.getX(DistanceUnit.CM);
            double currentY = currentPose.getY(DistanceUnit.CM);

            double deltaX = currentX - startX;
            double deltaY = currentY - startY;
            double distanceTraveled = Math.sqrt(deltaX * deltaX + deltaY * deltaY);

            if (distanceCM < 0 && distanceTraveled > 0) {
                distanceTraveled = -distanceTraveled;
            }

            double distanceError = distanceCM - distanceTraveled;
            double currentHeading = currentPose.getHeading(AngleUnit.DEGREES);

            double forward = forwardDistancePID.calculate(distanceTraveled, distanceCM);
            double rotation = headingPID.calculate(currentHeading, targetHeading);

            forward = Range.clip(forward, -Math.abs(power), Math.abs(power));
            rotation = Range.clip(rotation, -0.3, 0.3);

            opMode.telemetry.addData("Distance Target", distanceCM);
            opMode.telemetry.addData("Distance Traveled", distanceTraveled);
            opMode.telemetry.addData("Distance Error", distanceError);
            opMode.telemetry.addData("Heading", currentHeading);
            opMode.telemetry.update();

            if(Math.abs(distanceError) < 0.5) break;

            double heading = currentPose.getHeading(AngleUnit.RADIANS);
            double forwardX = forward * Math.cos(heading);
            double forwardY = forward * Math.sin(heading);

            double rotX = forwardX * Math.cos(-heading) - forwardY * Math.sin(-heading);
            double rotY = forwardX * Math.sin(-heading) + forwardY * Math.cos(-heading);

            frontLeft.setPower(rotY + rotX - rotation);
            backLeft.setPower(rotY - rotX - rotation);
            frontRight.setPower(rotY - rotX + rotation);
            backRight.setPower(rotY + rotX + rotation);
        }
        stop();
    }

    public void autoForward(double distance, double power) {
        pinpoint.update();

        double targetHeading = getHeading(AngleUnit.DEGREES);

        strafeDistancePID.reset();

        while (opMode.opModeIsActive()) {
            pinpoint.update();
            Pose2D robotPose = pinpoint.getPosition();

            double currentDistance = robotPose.getX(DistanceUnit.CM);
            double distanceError = distance - currentDistance;
            double heading = robotPose.getHeading(AngleUnit.DEGREES);

            double x = strafeDistancePID.calculate(currentDistance, distance);
            double rotation = headingPID.calculate(heading, targetHeading);

            x = Range.clip(x, -power, power);
            rotation = Range.clip(rotation, -power, power);

            opMode.telemetry.addData("Target Distance", distance);
            opMode.telemetry.addData("Current Distance", currentDistance);
            opMode.telemetry.addData("Distance Error", distanceError);
            opMode.telemetry.update();

            if (Math.abs(distanceError) < 0.4) break;

            frontLeft.setPower(x - rotation);
            backLeft.setPower(x - rotation);
            frontRight.setPower(x + rotation);
            backRight.setPower(x + rotation);
        }
        stop();
    }

    public void autoStrafe(double distance, double power) {
        pinpoint.update();

        double targetHeading = getHeading(AngleUnit.DEGREES);

        strafeDistancePID.reset();

        while (opMode.opModeIsActive()) {
            pinpoint.update();
            Pose2D robotPose = pinpoint.getPosition();

            double currentDistance = robotPose.getY(DistanceUnit.CM);
            double distanceError = distance - currentDistance;
            double heading = robotPose.getHeading(AngleUnit.DEGREES);

            double y = strafeDistancePID.calculate(currentDistance, distance);
            double rotation = headingPID.calculate(heading, targetHeading);

            y = Range.clip(y, -power, power);
            rotation = Range.clip(rotation, -power, power);

            opMode.telemetry.addData("Target Distance", distance);
            opMode.telemetry.addData("Current Distance", currentDistance);
            opMode.telemetry.addData("Distance Error", distanceError);
            opMode.telemetry.update();

            if (Math.abs(distanceError) < 0.7) break;

            frontLeft.setPower(-y - rotation);
            backLeft.setPower(y - rotation);
            frontRight.setPower(y + rotation);
            backRight.setPower(-y + rotation);
        }
        stop();
    }

    public void autoRotate(double targetHeading, double power) {
        while (opMode.opModeIsActive()) {
            pinpoint.update();
            Pose2D robotPose = pinpoint.getPosition();

            double currentHeading = robotPose.getHeading(AngleUnit.DEGREES);
            double error = targetHeading - currentHeading;
            while (error > 180) error -= 360;
            while (error < -180) error += 360;

            double rotation = headingPID.calculate(0, error);

            rotation = Range.clip(rotation, -power, power);

            opMode.telemetry.addData("Current Heading", currentHeading);
            opMode.telemetry.addData("Target Heading", targetHeading);
            opMode.telemetry.addData("Error", error);
            opMode.telemetry.update();

            if (Math.abs(error) < 0.5) break;

            frontLeft.setPower(-rotation);
            backLeft.setPower(-rotation);
            frontRight.setPower(rotation);
            backRight.setPower(rotation);
        }
        stop();
    }

    public void autoMoveRelative(double distanceCM, double power) {
        pinpoint.update();
        Pose2D startPose = pinpoint.getPosition();

        double startX = startPose.getX(DistanceUnit.CM);
        double startY = startPose.getY(DistanceUnit.CM);
        double targetHeading = startPose.getHeading(AngleUnit.DEGREES);

        forwardDistancePID.reset();
        headingPID.reset();

        while(opMode.opModeIsActive()) {
            pinpoint.update();
            Pose2D currentPose = pinpoint.getPosition();

            double currentX = currentPose.getX(DistanceUnit.CM);
            double currentY = currentPose.getY(DistanceUnit.CM);

            double deltaX = currentX - startX;
            double deltaY = currentY - startY;
            double distanceTraveled = Math.sqrt(deltaX * deltaX + deltaY * deltaY);

            double distanceError = Math.abs(distanceCM) - distanceTraveled;

            double forward = forwardDistancePID.calculate(distanceTraveled, Math.abs(distanceCM));

            double currentHeading = currentPose.getHeading(AngleUnit.DEGREES);
            double headingError = targetHeading - currentHeading;
            while (headingError > 180) headingError -= 360;
            while (headingError < -180) headingError += 360;
            double rotation = headingPID.calculate(0, headingError);

            if (distanceCM < 0) forward = -forward;

            forward = Range.clip(forward, -Math.abs(power), Math.abs(power));
            rotation = Range.clip(rotation, -0.3, 0.3);

            opMode.telemetry.addData("Target Distance", distanceCM);
            opMode.telemetry.addData("Distance Traveled", distanceTraveled);
            opMode.telemetry.addData("Distance Error", distanceError);
            opMode.telemetry.update();

            if(distanceError < 2.0) break;

            frontLeft.setPower(forward - rotation);
            backLeft.setPower(forward - rotation);
            frontRight.setPower(forward + rotation);
            backRight.setPower(forward + rotation);
        }
        stop();
    }

    public void resetPinpoint() {
        pinpoint.resetPosAndIMU();
    }

    public void stop() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    public Pose2D getPinpointPosition() {
        return pinpoint.getPosition();
    }
}