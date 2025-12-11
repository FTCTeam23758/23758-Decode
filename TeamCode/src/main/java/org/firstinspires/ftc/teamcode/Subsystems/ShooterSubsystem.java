package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ShooterSubsystem {
    private DcMotorEx shooterMotor;
    private Servo capServo;

    private boolean shooterActive = false;
    private double targetRPM = 2100;
    private double currentRPM = 0;
    private double hoodPos = 0.20;

    private static final double ticksRevolution = 28.0;
    private static final double normalRevolutions = 800;
    private static final double kS = 0.07;
    private static final double kV = 0.00017;
    private static final double kPShooter = 0.0004;
    private static final double readyFactor = 0.99;
    private static final double matrixSnapTolerance = 0.05;

    private final double[][] shooterMatrix = {
            {1.8, 1800, 0.40},
            {2.0, 1900, 0.38},
            {3.0, 2000, 0.20},
            {3.3, 2050, 0.10},
            {3.5, 2100, 0.0},
    };

    public void init(HardwareMap hardwareMap) {
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        capServo = hardwareMap.get(Servo.class, "capServo");
        hoodPos = 0.20;
        capServo.setPosition(hoodPos);
    }

    public void update() {
        double desiredRPM = shooterActive ? targetRPM : normalRevolutions;

        if (desiredRPM <= 0) {
            currentRPM = 0;
            shooterMotor.setPower(0);
            return;
        }

        double ticksPerSecond = shooterMotor.getVelocity();
        currentRPM = Math.abs(ticksPerSecond * 60.0 / ticksRevolution);

        double ff = kS + kV * desiredRPM;
        double error = desiredRPM - currentRPM;
        double power = ff + kPShooter * error;

        power = Math.max(0.0, Math.min(1.0, power));
        shooterMotor.setPower(power);
    }

    public boolean isReady() {
        return shooterActive &&
                targetRPM > 0 &&
                currentRPM >= targetRPM * readyFactor;
    }

    public void applyShooterSettings(double distance, double matrixDistanceMin, double matrixDistanceMax) {
        int closestIdx = 0;
        double minDiff = Double.MAX_VALUE;

        for (int i = 0; i < shooterMatrix.length; i++) {
            double diff = Math.abs(distance - shooterMatrix[i][0]);
            if (diff < minDiff) {
                minDiff = diff;
                closestIdx = i;
            }
        }

        if (minDiff <= matrixSnapTolerance) {
            targetRPM = shooterMatrix[closestIdx][1];
            hoodPos = shooterMatrix[closestIdx][2];
            capServo.setPosition(hoodPos);
        } else {
            double dist = Math.max(shooterMatrix[0][0],
                    Math.min(shooterMatrix[shooterMatrix.length-1][0], distance));

            int lowerIdx = 0;
            for (int i = 0; i < shooterMatrix.length - 1; i++) {
                if (dist >= shooterMatrix[i][0] && dist <= shooterMatrix[i+1][0]) {
                    lowerIdx = i;
                    break;
                }
            }

            int upperIdx = lowerIdx + 1;
            if (upperIdx >= shooterMatrix.length) {
                upperIdx = shooterMatrix.length - 1;
            }

            double dist1 = shooterMatrix[lowerIdx][0];
            double dist2 = shooterMatrix[upperIdx][0];
            double t = (dist - dist1) / (dist2 - dist1);

            if (Double.isNaN(t) || Double.isInfinite(t)) {
                t = 0;
            }

            double rpm1 = shooterMatrix[lowerIdx][1];
            double rpm2 = shooterMatrix[upperIdx][1];
            targetRPM = rpm1 + t * (rpm2 - rpm1);

            double hood1 = shooterMatrix[lowerIdx][2];
            double hood2 = shooterMatrix[upperIdx][2];
            hoodPos = hood1 + t * (hood2 - hood1);

            capServo.setPosition(hoodPos);
        }
    }

    public void setShooterActive(boolean active) {
        shooterActive = active;
    }

    public boolean isShooterActive() {
        return shooterActive;
    }

    public void adjustHood(double delta) {
        hoodPos += delta;
        if (hoodPos > 1.0) hoodPos = 1.0;
        if (hoodPos < 0.0) hoodPos = 0.0;
        capServo.setPosition(hoodPos);
    }

    public double getTargetRPM() {
        return shooterActive ? targetRPM : normalRevolutions;
    }

    public double getCurrentRPM() {
        return currentRPM;
    }

    public double getHoodPos() {
        return hoodPos;
    }

    public void stop() {
        shooterMotor.setPower(0);
    }

    public void setRPM(double rpm) {
        shooterActive = true;
        targetRPM = rpm;
    }

    public boolean isAtTargetRPM() {
        return currentRPM >= targetRPM * readyFactor;
    }

    public void setServo(double pos){
        capServo.setPosition(pos);
    }

}