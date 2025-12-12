package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import java.util.List;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeHopperSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;

@TeleOp(name = "Casper Blue Alliance TeleOp")
public class BlueAllianceTeleOp extends LinearOpMode {
    private DriveSubsystem drive;
    private LimelightSubsystem limelight;
    private IntakeHopperSubsystem intakeHopper;
    private ShooterSubsystem shooter;
    private enum RobotState {
        MANUAL,
        AUTO_ALIGNING,
        ALIGNED
    }
    private RobotState state = RobotState.MANUAL;

    private static final double kP_rotate = 0.02;
    private static final double minAlignPower = 0.12;
    private static final double maxAlignPower = 0.35;
    private static final double alignTxTolerance = 1.5;
    private static final double alignDistanceMin = 1.2;
    private static final double alignDistanceMax = 3.7;
    private static final double matrixDistanceMin = 1.4;
    private static final double matrixDistanceMax = 3.7;

    private final ElapsedTime alignTimer = new ElapsedTime();
    private static final double alignTimeout = 3.0;

    private double measuredDistance = 0.0;
    private double targetTx = 0.0;
    private LLResultTypes.FiducialResult targetFiducial = null;

    private boolean shooterToggleLatch = false;
    private boolean dpadUpLatch = false;
    private boolean dpadDownLatch = false;

    @Override
    public void runOpMode() {
        initHardware();

        telemetry.addLine("Limelight: ✓");
        telemetry.addLine("Pinpoint IMU: ✓");
        telemetry.addLine("BLUE ALLIANCE MODE");
        telemetry.update();

        waitForStart();
        drive.recalibrateIMU();

        while (opModeIsActive()) {

            if (gamepad1.y) {
                drive.resetToZero();
                telemetry.addLine("✓ Robot reseteado a 0°");
                telemetry.update();
                sleep(300);
            }

            drive.updatePinpoint();
            limelight.updateResult();
            shooter.update();

            switch (state) {
                case MANUAL:
                    handleManualControl();
                    break;
                case AUTO_ALIGNING:
                    handleAutoAlignment();
                    break;
                case ALIGNED:
                    handleAligned();
                    break;
            }
            updateTelemetry();
        }
        stopAllMotors();
    }

    private void initHardware() {
        drive = new DriveSubsystem();
        limelight = new LimelightSubsystem();
        intakeHopper = new IntakeHopperSubsystem();
        shooter = new ShooterSubsystem();

        drive.init(hardwareMap, this);
        limelight.init(hardwareMap);
        intakeHopper.init(hardwareMap);
        shooter.init(hardwareMap);

        drive.setBlueAlliance(true);
        limelight.setTargetAprilTag(20); // Blue High Basket
    }

    private void handleManualControl() {
        // DRIVE
        double forward = -gamepad1.left_stick_y;
        double strafe  = -gamepad1.left_stick_x;
        double rotate  =  gamepad1.right_stick_x;

        double normalSpeed = 0.8;
        double turboSpeed = 1.0;

        double speed = gamepad1.right_trigger > 0.1 ? turboSpeed : normalSpeed;
        drive.driveFieldOriented(forward, strafe, rotate, speed);

        // SHOOTER TOGGLE
        if (gamepad2.b && !shooterToggleLatch) {
            shooterToggleLatch = true;
            shooter.setShooterActive(!shooter.isShooterActive());

            if (shooter.isShooterActive()) {
                gamepad2.rumble(100);

                targetFiducial = limelight.findBestFiducial();
                if (targetFiducial != null) {
                    state = RobotState.AUTO_ALIGNING;
                    alignTimer.reset();
                    gamepad1.rumble(200);
                }
            }
        } else if (!gamepad2.b) {
            shooterToggleLatch = false;
        }

        // HOOD CONTROL
        if (gamepad2.dpad_up && !dpadUpLatch) {
            dpadUpLatch = true;
            shooter.adjustHood(0.1);
        } else if (!gamepad2.dpad_up) {
            dpadUpLatch = false;
        }

        if (gamepad2.dpad_down && !dpadDownLatch) {
            dpadDownLatch = true;
            shooter.adjustHood(-0.1);
        } else if (!gamepad2.dpad_down) {
            dpadDownLatch = false;
        }

        // INTAKE
        if (gamepad1.right_trigger > 0.3) {
            intakeHopper.setIntakePower(0.8);
        } else {
            intakeHopper.setIntakePower(0.0);
        }

        // HOPPER
        boolean override = gamepad2.left_trigger > 0.5;
        double hopperPower = 0.8;

        if (gamepad2.left_bumper) {
            intakeHopper.setHopperPower(-hopperPower);
        }
        else if (gamepad2.right_bumper) {
            boolean allowForward = !shooter.isShooterActive() || shooter.isReady() || override;
            intakeHopper.setHopperPower(allowForward ? hopperPower : 0.0);
        }
        else {
            intakeHopper.setHopperPower(0.0);
        }
    }

    private void handleAutoAlignment() {
        targetFiducial = limelight.findBestFiducial();

        if (targetFiducial == null) {
            state = RobotState.MANUAL;
            drive.stop();
            gamepad1.rumble(500);
            return;
        }

        measuredDistance = limelight.calculateDistance(targetFiducial);

        if (measuredDistance <= 0) {
            state = RobotState.MANUAL;
            drive.stop();
            gamepad1.rumble(1000);
            return;
        }

        if (alignTimer.seconds() > alignTimeout) {
            state = RobotState.MANUAL;
            drive.stop();
            gamepad1.rumble(1000);
            return;
        }

        targetTx = targetFiducial.getTargetXDegrees();

        boolean aligned = Math.abs(targetTx) < alignTxTolerance &&
                measuredDistance >= alignDistanceMin &&
                measuredDistance <= alignDistanceMax;

        if (aligned) {
            state = RobotState.ALIGNED;
            drive.stop();

            boolean inMatrixRange = measuredDistance >= matrixDistanceMin &&
                    measuredDistance <= matrixDistanceMax;

            if (inMatrixRange) {
                shooter.applyShooterSettings(measuredDistance, matrixDistanceMin, matrixDistanceMax);
                gamepad1.rumble(200);
                sleep(50);
                gamepad1.rumble(200);
            } else {
                gamepad1.rumble(100);
                sleep(50);
                gamepad1.rumble(100);
                sleep(50);
                gamepad1.rumble(100);
            }
            return;
        }

        double rotatePower = -targetTx * kP_rotate;

        if (Math.abs(rotatePower) < minAlignPower) {
            rotatePower = Math.signum(rotatePower) * minAlignPower;
        }
        rotatePower = Math.max(-maxAlignPower, Math.min(maxAlignPower, rotatePower));

        drive.driveRobotOriented(rotatePower);

        if (gamepad1.b) {
            state = RobotState.MANUAL;
            drive.stop();
        }
    }

    private void handleAligned() {
        handleManualControl();

        if (gamepad1.b) {
            state = RobotState.MANUAL;
        }

        targetFiducial = limelight.findBestFiducial();
        if (targetFiducial != null) {
            targetTx = targetFiducial.getTargetXDegrees();
            if (Math.abs(targetTx) > alignTxTolerance * 2) {
                state = RobotState.MANUAL;
            }
        }
    }

    private void stopAllMotors() {
        drive.stop();
        shooter.stop();
        intakeHopper.stop();
    }

    private void updateTelemetry() {
        telemetry.addLine("CASPER TELEOP [BLUE]");

        String stateStr = state == RobotState.MANUAL ? "MANUAL" :
                state == RobotState.AUTO_ALIGNING ? "ALIGNING" :
                        "ALIGNED";
        telemetry.addData("STATE:", stateStr);
        telemetry.addLine();

        telemetry.addData("Shooter", shooter.isShooterActive() ? "ON" : "IDLE");
        telemetry.addData("RPM", "/", shooter.getCurrentRPM(), shooter.getTargetRPM());
        if (shooter.isShooterActive()) {
            telemetry.addData("Ready", shooter.isReady() ? "YES" : "NO");
        }
        telemetry.addData("Hood", shooter.getHoodPos());
        telemetry.addData("Intake", intakeHopper.isIntakeRunning() ? "ON" : "OFF");
        telemetry.addLine();

        LLResult latestResult = limelight.getLatestResult();
        if (latestResult != null && latestResult.isValid()) {
            List<LLResultTypes.FiducialResult> fiducials = latestResult.getFiducialResults();
            if (fiducials != null && !fiducials.isEmpty()) {
                telemetry.addData("Tags", fiducials.size());
                if (targetFiducial != null && measuredDistance > 0) {
                    telemetry.addData("Distance", measuredDistance);
                    telemetry.addData("Tx", targetTx);
                    telemetry.addData("Target Tag", limelight.getTargetAprilTag());

                    boolean inRange = measuredDistance >= matrixDistanceMin &&
                            measuredDistance <= matrixDistanceMax;
                    telemetry.addData("  Matrix", inRange ? "✓ IN RANGE" : "✗ OUT OF RANGE");
                }
            }
        }

        telemetry.addData("Heading", drive.getHeading(AngleUnit.DEGREES));
        telemetry.update();
    }
}