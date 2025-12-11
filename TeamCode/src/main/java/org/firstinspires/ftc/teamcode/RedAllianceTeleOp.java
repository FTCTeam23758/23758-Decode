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

@TeleOp(name = "Casper Red Alliance TeleOp")
public class RedAllianceTeleOp extends LinearOpMode {
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
    private static final double kP_rotate = 0.03;
    private static final double minAlignPower = 0.15;
    private static final double maxAlignPower = 0.50;
    private static final double alignTxTolerance = 2.0;
    private static final double alignDistanceMin = 1.2;
    private static final double alignDistanceMax = 3.7;
    private static final double matrixDistanceMin = 1.4;
    private static final double matrixDistanceMax = 3.7;

    private final ElapsedTime alignTimer = new ElapsedTime();
    private static final double alignTimeout = 3.5;

    private double measuredDistance = 0.0;
    private double targetTx = 0.0;
    private LLResultTypes.FiducialResult targetFiducial = null;

    private boolean shooterToggleLatch = false;
    private boolean dpadUpLatch = false;
    private boolean dpadDownLatch = false;

    // ================================================================

    @Override
    public void runOpMode() {
        initHardware();

        telemetry.addLine("Limelight: ✓");
        telemetry.addLine("Pinpoint IMU: ✓");
        telemetry.update();

        waitForStart();
        drive.recalibrateIMU();

        while (opModeIsActive()) {
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
    }

    private void initHardware() {
        drive = new DriveSubsystem();
        limelight = new LimelightSubsystem();
        intakeHopper = new IntakeHopperSubsystem();
        shooter = new ShooterSubsystem();

        drive.init(hardwareMap);
        limelight.init(hardwareMap);
        intakeHopper.init(hardwareMap);
        shooter.init(hardwareMap);
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

        // SHOOTER TOGGLE + AUTO ALIGN
        if (gamepad2.b && !shooterToggleLatch) {
            shooterToggleLatch = true;

            if (!shooter.isShooterActive()) {
                shooter.setShooterActive(true);
                gamepad2.rumble(100);

                targetFiducial = limelight.findBestFiducial();
                if (targetFiducial != null) {
                    state = RobotState.AUTO_ALIGNING;
                    alignTimer.reset();
                    gamepad1.rumble(200);
                } else {
                    double defaultDistance = (matrixDistanceMin + matrixDistanceMax) / 2.0;
                    shooter.applyShooterSettings(defaultDistance);
                    gamepad2.rumble(50);
                    sleep(50);
                    gamepad2.rumble(50);
                }
            } else {
                shooter.setShooterActive(false);
                state = RobotState.MANUAL;
                gamepad2.rumble(200);
            }
        } else if (!gamepad2.b) {
            shooterToggleLatch = false;
        }

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

        if (gamepad1.right_trigger > 0.3) {
            intakeHopper.setIntakePower(0.8);
        } else {
            intakeHopper.setIntakePower(0.0);
        }

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

        // 🔥 CAMBIO: Usar Tx ajustado con offset
        targetTx = limelight.getAdjustedTx(targetFiducial);

        boolean txAligned = Math.abs(targetTx) < alignTxTolerance;
        boolean distanceInRange = measuredDistance >= alignDistanceMin &&
                measuredDistance <= alignDistanceMax;

        if (txAligned && distanceInRange) {
            state = RobotState.ALIGNED;
            drive.stop();

            boolean inMatrixRange = measuredDistance >= matrixDistanceMin &&
                    measuredDistance <= matrixDistanceMax;

            if (inMatrixRange) {
                shooter.applyShooterSettings(measuredDistance);
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
        // Permitir movimiento suave
        double forward = -gamepad1.left_stick_y * 0.4;
        double strafe  = -gamepad1.left_stick_x * 0.4;
        double rotate  =  gamepad1.right_stick_x * 0.4;
        drive.driveFieldOriented(forward, strafe, rotate, 1.0);

        // Control de intake/hopper
        if (gamepad1.right_trigger > 0.3) {
            intakeHopper.setIntakePower(0.8);
        } else {
            intakeHopper.setIntakePower(0.0);
        }

        boolean override = gamepad2.left_trigger > 0.5;
        if (gamepad2.left_bumper) {
            intakeHopper.setHopperPower(-0.8);
        } else if (gamepad2.right_bumper) {
            boolean allowForward = !shooter.isShooterActive() || shooter.isReady() || override;
            intakeHopper.setHopperPower(allowForward ? 0.8 : 0.0);
        } else {
            intakeHopper.setHopperPower(0.0);
        }

        if (gamepad1.b) {
            state = RobotState.MANUAL;
            return;
        }

        if (gamepad2.b && !shooterToggleLatch) {
            shooterToggleLatch = true;
            shooter.setShooterActive(false);
            state = RobotState.MANUAL;
        } else if (!gamepad2.b) {
            shooterToggleLatch = false;
        }

        // Verificar que no se perdió la alineación
        targetFiducial = limelight.findBestFiducial();
        if (targetFiducial != null) {
            // 🔥 CAMBIO: Usar Tx ajustado con offset
            targetTx = limelight.getAdjustedTx(targetFiducial);
            if (Math.abs(targetTx) > alignTxTolerance * 3) {
                state = RobotState.MANUAL;
            }
        } else {
            state = RobotState.MANUAL;
        }
    }

    private void updateTelemetry() {
        telemetry.addLine("=== CASPER TELEOP ===");

        String stateStr = state == RobotState.MANUAL ? "MANUAL" :
                state == RobotState.AUTO_ALIGNING ? "ALIGNING" : "ALIGNED";
        telemetry.addData("STATE", stateStr);
        telemetry.addLine();

        // SHOOTER INFO
        telemetry.addData("Shooter", shooter.isShooterActive() ? "✓ ON" : "✗ IDLE");
        telemetry.addData("Current RPM", shooter.getCurrentRPM());
        telemetry.addData("Target RPM", shooter.getTargetRPM());
        if (shooter.isShooterActive()) {
            telemetry.addData("Ready", shooter.isReady() ? "✓ YES" : "✗ NO");
        }
        telemetry.addData("Hood Pos", shooter.getHoodPos());
        telemetry.addData("Intake", intakeHopper.isIntakeRunning() ? "ON" : "OFF");
        telemetry.addLine();

        // LIMELIGHT & ALIGNMENT INFO
        LLResult latestResult = limelight.getLatestResult();
        if (latestResult != null && latestResult.isValid()) {
            List<LLResultTypes.FiducialResult> fiducials = latestResult.getFiducialResults();
            if (fiducials != null && !fiducials.isEmpty()) {
                telemetry.addData("AprilTags", fiducials.size());
                if (targetFiducial != null && measuredDistance > 0) {
                    telemetry.addData("Distance", measuredDistance);
                    telemetry.addData("Tx", targetTx);

                    boolean txOK = Math.abs(targetTx) < alignTxTolerance;
                    boolean distOK = measuredDistance >= alignDistanceMin &&
                            measuredDistance <= alignDistanceMax;

                    telemetry.addData("Tx Aligned", txOK ? "✓" : Math.abs(targetTx));
                    telemetry.addData("  Tolerance", alignTxTolerance);
                    telemetry.addData("Dist OK", distOK ? "✓" : "✗");

                    boolean inRange = measuredDistance >= matrixDistanceMin &&
                            measuredDistance <= matrixDistanceMax;
                    telemetry.addData("Matrix Range", inRange ? "✓ IN" : "✗ OUT");
                }
            } else {
                telemetry.addData("AprilTags", "✗ NONE");
            }
        } else {
            telemetry.addData("Limelight", "✗ NO SIGNAL");
        }
        telemetry.addLine();

        if (state == RobotState.AUTO_ALIGNING) {
            double timeLeft = alignTimeout - alignTimer.seconds();
            telemetry.addData("Align Timer", timeLeft);
        }

        telemetry.addData("Heading", drive.getHeading(AngleUnit.DEGREES));

        telemetry.update();
    }
}
