package org.firstinspires.ftc.teamcode.Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeHopperSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;

@Autonomous(name = "Red Alliance Front Auto")
public class RedAllianceFrontAuto extends LinearOpMode {
    private DriveSubsystem drive;
    private ShooterSubsystem shooter;
    private IntakeHopperSubsystem intake;

    @Override
    public void runOpMode() {
        initHardware();

        telemetry.addLine("✓");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            // Move and turn for first shoot
            drive.autoForward(150, 0.6);
            drive.autoRotate(170, 0.5);

            shooter.setServo(0.4);
            shooter.setRPM(1600);

            ElapsedTime timer = new ElapsedTime();

            // Accelerate Shooter
            while (opModeIsActive() && !shooter.isAtTargetRPM()) {
                shooter.update();
                telemetry.addData("ShooterRPM", shooter.getCurrentRPM());
                telemetry.update();
            }

            // First shoot
            timer.reset();
            while (opModeIsActive() && timer.seconds() < 6) {
                shooter.update();
                intake.setIntakePower(0.4);
                intake.setHopperPower(0.4);
                telemetry.addData("ShooterRPM", shooter.getCurrentRPM());
                telemetry.update();
            }

            shooter.stop();
            intake.setIntakePower(0);
            intake.setHopperPower(0);

            drive.autoRotate(-146, 0.5);
            sleep(200);

            drive.resetPinpoint();

            telemetry.addLine("Pinpoint reset");
            telemetry.addLine("Heading = 0");
            telemetry.update();
            sleep(200);

            drive.autoRotate(-90,0.5);
            intake.setIntakePower(0.6);
            intake.setHopperPower(0.4);

            drive.autoMoveRelative(90, 0.5);

            intake.setIntakePower(0);
            intake.setHopperPower(0);

            drive.autoMoveRelative(-90, 0.5);
            drive.autoRotate(-45,0.5);

            while (opModeIsActive() && !shooter.isAtTargetRPM()) {
                shooter.update();
                telemetry.addData("ShooterRPM", shooter.getCurrentRPM());
                telemetry.update();
            }

            timer.reset();
            while (opModeIsActive() && timer.seconds() < 8) {
                shooter.update();
                intake.setIntakePower(0.4);
                intake.setHopperPower(0.4);
                telemetry.addData("ShooterRPM", shooter.getCurrentRPM());
                telemetry.update();
            }

            drive.autoRotate(0,0.5);

            shooter.setRPM(0.0);
            intake.setIntakePower(0.0);
            intake.setHopperPower(0.0);

        drive.stop();
        }
    }

    private void initHardware() {
        drive = new DriveSubsystem();
        shooter = new ShooterSubsystem();
        intake = new IntakeHopperSubsystem();

        drive.init(hardwareMap, this);
        shooter.init(hardwareMap);
        intake.init(hardwareMap);
    }
}