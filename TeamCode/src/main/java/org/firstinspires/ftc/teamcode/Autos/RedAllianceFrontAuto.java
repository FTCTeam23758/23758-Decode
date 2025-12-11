package org.firstinspires.ftc.teamcode.Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeHopperSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;

@Autonomous(name = "Red Alliance Front Autonomous")
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
            drive.autoForward(170, 0.6);
            drive.autoRotate(-15, 0.5);

            shooter.setServo(0.4);

            shooter.setRPM(1600);

            ElapsedTime timer = new ElapsedTime();

            // First: get up to speed
            while (opModeIsActive() && !shooter.isAtTargetRPM()) {
                shooter.update();
                telemetry.addData("ShooterRPM", shooter.getCurrentRPM());
                telemetry.update();
            }

            // Second: hold speed for given seconds
            timer.reset();
            while (opModeIsActive() && timer.seconds() < 10) {
                shooter.update();
                intake.setIntakePower(0.4);
                intake.setHopperPower(0.4);
                telemetry.addData("ShooterRPM", shooter.getCurrentRPM());
                telemetry.update();
            }

            drive.autoRotate(-90, 0.5);

            intake.setIntakePower(0.6);
            intake.setHopperPower(0.4);

            drive.autoForward(90, 0.5);

            intake.setHopperPower(0);

            drive.autoForward(-90, -0.5);
            drive.autoRotate(-15, 0.5);

            shooter.setRPM(1600);

            ElapsedTime timer2 = new ElapsedTime();

            // First: get up to speed
            while (opModeIsActive() && !shooter.isAtTargetRPM()) {
                shooter.update();
                telemetry.addData("ShooterRPM", shooter.getCurrentRPM());
                telemetry.update();
            }

            // Second: hold speed for given seconds
            timer2.reset();
            while (opModeIsActive() && timer2.seconds() < 10) {
                shooter.update();
                intake.setIntakePower(0.4);
                intake.setHopperPower(0.4);
                telemetry.addData("ShooterRPM", shooter.getCurrentRPM());
                telemetry.update();
            }

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