package org.firstinspires.ftc.teamcode.Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;
@Autonomous(name = "Test Auto")
public class TestAuto extends LinearOpMode {
    private DriveSubsystem drive;

    @Override
    public void runOpMode() {
        initHardware();

        telemetry.addLine("✓");
        telemetry.update();

        waitForStart();
        //drive.recalibrateIMU();

        if (opModeIsActive()) {
            drive.autoForward(80, 0.4);
            drive.autoStrafe(40, 0.4);
            drive.autoRotate(90, 0.4);
            drive.stop();
        }
    }

    private void initHardware() {
        drive = new DriveSubsystem();

        drive.init(hardwareMap, this);
    }
}