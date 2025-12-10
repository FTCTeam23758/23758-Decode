package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import java.util.List;

public class LimelightSubsystem {
    private Limelight3A limelight;
    private LLResult latestResult;
    private static final double limelightOffset = 0.20;

    public void init(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();
        limelight.pipelineSwitch(0);
    }

    public void updateResult() {
        latestResult = limelight.getLatestResult();
    }

    public LLResult getLatestResult() {
        return latestResult;
    }

    public LLResultTypes.FiducialResult findBestFiducial() {
        if (latestResult == null || !latestResult.isValid()) {
            return null;
        }

        List<LLResultTypes.FiducialResult> fiducials = latestResult.getFiducialResults();
        if (fiducials == null || fiducials.isEmpty()) {
            return null;
        }

        LLResultTypes.FiducialResult best = null;
        double minTx = Double.MAX_VALUE;

        for (LLResultTypes.FiducialResult f : fiducials) {
            double absTx = Math.abs(f.getTargetXDegrees());
            if (absTx < minTx) {
                minTx = absTx;
                best = f;
            }
        }

        return best;
    }

    public double calculateDistance(LLResultTypes.FiducialResult fiducial) {
        Pose3D targetPose = fiducial.getTargetPoseCameraSpace();
        if (targetPose != null) {
            double distanceFromCamera = targetPose.getPosition().z;
            return distanceFromCamera + limelightOffset;
        }
        return 0.0;
    }
}