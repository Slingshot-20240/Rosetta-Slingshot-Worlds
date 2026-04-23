package org.firstinspires.ftc.teamcode.subsystems.vision;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.WhiteBalanceControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.concurrent.TimeUnit;

public class exposureLogi {
    AprilTagProcessor apriltagPipeline;

    BallProcessor ballPipeline;

    VisionPortal portal;

    final double ATsize = 6.5;
    final double theta = Math.toRadians(45.2189284116);
    final double[] resolution = new double[]{1920, 1080};

    public exposureLogi(HardwareMap hw) {
        apriltagPipeline = new AprilTagProcessor.Builder()
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
//                .setLensIntrinsics(0.187319959814, -0.575948480673, -0.00438930956954, 0.00126723944556)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .build();

        ballPipeline = new BallProcessor();

        int[] myPortalsList = VisionPortal.makeMultiPortalView(2, VisionPortal.MultiPortalLayout.HORIZONTAL);

        portal = new VisionPortal.Builder()
                .setCamera(hw.get(WebcamName.class, "Webcam 1"))
                .addProcessors(apriltagPipeline, ballPipeline)
                .setCameraResolution(new Size(1280, 720))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .setAutoStopLiveView(true)
                .setLiveViewContainerId(myPortalsList[0])
                .build();

        portal.setProcessorEnabled(apriltagPipeline, true);
        portal.setProcessorEnabled(ballPipeline, false);


        //Exposure Control
        ExposureControl exposureControl = portal.getCameraControl(ExposureControl.class);
        GainControl gainControl = portal.getCameraControl(GainControl.class);
        WhiteBalanceControl whiteControl = portal.getCameraControl(WhiteBalanceControl.class);

        // auto exposure is stupid
        exposureControl.setMode(ExposureControl.Mode.Manual);

        // Set a short exposure (e.g., 5 milliseconds) to "freeze" motion
        exposureControl.setExposure(exposureControl.getMinExposure(TimeUnit.MILLISECONDS), TimeUnit.MILLISECONDS);

        // Increase gain to compensate for the dark image
        gainControl.setGain(200); // Adjust based on your lighting
    }

    public void enableAT() {
        portal.setProcessorEnabled(apriltagPipeline, true);
    }

    public void enableAT(boolean enabled) {
        portal.setProcessorEnabled(apriltagPipeline, enabled);
    }

    public void enableBall() {
        portal.setProcessorEnabled(ballPipeline, true);
    }

    public void enableBall(boolean enabled) {
        portal.setProcessorEnabled(ballPipeline, enabled);
    }

    public double getATdist() {
        if(!portal.getProcessorEnabled(apriltagPipeline))
            return 0.0;

        for (AprilTagDetection detection : apriltagPipeline.getDetections()) {
            if (detection.id != 20 && detection.id != 24) {
                continue;
            }

            return detection.ftcPose.range;
        }
        return 0.0;
    }

    public double getATangle() {
        if(!portal.getProcessorEnabled(apriltagPipeline))
            return 0.0;

        for (AprilTagDetection detection : apriltagPipeline.getDetections()) {
            if (detection.id != 20 && detection.id != 24) {
                continue;
            }

            return detection.ftcPose.bearing;
        }
        return 0.0;
    }

    // extra 4" for cam-to-flywheel, 18" for AT-to-back-of-goal
    public double getTargetArtifactTravelDistanceX() {
        return getATdist() + 22;
    }
}
