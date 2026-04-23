package org.firstinspires.ftc.teamcode.NextFTC.misc.vision;

import android.util.Size;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;


public class Logi implements Subsystem {

    // ------------------ INSTANCES ------------------ //

    public static final Logi INSTANCE = new Logi();

    public Logi() {}

    AprilTagProcessor apriltagPipeline;

    VisionPortal portal;

    BallProcessor ballPipeline;

    // ------------------ COMMANDS ------------------ //

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

    public void enableAT() {
        portal.setProcessorEnabled(apriltagPipeline, true);
    }

    @Override
    public void initialize() {
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
                .setCamera(ActiveOpMode.hardwareMap().get(WebcamName.class, "Webcam 1"))
                .addProcessors(apriltagPipeline, ballPipeline)
                .setCameraResolution(new Size(1920, 1080))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .setAutoStopLiveView(true)
                .setLiveViewContainerId(myPortalsList[0])
                .build();

        portal.setProcessorEnabled(apriltagPipeline, true);
        portal.setProcessorEnabled(ballPipeline, false);
    }

    // extra 4" for cam-to-flywheel, 18" for AT-to-back-of-goal
    public double getTargetArtifactTravelDistanceX() {
        return getATdist() + 22;
    }

    @Override
    public void periodic() {}
}
