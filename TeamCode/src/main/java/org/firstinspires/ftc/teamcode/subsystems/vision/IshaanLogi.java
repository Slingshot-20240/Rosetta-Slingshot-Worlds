package org.firstinspires.ftc.teamcode.subsystems.vision;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class IshaanLogi {
    AprilTagProcessor apriltagPipeline;

    VisionPortal portal;



    public IshaanLogi(HardwareMap hw) {
        apriltagPipeline = new AprilTagProcessor.Builder()
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .build();


        portal = new VisionPortal.Builder()
                .setCamera(hw.get(WebcamName.class, "Webcam 1"))
                .addProcessors(apriltagPipeline)
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .setAutoStopLiveView(true)
                .enableLiveView(true)
                .build();

        portal.setProcessorEnabled(apriltagPipeline, true);

    }

    public void enableAT() {
        portal.setProcessorEnabled(apriltagPipeline, true);
    }

    public void enableAT(boolean enabled) {
        portal.setProcessorEnabled(apriltagPipeline, enabled);
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
}
