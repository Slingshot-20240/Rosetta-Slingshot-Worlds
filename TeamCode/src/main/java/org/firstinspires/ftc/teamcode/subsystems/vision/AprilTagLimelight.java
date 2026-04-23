package org.firstinspires.ftc.teamcode.subsystems.vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.Objects;
import java.util.stream.Stream;


public class AprilTagLimelight {
    Limelight3A limelight;
    boolean isBlue;

    public AprilTagLimelight(HardwareMap hw, boolean isBlue, Telemetry t) {
        limelight = hw.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();
        this.isBlue = isBlue;


        t.addLine(String.format("connected: %s, running: %s, status: %s, index: %s",
                limelight.isConnected(),
                limelight.isRunning(),
                limelight.getStatus(),
                limelight.getStatus().getPipelineIndex())
        );
    }
    public ObeliskLocation getObelisk(){
        return Stream.of(limelight.getLatestResult().getFiducialResults()).map((a)->(FiducialResult)a).filter(Objects::nonNull).map(FiducialResult::getFiducialId).map(ObeliskLocation::fromInt).filter(Objects::nonNull).findFirst().orElse(null);
    }

    /*
    note that the imu value will make it more accurate (but only if there is imu).
    imu is 0 when blue alliance is on the left (and red is on the right)

    default value is null
    */
    public Pose2D getBotPose(){
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            return new Pose2D(DistanceUnit.CM,
                              result.getBotpose().getPosition().toUnit(DistanceUnit.CM).x,
                              result.getBotpose().getPosition().toUnit(DistanceUnit.CM).y,
                              AngleUnit.RADIANS,
                              result.getBotpose().getOrientation().getYaw(AngleUnit.RADIANS));
        }
        return null;
    }
    public Pose2D getBotPose(double yaw) {

        LLResult result = limelight.getLatestResult();

        limelight.updateRobotOrientation(yaw-270);
        if (result != null && result.isValid()) {
            return new Pose2D(DistanceUnit.CM,
                    result.getBotpose().getPosition().toUnit(DistanceUnit.CM).x,
                    result.getBotpose().getPosition().toUnit(DistanceUnit.CM).y,
                    AngleUnit.DEGREES,
                    result.getBotpose().getOrientation().getYaw(AngleUnit.DEGREES));
        }
        return null;
    }

    public double getDistance(){
        for(FiducialResult result : limelight.getLatestResult().getFiducialResults())
        {
            if(correctAT(result.getFiducialId())){ //distance formula i think this works
                return Math.sqrt(Math.pow(result.getRobotPoseTargetSpace().getPosition().y, 2) +
                                 Math.pow(result.getRobotPoseTargetSpace().getPosition().x,2));
            }
        }
        return 0.0;
    }
    public double getAngle(){
        for(FiducialResult result : limelight.getLatestResult().getFiducialResults())
        {
            if(correctAT(result.getFiducialId())){
                return result.getTargetXDegrees();
            }
        }
        return 0.0;
    }




    private boolean correctAT(int id){
        return (id == 20 && isBlue) || (id == 24 && !isBlue);
    }
    public enum ObeliskLocation //measured by the location of the green
    {
        LEFT("GPP", 21), CENTER("PGP", 22), RIGHT("PPG", 23);

        public final String order;
        public final int ATnumber;
        ObeliskLocation(String o, int AT){
            order = o;
            ATnumber = AT;
        }

        public static ObeliskLocation fromInt(int i){
            for(ObeliskLocation ol : ObeliskLocation.values()){
                if (ol.ATnumber == i)
                    return ol;
            }
            return null;
        }
    }

}
