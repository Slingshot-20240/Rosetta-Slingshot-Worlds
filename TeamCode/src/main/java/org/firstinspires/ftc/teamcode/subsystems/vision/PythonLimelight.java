package org.firstinspires.ftc.teamcode.subsystems.vision;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class PythonLimelight {
    /*
    ACUTAL FORMAT:
    [
        most recent obelisk ID,
        most recent nav ID,
        angle from change yes (if the marker is on the right, then +, if on left, -),
        distance from tag (roughly +/- 1 in)
    ]

    IDEAL FORMAT:
    [
        most recent obelisk ID,
        most recent nav ID,
        relocalized x based on nav ID,
        relocalized y,
        relocalized z,
        relocalized yaw,
        relocalized pitch,
        relocalized roll
    ]
    */
    Limelight3A limelight;

    public PythonLimelight(HardwareMap hw) {
        limelight = hw.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
//        limelight.reloadPipeline();
        limelight.start();

        limelight.pipelineSwitch(0);
    }
    public PythonLimelight(HardwareMap hw, Telemetry t) {
        limelight = hw.get(Limelight3A.class, "limelight");
        t.addLine(String.format("connected: %s, running: %s, status: %s, index: %s",
                                limelight.isConnected(),
                                limelight.isRunning(),
                                limelight.getStatus(),
                                limelight.getStatus().getPipelineIndex())
        );

        limelight.setPollRateHz(100);
//        limelight.reloadPipeline();
        limelight.start();
        if(limelight.getStatus().getPipelineIndex() != 0)
            limelight.pipelineSwitch(0);
    }
    public ObeliskLocation getObelisk(){
        return ObeliskLocation.fromInt((int) limelight.getLatestResult().getPythonOutput()[0]);
    }

    //pos for turn right (clockwise), neg for turn left (counter clockwise) [radians]
    public double getAngle(){
        double[] llr = limelight.getLatestResult().getPythonOutput();
        if (llr == null || llr.length < 2)
            return 0;

        return llr[2];
    }

    public double getDistance() {
        double[] llr = limelight.getLatestResult().getPythonOutput();
        if (llr == null || llr.length < 3)
            return 0;

        return llr[3];
    }
    public double getLastNav() {
        double[] llr = limelight.getLatestResult().getPythonOutput();
        if (llr == null || llr.length < 1)
            return 0;
        return llr[1];

    }
    /*
    public Pose3D getPose() {
        double[] llResults = limelight.getLatestResult().getPythonOutput();
        return new Pose3D(new Position(DistanceUnit.INCH, //TODO: figure this out LOL
                                       llResults[2],
                                       llResults[3],
                                       llResults[4],
                                       System.nanoTime()),

                          new YawPitchRollAngles(AngleUnit.DEGREES, //TODO: figure this out LOLOLLLL
                                                 llResults[5],
                                                 llResults[6],
                                                 llResults[7],
                                                 System.nanoTime()));
    }
    */
    public void close(){
        limelight.stop();
        limelight.close();
        limelight.shutdown();
    }
    public enum ObeliskLocation //measured by the location of the green
    {
        LEFT("GPP", 21), CENTER("PGP", 22), RIGHT("PPG", 23), NONE("none", -1);

        public final String order;
        public final int ATnumber;
        ObeliskLocation(String o, int AT){
            order = o;
            ATnumber = AT;
        }

        public static ObeliskLocation fromInt(int i){
            if(i == 0)
                return NONE;
            for(ObeliskLocation ol : ObeliskLocation.values()){
                if (ol.ATnumber == i)
                    return ol;
            }
            return null;
        }
    }

}
