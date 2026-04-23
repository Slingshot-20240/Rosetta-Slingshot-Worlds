//package org.firstinspires.ftc.teamcode.NextFTC.misc;
//
//import com.qualcomm.robotcore.hardware.DigitalChannel;
//
//import dev.nextftc.core.subsystems.Subsystem;
//import dev.nextftc.ftc.ActiveOpMode;
//
//public class Breakbeamnf implements Subsystem {
//    public static final Breakbeamnf INSTANCE = new Breakbeamnf();
//    private Breakbeamnf() {}
//
//    private DigitalChannel bblow;
//    private DigitalChannel bbmid;
//    private DigitalChannel bbhigh;
//
//    private int count = 0;
//
//    @Override
//    public void initialize() {
//        bblow = ActiveOpMode.hardwareMap().get(DigitalChannel.class, "bbl");
//        bbmid = ActiveOpMode.hardwareMap().get(DigitalChannel.class, "bbm");
//        bbhigh = ActiveOpMode.hardwareMap().get(DigitalChannel.class, "bbh");
//
//        bblow.setMode(DigitalChannel.Mode.INPUT);
//        bbmid.setMode(DigitalChannel.Mode.INPUT);
//        bbhigh.setMode(DigitalChannel.Mode.INPUT);
//    }
//
//    public void resetCount(){
//        count = 0;
//    }
//
//    @Override
//    public void periodic(){
//        // True if the sensor is blocked by a ball
//        boolean lowBlocked = !bblow.getState();   // DigitalChannel returns true when NOT blocked
//        boolean midBlocked = !bbmid.getState();
//        boolean highBlocked = !bbhigh.getState();
//
//        // Count the number of balls in intake
//        count = 0;
//        if(lowBlocked) count++;
//        if(midBlocked) count++;
//        if(highBlocked) count++;
//
//        // Telemetry
//        ActiveOpMode.telemetry().addLine(lowBlocked ? "Low blocked" : "Low clear");
//        ActiveOpMode.telemetry().addLine(midBlocked ? "Mid blocked" : "Mid clear");
//        ActiveOpMode.telemetry().addLine(highBlocked ? "High blocked" : "High clear");
//        ActiveOpMode.telemetry().addData("Balls in intake", count);
//        ActiveOpMode.telemetry().update();
//    }
//
//    public int getCount(){
//        return count;
//    }
//}
