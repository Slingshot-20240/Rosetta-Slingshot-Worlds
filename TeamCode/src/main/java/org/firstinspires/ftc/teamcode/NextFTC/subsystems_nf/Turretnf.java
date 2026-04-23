package org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.AnalogInput;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.CRServoEx;
public class Turretnf implements Subsystem {

    public static double distance;
    // P is pretty much 1/range 
    public static PIDCoefficients turretPIDCoefficients =
            new PIDCoefficients(0.02, 0.0, 0);

    public static final Turretnf INSTANCE = new Turretnf();
    private Turretnf() {}

    private CRServoEx lTurret = new CRServoEx("lTurret", 0.01);
    private CRServoEx rTurret = new CRServoEx("rTurret", 0.01);


    public AnalogInput lTurretEncoder;
    public AnalogInput rTurretEncoder;

    public ControlSystem turretController = ControlSystem.builder()
            .posPid(turretPIDCoefficients)
            .build();

    public double power;
    public static Double targetTurretAng = 0.0;
    private static double offset = 4.0;
    private static final double GEAR_RATIO = 1.083;

    // Got MIN and MAX from 180/ Gear Ratio and then added some tolerance
    private static final double TURRET_MIN_DEG = -100;
    private static final double TURRET_MAX_DEG =  100;
    public static boolean AUTO_AIM = true;

    Pose goal = new Pose(141.5,141.5);


    @Override
    public void initialize() {
        //left turret encoder, right turret encoder
        lTurretEncoder = ActiveOpMode.hardwareMap().get(AnalogInput.class, "lte");
        rTurretEncoder = ActiveOpMode.hardwareMap().get(AnalogInput.class, "rte");
    }

    @Override
    public void periodic() {
        if (AUTO_AIM) {
            turretLoop();
        }
        turretController.setGoal(new KineticState(targetTurretAng));
        // (-) Power because gear rotates other gear opposite
        power = -turretController.calculate(new KineticState(getTurretDegrees()));
        lTurret.setPower(power);
        //TODO - may need to reverse this power
        rTurret.setPower(power);
    }

    public void turretLoop() {
        double dx = 141.5 - PedroComponent.follower().getPose().getX();
        double dy = 141.5 - PedroComponent.follower().getPose().getY();

        //Arc tangent of dy/dx 
        double goalFieldDeg = Math.toDegrees(Math.atan2(dy, dx));
        double headingDeg   = Math.toDegrees(PedroComponent.follower().getHeading());

        // turret relative to robot forward 0 degree facing to goal side
        double desired = normalizeAngle(goalFieldDeg - headingDeg);
        targetTurretAng = clamp(desired, TURRET_MIN_DEG, TURRET_MAX_DEG);

    }
    //SHHHHH
    public void velocityBasedTurret() {
        Vector robot2Goal = new Vector(
                141.5-PedroComponent.follower().getPose().getX(),
                141.5-PedroComponent.follower().getPose().getY()
        );
        Vector velocity = PedroComponent.follower().getVelocity();
        double heading = PedroComponent.follower().getHeading();
        double coordTheta = velocity.getTheta() - robot2Goal.getTheta();
        double perpComponent = Math.sin(coordTheta) * velocity.getMagnitude();

        distance = Math.hypot(141.5-PedroComponent.follower().getPose().getX(), 141.5-PedroComponent.follower().getPose().getY());
    }


    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    // Normalize angle to 180 so easy to work with 
    public double normalizeAngle(double angDeg) {
        double ang = angDeg;
        if (ang < 0.0) ang += 360.0;
        if (ang > 180.0) ang -= 360.0;
        return ang;
    }


    private double getTurretDegrees() {
        //TODO - may need to change max voltage
        //yeah so voltage is what feedback get from servo and its pretty much a linear graph so all you need to do is divide by max voltage which is 3.3. And then i just multiply by 360 to go into degrees you could do radians but i dont like radians
        double servoDeg = (lTurretEncoder.getVoltage() / 3.3) * 360.0;
        //TODO - NOTE we are NOT using the right encoder right now!!! we may want to average them we'll see

        // convert to turret degrees
        return normalizeAngle(servoDeg) / GEAR_RATIO;
    }
    public double turretDegrees(){
        return getTurretDegrees();
    }

    public double getPower(){
        return power;
    }

}