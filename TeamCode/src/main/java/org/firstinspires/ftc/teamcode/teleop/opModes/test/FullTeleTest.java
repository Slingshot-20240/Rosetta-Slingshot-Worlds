package org.firstinspires.ftc.teamcode.teleop.opModes.test;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.NextFTC.autonomous.PoseStorage;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.teleop.FSM;
import org.firstinspires.ftc.teamcode.teleop.gamepad.GamepadMapping;

@Config
@TeleOp(name="FullTeleTest", group="test")
public class FullTeleTest extends OpMode {

    private GamepadMapping controls;
    private FSM fsm;
    private Robot robot;

    private Follower follower;

    private boolean autoTurnVision = false;
    private boolean autoTurnOdo = false;

    private TelemetryManager telemetryM;
    public static double odoDistance;


    public static double tolerance = 0.01;

    // Vision tuning
    public static double visionTurn_kP = 0.04;
    public static double visionMinTurnPower = 0.1;
    public static double visionMiniTolerance = 0.01;

    // ODO target
    public static double GOAL_X = 138;
    public static double GOAL_Y = 138;

    // ODO tuning
    public static double odoTurn_kP = 0.3;
    public static double odoMinTurnPower = 0.08;

    @Override
    public void init() {

        controls = new GamepadMapping(gamepad1, gamepad2);
        robot = new Robot(hardwareMap, controls);
        fsm = new FSM(hardwareMap, controls, robot);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(PoseStorage.startingPose);
        follower.update();

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

    }

    @Override
    public void start() {
        follower.startTeleopDrive(true);
    }

    @Override
    public void loop() {

        fsm.update();
        follower.update();
        telemetryM.update();
        telemetry.update();

        Pose pose = follower.getPose();
        double heading = pose.getHeading();
        odoDistance = pose.distanceFrom(new Pose(138,138));


        boolean controllerBusy =
                Math.abs(gamepad1.left_stick_x) > 0.05 ||
                        Math.abs(gamepad1.left_stick_y) > 0.05 ||
                        Math.abs(gamepad1.right_stick_x) > 0.05;

        double forward = -gamepad1.left_stick_y;
        double strafe  = -gamepad1.left_stick_x;
        double rotate;



        //------------- error calculation -------------\\
        // Vision error
        double visionBearing = Math.toRadians(Robot.cam.getATangle());
        double visionHeadingError = angleWrap(visionBearing);
        boolean visionTurnFinished =
                Math.abs(visionHeadingError) < tolerance;

        // ODO error
        double dx = GOAL_X - pose.getX();
        double dy = GOAL_Y - pose.getY();
        double targetHeading = Math.atan2(dy, dx);
        double odoHeadingError = angleWrap(targetHeading - heading);
        boolean odoTurnFinished =
                Math.abs(odoHeadingError) < tolerance;

        //------------- rotate logic -------------\\

        if (autoTurnVision || autoTurnOdo) {

            forward = 0;
            strafe = 0;

            double error;
            double kP;
            double minPower;
            double miniTolerance;

            if (autoTurnVision) {
                error = visionHeadingError;
                kP = visionTurn_kP;
                minPower = visionMinTurnPower;
                miniTolerance = visionMiniTolerance;
            } else {
                error = odoHeadingError;
                kP = odoTurn_kP;
                minPower = odoMinTurnPower;
                miniTolerance = tolerance;
            }

            rotate = error * kP;

            if (Math.abs(rotate) < minPower && Math.abs(error) > miniTolerance) {
                rotate = Math.signum(rotate) * minPower;
            }

        } else {
            rotate = -gamepad1.right_stick_x * 0.55;
        }

        //follower.startTeleopDrive(true);

        follower.setTeleOpDrive(forward, strafe, rotate, true);
        //TODO - Check if even needed

        if (gamepad1.x) {
            follower.setPose(new Pose(72,8,Math.toRadians(90)));
        }

        // Path following




        /* ---------------- AUTO TURN TOGGLES ---------------- */

        if (gamepad1.a && !autoTurnVision) {
            autoTurnVision = true;
        }

        if (gamepad1.left_trigger > 0 && !autoTurnOdo) {
            autoTurnOdo = true;
        }

        // Turn cancellation

        if ((autoTurnVision && visionTurnFinished) || controllerBusy) {
            autoTurnVision = false;
        }

        if ((autoTurnOdo && odoTurnFinished) || controllerBusy) {
            autoTurnOdo = false;
        }

        // Telemetry
        telemetry.addData("State", fsm.state);

        telemetry.addData("Pose", pose);
        telemetry.addData("Heading (deg)", Math.toDegrees(heading));
        telemetry.addLine("---- AUTO TURN ----");
        telemetry.addData("Vision Error (deg)", Math.toDegrees(visionHeadingError));
        telemetry.addData("ODO Error (deg)", Math.toDegrees(odoHeadingError));
        telemetry.addData("AutoTurn Vision", autoTurnVision);
        telemetry.addData("AutoTurn ODO", autoTurnOdo);
    }

    @Override
    public void stop() {
        PoseStorage.startingPose = follower.getPose();
    }

    private double angleWrap(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
}
