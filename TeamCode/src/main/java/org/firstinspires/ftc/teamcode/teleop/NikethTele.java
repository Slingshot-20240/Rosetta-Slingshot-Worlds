package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.NextFTC.autonomous.PoseStorage;
import org.firstinspires.ftc.teamcode.teleop.gamepad.GamepadMapping;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@Config
@TeleOp
public class NikethTele extends OpMode {

    private GamepadMapping controls;
    private FSM fsm;
    private Robot robot;
    private Follower follower;
    private TelemetryManager telemetryM;
    VoltageSensor voltageSensor;



    public static double odoDistance;

    // ---------------- AUTO TURN ALIGN ----------------
    private static final double TX_THRESHOLD_DEG = 1.0;
    private static final double BLUE_TX_OFFSET_DEG = 5.0;
    private static final double RED_TX_OFFSET_DEG = -5.0;

    public static double MAX_TURN_POWER = 0.6;
    public static double kP_TURN = 0.001;
    public static double kD_TURN = 0.002;

    private double lastErr = 0.0;
    private boolean alignedLastLoop = false;

    @Override
    public void init() {

        controls = new GamepadMapping(gamepad1, gamepad2);
        robot = new Robot(hardwareMap, controls);
        fsm = new FSM(hardwareMap, controls, robot);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(PoseStorage.startingPose);
        //follower.setStartingPose(new Pose(126.2, 119, Math.toRadians(36)));
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

//        if (voltageSensor.getVoltage() < 9) {
//            stop();
//        }

        Pose pose = follower.getPose();
        double heading = pose.getHeading();
        odoDistance = pose.distanceFrom(new Pose(138,138));

        boolean controllerBusy =
                Math.abs(gamepad1.left_stick_x) > 0.05 ||
                        Math.abs(gamepad1.left_stick_y) > 0.05 ||
                        Math.abs(gamepad1.right_stick_x) > 0.05;


    // Drive Controls ----------------------------------------------------------------//

        double forward = -Math.pow(gamepad1.left_stick_y, 5);
        double strafe  = -Math.pow(gamepad1.left_stick_x, 5);
        double rotate;

        if (gamepad1.a) {
            forward = forward * 0.26;
            strafe = strafe * 0.2;
        }


//    if (gamepad1.aWasPressed()) {
//         follower.setPose(new Pose(141.5,0, Math.toRadians(90)));
//    }
//    if (gamepad1.y) {
//
//        double dx = GOAL_X - pose.getX();
//        double dy = GOAL_Y - pose.getY();
//        double targetHeading = Math.atan2(dy, dx);
//        double odoHeadingError = angleWrap(targetHeading - heading);
//        boolean odoTurnFinished =
//                Math.abs(odoHeadingError) < tolerance;
//
//        forward = 0;
//        strafe = 0;
//
//        double error;
//        double kP;
//        double minPower;
//        double miniTolerance;
//
//        error = odoHeadingError;
//        kP = odoTurn_kP;
//        minPower = odoMinTurnPower;
//        miniTolerance = tolerance;
//
//        rotate = error * kP;
//
//        if (Math.abs(rotate) < minPower && Math.abs(error) > miniTolerance) {
//            rotate = Math.signum(rotate) * minPower;
//        }
//
//    } else {
//        rotate = -gamepad1.right_stick_x * 0.55;
//    }


        // ---------------- AUTO ALIGN ----------------
        if (gamepad1.right_trigger > 0.1) {
            telemetry.addData("ALIGNING","true");
            double error = Robot.cam.getATangle();
            double derivative = error - lastErr;

            lastErr = error;

            if (Robot.cam.getATdist() > 100) {
                telemetry.addLine("Far");
                MAX_TURN_POWER = 0.6;
                kP_TURN = 0.009;
                kD_TURN = 0.001;
                double turnCmd = (kP_TURN * error) + (kD_TURN * derivative);

                rotate = Range.clip(turnCmd, -MAX_TURN_POWER, MAX_TURN_POWER);
                telemetry.addData("Rotate value", rotate);

            } else {
                telemetry.addLine("Close");
                MAX_TURN_POWER = 0.6;
                kP_TURN = 0.009;
                kD_TURN = 0.001;
                double turnCmd = (kP_TURN * error) + (kD_TURN * derivative);

                rotate = Range.clip(turnCmd, -MAX_TURN_POWER, MAX_TURN_POWER);
                telemetry.addData("Rotate value", rotate);
            }

            follower.setTeleOpDrive(0, 0, rotate, true);

        } else {
            lastErr = 0;
            rotate = -(Math.pow(gamepad1.right_stick_x, 3)) * 0.38;

            if (gamepad1.left_trigger > 0.1) {
                if (gamepad1.a) {
                    rotate = rotate * 0.2;
                }
                telemetry.addData("Mecanum Override", true);
                follower.setTeleOpDrive(forward, strafe, rotate, true);

            } else {
                if (gamepad1.a) {
                    rotate = rotate * 0.2;
                }
                follower.setTeleOpDrive(forward, 0, rotate, true);
            }
            telemetry.addData("Rotate value", rotate);
        }





        // Telemetry
        telemetry.addData("Pose", pose.toString());
        telemetry.addData("Heading (deg)", Math.toDegrees(heading));
        telemetry.addData("AT Distance", Robot.cam.getATdist());

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

//    public double lockHeading(double targetAngle) {
//        double wrappedTarget = angleWrap(targetAngle);
//        double pid = turnController.calculate(wrappedTarget, false);
//        return pid + turnF;
//    }



}
