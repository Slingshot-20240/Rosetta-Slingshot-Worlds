package org.firstinspires.ftc.teamcode.teleop.opModes.test;

import com.acmerobotics.dashboard.config.Config;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.NextFTC.autonomous.PoseStorage;
import org.firstinspires.ftc.teamcode.teleop.gamepad.GamepadMapping;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

@Config
@TeleOp
public class AutoAlignAT extends OpMode {

    private GamepadMapping controls;
    private Robot robot;
    private Shooter shooter;
    private Follower follower;

    private boolean autoTurnVision = false;

    //auto align
    public static double tolerance = 0.008;
    public static double turn_kP = 0.1;


    @Override
    public void init() {
        controls = new GamepadMapping(gamepad1, gamepad2);
        robot = new Robot(hardwareMap, controls);
        shooter = new Shooter(hardwareMap);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(PoseStorage.startingPose);
        follower.update();

    }

    @Override
    public void start() {
        follower.startTeleopDrive(true);
    }

    @Override
    public void loop() {
        follower.update();
        telemetry.update();
        //LED
        if (Robot.limelight.getDistanceInches() != 0) {
            robot.ledBoard0.setState(true);
            robot.ledBoard1.setState(true);
        } else {
            robot.ledBoard0.setState(false);
            robot.ledBoard1.setState(true);
        }


        //AUTO ALIGN
        double atBearing = Math.toRadians(Robot.limelight.getAngleBearing());
        double atHeadingError = angleWrap(atBearing);
        boolean visionTurnFinished = Math.abs(atHeadingError) < tolerance;


        double forward = -gamepad1.left_stick_y;
        double strafe  = -gamepad1.left_stick_x;
        double rotate;

        //Auto Align
        if (gamepad1.right_trigger >= 0.4) {
            rotate = atHeadingError * turn_kP;
        } else {
            rotate = -Math.pow(gamepad1.right_stick_x, 5);
        }
        rotate = Math.max(-1.0, Math.min(1.0, rotate));

        //TODO - if you don't want drive controls to be activated, just change forward and strafe to be 0
        follower.setTeleOpDrive(0, 0, rotate, true);


        if (gamepad1.a && !autoTurnVision) {
            autoTurnVision = true;
        }

        if (autoTurnVision && visionTurnFinished) {
            autoTurnVision = false;
        }

        telemetry.addData("AT angle", Robot.limelight.getAngleBearing());
        telemetry.addData("AT dist",  Robot.limelight.getDistanceInches());

        telemetry.addLine("--------------------------------");
        telemetry.addData("Vision AutoTurn", autoTurnVision);

    }

    private double angleWrap(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
}
