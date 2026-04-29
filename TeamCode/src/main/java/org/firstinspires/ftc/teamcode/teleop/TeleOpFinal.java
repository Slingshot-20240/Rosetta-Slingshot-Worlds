package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.NextFTC.autonomous.PoseStorage;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.teleop.FSM;
import org.firstinspires.ftc.teamcode.teleop.gamepad.GamepadMapping;

@Config
@TeleOp(name="ASling TeleOp Final")
public class TeleOpFinal extends OpMode {
    private Robot robot;
    private GamepadMapping controls;
    private IshaanFSM fsm;
    private DcMotorEx leftFront, rightFront, leftBack, rightBack;


    @Override
    public void init() {
        controls = new GamepadMapping(gamepad1, gamepad2);
        robot    = new Robot(hardwareMap, controls);
        fsm      = new IshaanFSM(hardwareMap, controls, robot);

        leftFront  = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftBack   = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack  = hardwareMap.get(DcMotorEx.class, "rightBack");

        leftFront .setDirection(DcMotor.Direction.REVERSE);
        leftBack  .setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack .setDirection(DcMotor.Direction.FORWARD);

        for (DcMotor m : new DcMotor[]{leftFront, rightFront, leftBack, rightBack}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    @Override
    public void start() {
        Robot.limelight.start();
    }

    @Override
    public void loop() {
        fsm.update();

        // Keep limelight fresh every loop
        YawPitchRollAngles orientation = robot.imu.getRobotYawPitchRollAngles();
        Robot.limelight.update(orientation);

        double forward = -gamepad1.left_stick_y;
        double strafe  = gamepad1.left_stick_x;
        double rotate  = gamepad1.right_stick_x * 0.75;

        if (gamepad1.a) {
            // FIX: autoAlign() already drives the motors internally (full power, no current limit).
            // Do NOT call mecanumDriveAutoAlign() here as well — that would overwrite the yaw
            // correction that autoAlign just applied, breaking the alignment entirely.
            boolean aligned = Robot.limelight.autoAlign(forward, strafe, gamepad1);

            telemetry.addLine(">> AUTO ALIGNING (hold A)");
            telemetry.addData("aligned",           aligned);
            telemetry.addData("bearing (tX)",      Robot.limelight.getAngleBearing());
            telemetry.addData("distance (inches)", Robot.limelight.getDistanceInches());
            telemetry.addData("has target",        Robot.limelight.hasTarget());
        } else {
            // Full driver control with current limiting
            mecanumDrive(forward, strafe, rotate);

            telemetry.addLine(">> DRIVER CONTROL (hold A to align)");
            telemetry.addData("has target",        Robot.limelight.hasTarget());
            telemetry.addData("bearing (tX)",      Robot.limelight.getAngleBearing());
            telemetry.addData("distance (inches)", Robot.limelight.getDistanceInches());
        }
        telemetry.addData("ol", robot.shooter.outtake1.getVelocity());
        telemetry.addData("or", robot.shooter.outtake2.getVelocity());

        telemetry.addData("Control Type", fsm.getControlType());
        telemetry.addData("Switch value", controls.switchMode.value());
        telemetry.update();
    }

    @Override
    public void stop() {
        Robot.limelight.stop();
    }

    /** Current-limited drive — used for normal driver control. */
    private void mecanumDrive(double axial, double lateral, double yaw) {
        double lf = axial + lateral + yaw;
        double rf = axial - lateral - yaw;
        double lb = axial - lateral + yaw;
        double rb = axial + lateral - yaw;

        double max = Math.max(1.0,
                Math.max(Math.abs(lf),
                        Math.max(Math.abs(rf),
                                Math.max(Math.abs(lb), Math.abs(rb)))));

        double lfCurrent = leftFront.getCurrent(CurrentUnit.AMPS);
        double rfCurrent = rightFront.getCurrent(CurrentUnit.AMPS);
        double lbCurrent = leftBack.getCurrent(CurrentUnit.AMPS);
        double rbCurrent = rightBack.getCurrent(CurrentUnit.AMPS);

        double averageCurrent = (lfCurrent + rfCurrent + lbCurrent + rbCurrent) / 4;
        if (averageCurrent > 6) {
            lf *= 0.5;
            rf *= 0.5;
            lb *= 0.5;
            rb *= 0.5;
        }

        leftFront .setPower(lf / max);
        rightFront.setPower(rf / max);
        leftBack  .setPower(lb / max);
        rightBack .setPower(rb / max);
    }

    /** Full-power drive (no current limiting) — kept here if ever needed locally. */
    private void mecanumDriveAutoAlign(double axial, double lateral, double yaw) {
        double lf = axial + lateral + yaw;
        double rf = axial - lateral - yaw;
        double lb = axial - lateral + yaw;
        double rb = axial + lateral - yaw;

        double max = Math.max(1.0,
                Math.max(Math.abs(lf),
                        Math.max(Math.abs(rf),
                                Math.max(Math.abs(lb), Math.abs(rb)))));

        leftFront .setPower(lf / max);
        rightFront.setPower(rf / max);
        leftBack  .setPower(lb / max);
        rightBack .setPower(rb / max);
    }
}