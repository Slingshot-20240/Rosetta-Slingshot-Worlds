package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.teleop.gamepad.GamepadMapping;

public class Intake {
    public final DcMotorEx transfer;
    public final DcMotorEx dropdownIntake;
    public final Servo pivotServo;

    private GamepadMapping controls;

    private boolean ballIn = false;
    private int cycles = 0;
    public Intake(HardwareMap hardwareMap, GamepadMapping controls) {
        transfer = hardwareMap.get(DcMotorEx.class, "transfer");
        transfer.setDirection(DcMotorSimple.Direction.REVERSE);

        dropdownIntake = hardwareMap.get(DcMotorEx.class, "dropdownIntake");

        pivotServo = hardwareMap.get(Servo.class, "pivotServo");

        this.controls = controls;
    }

    // Constructor for JUnit
    public Intake(DcMotorEx transfer, DcMotorEx intake2) {
        this.dropdownIntake = intake2;
        this.transfer       = transfer;
        this.pivotServo     = null;
    }

//-------------------------------------------------------------------------------

    public void setIntakePower(double power) {
        dropdownIntake.setPower(power);
        safeSetPower(power, controls.gamepad1);
    }
    public void intakeTransferOnClose() {
        dropdownIntake.setPower(1);
        safeSetPower(1, controls.gamepad1);
    }

    public void intakeTransferOnFar() {
        dropdownIntake.setPower(1);
        safeSetPower(0.75, controls.gamepad1);
    }

    public void intakeTransferOff() {
        dropdownIntake.setPower(0);
        safeSetPower(0, controls.gamepad1);
    }

    public void intakeTransferReverse() {
        dropdownIntake.setPower(-1);
        safeSetPower(-1, controls.gamepad1);
    }

    public void pivotUp() {
       pivotServo.setPosition(0.28);
    }

    public void pivotDown() {
        pivotServo.setPosition(0.9);
    }

    public void setPivotPos(double position) {
        pivotServo.setPosition(position);
    }
    public double getTransferCurrent() {
        return transfer.getCurrent(CurrentUnit.AMPS);
    }
    public void safeSetPower(double power, Gamepad gamepad) {
        if (gamepad.right_bumper) {
            ballIn = false;
            cycles = 0;
        }
        if (getTransferCurrent() > 2) {
            cycles = cycles +1;
        } else {
            cycles = 0;
        }
        if (cycles>5) {
            ballIn = true;
        }
        if (!ballIn) {
            transfer.setPower(power);
        }
        else {
            transfer.setPower(0);
        }
    }


}