package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {
    public final DcMotorEx transfer;
    public final DcMotorEx dropdownIntake;
    public final Servo pivotServo;

    public Intake(HardwareMap hardwareMap) {
        transfer = hardwareMap.get(DcMotorEx.class, "transfer");
        transfer.setDirection(DcMotorSimple.Direction.REVERSE);

        dropdownIntake = hardwareMap.get(DcMotorEx.class, "dropdownIntake");

        pivotServo = hardwareMap.get(Servo.class, "pivotServo");
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
        transfer.setPower(power);
    }
    public void intakeTransferOnClose() {
        dropdownIntake.setPower(-1);
        transfer.setPower(-1);
    }

    public void intakeTransferOnFar() {
        dropdownIntake.setPower(-1);
        transfer.setPower(-0.75);
    }

    public void intakeTransferOff() {
        transfer.setPower(0);
        dropdownIntake.setPower(0);
    }

    public void intakeTransferReverse() {
        dropdownIntake.setPower(1);
        transfer.setPower(1);
    }

    public void pivotUp() {
       pivotServo.setPosition(0.11);
    }

    public void pivotDown() {
        pivotServo.setPosition(0.67);
    }

    public void setPivotPos(double position) {
        pivotServo.setPosition(position);
    }


}