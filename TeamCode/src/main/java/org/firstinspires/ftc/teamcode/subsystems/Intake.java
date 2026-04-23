package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    public final DcMotorEx intake1;
    public final DcMotorEx dropdownIntake;

    //public final DigitalChannel beamBreakLow, beamBreakMid, beamBreakHigh;

    private int count = 0;
    private boolean lastDetected = false;


    public Intake(HardwareMap hardwareMap) {
        intake1 = hardwareMap.get(DcMotorEx.class, "intake1");
        dropdownIntake = hardwareMap.get(DcMotorEx.class, "dropdownIntake");
        intake1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dropdownIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    // constructor for JUnit
    public Intake(DcMotorEx intake1, DcMotorEx intake2) {
        this.dropdownIntake = intake2;
        this.intake1 = intake1;
    }

//-------------------------------------------------------------------------------

    public void intakeTransferOnClose() {
        dropdownIntake.setPower(-1);
        intake1.setPower(-1);
    }

    public void intakeTransferOnFar() {
        dropdownIntake.setPower(-1);
        intake1.setPower(-0.75);
    }

    public void intakeTransferOff() {
        intake1.setPower(0);
        dropdownIntake.setPower(0);
    }

    public void intakeTransferReverse() {
        dropdownIntake.setPower(1);
        intake1.setPower(1);
    }

//    public int getCount() {
//        boolean detected = beamBreakLow.getState();
//        if (detected && !lastDetected) {
//            count++;
//        }
//
//        lastDetected = detected;
//
//        return count;
//    }

}
