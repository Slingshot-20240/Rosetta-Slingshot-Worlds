package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {
    public final DcMotorEx transfer;
    public final DcMotorEx dropdownIntake;

    public final DigitalChannel beamBreakFront; // near front of intake - triggers pivot up + stops all intake
    public final DigitalChannel beamBreakTop;   // top - stops transfer motor, dropdown still runs

    public final Servo pivotServo;

    // Pivot servo positions
    public static final double PIVOT_UP   = 1.0;
    public static final double PIVOT_DOWN = 0.0;

    // True when the last (3rd) ball is detected — both sensors triggered
    public boolean hasBall = false;


    public Intake(HardwareMap hardwareMap) {
        transfer = hardwareMap.get(DcMotorEx.class, "transfer");
        dropdownIntake = hardwareMap.get(DcMotorEx.class, "dropdownIntake");
        transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dropdownIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        beamBreakFront = hardwareMap.get(DigitalChannel.class, "beamBreakFront");
        beamBreakTop   = hardwareMap.get(DigitalChannel.class, "beamBreakTop");

        beamBreakFront.setMode(DigitalChannel.Mode.INPUT);
        beamBreakTop.setMode(DigitalChannel.Mode.INPUT);

        pivotServo = hardwareMap.get(Servo.class, "pivotServo");
    }

    // Constructor for JUnit
    public Intake(DcMotorEx transfer, DcMotorEx intake2) {
        this.dropdownIntake = intake2;
        this.transfer       = transfer;
        this.beamBreakFront = null;
        this.beamBreakTop   = null;
        this.pivotServo     = null;
    }

//-------------------------------------------------------------------------------

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

//-------------------------------------------------------------------------------
// Pivot servo functions

    /** Pivots the servo up (ball loaded position) */
    public void pivotUp() {
        if (pivotServo != null) pivotServo.setPosition(PIVOT_UP);
    }

    /** Pivots the servo down (intaking position) */
    public void pivotDown() {
        if (pivotServo != null) pivotServo.setPosition(PIVOT_DOWN);
    }

//-------------------------------------------------------------------------------
// Beam break sensor helpers

    /** Returns true when the front beam break detects a ball */
    public boolean isFrontBeamBreakTriggered() {
        return beamBreakFront != null && !beamBreakFront.getState();
    }

    /** Returns true when the top beam break detects a ball */
    public boolean isTopBeamBreakTriggered() {
        return beamBreakTop != null && !beamBreakTop.getState();
    }

//-------------------------------------------------------------------------------
// Ball counting using beam breaks

    /**
     * Returns the number of balls currently detected inside the robot.
     *
     * beamBreakFront = ball entering intake (stage 1)
     * beamBreakTop   = ball fully loaded at top (stage 2 / final)
     *
     * hasBall is set to true when count reaches 2 (both sensors triggered),
     * representing the robot being full with 3 balls.
     */
    public int getCount() {
        int count = 0;
        if (isFrontBeamBreakTriggered()) count++;
        if (isTopBeamBreakTriggered())   count++;

        hasBall = (count >= 2);

        return count;
    }

//-------------------------------------------------------------------------------
// Smart intake functions using beam breaks

    /**
     * Call this in your opmode loop during intaking.
     *
     * Behavior:
     *   - Both dropdownIntake and transfer run until top beam break sees a ball
     *   - Once top beam break is triggered: dropdown still runs, transfer stops
     *   - Once front beam break is triggered: pivot goes up, both motors stop
     */
    public void updateSmartIntake() {
        getCount(); // keep hasBall updated every loop

        if (isFrontBeamBreakTriggered()) {
            pivotUp();
            transfer.setPower(0);
            dropdownIntake.setPower(0);
        } else if (isTopBeamBreakTriggered()) {
            dropdownIntake.setPower(-1);
            transfer.setPower(0);
        } else {
            dropdownIntake.setPower(-1);
            transfer.setPower(-1);
        }
    }
}