package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {
    public final DcMotorEx intake1;
    public final DcMotorEx dropdownIntake;

    public final DigitalChannel beamBreakFront;  // near front of intake - triggers pivot up + stops all intake
    public final DigitalChannel beamBreakMid;    // middle - unused for now, kept for future use
    public final DigitalChannel beamBreakTop;    // top - stops transfer motor, dropdown still runs

    public final Servo pivotServo;

    // Pivot servo positions
    public static final double PIVOT_UP   = 1.0;
    public static final double PIVOT_DOWN = 0.0;

    private boolean lastDetected = false;


    public Intake(HardwareMap hardwareMap) {
        intake1 = hardwareMap.get(DcMotorEx.class, "intake1");
        dropdownIntake = hardwareMap.get(DcMotorEx.class, "dropdownIntake");
        intake1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dropdownIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        beamBreakFront = hardwareMap.get(DigitalChannel.class, "beamBreakFront");
        beamBreakMid   = hardwareMap.get(DigitalChannel.class, "beamBreakMid");
        beamBreakTop   = hardwareMap.get(DigitalChannel.class, "beamBreakTop");

        beamBreakFront.setMode(DigitalChannel.Mode.INPUT);
        beamBreakMid.setMode(DigitalChannel.Mode.INPUT);
        beamBreakTop.setMode(DigitalChannel.Mode.INPUT);

        pivotServo = hardwareMap.get(Servo.class, "pivotServo");
    }

    // constructor for JUnit
    public Intake(DcMotorEx intake1, DcMotorEx intake2) {
        this.dropdownIntake = intake2;
        this.intake1 = intake1;
        this.beamBreakFront = null;
        this.beamBreakMid   = null;
        this.beamBreakTop   = null;
        this.pivotServo     = null;
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

    /** Returns true when the middle beam break detects a ball */
    public boolean isMidBeamBreakTriggered() {
        return beamBreakMid != null && !beamBreakMid.getState();
    }

    /** Returns true when the top beam break detects a ball */
    public boolean isTopBeamBreakTriggered() {
        return beamBreakTop != null && !beamBreakTop.getState();
    }

//-------------------------------------------------------------------------------
// Ball counting using beam breaks

    /**
     * Returns the number of balls currently detected inside the robot.
     * Each beam break that is triggered counts as one ball.
     *
     * beamBreakFront = ball entering intake (stage 1)
     * beamBreakMid   = ball in transfer (stage 2)
     * beamBreakTop   = ball fully loaded at top (stage 3)
     */
    public int getCount() {
        int count = 0;
        if (isFrontBeamBreakTriggered()) count++;
        if (isMidBeamBreakTriggered())   count++;
        if (isTopBeamBreakTriggered())   count++;
        return count;
    }

//-------------------------------------------------------------------------------
// Smart intake functions using beam breaks

    /**
     * Call this in your opmode loop during intaking.
     *
     * Behavior:
     *   - Both dropdownIntake and transfer (intake1) run until top beam break sees a ball
     *   - Once top beam break is triggered: dropdown still runs, transfer stops
     *   - Once front beam break is triggered: pivot goes up, both motors stop
     */
    public void updateSmartIntake() {
        if (isFrontBeamBreakTriggered()) {
            // Ball detected at front - pivot up and kill all intake
            pivotUp();
            intake1.setPower(0);
            dropdownIntake.setPower(0);
        } else if (isTopBeamBreakTriggered()) {
            // Ball at top - keep dropdown running, stop transfer
            dropdownIntake.setPower(-1);
            intake1.setPower(0);
        } else {
            // No ball detected yet - run both at full
            dropdownIntake.setPower(-1);
            intake1.setPower(-1);
        }
    }


}