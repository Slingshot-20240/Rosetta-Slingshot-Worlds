package org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf;

import com.qualcomm.robotcore.hardware.DigitalChannel;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.controllable.MotorGroup;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;
import dev.nextftc.hardware.powerable.SetPower;

public class Intakenf implements Subsystem {

    public static final Intakenf INSTANCE = new Intakenf();

    private Intakenf() { }

    public MotorEx intake1;
    public MotorEx dropDownIntake;
    MotorGroup intake;

    // Beam break sensors
    public DigitalChannel beamBreakFront; // front of intake - triggers pivot up + stops all intake
    public DigitalChannel beamBreakMid;   // middle - available for future use
    public DigitalChannel beamBreakTop;   // top - stops transfer, dropdown keeps running

    // Pivot servo
    public ServoEx pivotServo;
    public static final double PIVOT_UP   = 1.0;
    public static final double PIVOT_DOWN = 0.0;

//-------------------------------------------------------------------------------
// Original commands

    public Command in() {
        return new SetPower(intake, 0.9);
    }
    public Command idle() {
        return new SetPower(intake, 0);
    }
    public Command out() {
        return new SetPower(intake, -1.0);
    }

    public Command setIntakePower(double power) {
        return new SetPower(intake, power);
    }

//-------------------------------------------------------------------------------
// Pivot servo functions
    /** Pivots the servo up (ball loaded position) */
    public Command pivotUp() {
        return new SetPosition(pivotServo, PIVOT_UP);
    }

    /** Pivots the servo down (intaking position) */
    public Command pivotDown() {
        return new SetPosition(pivotServo, PIVOT_DOWN);
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
// Ball counting

    /**
     * Returns the number of balls currently detected inside the robot.
     *
     * beamBreakFront = ball entering intake (stage 1)
     * beamBreakMid   = ball in transfer   (stage 2)
     * beamBreakTop   = ball at top        (stage 3)
     */
    public int getCount() {
        int count = 0;
        if (isFrontBeamBreakTriggered()) count++;
        if (isMidBeamBreakTriggered())   count++;
        if (isTopBeamBreakTriggered())   count++;
        return count;
    }

//-------------------------------------------------------------------------------
// Smart intake logic - call these in periodic() or your opmode loop

    /**
     * Smart intake at full transfer speed.
     *
     * Behavior:
     *   - Both motors run until top beam break sees a ball
     *   - Top triggered: dropdown keeps running, transfer stops
     *   - Front triggered: pivot up, both motors stop
     */
    public void updateSmartIntake() {
        if (isFrontBeamBreakTriggered()) {
            pivotUp();
            intake1.setPower(0);
            dropDownIntake.setPower(0);
        } else if (isTopBeamBreakTriggered()) {
            dropDownIntake.setPower(-1);
            intake1.setPower(0);
        } else {
            dropDownIntake.setPower(-1);
            intake1.setPower(-1);
        }
    }


//-------------------------------------------------------------------------------

    @Override
    public void initialize() {
        intake1 = new MotorEx("intake1");
        dropDownIntake = new MotorEx("dropdownIntake");
        intake1.reverse();
        dropDownIntake.reverse();
        intake = new MotorGroup(intake1, dropDownIntake);

        // Beam breaks
        beamBreakFront = ActiveOpMode.hardwareMap().get(DigitalChannel.class, "beamBreakFront");
        beamBreakMid   = ActiveOpMode.hardwareMap().get(DigitalChannel.class, "beamBreakMid");
        beamBreakTop   = ActiveOpMode.hardwareMap().get(DigitalChannel.class, "beamBreakTop");


        beamBreakFront.setMode(DigitalChannel.Mode.INPUT);
        beamBreakMid.setMode(DigitalChannel.Mode.INPUT);
        beamBreakTop.setMode(DigitalChannel.Mode.INPUT);

        // Pivot servo
        pivotServo = new ServoEx("pivotServo");
    }

    @Override
    public void periodic() {}

}