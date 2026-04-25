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

    public MotorEx transfer;
    public MotorEx dropDownIntake;
    MotorGroup intake;

    // Beam break sensors
    public DigitalChannel beamBreakFront; // front of intake - triggers pivot up + stops all intake
    public DigitalChannel beamBreakTop;   // top - stops transfer, dropdown keeps running

    // Ball count trigger
    public boolean hasBall = false; // true when 3rd (last) ball is detected at top

    // Pivot servo
    public ServoEx pivotServo;

//-------------------------------------------------------------------------------


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

    public Command pivotUp() {
        return new SetPosition(pivotServo, 1.0);
    }

    public Command pivotDown() {
        return new SetPosition(pivotServo, 0);
    }

//-------------------------------------------------------------------------------
// Beam break sensor helpers

    public boolean isFrontBeamBreakTriggered() {
        return beamBreakFront != null && !beamBreakFront.getState();
    }

    public boolean isTopBeamBreakTriggered() {
        return beamBreakTop != null && !beamBreakTop.getState();
    }

//-------------------------------------------------------------------------------
// Ball counting

    /**
     * Returns the number of balls currently detected inside the robot.
     *
     * beamBreakFront = ball entering intake (stage 1)
     * beamBreakTop   = ball at top         (stage 2 / final)
     *
     * hasBall is set to true when count reaches 3 (top beam break triggered
     * while front beam break is also triggered, indicating robot is full).
     */
    public int getCount() {
        int count = 0;
        if (isFrontBeamBreakTriggered()) count++;
        if (isTopBeamBreakTriggered())   count++;

        // hasBall triggers true when the last (3rd) ball reaches the top
        hasBall = (count >= 2);

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
        getCount(); // keep hasBall updated every loop

        if (isFrontBeamBreakTriggered()) {
            pivotUp();
            transfer.setPower(0);
            dropDownIntake.setPower(0);
        } else if (isTopBeamBreakTriggered()) {
            dropDownIntake.setPower(-1);
            transfer.setPower(0);
        } else {
            dropDownIntake.setPower(-1);
            transfer.setPower(-1);
        }
    }

//-------------------------------------------------------------------------------

    @Override
    public void initialize() {
        transfer = new MotorEx("transfer");
        dropDownIntake = new MotorEx("dropdownIntake");
        transfer.reverse();
        dropDownIntake.reverse();
        intake = new MotorGroup(transfer, dropDownIntake);

        // Beam breaks
        beamBreakFront = ActiveOpMode.hardwareMap().get(DigitalChannel.class, "beamBreakFront");
        beamBreakTop   = ActiveOpMode.hardwareMap().get(DigitalChannel.class, "beamBreakTop");

        beamBreakFront.setMode(DigitalChannel.Mode.INPUT);
        beamBreakTop.setMode(DigitalChannel.Mode.INPUT);

        // Pivot servo
        pivotServo = new ServoEx("pivotServo");
    }

    @Override
    public void periodic() {
        getCount(); // continuously update hasBall each loop
    }

}