package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Stopper;
import org.firstinspires.ftc.teamcode.teleop.gamepad.GamepadMapping;


public class IshaanFSM {
    // --------------- Robot & States ---------------
    public Robot robot;
    public FSMStates state = FSMStates.BASE_STATE;
    public ControlType type = ControlType.PID_CONTROL;
    private final GamepadMapping gamepad;

    // --------------- SUBSYSTEMS ---------------
    private final Intake intake;
    private final Stopper stopper;
    private final Shooter shooter;


    // --------------- MISC ---------------
    public double lastVelo = 800;
    public double lastHoodPos = 0.45; // default mid-range hood position
    private ControlType savedType;
    private double FAR_THRESHOLD = 110; // TODO tune threshold

    // furthest forward for far shots
    private double MAX_HOOD_POS = 0.1;
    // furthest down for close shots
    private double MIN_HOOD_POS = 0.6;
    private double HOOD_OFFSET = 0.1; // TODO tune offset

    public IshaanFSM(HardwareMap hardwareMap, GamepadMapping gamepad, Robot robot) {
        this.robot = robot;
        this.gamepad = robot.controls;

        intake = robot.intake;
        stopper = robot.stopper;
        shooter = robot.shooter;

        savedType = ControlType.PID_CONTROL;
    }

    public void update() {
        gamepad.update();

        switch (state) {
            case BASE_STATE:

                // Intake toggle
                if (gamepad.intake.value()) {
                    intake.intakeTransferOnClose();
                    intake.pivotDown();
                } else if (!gamepad.transfer.locked()) {
                    intake.intakeTransferOff();
                    intake.pivotUp();
                }

                // Stopper hold
                if (gamepad.transfer.locked() && type == ControlType.PID_CONTROL) {
                    intake.intakeTransferOnClose();
                    stopper.release();
                    intake.pivotDown();
                } else {
                    stopper.stop();
                }

                // Outtake hold
                if (gamepad.outtake.locked()) {
                    state = FSMStates.OUTTAKING;
                }

                // --------------- PID Only ---------------

                if (type == ControlType.PID_CONTROL) {

                    // --- Regression data points: {distanceInches, value} ---
                    // Add more points from real shooting tests to improve accuracy
                    double[][] velocityPoints = {
                            {60,  2800},
                            {90,  3050},
                            {120, 3300},
                            {150, 3550},
                            {180, 3800}
                    };

                    double[][] hoodPoints = {
                            {60,  0.30},
                            {90,  0.45},
                            {120, 0.58},
                            {150, 0.70},
                            {180, 0.80}
                    };

                    // --- Linear regression ---
                    double velocitySlope = calcSlope(velocityPoints);
                    double velocityIntercept = calcIntercept(velocityPoints, velocitySlope);

                    double hoodSlope = calcSlope(hoodPoints);
                    double hoodIntercept = calcIntercept(hoodPoints, hoodSlope);

                    // --- Get distance and compute setpoints ---
                    double distance = Robot.limelight.getDistanceInches();

                    double targetVelocity = velocitySlope * distance + velocityIntercept;
                    double targetHoodPos  = hoodSlope    * distance + hoodIntercept;

                    // --- Limelight dropout protection ---
                    if (distance != 0) {
                        lastVelo    = targetVelocity;
                        lastHoodPos = targetHoodPos;
                    }

                    if (Robot.limelight.getTargetArtifactTravelDistanceX() == 22) {
                        // Tag not visible — hold last known setpoints
                        robot.shooter.setHoodAngle(lastHoodPos);
                        robot.shooter.setShooterVelocity(lastVelo);
                    } else {
                        robot.shooter.setHoodAngle(targetHoodPos);
                        robot.shooter.setShooterVelocity(targetVelocity);
                    }
                }

                // --------------- Hardcoded Only ---------------
                if (gamepad.switchMode.value()) {
                    // if saved is PID (in PID mode), switch to Hardcoded
                    if (savedType == ControlType.PID_CONTROL) {
                        type = ControlType.HARDCODED_CONTROL;
                        savedType = ControlType.HARDCODED_CONTROL;
                        gamepad.switchMode.set(false);
                        // if saved is Hardcoded (in Hardcoded mode), switch to PID
                    } else if (savedType == ControlType.HARDCODED_CONTROL) {
                        type = ControlType.PID_CONTROL;
                        savedType = ControlType.PID_CONTROL;
                        gamepad.switchMode.set(false);
                    }
                }

                if (type == ControlType.HARDCODED_CONTROL) {
                    shooter.shootFromFront();
                    shooter.hoodToFront();
                }

                if (gamepad.shootBack.locked() && type == ControlType.HARDCODED_CONTROL) {
                    state = FSMStates.SHOOT_BACK;
                }

                if (gamepad.shootFront.locked() && type == ControlType.HARDCODED_CONTROL) {
                    state = FSMStates.SHOOT_FRONT;
                }

                break;

            case OUTTAKING:
                intake.intakeTransferReverse();

                if (!gamepad.outtake.locked()) {
                    state = FSMStates.BASE_STATE;
                    gamepad.resetMultipleControls(gamepad.transfer);
                }
                break;


            // --------------- Hardcoded Only ---------------

            case SHOOT_BACK:

                shooter.shootFromBack();
                shooter.hoodToBack();
                intake.intakeTransferOnClose();

                stopper.stop();

                if (!gamepad.shootBack.locked()) {
                    state = FSMStates.BASE_STATE;
                    gamepad.resetMultipleControls(gamepad.transfer);
                }

                break;

            case SHOOT_FRONT:

                intake.intakeTransferOnFar();
                shooter.shootFromFront();
                shooter.hoodToFront();

                stopper.stop();

                if (!gamepad.shootFront.locked()) {
                    state = FSMStates.BASE_STATE;
                    gamepad.resetMultipleControls(gamepad.shootBack, gamepad.shootFront, gamepad.transfer);
                }

                break;
        }
    }

    public void setState(FSMStates newState) {
        state = newState;
    }

    public FSMStates getState() {
        return state;
    }

    public void setControlType(ControlType newCType) {
        type = newCType;
    }

    public ControlType getControlType() {
        return type;
    }

    public enum FSMStates {
        BASE_STATE,
        SHOOT_FRONT,
        SHOOT_BACK,
        OUTTAKING
    }

    public enum ControlType {
        HARDCODED_CONTROL,
        PID_CONTROL
    }

    private double calcSlope(double[][] points) {
        int n = points.length;
        double sumX = 0, sumY = 0, sumXY = 0, sumX2 = 0;
        for (double[] p : points) {
            sumX  += p[0];
            sumY  += p[1];
            sumXY += p[0] * p[1];
            sumX2 += p[0] * p[0];
        }
        return (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);
    }

    private double calcIntercept(double[][] points, double slope) {
        int n = points.length;
        double sumX = 0, sumY = 0;
        for (double[] p : points) { sumX += p[0]; sumY += p[1]; }
        return (sumY - slope * sumX) / n;
    }
}