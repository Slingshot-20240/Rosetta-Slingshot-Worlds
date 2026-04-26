package org.firstinspires.ftc.teamcode.NextFTC.autonomous.Far;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.NextFTC.autonomous.PoseStorage;
import org.firstinspires.ftc.teamcode.NextFTC.sequences_and_groups.f;
import org.firstinspires.ftc.teamcode.NextFTC.sequences_and_groups.s;
import org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf.BaseShooternf;
import org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf.Hoodnf;
import org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf.Intakenf;
import org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf.Stoppernf;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.delays.WaitUntil;
import dev.nextftc.core.commands.groups.ParallelDeadlineGroup;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;


@Config
@Autonomous(name = "Blue Spike3 Far")
public class BlueSpike3Far extends NextFTCOpMode {
    public BlueSpike3Far() {
        addComponents(
                new SubsystemComponent(
                        f.i, s.i,
                        Intakenf.INSTANCE, Hoodnf.INSTANCE,
                        BaseShooternf.INSTANCE, Stoppernf.INSTANCE
                ),
                new PedroComponent(Constants::createFollower),
                BulkReadComponent.INSTANCE
        );
    }


    public Pose scorePose = new Pose(54, 15);
    public double scoreHeading = 114;
    public PathChain shootPreloads;
    public PathChain grabbingSomethingidklowkey;
    public PathChain shootSet2;
    public PathChain grabHpBottom;
    public PathChain shootHpBottom;
    public PathChain grabHpTop;
    public PathChain shootHpTop;

    public PathChain GETOUT;


    public void buildPaths() {
        follower().setStartingPose(new Pose(54.80101647446459, 8.777209225700156, Math.toRadians(90)));

        shootPreloads = follower().pathBuilder().addPath(
                        new BezierLine(
                                new Pose(54.801, 8.777),

                                new Pose(54.000, 15.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(114))
                .build();

        grabbingSomethingidklowkey = follower().pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(54.000, 15.000),
                                new Pose(50.344, 36.189),
                                new Pose(43.245, 36.189),
                                new Pose(12.000, 36.189)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(114), Math.toRadians(180))

                .build();

        shootSet2 = follower().pathBuilder().addPath(
                        new BezierLine(
                                new Pose(12.000, 36.189),

                                new Pose(54.000, 15.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(114))
                .build();

        grabHpBottom = follower().pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(54.000, 15.000),
                                new Pose(33.566, 8.219),
                                new Pose(11.000, 8.219)
                        )
                ).setTangentHeadingInterpolation()

                .build();

        shootHpBottom = follower().pathBuilder().addPath(
                        new BezierLine(
                                new Pose(11.000, 8.219),

                                new Pose(54.000, 15.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(114))

                .build();
        grabHpTop = follower().pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(54.000, 15.000),
                                new Pose(31.205, 24.072),
                                new Pose(11.000, 24.000)
                        )
                ).setTangentHeadingInterpolation()

                .build();

        shootHpTop = follower().pathBuilder().addPath(
                        new BezierLine(
                                new Pose(11.000, 24.000),

                                new Pose(54.000, 15.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(114))

                .build();
        GETOUT = follower().pathBuilder().addPath(
                    new BezierLine(
                            new Pose(54.000, 15.000),

                            new Pose(54.000, 33.957)
                    )
                ).setConstantHeadingInterpolation(Math.toRadians(114))
                .build();
    }




    private Command init_bot() {
        return new ParallelGroup(
                Hoodnf.INSTANCE.setHoodPos(0.37),
                Stoppernf.INSTANCE.close(),
                Intakenf.INSTANCE.pivotDown()
        );
    }

    private Command autonomous() {
        return new SequentialGroup(
                /*
                //Score Preloads
                new ParallelDeadlineGroup(
                        f.i.follow(shootPreloads, true), //if no move on, check to see if open command finishes
                        BaseShooternf.INSTANCE.setShooterVel(1030)
                ),
                s.i.shoot(0.7),


                //Middle Set

                new ParallelGroup(
                        new SequentialGroup(
                                f.i.follow(grabMiddleSet, false),
                                Intakenf.INSTANCE.off(),
                                f.i.follow(shootMiddleSet, true)

                        ),
                        new SequentialGroup(
                                new WaitUntil(() -> shootMiddleSet.lastPath().getDistanceRemaining() < 2),
                                s.i.shoot(0.6)
                        )
                ),


                //Gate Cycles
                new ParallelGroup(
                        new SequentialGroup(
                                f.i.follow(gateIntake, false),
                                new Delay(1.4),
                                Intakenf.INSTANCE.off(),
                                f.i.follow(shootGate, true)

                        ),

                        new SequentialGroup(
                                new WaitUntil(() -> shootGate.lastPath().getDistanceRemaining() < 2),
                                s.i.shoot(0.6)
                        )
                ),
                new ParallelGroup(
                        new SequentialGroup(
                                f.i.follow(gateIntake, false),
                                new Delay(1.4),
                                Intakenf.INSTANCE.off(),
                                f.i.follow(shootGate, true)

                        ),

                        new SequentialGroup(
                                new WaitUntil(() -> shootGate.lastPath().getDistanceRemaining() < 2),
                                s.i.shoot(0.6)
                        )
                ),
                new ParallelGroup(
                        new SequentialGroup(
                                f.i.follow(gateIntake, false),
                                new Delay(1.4),
                                Intakenf.INSTANCE.off(),
                                f.i.follow(shootGate, true)

                        ),

                        new SequentialGroup(
                                new WaitUntil(() -> shootGate.lastPath().getDistanceRemaining() < 2),
                                s.i.shoot(0.6)
                        )
                ),

                //4th Gate Cycle, might be possible since no gate hold, comment out not enough time
                new ParallelGroup(
                        new SequentialGroup(
                                f.i.follow(gateIntake, false),
                                new Delay(1.4),
                                Intakenf.INSTANCE.off(),
                                f.i.follow(shootGate, true)

                        ),

                        new SequentialGroup(
                                new WaitUntil(() -> shootGate.lastPath().getDistanceRemaining() < 2),
                                s.i.shoot(0.6)
                        )
                ),


                //Set2
                new ParallelGroup(
                        new SequentialGroup(
                                f.i.follow(grabSet2, false),
                                Intakenf.INSTANCE.off(),
                                f.i.follow(shootGate, true)

                        ),

                        new SequentialGroup(
                                new WaitUntil(() -> shootSet2.lastPath().getDistanceRemaining() < 2),
                                s.i.shoot(0.6)
                        )
                )
            */
        );
    }


    @Override
    public void onInit() {
        buildPaths();
        init_bot().schedule();
        BaseShooternf.INSTANCE.disable();
    }

    @Override
    public void onStartButtonPressed() {
        autonomous().schedule();
        BaseShooternf.INSTANCE.enable();
    }

    @Override
    public void onUpdate() {}

    @Override
    public void onStop() {
        PoseStorage.startingPose = follower().getPose();
    }
}

