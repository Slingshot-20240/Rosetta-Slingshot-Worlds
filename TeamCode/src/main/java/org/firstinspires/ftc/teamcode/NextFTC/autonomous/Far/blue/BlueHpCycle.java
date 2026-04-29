package org.firstinspires.ftc.teamcode.NextFTC.autonomous.Far.blue;

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
@Autonomous(name = "Blue Hp Cycle")
public class BlueHpCycle extends NextFTCOpMode {
    public BlueHpCycle() {
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
    public PathChain grabHpBottom;
    public PathChain shootHpBottom;
    public PathChain grabHpTop;
    public PathChain shootHpTop;
    public PathChain GETOUT;


    public void buildPaths() {
        follower().setStartingPose(new Pose(54.8, 8.8, Math.toRadians(90)));

        shootPreloads = follower().pathBuilder().addPath(
                        new BezierLine(
                                new Pose(54.801, 8.777),

                                scorePose
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(scoreHeading))
                .build();

        grabHpBottom = follower().pathBuilder().addPath(
                        new BezierCurve(
                                scorePose,
                                new Pose(33.566, 8.219),
                                new Pose(11, 8.219)
                        )
                ).setTangentHeadingInterpolation()

                .build();

        shootHpBottom = follower().pathBuilder().addPath(
                        new BezierLine(
                                new Pose(11, 8.219),

                                scorePose
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(scoreHeading))

                .build();

        grabHpTop = follower().pathBuilder().addPath(
                        new BezierCurve(
                                scorePose,
                                new Pose(31.205, 24.072),
                                new Pose(11, 24)
                        )
                ).setTangentHeadingInterpolation()

                .build();

        shootHpTop = follower().pathBuilder().addPath(
                        new BezierLine(
                                new Pose(11, 24),

                                scorePose
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(scoreHeading))

                .build();

        GETOUT = follower().pathBuilder().addPath(
                    new BezierLine(
                            scorePose,

                            new Pose(54, 33.957)
                    )
                ).setConstantHeadingInterpolation(Math.toRadians(scoreHeading))
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
        return new ParallelGroup(
                BaseShooternf.INSTANCE.setShooterVel(1030),
                new SequentialGroup(

                        //Score Preloads
                        new ParallelDeadlineGroup(
                                f.i.follow(shootPreloads), //if no move on, check to see if open command finishes
                                Stoppernf.INSTANCE.open()
                        ),
                        s.i.shoot(1),


                        //Middle Set
                        new ParallelGroup(
                                new SequentialGroup(
                                        new ParallelGroup(
                                                f.i.follow(grabHpBottom),
                                                Stoppernf.INSTANCE.close()
                                        ),
                                        new Delay(0.2),
                                        Intakenf.INSTANCE.off(),
                                        new ParallelGroup(
                                                f.i.follow(shootHpBottom),
                                                Stoppernf.INSTANCE.open()
                                        )
                                ),
                                new SequentialGroup(
                                        new WaitUntil(() -> shootHpBottom.lastPath().getDistanceRemaining() < 2),
                                        s.i.shoot(0.6)
                                )
                        ),


                        //Gate Cycles
                        new ParallelGroup(
                                new SequentialGroup(
                                        new ParallelGroup(
                                                f.i.follow(grabHpTop),
                                                Stoppernf.INSTANCE.close()
                                        ),
                                        new Delay(0.2),
                                        Intakenf.INSTANCE.off(),
                                        new ParallelGroup(
                                                f.i.follow(shootHpTop),
                                                Stoppernf.INSTANCE.open()

                                        )
                                ),

                                new SequentialGroup(
                                        new WaitUntil(() -> shootHpTop.lastPath().getDistanceRemaining() < 2),
                                        s.i.shoot(0.6)
                                )
                        ),
                        new ParallelGroup(
                                new SequentialGroup(
                                        new ParallelGroup(
                                                f.i.follow(grabHpBottom),
                                                Stoppernf.INSTANCE.close()
                                        ),
                                        new Delay(0.2),
                                        Intakenf.INSTANCE.off(),
                                        new ParallelGroup(
                                                f.i.follow(shootHpBottom),
                                                Stoppernf.INSTANCE.open()

                                        )
                                ),

                                new SequentialGroup(
                                        new WaitUntil(() -> shootHpBottom.lastPath().getDistanceRemaining() < 2),
                                        s.i.shoot(0.6)
                                )
                        ),
                        new ParallelGroup(
                                new SequentialGroup(
                                        new ParallelGroup(
                                                f.i.follow(grabHpTop),
                                                Stoppernf.INSTANCE.close()
                                        ),
                                        new Delay(0.2),
                                        Intakenf.INSTANCE.off(),
                                        new ParallelGroup(
                                                f.i.follow(shootHpTop),
                                                Stoppernf.INSTANCE.open()

                                        )
                                ),

                                new SequentialGroup(
                                        new WaitUntil(() -> shootHpTop.lastPath().getDistanceRemaining() < 2),
                                        s.i.shoot(0.6)
                                )
                        ),
                        new ParallelGroup(
                                new SequentialGroup(
                                        new ParallelGroup(
                                                f.i.follow(grabHpBottom),
                                                Stoppernf.INSTANCE.close()
                                        ),
                                        new Delay(0.2),
                                        Intakenf.INSTANCE.off(),
                                        new ParallelGroup(
                                                f.i.follow(shootHpBottom),
                                                Stoppernf.INSTANCE.open()

                                        )
                                ),

                                new SequentialGroup(
                                        new WaitUntil(() -> shootHpBottom.lastPath().getDistanceRemaining() < 2),
                                        s.i.shoot(0.6)
                                )
                        ),
                        new ParallelGroup(
                                new SequentialGroup(
                                        new ParallelGroup(
                                                f.i.follow(grabHpTop),
                                                Stoppernf.INSTANCE.close()
                                        ),
                                        new Delay(0.2),
                                        Intakenf.INSTANCE.off(),
                                        new ParallelGroup(
                                                f.i.follow(shootHpTop),
                                                Stoppernf.INSTANCE.open()

                                        )
                                ),

                                new SequentialGroup(
                                        new WaitUntil(() -> shootHpTop.lastPath().getDistanceRemaining() < 2),
                                        s.i.shoot(0.6)
                                )
                        )


                )
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

