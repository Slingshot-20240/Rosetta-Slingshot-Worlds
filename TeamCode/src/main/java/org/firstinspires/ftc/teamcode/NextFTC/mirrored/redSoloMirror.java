package org.firstinspires.ftc.teamcode.NextFTC.mirrored;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;

import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.NextFTC.autonomous.PoseStorage;
import org.firstinspires.ftc.teamcode.NextFTC.sequences_and_groups.s;
import org.firstinspires.ftc.teamcode.NextFTC.sequences_and_groups.f;
import org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf.Hoodnf;
import org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf.Intakenf;
//import org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf.Lednf;
import org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf.BaseShooternf;
import org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf.Stoppernf;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.opencv.core.Mat;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.delays.WaitUntil;
import dev.nextftc.core.commands.groups.ParallelDeadlineGroup;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;


@Config
@Autonomous(name = "redSoloMirror")
public class redSoloMirror extends NextFTCOpMode {
    public redSoloMirror() {
        addComponents(
                new SubsystemComponent(
                        f.i, s.i,
                        Intakenf.INSTANCE, Hoodnf.INSTANCE,
                        BaseShooternf.INSTANCE, Stoppernf.INSTANCE
//                        Lednf.INSTANCE
                ),
                new PedroComponent(Constants::createFollower),
                BulkReadComponent.INSTANCE
        );
    }

    public PathChain scorePreloads;
    public PathChain grabSet2;
    public PathChain scoreSet2;
    public PathChain grabSet3;
    public PathChain gateSet3;
    public PathChain scoreSet3;
    public PathChain grabSet4;
    public PathChain scoreSet4;
    public PathChain grabHp;
    public PathChain scoreHp;
    public PathChain park;

    public Pose scorePose = new Pose(90,88);
    public double scoreHeading = 40;

    public void buildPaths() {
        follower().setStartingPose(new Pose(126.2, 119, Math.toRadians(36)));

        scorePreloads = follower()
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(126.2, 119), scorePose)
                )
                .setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(44))
//                .setTangentHeadingInterpolation().setReversed()
                .build();



        grabSet2 = PedroComponent.follower()
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                scorePose,
                                new Pose(92.292, 77),
                                new Pose(125.9, 82.9)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        scoreSet2 = PedroComponent.follower().pathBuilder().addPath(
                        new BezierLine(
                                new Pose(125.9, 82.9),
                                scorePose
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(scoreHeading))

                .build();

        grabSet3 = PedroComponent.follower().pathBuilder().addPath(
                        new BezierCurve(
                                scorePose,
                                new Pose(88.941, 53.686),
                                new Pose(96.582, 59.502),
                                new Pose(130, 58)
                        )
                ).setHeadingInterpolation(
                        HeadingInterpolator.piecewise(
                                new HeadingInterpolator.PiecewiseNode(
                                        0,
                                        0.55,
                                        HeadingInterpolator.linear(
                                                Math.toRadians(scoreHeading),
                                                Math.toRadians(0)
                                        )
                                ),
                                new HeadingInterpolator.PiecewiseNode(
                                        0.55,
                                        1.0,
                                        HeadingInterpolator.constant(Math.toRadians(0))
                                )
                        )
                )

                .build();

        gateSet3 = PedroComponent.follower().pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(130, 58),
                                new Pose(105.151, 58.012),
                                new Pose(127.2, 74)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                .build();

        scoreSet3 = PedroComponent.follower().pathBuilder().addPath(
                        new BezierLine(
                                new Pose(127.2, 74),
                                scorePose
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(scoreHeading))
                .build();

        grabSet4 = PedroComponent.follower().pathBuilder().addPath(
                        new BezierCurve(
                                scorePose,
                                new Pose(85, 29),
                                new Pose(94, 36),
                                new Pose(130, 34.2)
                        )
                ).setHeadingInterpolation(
                        HeadingInterpolator.piecewise(
                                new HeadingInterpolator.PiecewiseNode(
                                        0,
                                        0.6,
//                                        HeadingInterpolator.linear(
//                                                Math.toRadians(scoreHeading),
//                                                Math.toRadians(0)
//                                        )
                                        HeadingInterpolator.tangent
                                ),
                                new HeadingInterpolator.PiecewiseNode(
                                        0.6,
                                        1.0,
                                        HeadingInterpolator.constant(Math.toRadians(0))
                                )
                        )
                )

                .build();

        scoreSet4 = PedroComponent.follower().pathBuilder().addPath(
                        new BezierLine(
                                new Pose(130, 34.2),
                                scorePose
                        )
                )
                .setHeadingInterpolation(
                        HeadingInterpolator.piecewise(
                                new HeadingInterpolator.PiecewiseNode(
                                        0,
                                        0.06,
                                        HeadingInterpolator.constant(Math.toRadians(0))
                                ),
                                new HeadingInterpolator.PiecewiseNode(
                                        0.06,
                                        0.6,
                                        HeadingInterpolator.tangent.reverse()
                                ),
                                new HeadingInterpolator.PiecewiseNode(
                                        0.6,
                                        1.0,
                                        HeadingInterpolator.constant(Math.toRadians(scoreHeading+1))
                                )
                        )
                )
                .build();

        grabHp = PedroComponent.follower().pathBuilder().addPath(
                        new BezierCurve(
                                scorePose,
                                new Pose(129, 58.000),
                                new Pose(132, 12.000)
                        )
                ).setTangentHeadingInterpolation()

                .build();

        scoreHp = PedroComponent.follower().pathBuilder().addPath(
                        new BezierLine(
                                new Pose(132, 12.000),
                                new Pose(88, 110)
                        )
                ).setHeadingInterpolation(
                        HeadingInterpolator.piecewise(
                                new HeadingInterpolator.PiecewiseNode(
                                        0,
                                        0.9,
                                        HeadingInterpolator.tangent.reverse()
                                ),
                                new HeadingInterpolator.PiecewiseNode(
                                        0.9,
                                        1.0,
                                        HeadingInterpolator.constant(Math.toRadians(30))
                                )
                        )
                )
                .build();

        park = follower()
                .pathBuilder()

                .addPath(
                        new BezierLine(new Pose(88,110), new Pose(100,68))
                )
                .setLinearHeadingInterpolation(Math.toRadians(29), Math.toRadians(90))

                .build();



    }

    private Command init_bot() {
        return new ParallelGroup(
                Hoodnf.INSTANCE.setHoodPos(0.345),
                Stoppernf.INSTANCE.close()
        );

    }

    private Command autonomous() {
        return new SequentialGroup(

                new ParallelDeadlineGroup(
                        new FollowPath(scorePreloads),
                        BaseShooternf.INSTANCE.setShooterVel(1100),

                        Intakenf.INSTANCE.in()
                ),
                new Delay(0.5),
                s.i.shoot(0.7),

                new FollowPath(grabSet2),
                new Delay(0.1),

                new ParallelGroup(
                        new FollowPath(scoreSet2),
                        Hoodnf.INSTANCE.setHoodPos(0.352),

                        new SequentialGroup(
                                new WaitUntil(() -> scoreSet2.lastPath().getDistanceRemaining() < 0.2),
                                new Delay(0.3),
                                s.i.shoot(0.5)
                        )
                ),


                new FollowPath(grabSet3),
                new Delay(0.1),
                new FollowPath(gateSet3),
                new Delay(0.5),

                new ParallelGroup(
                        new FollowPath(scoreSet3),

                        new SequentialGroup(
                                new WaitUntil(() -> scoreSet3.lastPath().getDistanceRemaining() < 0.2),
                                new Delay(0.3),
                                s.i.shoot(0.5)
                        )
                ),
                s.i.shoot(1),

                new FollowPath(grabSet4),
                new Delay(0.1),
                new ParallelGroup(
                        new FollowPath(scoreSet4),

                        new SequentialGroup(
                                new WaitUntil(() -> scoreSet4.lastPath().getDistanceRemaining() < 0.2),
                                new Delay(0.3),
                                s.i.shoot(0.5)
                        )
                ),
                new Delay(0.5),

                new ParallelGroup(
                        new FollowPath(grabHp),
                        s.i.shooterState(960,0.5)

                ),
                new Delay(0.1),
                new FollowPath(scoreHp),
                new Delay(0.4),
                Stoppernf.INSTANCE.open()
//                s.i.shoot(1)
//                new FollowPath(park)



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
    public void onStop() {
        PoseStorage.startingPose = follower().getPose();
    }
}