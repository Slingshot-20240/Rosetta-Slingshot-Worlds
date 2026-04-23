package org.firstinspires.ftc.teamcode.NextFTC.autonomous.alliance;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;

import com.pedropathing.paths.HeadingInterpolator;
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

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.delays.WaitUntil;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;


@Config
@Autonomous(name = "Blue No HP 15")
public class BlueNoHp extends NextFTCOpMode {
    public BlueNoHp() {
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
    public PathChain gateSet2;
    public PathChain scoreSet2;
    public PathChain grabSet3;
    public PathChain gateSet3;
    public PathChain scoreSet3;
    public PathChain grabSet4;
    public PathChain scoreSet4;


    public Pose scorePose = new Pose(92,88).mirror();
    public double scoreHeading = 180-45;

    public void buildPaths() {
        follower().setStartingPose(new Pose(126.2, 119, Math.toRadians(36)).mirror());

        scorePreloads = follower()
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(126.2, 119).mirror(), scorePose)
                )
                .setLinearHeadingInterpolation(Math.toRadians(180-36), Math.toRadians(180-43))
//                .setTangentHeadingInterpolation().setReversed()
                .build();



        grabSet2 = PedroComponent.follower()
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                scorePose,
                                new Pose(92.292, 77).mirror(),
                                new Pose(126.5, 82.9).mirror()
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180-0))
                .build();

        gateSet2 = PedroComponent.follower().pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(126.5, 82.9).mirror(),
                                new Pose(110.481, 71.074).mirror(),
                                new Pose(126.5, 74).mirror()
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180-0), Math.toRadians(180-0))

                .build();

        scoreSet2 = PedroComponent.follower().pathBuilder().addPath(
                        new BezierLine(
                                new Pose(126.5, 74).mirror(),
                                scorePose
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180-0), Math.toRadians(scoreHeading))

                .build();

        grabSet3 = PedroComponent.follower().pathBuilder().addPath(
                        new BezierCurve(
                                scorePose,
                                new Pose(88.941, 53.686).mirror(),
                                new Pose(96.582, 59.502).mirror(),
                                new Pose(131, 58).mirror()
                        )
                ).setHeadingInterpolation(
                        HeadingInterpolator.piecewise(
                                new HeadingInterpolator.PiecewiseNode(
                                        0,
                                        0.55,
                                        HeadingInterpolator.linear(
                                                Math.toRadians(scoreHeading),
                                                Math.toRadians(180-0)
                                        )
                                ),
                                new HeadingInterpolator.PiecewiseNode(
                                        0.55,
                                        1.0,
                                        HeadingInterpolator.constant(Math.toRadians(180-0))
                                )
                        )
                )

                .build();

        gateSet3 = PedroComponent.follower().pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(131, 58).mirror(),
                                new Pose(105.151, 58.012).mirror(),
                                new Pose(126, 74).mirror()
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180-0), Math.toRadians(180-0))

                .build();

        scoreSet3 = PedroComponent.follower().pathBuilder().addPath(
                        new BezierLine(
                                new Pose(126, 74).mirror(),
                                scorePose
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180-0), Math.toRadians(scoreHeading))
                .build();

        grabSet4 = PedroComponent.follower().pathBuilder().addPath(
                        new BezierCurve(
                                scorePose,
                                new Pose(85, 28).mirror(),
                                new Pose(90, 37).mirror(),
                                new Pose(131, 35.5).mirror()
                        )
                ).setHeadingInterpolation(
                        HeadingInterpolator.piecewise(
                                new HeadingInterpolator.PiecewiseNode(
                                        0,
                                        0.1,
                                        HeadingInterpolator.linear(
                                                Math.toRadians(scoreHeading),
                                                Math.toRadians(180-0)
                                        )
//                                        HeadingInterpolator.tangent
                                ),
                                new HeadingInterpolator.PiecewiseNode(
                                        0.1,
                                        1.0,
                                        HeadingInterpolator.constant(Math.toRadians(180-0))
                                )
                        )
                )

                .build();

        scoreSet4 = PedroComponent.follower().pathBuilder().addPath(
                        new BezierLine(
                                new Pose(131, 35.5).mirror(),
                                scorePose
                        )
                )
                .setHeadingInterpolation(
                        HeadingInterpolator.piecewise(
                                new HeadingInterpolator.PiecewiseNode(
                                        0,
                                        0.06,
                                        HeadingInterpolator.constant(Math.toRadians(180-0))
                                ),
                                new HeadingInterpolator.PiecewiseNode(
                                        0.06,
                                        0.6,
                                        HeadingInterpolator.tangent.reverse()
                                ),
                                new HeadingInterpolator.PiecewiseNode(
                                        0.6,
                                        1.0,
                                        HeadingInterpolator.constant(Math.toRadians(scoreHeading))
                                )
                        )
                )
                .build();



    }

    private Command init_bot() {
        return new SequentialGroup(
                Hoodnf.INSTANCE.setHoodPos(0.37),
                Stoppernf.INSTANCE.close()
        );

    }

    private Command autonomous() {
        return new SequentialGroup(
                new ParallelGroup(
                        new FollowPath(scorePreloads),

                        s.i.shooterState(1110),
                        Intakenf.INSTANCE.in(),

                        new SequentialGroup(
                                new WaitUntil(() -> scorePreloads.lastPath().getDistanceRemaining() < 2),
                                new Delay(0.6),
                                s.i.shoot(1)
                        )
                ),
                new FollowPath(grabSet2),
                new FollowPath(gateSet2),
                new ParallelGroup(
                        new FollowPath(scoreSet2),

                        new SequentialGroup(
                                new WaitUntil(() -> scoreSet2.lastPath().getDistanceRemaining() < 0.2),
                                new Delay(0.4),
                                s.i.shoot(0.6)
                        )
                ),


                new FollowPath(grabSet3),
                new FollowPath(gateSet3),
                new Delay(0.4),
                new ParallelGroup(
                        new FollowPath(scoreSet3),

                        new SequentialGroup(
                                new WaitUntil(() -> scoreSet3.lastPath().getDistanceRemaining() < 0.2),
                                new Delay(0.2),
                                s.i.shoot(0.6)
                        )
                ),
//                    s.i.shoot(1),

                new FollowPath(grabSet4),
                new ParallelGroup(
                        new FollowPath(scoreSet4),

                        new SequentialGroup(
                                new WaitUntil(() -> scoreSet4.lastPath().getDistanceRemaining() < 0.2),
                                new Delay(0.2),
                                s.i.shoot(0.6)
                        )
                ),

                //Set 4 again
                new FollowPath(grabSet4),
                new ParallelGroup(
                        new FollowPath(scoreSet4),

                        new SequentialGroup(
                                new WaitUntil(() -> scoreSet4.lastPath().getDistanceRemaining() < 0.2),
                                new Delay(0.2),
                                s.i.shoot(0.6)
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
    public void onStop() {
        PoseStorage.startingPose = follower().getPose();
    }
}

