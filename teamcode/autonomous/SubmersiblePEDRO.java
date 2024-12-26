package org.firstinspires.ftc.teamcode.autonomous;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.*;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathBuilder;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Config
@Autonomous(group = "drive")
public class SubmersiblePEDRO extends LinearOpMode {
    public static double DISTANCE = 24; // in

    public Slide slide;
    public Motors panningMotor;
    public Servos claw;
    public Servos panningServo;
    public Servos orientation;
    public Motors pulley;

    private Follower follower;
    private PathChain submersible0;
    private PathChain submersible1;
    private PathChain submersible2;
    private PathChain submersible3;
    private PathChain submersible4;
    private PathChain submersible5;
    private PathChain submersible6;

    private Pose currentPose;
    int num1 = 0;
    double time;

    Thread bigBootyThread;

    public int runFrames;

    public HuskyLens.Block currentTarget;

    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        slide = new Slide(hardwareMap, telemetry, "slide");
        panningMotor = new Motors(hardwareMap, "panningmotor");
        claw = new Servos(hardwareMap, "claw");
        panningServo = new Servos(hardwareMap, "panning");
        orientation = new Servos(hardwareMap, "orientation");
        pulley = new Motors(hardwareMap, "pulley");
//        frontLens = new HuskyLenses(hardwareMap, "frontlens", "color");
        follower = new Follower(hardwareMap);
        follower.setStartingPose(new Pose(134.91692307692307, 79.97538461538461,0));

        runFrames = 0;
        claw.moveForwardMAX();
        panningServo.moveBackwardMIN();
        orientation.moveBackwardMIN();
        slide.MoveToLevel(Slide.level.zero);

        PathBuilder builder0 = new PathBuilder();
        PathBuilder builder1 = new PathBuilder();
        PathBuilder builder2 = new PathBuilder();
        PathBuilder builder3 = new PathBuilder();
        PathBuilder builder4 = new PathBuilder();
        PathBuilder builder5 = new PathBuilder();
        PathBuilder builder6 = new PathBuilder();

        builder0
             .addPath(
                // Line 1
                new BezierCurve(
                        new Point(135.138, 80.640, Point.CARTESIAN),
                        new Point(120.517, 78.425, Point.CARTESIAN),
                        new Point(105.452, 80.418, Point.CARTESIAN)
                )
        )
                .setConstantHeadingInterpolation(Math.toRadians(0));
        builder1
                .addPath(
                        // Line 2
                        new BezierCurve(
                                new Point(105.452, 80.418, Point.CARTESIAN),
                                new Point(130.929, 100.800, Point.CARTESIAN),
                                new Point(104.788, 107.668, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 3
                        new BezierCurve(
                                new Point(104.788, 107.668, Point.CARTESIAN),
                                new Point(80.862, 105.452, Point.CARTESIAN),
                                new Point(83.298, 123.175, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 4
                        new BezierCurve(
                                new Point(83.298, 123.175, Point.CARTESIAN),
                                new Point(106.117, 119.852, Point.CARTESIAN),
                                new Point(122.289, 123.175, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 5
                        new BezierCurve(
                                new Point(122.289, 123.175, Point.CARTESIAN),
                                new Point(101.465, 106.338, Point.CARTESIAN),
                                new Point(83.520, 132.258, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 6
                        new BezierCurve(
                                new Point(83.520, 132.258, Point.CARTESIAN),
                                new Point(103.237, 134.252, Point.CARTESIAN),
                                new Point(131.151, 133.145, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0));

                builder2
                        .addPath(
                                // Line 7
                                new BezierCurve(
                                        new Point(131.151, 133.145, Point.CARTESIAN),
                                        new Point(131.151, 79.754, Point.CARTESIAN),
                                        new Point(105.452, 75.545, Point.CARTESIAN)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(0));
                builder3
                        .addPath(
                                // Line 8
                                new BezierCurve(
                                        new Point(105.452, 75.545, Point.CARTESIAN),
                                        new Point(142.228, 79.089, Point.CARTESIAN),
                                        new Point(87.286, 101.908, Point.CARTESIAN),
                                        new Point(130.708, 118.745, Point.CARTESIAN)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(0));

                builder4
                        .addPath(
                                // Line 9
                                new BezierCurve(
                                        new Point(130.708, 118.745, Point.CARTESIAN),
                                        new Point(128.492, 65.132, Point.CARTESIAN),
                                        new Point(105.231, 70.006, Point.CARTESIAN)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(0));

                builder5
                        .addPath(
                                // Line 10
                                new BezierCurve(
                                        new Point(105.231, 70.006, Point.CARTESIAN),
                                        new Point(142.228, 79.089, Point.CARTESIAN),
                                        new Point(87.951, 101.686, Point.CARTESIAN),
                                        new Point(130.708, 118.745, Point.CARTESIAN)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(0));
                builder6
                        .addPath(
                                // Line 11
                                new BezierCurve(
                                        new Point(130.708, 118.745, Point.CARTESIAN),
                                        new Point(133.809, 86.622, Point.CARTESIAN),
                                        new Point(104.123, 72.443, Point.CARTESIAN)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(0));

        submersible0 = builder0.build();
        submersible1 = builder1.build();
        submersible2 = builder2.build();
        submersible3 = builder3.build();
        submersible4 = builder4.build();
        submersible5 = builder5.build();
        submersible6 = builder6.build();

        waitForStart();

        if (isStopRequested()) return;

        while (getRuntime() < 100) {
            follower.update();
            currentPose = follower.getPose();
            time = getRuntime();
            runAuto(num1, time);

            telemetry.addData("X:", currentPose.getX());
            telemetry.addData("Y:", currentPose.getY());
        }
    }

    public void runAuto(int num, double time1) {
        switch (num) {
            case 0:
                follower.followPath(submersible0);
                num1 = 1;
                break;
            case 1:
                if (!follower.isBusy()) {
                    panningServo.moveSpecificPos(.35);
                    panningMotor.rotateForward(1, 300);
                    follower.followPath(submersible1);
                    follower.update();
                    claw.moveBackwardMIN();
                    sleep(300);
                    follower.update();
                    panningMotor.rotateForward(-0.8, 500);
                    follower.update();
                    panningServo.moveForwardMAX();
                    claw.moveSpecificPos(0.3);
                    follower.update();
                    panningServo.moveSpecificPos(0.4);
                    follower.update();
                    num1 = 2;
                }
                break;
            case 2:
                if (!follower.isBusy() && currentPose.getY() > 120) {
                    claw.moveForwardMAX();
                    sleep(350);
                    follower.followPath(submersible2);
                    follower.update();
                    panningServo.moveBackwardMIN();
                    orientation.moveForwardMAX();
                    num1 = 3;
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    panningServo.moveSpecificPos(.3);
                    panningMotor.rotateForward(1, 400);
                    follower.followPath(submersible3);
                    follower.update();
                    claw.moveBackwardMIN();
                    sleep(300);
                    orientation.moveBackwardMIN();
                    follower.update();
                    panningMotor.rotateForward(-0.8, 500);
                    panningServo.moveForwardMAX();
                    claw.moveSpecificPos(0.3);
                    panningServo.moveSpecificPos(0.4);
                    follower.update();

                    num1 = 4;
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    claw.moveForwardMAX();
                    sleep(350);
                    follower.followPath(submersible4);
                    follower.update();
                    panningServo.moveBackwardMIN();
                    orientation.moveForwardMAX();
                    num1 = 5;
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    panningServo.moveSpecificPos(.3);
                    panningMotor.rotateForward(1, 400);
                    follower.followPath(submersible5);
                    follower.update();
                    claw.moveBackwardMIN();
                    sleep(300);
                    follower.update();
                    orientation.moveBackwardMIN();
                    panningMotor.rotateForward(-0.8, 500);
                    panningServo.moveForwardMAX();
                    claw.moveSpecificPos(0.3);
                    panningServo.moveSpecificPos(0.4);
                    follower.update();

                    num1 = 6;
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    claw.moveForwardMAX();
                    sleep(350);
                    follower.followPath(submersible6);
                    follower.update();
                    panningServo.moveBackwardMIN();
                    orientation.moveForwardMAX();
                    num1 = 7;
                }
                break;
            case 7:
                if (!follower.isBusy()) {
                    panningServo.moveSpecificPos(.3);
                    panningMotor.rotateForward(1, 400);
                    panningServo.moveSpecificPos(0);
                    sleep(500);
                    claw.moveBackwardMIN();
                    sleep(300);

                    num1 = -1;
                }
                break;
        }
    }

}