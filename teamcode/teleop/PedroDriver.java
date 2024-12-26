package org.firstinspires.ftc.teamcode.teleop;

import static java.lang.Thread.sleep;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.parts.Claw;
import org.firstinspires.ftc.teamcode.parts.HuskyLenses;
import org.firstinspires.ftc.teamcode.parts.MotorReal;
import org.firstinspires.ftc.teamcode.parts.Servos;

import org.firstinspires.ftc.teamcode.parts.Slides;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.*;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathBuilder;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

@TeleOp(name="PedroDriverControl", group="Linear Opmode") // @Autonomous(...) is the other common choice
public class PedroDriver extends LinearOpMode /*implements Runnable*/ {
    public MotorReal panningMotor;
    public Claw claw;
    public Servos orientation;
    public Servos panningServo;
    public HuskyLenses clawLens;
    public Slides slides;

    // gamepad old for button
    Gamepad oldGamepad1;
    Gamepad oldGamepad2;

    // random
    HuskyLens.Block block;
    double orientationPos = 0;
    double velocity;

    // auto to bucket stuff
    private Follower follower;
    private Pose currentPose;
    private PathChain toBucket, toSubmersible;
    private Pose bucketPose, subPose;
    private PathBuilder builderBucket;
    private PathChain builderSub;
    int bucketCase = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        bucketPose = new Pose(83, 47.6, 135);
        clawLens = new HuskyLenses(hardwareMap, "frontlens", "color");
        slides = new Slides(hardwareMap, telemetry);
        panningMotor = new MotorReal(hardwareMap, "panningmotor");
        claw = new Claw(hardwareMap);
        orientation = new Servos(hardwareMap, "orientation");
        panningServo = new Servos(hardwareMap, "panning");
        follower = new Follower(hardwareMap);

        orientation.moveToMin();
        follower.startTeleopDrive();
        follower.setStartingPose(new Pose(133.809, 86.622,0));

        while (opModeIsActive()) {
            updateVelocity();
            cvOrientation();
            cvSlide();
            manualPanServo();
            manualOrientation();
            manualSlide();
            slidePosition();
            manualPanning();
            bucketCaseChange();
            updateFollower();

            runToBucket(bucketCase);

            oldGamepad1 = gamepad1;
            oldGamepad2 = gamepad2;
        }
    }
    public void runToBucket(int num) {
        switch (num) {
            case 1:
                follower.breakFollowing();
                follower.setMaxPower(0.65);

                toBucket = builderBucket
                        .addPath(new BezierCurve(new Point(follower.getPose()), new Point(bucketPose)))
                        .setLinearHeadingInterpolation(follower.getPose().getHeading(), bucketPose.getHeading())
                        .build();

                follower.followPath(toBucket, true);
                follower.update();
                movePanningUp(500);
                slides.runSlidesForward(1600);
                bucketCase = 2;
                break;
            case 2:
                if (!follower.isBusy()) {
                    slides.runSlidesForward(200);
                    bucketDrop();
                    slides.runSlidesBackward(1000);
                }
                break;
        }
    }

    // Teleop functions
    public void updateVelocity() {
        if (gamepad1.right_trigger > 0.8) {
            velocity = 1;
        } else if (gamepad1.left_trigger > 0.8) {
            velocity = 0.25;
        } else {
            velocity = 0.6;
        }
    }

    public void cvOrientation() {
        if (gamepad1.a) {
            block = clawLens.getFirstObject();
            if (block != null) {
                moveToPos(clawLens.calculateServoPosition(block.height, block.width));
            }
        }
    }

    public void cvSlide() {
        if (gamepad1.x) {
            panningServo.moveSpecificPos(0.7);
            block = clawLens.getFirstObject();
            while (block != null && block.width < 20 && block.height < 20 ) {
                block = clawLens.getFirstObject();
                slides.runSlidesForward(0);
            }
            moveToPos(clawLens.calculateServoPosition(block.height, block.width));
        }
    }

    public void manualPanServo() {
        if (gamepad2.y) {
            panningServo.moveSpecificPos(0.35);
        } else if (gamepad2.a) {
            panningServo.moveToMax();
        }
    }

    public void manualOrientation() {
        if (gamepad2.x) {
            orientation.moveToMin();
        } else if (gamepad2.b) {
            orientation.moveSpecificPos(0.4);
        }
    }

    public void manualSlide() {
        if (gamepad1.right_bumper) {
            slides.runSlidesForward(0);
        } else if (gamepad1.left_bumper) {
            slides.runSlidesBackward(0);
        } else {
            slides.stopSlides();
        }
    }

    public void slidePosition() {
        if (gamepad1.y) {
            slides.runSlidesForward(1000);
        }
    }

    public void manualPanning() {
        if (gamepad2.right_trigger > 0.8) {
           movePanningUp(0);
        } else if (gamepad2.left_trigger > 0.8) {
            movePanningDown(0);
        } else {
            panningMotor.stopRotation();
        }
    }

    public void clawMovement() {
        if (gamepad2.dpad_left) {
            claw.closeClaw();
        } else if (gamepad2.dpad_right) {
            claw.openClaw();
        }
    }

    public void autoGrab() {
        if (gamepad2.dpad_up) {
            panningServo.moveToMax();
            claw.closeClaw();
        }
    }

    public void bucketCaseChange() {
        if (gamepad1.b) {
            if (bucketCase != 1) {
                bucketCase = -1;
                sleep(200);
            } else {
                bucketCase = 1;
                sleep(200);
            }
        }
    }

    public void updateFollower() {
        follower.setTeleOpMovementVectors(-gamepad1.left_stick_y * velocity , -gamepad1.left_stick_x * velocity, -gamepad1.right_stick_x * velocity);
        follower.update();
        currentPose = follower.getPose();
    }

    // Other functions
    public void movePanningDown(int time) {
        panningMotor.rotateBackward(-1.0, time);
    }

    public void movePanningUp(int time) {
        panningMotor.rotateForward(1.0, time);
    }

    public void moveToPos(double pos) {
        orientationPos = pos;
        orientation.moveSpecificPos(pos);
    }

    public void bucketDrop() {
        panningServo.moveSpecificPos(0.35);
        sleep(200);
        claw.openClaw();
        sleep(200);
        panningServo.moveSpecificPos(0.7);
        sleep(200);
    }
}
