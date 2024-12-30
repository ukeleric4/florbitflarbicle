package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class PIDF_Test extends OpMode {
    Slides slides;
    Panning panning;

    @Override
    public void init() {
        slides = new Slides(hardwareMap);
        panning = new Panning(hardwareMap);
    }

    @Override
    public void loop() {
        if (gamepad1.right_bumper) {
            slides.runSlidesForward();
        } else if (gamepad1.left_bumper) {
            slides.runSlidesBackward();
        }

        if (gamepad1.right_trigger > 0.8) {
            panning.setTarget(500);
        } else if (gamepad1.left_trigger > 0.8) {
            panning.setTarget(0);
        }

        if (gamepad1.a) {
            slides.setPosition(1000);
        }

        if (gamepad1.b) {
            slides.setPosition(0);
        }

        panning.update();
        slides.updateSlides();
    }
}
