package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class Slides {
    Slide1 slide1;
    Slide2 slide2;

    public Slides(HardwareMap hw) {
        slide1 = new Slide1(hw);
        slide2 = new Slide2(hw);
    }

    public void setPosition(int target) {
        slide1.setTarget(target);
        slide2.setTarget(target);
    }

    public void updateSlides() {
        slide1.update();
        slide2.update();
    }

    public void runSlidesForward() {
        if (slide1.getTarget() <= 1800) {
            slide1.setTarget(slide1.getTarget() + 5);
            slide2.setTarget(slide2.getTarget() + 5);
        }
    }

    public void runSlidesBackward() {
        if (slide1.getTarget() >= 5) {
            slide1.setTarget(slide1.getTarget() - 5);
            slide2.setTarget(slide2.getTarget() - 5);
        }
    }

    public int getTarget() {
        return slide1.getTarget();
    }
}
