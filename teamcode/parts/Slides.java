package org.firstinspires.ftc.teamcode.parts;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Slides {
    public SlideReal slide1;
    public SlideReal slide2;
    public SlideAuto autoSlide1;
    public SlideAuto autoSlide2;
    public Slides(HardwareMap hw, Telemetry telemetry) {
        slide1 = new SlideReal(hw, "slide1");
        slide2 = new SlideReal(hw, "slide2");
        autoSlide1 = new SlideAuto(hw, telemetry, "slide1");
        autoSlide2 = new SlideAuto(hw, telemetry, "slide2");
    }

    public void runSlidesAsync(int level) {
        autoSlide1.MoveToLevelAsync(level);
        autoSlide2.MoveToLevelAsync(level);
    }

    public void runSlidesToPos(int level) {
        autoSlide1.MoveToLevelAsync(level);
        autoSlide2.MoveToLevelAsync(level);
    }

    public void runSlidesForward(int time) {
        slide1.runForward(1.0, time);
        slide2.runForward(1.0, time);
    }

    public void runSlidesBackward(int time) {
        slide1.runBackward(1.0, time);
        slide2.runBackward(1.0, time);
    }

    public void stopSlides() {
        slide1.stopSlide();
        slide2.stopSlide();
    }
}
