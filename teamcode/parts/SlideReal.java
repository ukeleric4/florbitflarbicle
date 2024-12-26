package org.firstinspires.ftc.teamcode.parts;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class SlideReal {
    public MotorReal slideMotor;

    public SlideReal(HardwareMap hw, String name) {
        slideMotor.resetEncoder();
        slideMotor = new MotorReal(hw, "slide");
    }

    public void runForward(double power, int timeMs) {
        slideMotor.setModeNoEncoder();
        slideMotor.setDirectionForward();
        slideMotor.motor.setPower(power);
        if (timeMs > 0 ) {
            try {
                sleep(timeMs);
            } catch (InterruptedException e) {
            }
            slideMotor.stopRotation();
        }
    }

    public void runBackward(double power, int timeMs) {
        slideMotor.setModeNoEncoder();
        slideMotor.setDirectionReverse();
        slideMotor.motor.setPower(power);
        if (timeMs > 0 ) {
            try {
                sleep(timeMs);
            } catch (InterruptedException e) {
            }
            slideMotor.stopRotation();
        }
    }

    public void stopSlide() {
        slideMotor.stopRotation();
    }
}
