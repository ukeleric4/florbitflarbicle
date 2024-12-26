package org.firstinspires.ftc.teamcode.parts;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MotorReal {
    public DcMotor motor;

    public MotorReal(HardwareMap hw, String name) {
        motor = hw.get(DcMotor.class, name);
        resetEncoder();
    }

    public void resetEncoder() {motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);}
    public void setModeRunToPosition() {
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void setModeEncoder() {
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void setModeNoEncoder() {
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void setTargetPosition(int targetPosition) {motor.setTargetPosition(targetPosition);}
    public void setDirectionForward() {
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
    }
    public void setDirectionReverse() {
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    public void stopRotation() {
        motor.setPower(0);
    }

    public void rotateForward(double power, int timeMs) {
        setDirectionForward();
        motor.setPower(power);
        if (timeMs > 0 ) {
            try {
                sleep(timeMs);
            } catch (InterruptedException e) {
            }
            stopRotation();
        }
    }

    public void rotateBackward(double power, int timeMs) {
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setPower(power);
        if (timeMs > 0 ) {
            try {
                sleep(timeMs);
            } catch (InterruptedException e) {
            }
            stopRotation();
        }
    }
}
