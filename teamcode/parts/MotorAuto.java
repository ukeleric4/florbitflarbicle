package org.firstinspires.ftc.teamcode.parts;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MotorAuto {

    /* Declare OpMode members. */
    //private ElapsedTime runtime = new ElapsedTime();
    protected DcMotorEx motor = null;
    private Telemetry telemetry = null;
    protected String name = "";

    public MotorAuto(HardwareMap hardwareMap, Telemetry t) {

        //motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        telemetry = t;
    }

    protected void setModeRunToPosition() {
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    protected void setModeResetEncoder() {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    protected void setModeEncoder() {
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void setVelocity(double motorVelocity) {
        motor.setVelocity(motorVelocity);
    }

    protected void setTargetPosition(int targetPosition) {
        motor.setTargetPosition(targetPosition);
    }

    public void stop() {
        //motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        setVelocity(0);

    }

    public void holdPosition () {
        setModeRunToPosition ();
        int position = motor.getCurrentPosition();
        RobotLog.d ("%s - Current position = %d", name, position);
//motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//setVelocity (0);
        setTargetPosition(position);
       // motor.setPower(0.5);

    }

    //using encoders
    public void runforward(int targetPosition) {

        //RobotLog.d("Motor.runforward");
        motor.setDirection(DcMotor.Direction.FORWARD);
        setTargetPosition(targetPosition);
        //motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(1);
        while (motor.isBusy());
        RobotLog.d("Motor.runforward done");
        //motor.setPower(0);
    }

    //using encoders
    public void runbackward(int targetPosition) {

        //RobotLog.d("Motor.runbackward target = %d, current = %d", targetPosition, motor.getCurrentPosition());
        motor.setDirection(DcMotor.Direction.REVERSE);
        motor.setTargetPosition(targetPosition);
        //motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(1);
        while (motor.isBusy());
        //motor.setPower(0);
    }

    public void runforward(double motorVelocity, int timeMs) {

        motor.setDirection(DcMotor.Direction.FORWARD);
        //velocity input
        setVelocity(motorVelocity);
        int position = motor.getCurrentPosition();
        RobotLog.d ("%s - Current position = %d", name, position);
        //time input
        if (timeMs > 0) {
            try {
                sleep(timeMs);
            } catch (InterruptedException e) {
            }
            stop();
        }

    }

    public void runbackward(double motorVelocity, int timeMs) {

        motor.setDirection(DcMotor.Direction.REVERSE);
        //velocity input
        setVelocity(motorVelocity);
        int position = motor.getCurrentPosition();
        RobotLog.d ("%s - Current position = %d", name, position);
        //time input
        if (timeMs > 0) {
            try {
                sleep(timeMs);
            } catch (InterruptedException e) {
            }
            stop();
        }

    }


    public void showTelemetery() {
        telemetry.addData("Encoder value - motor", motor.getCurrentPosition());
        telemetry.update();
    }
}