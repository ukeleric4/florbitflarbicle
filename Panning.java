package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Panning {
    private PIDController controller;

    public static double p = 0, i = 0, d = 0;
    public static double f = 0;

    public static int target = 0;

    private final DcMotorEx motor;

    private Telemetry t;

    public Panning(HardwareMap hardwareMap) {
        controller = new PIDController(p, i, d);
        t = new MultipleTelemetry(t, FtcDashboard.getInstance().getTelemetry());
        motor = hardwareMap.get(DcMotorEx.class, "panningmotor");
    }

    public void update() {
        controller.setPID(p, i, d);
        int motorPos = motor.getCurrentPosition();
        double pid = controller.calculate(motorPos, target);
        double ticks_in_degree = 384.5 / 180.0;
        double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;

        double power = pid + ff;

        motor.setPower(power);

        t.addData("pos:", motorPos);
        t.addData("target:", target);
        t.update();
    }

    public void setTarget(int target) {
        Panning.target = target;
    }

    public int getTarget() {
        return Panning.target;
    }
}
