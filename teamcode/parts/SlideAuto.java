package org.firstinspires.ftc.teamcode.parts;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SlideAuto extends MotorAuto implements Runnable {
    boolean do_async = false;
    boolean exit = false;
    int targetLevel = 0;

    @Override
    public void run() {
        while (exit == false) {
            if (do_async) {
                MoveToLevel(targetLevel);
                do_async = false;
            }
        }
    }

    public SlideAuto(HardwareMap hardwareMap, Telemetry t, String device_name) {
        super(hardwareMap, t);
        motor = hardwareMap.get(DcMotorEx.class, device_name);
        name = device_name;
        super.setTargetPosition(0);
        setModeResetEncoder();
        setModeRunToPosition();
        Thread te = new Thread(this, device_name);
        te.start();
    }

    public void terminate () {exit = true;}
    private void move (int l) {
        if (name == "panningmotor"){
            runforward(l);
        } else {
            runforward(l);
        }
    }

    public void setmanualcontrol (boolean manual){
        if (manual == true){
            setModeResetEncoder();
        } else {
            setModeRunToPosition();
        }
    }
    public boolean MoveToLevel(int l){
        move(l);
        return true;
    }

    public boolean MoveToLevelAsync (int l) {
        targetLevel = l;
        do_async = true;
        return true;
    }

}