package org.firstinspires.ftc.teamcode.parts;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class Claw {
    public Servos left;
    public Servos right;

    public Claw(HardwareMap hw) {
        left = new Servos(hw, "left");
        right = new Servos(hw, "right");
    }

    public void closeClaw() {
        left.moveToMax();
        right.moveToMax();
    }

    public void openClaw() {
        left.moveToMin();
        right.moveToMin();
    }

    public void moveClawToSpecificPos(double pos) {
        left.moveSpecificPos(pos);
        right.moveSpecificPos(pos);
    }
}
