package org.firstinspires.ftc.teamcode.parts;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class HuskyLenses {
    public HuskyLens huskyLens;
    public int SCREENWIDTH = 320;
    public int MAXHEIGHT = 240;
    public int redid = 3;
    public int blueid = 2;
    public int yellowid = 1;

    public HuskyLens.Block[] observedObjects = new HuskyLens.Block[30];
    public HuskyLens.Block currentTarget;

    public HuskyLenses(HardwareMap hw, String name, String mode) {
        huskyLens = hw.get(HuskyLens.class, name);
        switch (mode) {
            case "color":
                huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);
                break;
            case "obj_track":
                huskyLens.selectAlgorithm(HuskyLens.Algorithm.OBJECT_RECOGNITION);
                break;
            case "face":
                huskyLens.selectAlgorithm(HuskyLens.Algorithm.FACE_RECOGNITION);
                break;
            case "tag":
                huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
                break;
        }
    }

    public HuskyLens.Block getFirstObject() {
        updateObservedObjects();
        if (observedObjects.length > 0) {
            return observedObjects[0];
        }
        return null;
    }

    public double calculateServoPosition(double height, double width) {
        // Calculate the block's angle in degrees
        double blockAngle = Math.toDegrees(Math.atan2(height, width));

        // Normalize the angle to [0, 360)
        blockAngle = blockAngle % 360;
        if (blockAngle < 0) {
            blockAngle += 360;
        }

        // Map angle to the servo's range of [0, 180]
        double mappedAngle;
        if (blockAngle > 180) {
            // For angles greater than 180, map to equivalent 0-180 range
            mappedAngle = blockAngle - 180;
        } else {
            // For angles in range 0-180, retain as is
            mappedAngle = blockAngle;
        }

        // Map angle to servo position [0, 1]
        double servoPosition = mappedAngle / 180.0;
        return servoPosition;
    }

    public void updateObservedObjects() {
        observedObjects = huskyLens.blocks();
    }

    public int getTargetId() {
        return currentTarget.id;
    }

    public double getProportion() {
        return (double) currentTarget.height / currentTarget.width;
    }
}
