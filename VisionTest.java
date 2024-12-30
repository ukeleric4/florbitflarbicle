package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;

import java.util.List;

@Autonomous
public class VisionTest extends OpMode {
    List<ColorBlobLocatorProcessor.Blob> blobs;
    ColorBlobLocatorProcessor colorLocator;
    ColorBlobLocatorProcessor.BlobFilter areaFilter;
    VisionPortal portal;
    ColorBlobLocatorProcessor.Blob b;
    double otherAngle = 0;
    Point[] points;

    @Override
    public void init() {
        // get claw orientation servo
        colorLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.BLUE)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.asUnityCenterCoordinates(-0.5, 0.5, 0.5, -0.5))
                .setDrawContours(true)
                .build();
        portal = new VisionPortal.Builder()
                .addProcessor(colorLocator)
                .setCameraResolution(new Size(320, 240))
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .build();
        areaFilter = new ColorBlobLocatorProcessor.BlobFilter(ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA,50, 20000);
    }

    @Override
    public void loop() {
        blobs = colorLocator.getBlobs();
        if (!blobs.isEmpty()) {
            b = blobs.get(0);
            RotatedRect boxFit = b.getBoxFit();
            boxFit.points(points);
            double angle = boxFit.angle;
            if (boxFit.size.width < boxFit.size.height) {
                otherAngle = angle - 90;
            }
            telemetry.addLine(String.format("%5d  %4.2f   %5.2f  (%3d,%3d)",
                    b.getContourArea(), b.getDensity(), b.getAspectRatio(), (int) boxFit.center.x, (int) boxFit.center.y));
            telemetry.addData("angle:", angle);
            telemetry.addData("weird angle:", otherAngle);
        }

        telemetry.update();
        try {
            Thread.sleep(50);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }
}
