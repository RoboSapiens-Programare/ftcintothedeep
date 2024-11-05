//TODO: account for multiple game elements of the same color
//TODO: get directions according to color positions

package org.firstinspires.ftc.teamcode.drive.opmodetele;

import android.graphics.Color;
import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.core.Point;
import org.opencv.imgproc.Imgproc;

@TeleOp(name = "Concept: Vision Color-Sensor", group = "Concept")
public class VisionColorSensor extends LinearOpMode {
    private OpenCvCamera webcam;

    public int yellow(int rgb) {
        int red = (rgb >> 16) & 0xFF;
        int green = (rgb >> 8) & 0xFF;
        int blue = rgb & 0xFF;
        int yellowComponent = (red + green - blue) / 2;
        return Math.max(0, Math.min(255, yellowComponent));
    }

    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Camera"), cameraMonitorViewId);
        webcam.setPipeline(new ColorDetectionPipeline());

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Error", errorCode);
                telemetry.update();
            }
        });

        waitForStart();

        while (opModeIsActive()) {
            // Telemetry is updated within the pipeline
            sleep(100);
        }
    }

    class ColorDetectionPipeline extends OpenCvPipeline {
        @Override
        public Mat processFrame(Mat input) {
            Mat maskRed = new Mat();
            Mat maskBlue = new Mat();
            Mat maskYellow = new Mat();

            // Define color ranges (HSV)
            Scalar lowerRed = new Scalar(0, 100, 100);
            Scalar upperRed = new Scalar(10, 255, 255);
            Scalar lowerBlue = new Scalar(100, 150, 0);
            Scalar upperBlue = new Scalar(140, 255, 255);
            Scalar lowerYellow = new Scalar(20, 100, 100);
            Scalar upperYellow = new Scalar(30, 255, 255);

            // Convert frame to HSV
            Mat hsv = new Mat();
            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

            // Create masks for each color
            Core.inRange(hsv, lowerRed, upperRed, maskRed);
            Core.inRange(hsv, lowerBlue, upperBlue, maskBlue);
            Core.inRange(hsv, lowerYellow, upperYellow, maskYellow);

            // Find the average position of each color
            Point avgRed = findAveragePosition(maskRed);
            Point avgBlue = findAveragePosition(maskBlue);
            Point avgYellow = findAveragePosition(maskYellow);

            // Print to telemetry
            telemetry.addData("Red Position", avgRed.toString());
            telemetry.addData("Blue Position", avgBlue.toString());
            telemetry.addData("Yellow Position", avgYellow.toString());
            telemetry.update();

            // Release temporary mats
            maskRed.release();
            maskBlue.release();
            maskYellow.release();
            hsv.release();

            return input;
        }

        private Point findAveragePosition(Mat mask) {
            Moments moments = Imgproc.moments(mask, true);
            double m00 = moments.get_m00();
            double m10 = moments.get_m10();
            double m01 = moments.get_m01();
            if (m00 != 0) {
                return new Point(m10 / m00, m01 / m00);
            } else {
                return new Point(-1, -1); // Invalid position
            }
        }
    }
}
