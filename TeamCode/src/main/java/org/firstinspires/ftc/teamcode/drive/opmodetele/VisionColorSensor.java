/*
 * Copyright (c) 2024 Phil Malone
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

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

/*
 * This OpMode illustrates how to use a video source (camera) as a color sensor
 *
 * A "color sensor" will typically determine the color of the object that it is pointed at.
 *
 * This sample performs the same function, except it uses a video camera to inspect an object or scene.
 * The user may choose to inspect all, or just a Region of Interest (ROI), of the active camera view.
 * The user must also provide a list of "acceptable colors" (Swatches) from which the closest matching color will be selected.
 *
 * To perform this function, a VisionPortal runs a PredominantColorProcessor process.
 *   The PredominantColorProcessor process is created first, and then the VisionPortal is built to use this process.
 *   The PredominantColorProcessor analyses the ROI and splits the colored pixels into several color-clusters.
 *   The largest of these clusters is then considered to be the "Predominant Color"
 *   The process then matches the Predominant Color with the closest Swatch and returns that match.
 *
 * To aid the user, a colored rectangle is drawn on the camera preview to show the RegionOfInterest,
 * The Predominant Color is used to paint the rectangle border, so the user can verify that the color is reasonable.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */


@TeleOp(name = "Concept: Vision Color-Sensor", group = "Concept")
public class VisionColorSensor extends LinearOpMode
{
    @Override
    public void runOpMode()
    {
        /* Build a "Color Sensor" vision processor based on the PredominantColorProcessor class.
         *
         * - Focus the color sensor by defining a RegionOfInterest (ROI) which you want to inspect.
         *     This can be the entire frame, or a sub-region defined using:
         *     1) standard image coordinates or 2) a normalized +/- 1.0 coordinate system.
         *     Use one form of the ImageRegion class to define the ROI.
         *         ImageRegion.entireFrame()
         *
         *         ImageRegion.asImageCoordinates(50, 50,  150, 150)  100x100 pixel square near the upper left corner
         *         ImageRegion.asUnityCenterCoordinates(-0.1, 0.1, 0.1, -0.1)  10% width/height square centered on screen
         *
         * - Set the list of "acceptable" color swatches (matches).
         *     Only colors that you assign here will be returned.
         *     If you know the sensor will be pointing to one of a few specific colors, enter them here.
         *     Or, if the sensor may be pointed randomly, provide some additional colors that may match the surrounding.
         *     Possible choices are:
         *         RED, ORANGE, YELLOW, GREEN, CYAN, BLUE, PURPLE, MAGENTA, BLACK, WHITE;
         *
         *     Note that in the example shown below, only some of the available colors are included.
         *     This will force any other colored region into one of these colors.
         *     eg: Green may be reported as YELLOW, as this may be the "closest" match.
         */


        // The color sensor numbering system represents it's coordinates in a matrix
        // Example: colorSensor11 would be at coordonates 1:1 of the matrix

        PredominantColorProcessor colorSensor11 = new PredominantColorProcessor.Builder()
                .setRoi(ImageRegion.asImageCoordinates(0, 0,  120, 80) )
                .setSwatches(
                        PredominantColorProcessor.Swatch.RED,
                        PredominantColorProcessor.Swatch.BLUE,
                        PredominantColorProcessor.Swatch.YELLOW)
                .build();

        PredominantColorProcessor colorSensor12 = new PredominantColorProcessor.Builder()
                .setRoi(ImageRegion.asImageCoordinates(120, 0,  200, 80) )
                .setSwatches(
                        PredominantColorProcessor.Swatch.RED,
                        PredominantColorProcessor.Swatch.BLUE,
                        PredominantColorProcessor.Swatch.YELLOW)
                .build();

        PredominantColorProcessor colorSensor13 = new PredominantColorProcessor.Builder()
                .setRoi(ImageRegion.asImageCoordinates(200, 0,  320, 80) )
                .setSwatches(
                        PredominantColorProcessor.Swatch.RED,
                        PredominantColorProcessor.Swatch.BLUE,
                        PredominantColorProcessor.Swatch.YELLOW)
                .build();

        PredominantColorProcessor colorSensor21 = new PredominantColorProcessor.Builder()
                .setRoi(ImageRegion.asImageCoordinates(0, 80,  120, 160) )
                .setSwatches(
                        PredominantColorProcessor.Swatch.RED,
                        PredominantColorProcessor.Swatch.BLUE,
                        PredominantColorProcessor.Swatch.YELLOW)
                .build();

        PredominantColorProcessor colorSensor22 = new PredominantColorProcessor.Builder()
                .setRoi(ImageRegion.asImageCoordinates(120, 80,  200, 160) )
                .setSwatches(
                        PredominantColorProcessor.Swatch.RED,
                        PredominantColorProcessor.Swatch.BLUE,
                        PredominantColorProcessor.Swatch.YELLOW)
                .build();

        PredominantColorProcessor colorSensor23 = new PredominantColorProcessor.Builder()
                .setRoi(ImageRegion.asImageCoordinates(200, 80,  320, 160) )
                .setSwatches(
                        PredominantColorProcessor.Swatch.RED,
                        PredominantColorProcessor.Swatch.BLUE,
                        PredominantColorProcessor.Swatch.YELLOW)
                .build();

        PredominantColorProcessor colorSensor31 = new PredominantColorProcessor.Builder()
                .setRoi(ImageRegion.asImageCoordinates(0, 160,  120, 240) )
                .setSwatches(
                        PredominantColorProcessor.Swatch.RED,
                        PredominantColorProcessor.Swatch.BLUE,
                        PredominantColorProcessor.Swatch.YELLOW)
                .build();

        PredominantColorProcessor colorSensor32 = new PredominantColorProcessor.Builder()
                .setRoi(ImageRegion.asImageCoordinates(120, 160,  200, 240) )
                .setSwatches(
                        PredominantColorProcessor.Swatch.RED,
                        PredominantColorProcessor.Swatch.BLUE,
                        PredominantColorProcessor.Swatch.YELLOW)
                .build();

        PredominantColorProcessor colorSensor33 = new PredominantColorProcessor.Builder()
                .setRoi(ImageRegion.asImageCoordinates(200, 160,  320, 240) )
                .setSwatches(
                        PredominantColorProcessor.Swatch.RED,
                        PredominantColorProcessor.Swatch.BLUE,
                        PredominantColorProcessor.Swatch.YELLOW)
                .build();



        /*
         * Build a vision portal to run the Color Sensor process.
         *
         *  - Add the colorSensor process created above.
         *  - Set the desired video resolution.
         *      Since a high resolution will not improve this process, choose a lower resolution that is
         *      supported by your camera.  This will improve overall performance and reduce latency.
         *  - Choose your video source.  This may be
         *      .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))  .....   for a webcam
         *  or
         *      .setCamera(BuiltinCameraDirection.BACK)    ... for a Phone Camera
         */
        VisionPortal portal = new VisionPortal.Builder()
                .addProcessors(colorSensor11, colorSensor12, colorSensor13, colorSensor21, colorSensor22, colorSensor23, colorSensor31, colorSensor32, colorSensor33)
                .setCameraResolution(new Size(320, 240))
                .setCamera(hardwareMap.get(WebcamName.class, "Camera"))
                .build();

        telemetry.setMsTransmissionInterval(50);  // Speed up telemetry updates, Just use for debugging.

        // WARNING:  To be able to view the stream preview on the Driver Station, this code runs in INIT mode.
        while (opModeIsActive() || opModeInInit())
        {
            telemetry.addData("DS preview on/off", "3 dots, Camera Stream\n");

            // Request the most recent color analysis.
            // This will return the closest matching colorSwatch and the predominant RGB color.
            // Note: to take actions based on the detected color, simply use the colorSwatch in a comparison or switch.
            //  eg:
            //      if (result.closestSwatch == PredominantColorProcessor.Swatch.RED) {... some code  ...}
            PredominantColorProcessor.Result result11 = colorSensor11.getAnalysis();
            PredominantColorProcessor.Result result12 = colorSensor12.getAnalysis();
            PredominantColorProcessor.Result result13 = colorSensor13.getAnalysis();
            PredominantColorProcessor.Result result21 = colorSensor21.getAnalysis();
            PredominantColorProcessor.Result result22 = colorSensor22.getAnalysis();
            PredominantColorProcessor.Result result23 = colorSensor23.getAnalysis();
            PredominantColorProcessor.Result result31 = colorSensor31.getAnalysis();
            PredominantColorProcessor.Result result32 = colorSensor32.getAnalysis();
            PredominantColorProcessor.Result result33 = colorSensor33.getAnalysis();



            // Display the Color Sensor result.
            telemetry.addData("Best Match 1:1:", result11.closestSwatch);
            telemetry.addLine(String.format("R %3d, G %3d, B %3d", Color.red(result11.rgb), Color.green(result11.rgb), Color.blue(result11.rgb)));
            telemetry.addData("Best Match 1:2:", result12.closestSwatch);
            telemetry.addLine(String.format("R %3d, G %3d, B %3d", Color.red(result12.rgb), Color.green(result12.rgb), Color.blue(result12.rgb)));
            telemetry.addData("Best Match 1:3:", result13.closestSwatch);
            telemetry.addLine(String.format("R %3d, G %3d, B %3d", Color.red(result13.rgb), Color.green(result13.rgb), Color.blue(result13.rgb)));
            telemetry.addData("Best Match 2:1:", result21.closestSwatch);
            telemetry.addLine(String.format("R %3d, G %3d, B %3d", Color.red(result21.rgb), Color.green(result21.rgb), Color.blue(result21.rgb)));
            telemetry.addData("Best Match 2:2:", result22.closestSwatch);
            telemetry.addLine(String.format("R %3d, G %3d, B %3d", Color.red(result22.rgb), Color.green(result22.rgb), Color.blue(result22.rgb)));
            telemetry.addData("Best Match 2:3:", result23.closestSwatch);
            telemetry.addLine(String.format("R %3d, G %3d, B %3d", Color.red(result23.rgb), Color.green(result23.rgb), Color.blue(result23.rgb)));
            telemetry.addData("Best Match 3:1:", result31.closestSwatch);
            telemetry.addLine(String.format("R %3d, G %3d, B %3d", Color.red(result31.rgb), Color.green(result31.rgb), Color.blue(result31.rgb)));
            telemetry.addData("Best Match 3:2:", result32.closestSwatch);
            telemetry.addLine(String.format("R %3d, G %3d, B %3d", Color.red(result32.rgb), Color.green(result32.rgb), Color.blue(result32.rgb)));
            telemetry.addData("Best Match 3:3:", result33.closestSwatch);
            telemetry.addLine(String.format("R %3d, G %3d, B %3d", Color.red(result33.rgb), Color.green(result33.rgb), Color.blue(result33.rgb)));
            telemetry.update();

            sleep(20);
        }
    }
}
