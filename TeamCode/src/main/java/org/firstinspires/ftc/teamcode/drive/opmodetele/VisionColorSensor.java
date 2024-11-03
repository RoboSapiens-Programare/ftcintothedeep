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

@TeleOp(name = "Concept: Vision Color-Sensor", group = "Concept")

public class VisionColorSensor extends LinearOpMode
{
    public int yellow(int rgb) {
        //Extract red, green, and blue components from the RGB value

        int red = (rgb >> 16) & 0xFF;
        int green = (rgb >> 8) & 0xFF;
        int blue = rgb & 0xFF;

        //Calculate yellow component by combining red and green, minus blue influence

        int yellowComponent = (red + green - blue) / 2;

        //Ensure the result is within 0-255 range

        return Math.max(0, Math.min(255, yellowComponent));
    }

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
         *     eg: Green may be reported as YELLOW, as this may be the "closest" match.https://github.com/RoboSapiens-Programare/ftcintothedeep/tree/concept-vision-color-sensor
         */

        /*
         * We will use a Swatch vector to define the acceptable colors.
         *  We will use the image coordinates to define the Region of Interest (ROI) in the roiCoordinates matrix.
         *  The image coordinates start at the top left corner of the image and are: left, top, right, bottom; they all start from the top left.
         *  We will use a PredominantColorProcessor vector to define all of the sectors.
         *  Then we will build all of the sectors using the Swatch vector, the roiCoordinates matrix and the sectors.
         */


        final PredominantColorProcessor.Swatch swatches[] = {
                PredominantColorProcessor.Swatch.RED,
                PredominantColorProcessor.Swatch.BLUE,
                PredominantColorProcessor.Swatch.YELLOW
        };

        int[][] roiCoordinates = {
                {0, 0, 120, 80}, {120, 0, 200, 80}, {200, 0, 320, 80},
                {0, 80, 120, 160}, {120, 80, 200, 160}, {200, 80, 320, 160},
                {0, 160, 120, 240}, {120, 160, 200, 240}, {200, 160, 320, 240}
        };

        PredominantColorProcessor[] sectors = new PredominantColorProcessor[roiCoordinates.length];

        for (int i = 0; i < roiCoordinates.length; i++) {
            int[] coords = roiCoordinates[i];
            sectors[i] = new PredominantColorProcessor.Builder()
                    .setRoi(ImageRegion.asImageCoordinates(coords[0], coords[1], coords[2], coords[3]))
                    .setSwatches(swatches)
                    .build();
        }



        /*
         * Build a vision portal to run the sectors created above.
         *  - Add the colorSensor process created above.
         *  - Set the desired video resolution.
         *      Since a high resolution will not improve this process, choose a lower resolution that is
         *      supported by your camera.  This will improve overall performance and reduce latency.
         *  - Choose your video source.
         */
        
        VisionPortal portal = new VisionPortal.Builder()
                .addProcessors(sectors)
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


            PredominantColorProcessor.Result[] results = new PredominantColorProcessor.Result[sectors.length];
            for (int i = 0; i < sectors.length; i++) {
                results[i] = sectors[i].getAnalysis();
            }

            //Returns the sector with the most red in it after the threshold of 190, if it didn't get past 190 then it will return sector 0.

            int maxRed = 190;
            int resultMaxRed = 0;

            for (int i = 0; i < 9; i++) {
                int red = Color.red(results[i].rgb);
                if (red > maxRed) {
                    maxRed = red;
                    resultMaxRed = i;
                }
            }


            //Returns the sector with the most blue in it after the threshold of 145, if it didn't get past 145 then it will return sector 0.

            int maxBlue = 145;
            int resultMaxBlue = 0;
            for (int i = 0; i < 9; i++) {
                int blue = Color.blue(results[i].rgb);
                if (blue > maxBlue) {
                    maxBlue = blue;
                    resultMaxBlue = i;
                }
            }

            //Returns the sector with the most yellow in it after the threshold of 145, if it didn't get past 145 then it will return sector 0.

            int maxYellow = 145;
            int resultMaxYellow = 0;
            for (int i = 0; i < 9; i++) {
                int yellow = yellow(results[i].rgb);
                if (yellow > maxYellow) {
                    maxYellow = yellow;
                    resultMaxYellow = i;
                }
            }

            //Adds to telemetry the values of the max of each sector and the most red sector, the most blue sector, and the most yellow sector after the threshold.

            telemetry.addData("max yellow: ", maxYellow);
            telemetry.addData("max red: ", maxRed);
            telemetry.addData("max blue: ", maxBlue);
            telemetry.addData("Best Red Match:", resultMaxRed);
            telemetry.addData("Best Blue Match:", resultMaxBlue);
            telemetry.addData("Best Yellow Match:", resultMaxYellow);
            telemetry.update();
        }
    }
}