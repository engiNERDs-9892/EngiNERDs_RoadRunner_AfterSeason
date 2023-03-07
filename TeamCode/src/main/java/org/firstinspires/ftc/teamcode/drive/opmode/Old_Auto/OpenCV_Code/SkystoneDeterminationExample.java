package org.firstinspires.ftc.teamcode.drive.opmode.Old_Auto.OpenCV_Code;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;


///////////////////////////////////////////////////////////////////////////////////////////////////
// This is code so that a separate code can refer to it | This is how the camera can get its data//
///////////////////////////////////////////////////////////////////////////////////////////////////

@TeleOp
@Disabled
public class SkystoneDeterminationExample extends LinearOpMode
{

   // This is how we identify our webcam
    OpenCvWebcam webcam;


    // Creating a pipeline to use later
    SkystoneDeterminationPipeline pipeline;


    @Override
    public void runOpMode()
    {


        // Looking at the camera preview
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        // Hardware map for the camera --> Use this to talk to the Control Hub to give the camera name
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        // Creating a pipeline to use in a separate code
        pipeline = new SkystoneDeterminationPipeline();

        // Even More Pipeline Shenanigans
        webcam.setPipeline(pipeline);

        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.
        webcam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {

            // This is in what viewing window the camera is seeing through and it doesn't matter
            // what orientation it is | UPRIGHT, SIDEWAYS_LEFT, SIDEWAYS_RIGHT, etc.

                webcam.startStreaming(1280,720, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }

            @Override
            public void onError(int errorCode)
            {
                // This will be called if the camera could not be opened
            }
        });

        waitForStart();

        while (opModeIsActive())
        {
            telemetry.addData("Analysis", pipeline.getAnalysis());
            telemetry.update();

            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
        }
    }

    public static class SkystoneDeterminationPipeline extends OpenCvPipeline
    {
        // Enum to call a variable -- Has no actual value and can be used throughout any code
        public enum SkystonePosition
        {

        //These variables are used to detect on where the object is place, or what rectangle its in

            LEFT, // Left Rectangle
            CENTER, // Center Rectangle
            RIGHT // Right Rectangle
        }

       // Colors that you will use to make the rectangles - Technically it doesn't matter what color
        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 255, 0);
        // This creates a area where the camera can detect where your object is (Rectangle)
        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(720,75);
        static final Point REGION2_TOPLEFT_ANCHOR_POINT = new Point(720,155);
        static final Point REGION3_TOPLEFT_ANCHOR_POINT = new Point(720,255);
        static final int REGION_WIDTH = 120;
        static final int REGION_HEIGHT = 20;

        /*
         * Points which actually define the sample region rectangles, derived from above values
         *
         * Example of how points A and B work to define a rectangle
         *
         *   ------------------------------------
         *   | (0,0) Point A                    |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                  Point B (70,50) |
         *   ------------------------------------
         *
         */


        // All this does is create the rectangles and what range to calculate it in
        Point region1_pointA = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x,
                REGION1_TOPLEFT_ANCHOR_POINT.y);
        Point region1_pointB = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
        Point region2_pointA = new Point(
                REGION2_TOPLEFT_ANCHOR_POINT.x,
                REGION2_TOPLEFT_ANCHOR_POINT.y);
        Point region2_pointB = new Point(
                REGION2_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION2_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
        Point region3_pointA = new Point(
                REGION3_TOPLEFT_ANCHOR_POINT.x,
                REGION3_TOPLEFT_ANCHOR_POINT.y);
        Point region3_pointB = new Point(
                REGION3_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION3_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);



        // Variables created in a Matrix
        Mat region1_Cb, region2_Cb, region3_Cb;

        // Creating a Matrix to store variables --> Variables = Colors
        // Y = Luminance   Cr = Chroma: Red     Cb = Chroma: Blue
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();

        // Variables created to use later
        int avg1, avg2, avg3;

        // Volatile since accessed by OpMode thread w/o synchronization
        private volatile SkystonePosition position = SkystonePosition.LEFT;


        // This function takes the RGB frame, converts to YCrCb,
        // and extracts the Cb channel to the 'Cb' variable

        void inputToCb(Mat input)
        {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 2);
        }


        @Override
        public void init(Mat firstFrame)
        {
            /*
             * We need to call this in order to make sure the 'Cb'
             * object is initialized, so that the submats we make
             * will still be linked to it on subsequent frames. (If
             * the object were to only be initialized in processFrame,
             * then the submats would become delinked because the backing
             * buffer would be re-allocated the first time a real frame
             * was crunched)
             */
            inputToCb(firstFrame);

            // Sub matrix to convert the rectangle
            region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
            region2_Cb = Cb.submat(new Rect(region2_pointA, region2_pointB));
            region3_Cb = Cb.submat(new Rect(region3_pointA, region3_pointB));
        }

        @Override
        public Mat processFrame(Mat input)
        {


             ///////////////////////////////////////////////////////////////////////////////////////
             // Overview of what we're doing:

             // We first convert to YCrCb color space, from RGB color space.
             // we do this because in the RGB color space, chroma and luminance are intertwined
             // In YCrCb, chroma and luminance are separated.
             // YCrCb is a 3-channel color space, just like RGB. YCrCb's 3 channels
             // are Y Channel (luminance) the Cr Channel (Chroma Red) Cb Channel (Chroma Blue)
             // Cr Channel Differences from red and the Cb Channel Differences from Blue

             // We also draw rectangles on the screen showing where the sample regions
             // are, as well as drawing a solid rectangle over the sample region it detected

             // Since we are trying to find yellow and Yellow has one of the Lowest CB
             // We ant to set it to minimum

            ///////////////////////////////////////////////////////////////////////////////////////



             // Get the Cb channel of the input frame after conversion to YCrCb

            inputToCb(input);


             // Compute the average pixel value of each sub matrix region. We're
             // taking the average of a single channel buffer, so the value
             // we need is at index 0. We could have also taken the average
             // pixel value of the 3-channel image, and referenced the value
             // at index 2 here.


            avg1 = (int) Core.mean(region1_Cb).val[0];
            avg2 = (int) Core.mean(region2_Cb).val[0];
            avg3 = (int) Core.mean(region3_Cb).val[0];

             // Draw a rectangle showing sample region 1 on the screen.
             // Simply a visual aid. Serves no functional purpose and only for camera preview.

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines


             // Draw a rectangle showing sample region 2 on the screen.
             // Simply a visual aid. Serves no functional purpose and only for camera preview..

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region2_pointA, // First point which defines the rectangle
                    region2_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines


            // Draw a rectangle showing sample region 1 on the screen.
            // Simply a visual aid. Serves no functional purpose and only for camera preview.
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region3_pointA, // First point which defines the rectangle
                    region3_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines



            // This is trying to find the averages of the calculations up above to find YELLOW
            int maxOneTwo = Math.max(avg1, avg2);
            int max = Math.max(maxOneTwo, avg3);

            /*
             * Now that we found the max, we actually need to go and
             * figure out which sample region that value was from
             */

            if(max == avg1) // Was it from region 1?
            {
                position = SkystonePosition.LEFT; // Record our analysis - Puts it in telemetry

                Imgproc.rectangle(
                        input, // Buffer to draw on
                        region1_pointA, // First point which defines the rectangle
                        region1_pointB, // Second point which defines the rectangle
                        GREEN, // The color the rectangle is drawn in
                        -1); // Negative means solid fill

                // In a nut shell if it detects it in region 1, it should solid fill with green
            }
            else if(max == avg2) // Was it from region 2?
            {
                position = SkystonePosition.CENTER; // Record our analysis - Puts it in telemetry

                Imgproc.rectangle(
                        input, // Buffer to draw on
                        region2_pointA, // First point which defines the rectangle
                        region2_pointB, // Second point which defines the rectangle
                        GREEN, // The color the rectangle is drawn in
                        -1); // Negative thickness means solid fill
                // In a nut shell if it detects it in the region 2 it should solid fill with green
            }
            else if(max == avg3) // Was it from region 3?
            {
                position = SkystonePosition.RIGHT; // Record our analysis - Puts it in telemetry

                Imgproc.rectangle(
                        input, // Buffer to draw on
                        region3_pointA, // First point which defines the rectangle
                        region3_pointB, // Second point which defines the rectangle
                        GREEN, // The color the rectangle is drawn in
                        -1); // Negative thickness means solid fill


                // In a nut shell if it detects it in the region 3 it should solid fill with green
            }

           // Shows the the rectangles
            return input;
        }

        public SkystonePosition getAnalysis()
        {
            return position;
        }
    }
}
