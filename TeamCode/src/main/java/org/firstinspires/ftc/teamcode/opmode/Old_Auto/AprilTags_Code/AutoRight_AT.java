
/*
 * Copyright (c) 2021 OpenFTC Team
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

package org.firstinspires.ftc.teamcode.opmode.Old_Auto.AprilTags_Code;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous
@Disabled
public class AutoRight_AT extends LinearOpMode
{
    //Motors
    // The private/public DcMotor _____; are used to identify a motor that can be used throughout the code.
    // Note that it does not matter if you use Public or a Private class identity for the motor
    // EX: Doesn't matter if you do Private DcMotor or Public DcMotor. They are interchangeable
    // should look like ______ DcMotor "______"
    //                  Class            Name
    private DcMotor motorFL;
    private DcMotor motorFR;
    private DcMotor motorBL;
    private DcMotor motorBR;
    private DcMotor motorRiseyRise;

    //Servos
    // The Servo ____; are used to identify a servo that can be used throughout the code.
    //           Name
    Servo servoFAL;
    Servo servoFAR;



    ///////////////////////////////////////////////
    // April-Tag Things (Camera Included)     /////
    ///////////////////////////////////////////////

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    //These are The QR Code things that you need
    //int ID_TAG_OF_INTEREST = 18; // Tag ID 18 from the 36h11 family
    int Left = 11; //Detects april tag id#10 - Attached to sleeve template position one
    int Middle = 12; //Detects april tag id#20 - Attached to sleeve template position one
    int Right = 13; //Detects april tag id#30 - Attached to sleeve template position one

    AprilTagDetection tagOfInterest = null;
///////////////////////////////////////////////////////////////////////////////////////////////////

    // Global Variables

    /* Motor Tick Count
               Andy Mark 3.7:1 = 103.6
               Andy Mark 20:1  = 537.6
               Andy Mark 60:1  = 1680

           To Calculate: # ticks per revolution / Distance of Revolution =
        */
    int in = 45; //Used for the wheels to drive: = 537.6 / (pi * 96 mm) = 537.6 / (pi * 3.77953 in)
    int up = 360 ; //Used for the linear slide distance

    @Override
    public void runOpMode()
    {
        // HardwareMap Section (Used to talk to the driver hub for the configuration)

        // Motors
        motorFL = hardwareMap.dcMotor.get("motorFL");
        motorFR = hardwareMap.dcMotor.get("motorFR");
        motorBL = hardwareMap.dcMotor.get("motorBL");
        motorBR = hardwareMap.dcMotor.get("motorBR");
        motorRiseyRise = hardwareMap.dcMotor.get("motorRiseyRise");

        // Servos
        servoFAL = hardwareMap.servo.get("servoFAL");
        servoFAR = hardwareMap.servo.get("servoFAR");

        //Encoders
        // Has the Run_Using_Encoders because all the motors use Encoders throughout the code.
        // You have to have this if you are running encoders.
        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRiseyRise.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // Camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */


        ////////////////////////////////
        // Start of Code or Initialize//
        ////////////////////////////////

        while (!isStarted() && !isStopRequested())
        {
            // Calls to the Pipline
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    //         QR 10              QR 20               QR 30
                    if(tag.id == Left || tag.id == Middle || tag.id == Right)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                // If ANY QR is found Display the Green Text in Telemetry with the QR that is found
                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }

                // If NO QR is found Display one of the following Messages in Green Text
                else
                {
                    // If No Tag is seen then display the text below in the telemetry
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {

                        // If No Tag is seen then display the text below in the telemetry
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        // Code to display in the telemetry of the last seen QR Code
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }
            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            Close_Claws();
            telemetry.update();
            sleep(20);
        }


        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        // If a QR is detected, Then display the one it found in the telemetry
        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            Close_Claws();
            telemetry.update();
        }

        // If no QR Code is detected
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            Close_Claws();
            telemetry.update();
        }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////
    // All the Code above is how the camera detects the april tage (^-- That code)    ////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////////////

        //////////////////////
        // Autonomous  Code //
        //////////////////////

        if (tagOfInterest == null) {
           //put default code here
            Move(directions.STARBOARD, 58, .5);
       }

        // The Left is Sleeve 11 (QR Code 11)
        else if (tagOfInterest.id == Left ) {
            Autonomous();
            Move(directions.FORWARDS, 23, 0.5);
       }

        // The Middle is Sleeve 12 (QR Code 12)
        else if (tagOfInterest.id == Middle){
            Autonomous();
            Move(directions.BACKWARDS, 1, 0.5);
        }


        // The third else or in this case Right is Sleeve 13 (QR Code 13)
        else {
            Autonomous();
            Move(directions.BACKWARDS, 25, 0.5);
       }
    }


    ////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////                   Functions                         ///////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////


    // Function to Give telemetry on which QR Code is Detected
    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));

    }

    // Function to ONLY Raise the Linear Slide / Arm
    private void Raise(int target, double speed) {
        // The stop and reset encoders is needed to reset and start the encoders (You need one at the
        // end because it tells the robot where the drive code starts and ends, kinda like brackets)
        motorRiseyRise.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // This sets the direction for the motor for the wheels to drive forward
        motorRiseyRise.setDirection(DcMotorSimple.Direction.REVERSE);
        // Gives it a position to run to
        motorRiseyRise.setTargetPosition(target * up);
        // tells it to go to the position that is set
        motorRiseyRise.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // the motor speed for Wheels
        motorRiseyRise.setPower(speed);
        // While loop keeps the code running until motors reach the desired position
        while (opModeIsActive() && (motorRiseyRise.isBusy())) {
        }
        motorRiseyRise.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    // Function to ONLY Lower the Linear Slide / Arm
    private void Lower(int target, double speed) {
        motorRiseyRise.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // The stop and reset encoders is needed to reset and start the encoders (You need one at the
        // end because it tells the robot where the drive code starts and ends, kinda like brackets)
        motorRiseyRise.setTargetPosition(target * up);
        // tells it to go to the position that is set
        motorRiseyRise.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // the motor speed for Wheel
        motorRiseyRise.setPower(speed);
        // While loop keeps the code running until motors reach the desired position
        while (opModeIsActive() && (motorRiseyRise.isBusy())) {
        }
        motorRiseyRise.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    // Function to Open the Claws on the Robot
    private void Open_Claws() {
        servoFAL.setPosition(.15);
        servoFAR.setDirection(Servo.Direction.REVERSE);
        servoFAR.setPosition(.15);
    }

    // Function to close the claws on the Robot
    private void Close_Claws() {
        servoFAL.setPosition(0);
        servoFAR.setDirection(Servo.Direction.REVERSE);
        servoFAR.setPosition(0);
    }

    // Function to Move to the Right and Raise the Linear Slide / Arm
    private void Right_Raise(int move_target, double move_speed, int claw_target, double claw_speed) {

        // Stops and Resets the encoders in order to use the motors
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRiseyRise.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // This sets the direction for the motor for the wheels to drive forward
        motorFL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBL.setDirection(DcMotorSimple.Direction.FORWARD);
        motorFR.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBR.setDirection(DcMotorSimple.Direction.FORWARD);
        motorRiseyRise.setDirection(DcMotorSimple.Direction.REVERSE);

        // Gives it a position to run to
        motorFL.setTargetPosition(move_target * in);
        motorFR.setTargetPosition(move_target * in);
        motorBL.setTargetPosition(move_target * in);
        motorBR.setTargetPosition(move_target * in);
        motorRiseyRise.setTargetPosition(claw_target * up);

        // tells it to go to the position that is set
        motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRiseyRise.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // the motor speed for Wheels
        motorFL.setPower(move_speed);
        motorFR.setPower(move_speed);
        motorBL.setPower(move_speed);
        motorBR.setPower(move_speed);
        motorRiseyRise.setPower(claw_speed);

        // While loop keeps the code running until motors reach the desired position
        while (opModeIsActive() && (motorFL.isBusy() || motorFR.isBusy() || motorRiseyRise.isBusy())) {
        }
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRiseyRise.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    // Function to Move to the Left and Lower the Linear Slide / Arm
    private void Left_Lower(int move_target, double move_speed, int claw_target, double claw_speed) {
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRiseyRise.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // This sets the direction for the motor for the wheels to drive forward
        motorFL.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFR.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBR.setDirection(DcMotorSimple.Direction.REVERSE);
        motorRiseyRise.setDirection(DcMotorSimple.Direction.REVERSE);
        // Gives it a position to run to
        motorFL.setTargetPosition(move_target * in);
        motorFR.setTargetPosition(move_target * in);
        motorBL.setTargetPosition(move_target * in);
        motorBR.setTargetPosition(move_target * in);
        motorRiseyRise.setTargetPosition(claw_target * up);
        // tells it to go to the position that is set
        motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRiseyRise.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // the motor speed for Wheels
        motorFL.setPower(move_speed);
        motorFR.setPower(move_speed);
        motorBL.setPower(move_speed);
        motorBR.setPower(move_speed);
        motorRiseyRise.setPower(claw_speed);

        // While loop keeps the code running until motors reach the desired position
        while (opModeIsActive() && (motorFL.isBusy() || motorFR.isBusy() || motorRiseyRise.isBusy())) {
        }
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRiseyRise.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }


    // Function for each motor/driving direction
    private void Move(directions direction, int target, double speed) {
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // This sets the direction for the motor for the wheels to drive forward
        if (direction == directions.FORWARDS) {
            motorFL.setDirection(DcMotorSimple.Direction.REVERSE);
            motorBL.setDirection(DcMotorSimple.Direction.REVERSE);
            motorFR.setDirection(DcMotorSimple.Direction.FORWARD);
            motorBR.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        // Sets the motor direction to move Backwards
        else if (direction == directions.BACKWARDS) {
            motorFL.setDirection(DcMotorSimple.Direction.FORWARD);
            motorBL.setDirection(DcMotorSimple.Direction.FORWARD);
            motorFR.setDirection(DcMotorSimple.Direction.REVERSE);
            motorBR.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        // Sets the motor direction to move to the Left ( Note * Port = Left)
        else if (direction == directions.PORT) {
            motorFL.setDirection(DcMotorSimple.Direction.FORWARD);
            motorBL.setDirection(DcMotorSimple.Direction.REVERSE);
            motorFR.setDirection(DcMotorSimple.Direction.FORWARD);
            motorBR.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        // Sets the motor direction to move to the Right (Note * Starboard = Right)
        else if (direction == directions.STARBOARD) {
            motorBR.setDirection(DcMotorSimple.Direction.FORWARD);
            motorFR.setDirection(DcMotorSimple.Direction.REVERSE);
            motorBL.setDirection(DcMotorSimple.Direction.FORWARD);
            motorFL.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        // Sets the motor direction to rotate Clockwise
        else if (direction == directions.CLOCKWISE) {
            motorBR.setDirection(DcMotorSimple.Direction.REVERSE);
            motorFR.setDirection(DcMotorSimple.Direction.REVERSE);
            motorBL.setDirection(DcMotorSimple.Direction.REVERSE);
            motorFL.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        // Sets the motor direction to rotate Counter Clockwise
        else if (direction == directions.COUNTER_CLOCKWISE) {
            motorBR.setDirection(DcMotorSimple.Direction.FORWARD);
            motorFR.setDirection(DcMotorSimple.Direction.FORWARD);
            motorBL.setDirection(DcMotorSimple.Direction.FORWARD);
            motorFL.setDirection(DcMotorSimple.Direction.FORWARD);
        }


        // Gives it a position to run to
        motorFL.setTargetPosition(target * in);
        motorFR.setTargetPosition(target * in);
        motorBL.setTargetPosition(target * in);
        motorBR.setTargetPosition(target * in);

        // tells it to go to the position that is set
        motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // the motor speed for Wheels
        motorFL.setPower(speed);
        motorFR.setPower(speed);
        motorBL.setPower(speed);
        motorBR.setPower(speed);


        // While loop keeps the code running until motors reach the desired position
        while (opModeIsActive() && (motorFL.isBusy() || motorFR.isBusy())) {
        }
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }


    // Our Autonomous Code
    private void Autonomous () {

        // Raise the claws up to the High Junction while moving the Signal Sleeve cone out of the way

        Right_Raise(76, 0.5, 33, 0.8);

        // Moves to the Left to align itself up with the High Junction

        Move(directions.PORT, 4, 0.5);

        // Moves Forwards so the Cone is over the High Junction

        Move(directions.FORWARDS, 4, 0.5);



        // Waits for the High Junction to stop shaking, then lowers the Arm

        sleep(3000);
        Lower(-2, 0.8);

        // Opens the claws to drop the cone on the High Junction
        Open_Claws();

        // Moves Backwards to get off of the High Junction

        sleep(200);
        Move(directions.BACKWARDS, 5, 0.5);

        // Moves Left to align itself for parking while also lowering the claw
        Left_Lower(18, 0.5, -31, 0.7);
    }

    // Names to for the Directions used in the Move Function
    enum directions{
        FORWARDS,
        BACKWARDS,
        PORT,//LEFT
        STARBOARD,//RIGHT
        CLOCKWISE, //TURN CLOCKWISE
        COUNTER_CLOCKWISE, //TURN COUNTER-CLOCKWISE
    }


}