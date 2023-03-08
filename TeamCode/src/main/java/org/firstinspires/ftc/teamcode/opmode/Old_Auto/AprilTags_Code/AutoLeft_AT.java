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
public class AutoLeft_AT extends LinearOpMode
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

    //int ID_TAG_OF_INTEREST = 18; // Tag ID 18 from the 36h11 family
    int Left = 11; //Detects april tag id#5 - Attached to sleeve template position one
    int Middle = 12; //Detects april tag id#6 - Attached to sleeve template position one
    int Right = 13; //Detects april tag id#7 - Attached to sleeve template position one
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
    int up = 360; //(Old360) Used for the linear slide distance spool 50in dia

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
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam Left"), cameraMonitorViewId);
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

        while (!isStarted() && !isStopRequested()) {

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

        // All the Code above is how the camera detects the april tage (^-- That code)
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////

        //////////////////////
        // Autonomous  Code //
        //////////////////////

        if (tagOfInterest == null) {
           //put default code here
            Move(directions.STARBOARD, 58, .5);
       }

        // The Left is Sleeve 10 (QR Code 10)
        else if (tagOfInterest.id == Left ) {
            Autonomous();
            Backwards_Raise(5, 0.5, -10, .8);
       }

        // The Middle is Sleeve 20 (QR Code 20)
        else if (tagOfInterest.id == Middle){
            Autonomous();
            Backwards_Raise(27, 0.7, -10, .8);
        }


        // The third else or in this case Right is Sleeve 30 (QR Code 30)
        else {
            Autonomous();
            Backwards_Raise(50, 1, -10, 0.8);

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
        //
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
        // The stop and reset encoders is needed to reset and start the encoders (You need one at the
        // end because it tells the robot where the drive code starts and ends, kinda like brackets)
        motorRiseyRise.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Gives it a position to run to
        motorRiseyRise.setDirection(DcMotorSimple.Direction.FORWARD);

        motorRiseyRise.setTargetPosition(target * up);
        // tells it to go to the position that is set
        motorRiseyRise.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // the motor speed
        motorRiseyRise.setPower(speed);
        // While loop keeps the code running until motors reach the desired position
        while (opModeIsActive() && (motorRiseyRise.isBusy())) {
        }
        motorRiseyRise.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    // Function to Open the Claws
    private void Open_Claws() {
        servoFAL.setPosition(.25);
        servoFAR.setDirection(Servo.Direction.REVERSE);
        servoFAR.setPosition(.25);
    }

    // Function to Close the Claws
    private void Close_Claws() {
        servoFAL.setPosition(0);
        servoFAR.setDirection(Servo.Direction.REVERSE);
        servoFAR.setPosition(0);
    }

    // Function to Move to the Right and Raise the Linear Slide / Arm
    private void Right_Raise(int move_target, double move_speed, int claw_target, double claw_speed) {
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
        motorRiseyRise.setDirection(DcMotorSimple.Direction.FORWARD);

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

    // Function to Move Backwards and Raise the Linear Slide / Arm
    private void Backwards_Raise(int move_target, double move_speed, int claw_target, double claw_speed) {
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRiseyRise.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // This sets the direction for the motor for the wheels to drive forward
        motorFL.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBL.setDirection(DcMotorSimple.Direction.FORWARD);
        motorFR.setDirection(DcMotorSimple.Direction.REVERSE);
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


    // Directions
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

    // Names to for the Directions used in the Move Function
    enum directions{
        FORWARDS,
        BACKWARDS,
        PORT, //LEFT
        STARBOARD, //RIGHT
        CLOCKWISE, //TURN CLOCKWISE
        COUNTER_CLOCKWISE, //TURN COUNTER-CLOCKWISE
    }

    private void Autonomous() {

        // Key
        // Arm == Linear Slides / Motor RiseyRise



        ////////////////////////////////
        // Pre-Loaded Cone or Cone #1///
        ////////////////////////////////

        // To Capture / Hold Pre-Loaded Cone



        // Moves to the Small Junction on the left side of the field while lifting the arm

        Right_Raise(44, 0.5, 16, 0.8);

        // Moves forward so the cone is over the Small Junction

        Move(directions.FORWARDS,5, 0.5);

        // Lowers the cone over the junction so it is stable when dropping the cone
        sleep(250);

        // Opens the claws to drop the cone

        Open_Claws();

        // Moves Backwards to get off of the Small Junction while the claws are still open

        Move(directions.BACKWARDS, 4, 0.5);

        // Closes the claws since there is no cone for stability
        Close_Claws();


        ///////////////////////////
        // Stack Cones or Cone #2//
        ///////////////////////////

        // Moves right to align itself up with the stack of cones on the left side of the field

        sleep(100);
        Move(directions.STARBOARD, 16, 0.5);

        // Moves to the left to align itself up with the stack of cones on the left side of the field

        sleep(100);
        Move(directions.PORT, 3, 0.5);

        // Moves Forwards going towards the stacked cones

        Move(directions.FORWARDS, 26, 0.35);

        // Opens Claws to pick up the cones

        Open_Claws();

        // Lowers the Arm in order to pick up one of the cones from the stack of cones

        Lower(12, 0.8);

        // Close the claws in order to capture/hold a stacked cone

        Close_Claws();

        // Raise the Arm to get the Cone that it picked up off of the stack

        Raise(7, 0.8);

        // Going backwards to re-align itself with the Small Junction while also raising the Arm to give it height
        // to place it on the small junction

        Backwards_Raise(26, 0.5, 4, 0.8);

        // Moves to the Left to play on the Small Junction

        Move(directions.PORT, 13, 0.5);

        // Moves Forward to place the cone over the Small Junction

        Move(directions.FORWARDS, 5, 0.5);

        // Open Claws in order to drop the cone over the Small Junction
        sleep(250);
        Open_Claws();

        // Moves Backwards to get off of the Small Junction while the claws are still open

        Move(directions.BACKWARDS, 5, 0.4);



        /////////////////////////////////////////
        // Picking up Cones for Driver Control //
        /////////////////////////////////////////

        // Moves Right to align itself up with the stack of cones on the left side of the field


        Move(directions.STARBOARD, 13, 0.5);

        // Moves Forward to Pick up a cone from the stack for Driver Control

        Move(directions.FORWARDS, 27, 0.5);

        // Lowers the arm so it can pick up a cone from the stack on the left side of the field

        Lower(10, 0.8);

        // Close Claws to pick up a cone for Driver Control

        Close_Claws();

        // To raise the cone off of the stack.

        Raise(6, 0.8);


        /////////////////////////////////////////////////////////////////////////////
        // The rest is given up above in the code based on which sleeve is detected//
        /////////////////////////////////////////////////////////////////////////////


    }

 }
