package org.firstinspires.ftc.teamcode.opmode.Old_Auto.OpenCV_Code;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name="Auto_Parking",group="used")
@Disabled
public class Auto_Parking extends LinearOpMode {

//Declare Motors and Devices needed to run the code.

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

    //Webcam
    OpenCvWebcam webcam;
    SkystoneDeterminationExample.SkystoneDeterminationPipeline pipeline;
    SkystoneDeterminationExample.SkystoneDeterminationPipeline.SkystonePosition snapshotAnalysis = SkystoneDeterminationExample.SkystoneDeterminationPipeline.SkystonePosition.LEFT; // default


    //GLOBAL VARIABLES
    // The int ____ = ______; are used to identify variables used through out the code.
    //         Name   amount

    /* Motor Tick Count
            Andy Mark 3.7:1 = 103.6
            Andy Mark 20:1  = 537.6
            Andy Mark 60:1  = 1680

        To Calculate: # ticks per revolution / Distance of Revolution =
     */

    int in = 45; //Used for the wheels to drive: = 537.6 / (pi * 96 mm) = 537.6 / (pi * 3.77953 in)
    int up = 360; //Used for the linear slide distance

    @Override
    public void runOpMode() {

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
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new SkystoneDeterminationExample.SkystoneDeterminationPipeline();
        webcam.setPipeline(pipeline);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                // This is in what viewing window the camera is seeing through and it doesn't matter
                // what orientation it is | UPRIGHT, SIDEWAYS_LEFT, SIDEWAYS_RIGHT, etc.
                webcam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });


        // Close claws when initialized
        Close_Claws();
        //servoFAL.setPosition(0);
        //servoFAR.setDirection(Servo.Direction.REVERSE);
        //servoFAR.setPosition(0);

        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("Realtime analysis", pipeline.getAnalysis());
            telemetry.update();
            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
        }

        snapshotAnalysis = pipeline.getAnalysis();

        telemetry.addData("Snapshot post-START analysis", snapshotAnalysis);
        telemetry.update();


        telemetry.addData("Status", "\uD83C\uDD97");

        telemetry.clear();
        telemetry.update();


        waitForStart();

        /////////////////////////////////////////////////////////////////////////////////////////
        ///////////////////////////////////   CODE   ///////////////////////////////////////////
        ///////////////////////////////////////////////////////////////////////////////////////

        // Plan

        // Before start, the claw is pre-loaded, and its left side
        // is along the wall. The robot faces *away* from the drop station,
        // and the claw is above the left edge of the "center"
        // tile. The camera is mounted to scan the sleeve from this
        // starting position.

        // 1. Identify the sleeve
        // 2. Park depending on sleeve


        //Runs 1 of 3 codes based on the Sleeve
        switch (snapshotAnalysis) {
            case LEFT: // Sleeve 1
            {
                Move(directions.STARBOARD, 32, .6);
                Move(directions.PORT,3,.6);
                Move(directions.FORWARDS,23,.6);
                break;
            }


            case CENTER: // Sleeve 2
            {

                // Moves to the Right just so it parks in the Middle Parking space (Numbers Might be off a tad)

                Move(directions.STARBOARD, 57, .6);


                break;


            }

            case RIGHT: // Sleeve 3
            {

                Move(directions.STARBOARD, 57, .6);
                Move(directions.BACKWARDS,23,.6);
                break;
            }


        }
    }


    ////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////                   Functions                         ///////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////


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



        // Moves to the the right far enough to move the signal sleeve cone out of the way so we don't get
        // caught on it

        Right_Raise(60, 0.75, 15, 0.8);

        // Moves to the left to line up so the robot is in alignment with the Small Junction

        Move(directions.PORT, 16, .6);

        // Moves forward so the cone is over the Small Junction

        Move(directions.FORWARDS,4, 0.6);

        // Lowers the cone over the junction so it is stable when dropping the cone

        Lower(3, 0.6);

        // Opens the claws to drop the cone

        Open_Claws();

        // Moves Backwards to get off of the Small Junction while the claws are still open

        Move(directions.BACKWARDS, 4, 0.7);

        // Closes the claws since there is no cone for stability
        Close_Claws();


        ///////////////////////////
        // Stack Cones or Cone #2//
        ///////////////////////////

        // Moves right to align itself up with the stack of cones on the left side of the field


        Move(directions.STARBOARD, 12, 0.6);

        // Moves Forwards going towards the stacked cones

        Move(directions.FORWARDS, 25, 0.55);

        // Opens Claws to pick up the cones

        Open_Claws();

        // Lowers the Arm in order to pick up one of the cones from the stack of cones

        Lower(9, 0.8);

        // Close the claws in order to capture/hold a stacked cone

        Close_Claws();

        // Raise the Arm to get the Cone that it picked up off of the stack

        Raise(7, 0.8);

        // Going backwards to re-align itself with the Small Junction while also raising the Arm to give it height
        // to place it on the small junction

        Backwards_Raise(26, 0.6, 4, 0.8);

        // Moves to the Left to play on the Small Junction

        Move(directions.PORT, 14, 0.6);

        // Moves Forward to place the cone over the Small Junction

        Move(directions.FORWARDS, 6, 0.6);

        Lower(3,.8);

        // Open Claws in order to drop the cone over the Small Junction

        Open_Claws();

        // Moves Backwards to get off of the Small Junction while the claws are still open

        Move(directions.BACKWARDS, 5, 0.4);



        /////////////////////////////////////////
        // Picking up Cones for Driver Control //
        /////////////////////////////////////////

        // Moves Right to align itself up with the stack of cones on the left side of the field

        Move(directions.STARBOARD, 12, 0.6);

        // Moves Forward to Pick up a cone from the stack for Driver Control

        Move(directions.FORWARDS, 27, 0.6);

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

//Closes Linear Op Mode
}




