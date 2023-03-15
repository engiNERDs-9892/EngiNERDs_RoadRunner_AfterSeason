/* Copyright (c) 2022 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

// Doesn't Work how it was intended
package org.firstinspires.ftc.teamcode.opmode.Old_Auto.Gyro_Code;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(name="Auto_Circut_Blue", group="Robot")
@Disabled
public class Auto_Circut_Blue extends LinearOpMode {
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    /* Declare OpMode members. */
    private DcMotor         motorFL  = null;
    private DcMotor         motorFR  = null;
    private DcMotor         motorBL = null;
    private DcMotor         motorBR = null;
    private DcMotor         motorRiseyRise = null;
    private DcMotor         motorSlideySlide= null;

    Servo servoFAL;
    Servo servoFAR;

    int in = 45;
    int up = 90;

    private BNO055IMU       imu         = null;      // Gyro Code

    private double          robotHeading  = 0;
    private double          headingOffset = 0;
    private double          headingError  = 0;

    // These variable are declared here (as class members) so they can be updated in various methods,
    // but still be displayed by sendTelemetry()
    private double  targetHeading  = 0;
    private double  driveSpeed     = 0;
    private double  turnSpeed      = 0;
    private double  motorFL_Speed  = 0;
    private double  motorBL_Speed  = 0;
    private double  motorFR_Speed  = 0;
    private double  motorBR_Speed  = 0;
    private int     motorFL_Target = 0;
    private int     motorBL_Target = 0;
    private int     motorFR_Target = 0;
    private int     motorBR_Target = 0;
    private int     leftTarget     = 0;
    private int     rightTarget    = 0;



    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;   // eg: GoBILDA 312 RPM Yellow Jacket
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    // These constants define the desired driving/control characteristics
    // They can/should be tweaked to suit the specific robot drive train.
    static final double     DRIVE_SPEED             = 0.6;     // Max driving speed for better distance accuracy.
    static final double     TURN_SPEED              = 0.4;     // Max Turn speed to limit turn rate
    static final double     HEADING_THRESHOLD       = 1.0 ;    // How close must the heading get to the target before moving to next step.
                                                               // Requiring more accuracy (a smaller number) will often make the turn take longer to get into the final position.
    // Define the Proportional control coefficient (or GAIN) for "heading control".
    // We define one value when Turning (larger errors), and the other is used when Driving straight (smaller errors).
    // Increase these numbers if the heading does not corrects strongly enough (eg: a heavy robot or using tracks)
    // Decrease these numbers if the heading does not settle on the correct value (eg: very agile robot with omni wheels)
    static final double     P_TURN_GAIN            = 0.02;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_GAIN           = 0.03;     // Larger is more responsive, but also less stable

    //int ID_TAG_OF_INTEREST = 18; // Tag ID 18 from the 36h11 family
    int Left = 11; //Detects april tag id#5 - Attached to sleeve template position one
    int Middle = 12; //Detects april tag id#6 - Attached to sleeve template position one
    int Right = 13; //Detects april tag id#7 - Attached to sleeve template position one
    AprilTagDetection tagOfInterest = null;
///////////////////////////////////////////////////////////////////////////////////////////////////

    @Override
    public void runOpMode() {

        // Initialize the drive system variables.
        motorFR  = hardwareMap.get(DcMotor.class, "motorFR");
        motorFL  = hardwareMap.get(DcMotor.class, "motorFL");
        motorBR  = hardwareMap.get(DcMotor.class, "motorBR");
        motorBL  = hardwareMap.get(DcMotor.class, "motorBL");
        motorRiseyRise = hardwareMap.get(DcMotor.class, "motorRiseyRise");
        motorSlideySlide = hardwareMap.get(DcMotor.class, "motorSlideySlide");

        servoFAL = hardwareMap.servo.get("servoFAL");
        servoFAR = hardwareMap.servo.get("servoFAR");


        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        motorBL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFR.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBR.setDirection(DcMotorSimple.Direction.FORWARD);
        motorRiseyRise.setDirection(DcMotorSimple.Direction.REVERSE);
        motorSlideySlide.setDirection(DcMotorSimple.Direction.FORWARD);


        // define initialization values for IMU, and then initialize it.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit            = BNO055IMU.AngleUnit.DEGREES;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Ensure the robot is stationary.  Reset the encoders and set the motors to BRAKE mode
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRiseyRise.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorSlideySlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        double fx = 578.272;
        double fy = 578.272;
        double cx = 402.145;
        double cy = 221.506;

        // UNITS ARE METERS
        double tagsize = 0.166;

        // Camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam Left"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        // Set the encoders for closed loop speed control, and reset the heading.
        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRiseyRise.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorSlideySlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (!isStarted() && !isStopRequested()) {

            resetHeading();
            // Calls to the Pipline
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    //         QR 11               QR 12              QR 13
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
                    telemetry.addData(">", "Robot Heading = %4.0f", getRawHeading());
                }

                // If NO QR is found Display one of the following Messages in Green Text
                else
                {
                    // If No Tag is seen then display the text below in the telemetry
                    telemetry.addLine("Don't see tag of interest :(");
                    telemetry.addData(">", "Robot Heading = %4.0f", getRawHeading());

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
                        telemetry.addData(">", "Robot Heading = %4.0f", getRawHeading());


                    }
                }
            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");
                telemetry.addData(">", "Robot Heading = %4.0f", getRawHeading());

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                    telemetry.addData(">", "Robot Heading = %4.0f", getRawHeading());
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                    telemetry.addData(">", "Robot Heading = %4.0f", getRawHeading());
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
            telemetry.addData(">", "Robot Heading = %4.0f", getRawHeading());
            telemetry.update();
            Close_Claws();

        }

        // If no QR Code is detected
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.addData(">", "Robot Heading = %4.0f", getRawHeading());
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

        }

        // The Left is Sleeve 11 (QR Code 11)
        else if (tagOfInterest.id == Left) {
            Autonomous();
            Left_Lower(46, .8,15,.8);
        }

        // The Middle is Sleeve 12 (QR Code 12)
        else if (tagOfInterest.id == Middle) {
            Autonomous();
            Left_Lower(16,.8,15,.8);

        }


        // The third else or in this case Right is Sleeve 13 (QR Code 13)
        else {
            Autonomous();
            Right_Lower(16,.8,15,.8);

        }
    }



    // **********  HIGH Level driving functions.  ********************

    // Function to Give telemetry on which QR Code is Detected
    void tagToTelemetry(AprilTagDetection detection) {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
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

    public void driveStraight(double speed, double distance, double heading) {

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            int moveCounts = (int)(distance * COUNTS_PER_INCH);
            motorFL_Target = motorFL.getCurrentPosition() + moveCounts;
            motorFR_Target = motorFR.getCurrentPosition() + moveCounts;
            motorBL_Target = motorBL.getCurrentPosition() + moveCounts;
            motorBR_Target = motorBR.getCurrentPosition() + moveCounts;

            motorBL.setDirection(DcMotorSimple.Direction.REVERSE);
            motorFR.setDirection(DcMotorSimple.Direction.FORWARD);
            motorBR.setDirection(DcMotorSimple.Direction.FORWARD);

            // Set Target FIRST, then turn on RUN_TO_POSITION
            motorFL.setTargetPosition(motorFL_Target);
            motorFR.setTargetPosition(motorFR_Target);
            motorBL.setTargetPosition(motorBL_Target);
            motorBR.setTargetPosition(motorBR_Target);

            motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // Set the required driving speed  (must be positive for RUN_TO_POSITION)
            // Start driving straight, and then enter the control loop
            speed = Math.abs(speed);
            moveRobot(speed, 0);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() && (motorFL.isBusy() && motorBR.isBusy())) {

                // Determine required steering to keep on heading
                turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    turnSpeed *= -1.0;

                // Apply the turning correction to the current driving speed.
                moveRobot(driveSpeed, turnSpeed);

                // Display drive status for the driver.
                sendTelemetry(true);
            }

            // Stop all motion & Turn off RUN_TO_POSITION
            moveRobot(0, 0);

            motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void turnToHeading(double turnspeed, double heading) {

        motorBL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFR.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBR.setDirection(DcMotorSimple.Direction.FORWARD);

        // Run getSteeringCorrection() once to pre-calculate the current error
        getSteeringCorrection(heading, P_DRIVE_GAIN);

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && (Math.abs(headingError) > HEADING_THRESHOLD)) {

            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -turnspeed, turnspeed);

            // Pivot in place by applying the turning correction
            moveRobot(0, turnSpeed);

            // Display drive status for the driver.
            sendTelemetry(false);
        }

        // Stop all motion;
        moveRobot(0, 0);
    }


    public void holdHeading(double maxTurnSpeed, double heading, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();

        // keep looping while we have time remaining.
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            moveRobot(0, turnSpeed);

            // Display drive status for the driver.
            sendTelemetry(false);
        }

        // Stop all motion;
        moveRobot(0, 0);
    }


    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        targetHeading = desiredHeading;  // Save for telemetry

        // Get the robot heading by applying an offset to the IMU heading
        robotHeading = getRawHeading() - headingOffset;

        // Determine the heading current error
        headingError = targetHeading - robotHeading;

        // Normalize the error to be within +/- 180 degrees
        while (headingError > 180)  headingError -= 360;
        while (headingError <= -180) headingError += 360;

        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
        return Range.clip(headingError * proportionalGain, -1, 1);
    }


    public void moveRobot(double drive, double turn) {
        driveSpeed = drive;     // save this value as a class member so it can be used by telemetry.
        turnSpeed  = turn;      // save this value as a class member so it can be used by telemetry.


        motorFL_Speed = drive + turn;
        motorBL_Speed = drive - turn;
        motorFR_Speed = drive + turn;
        motorBR_Speed = drive + turn;

        // Scale speeds down if either one exceeds +/- 1.0;
        double max = Math.max(Math.abs(motorFL_Speed), Math.abs(motorFR_Speed));
        double max2 = Math.max(Math.abs(motorBL_Speed), Math.abs(motorBR_Speed));
        if (max > 1.0)
        {
            motorFL_Speed /= max;
            motorFR_Speed /= max;
        }

        if (max2 > 1.0)
        {
            motorBL_Speed /= max;
            motorBR_Speed /= max;
        }

        motorFL.setPower(motorFL_Speed);
        motorFR.setPower(motorFR_Speed);
        motorBL.setPower(motorBL_Speed);
        motorBR.setPower(motorBR_Speed);
    }

    /**
     *  Display the various control parameters while driving
     *
     * @param straight  Set to true if we are driving straight, and the encoder positions should be included in the telemetry.
     */
    private void sendTelemetry(boolean straight) {

        if (straight) {
            telemetry.addData("Motion", "Drive Straight");
            telemetry.addData("Target Pos (Motors)",  "%7d:%7d:%7d:%7d",      motorFL_Target,  motorFR_Target, motorBL_Target, motorBR_Target);
            telemetry.addData("Actual Pos (Motors)",  "%7d:%7d:%7d:%7d",      motorFL.getCurrentPosition(), motorFR.getCurrentPosition(), motorBL.getCurrentPosition(), motorBR.getCurrentPosition());
        }

        else {
            telemetry.addData("Motion", "Turning");
        }

        telemetry.addData("Angle Target:Current", "%5.2f:%5.0f", targetHeading, robotHeading);
        telemetry.addData("Error:Steer",  "%5.1f:%5.1f", headingError, turnSpeed);
        telemetry.addData("Wheel Speeds", "%5.2f : %5.2f", motorFL_Speed, motorFR_Speed, motorBL_Speed, motorBR_Speed);
        telemetry.update();
    }

    public double getRawHeading() {
        Orientation angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    public void resetHeading() {
        // Save a new heading offset equal to the current raw heading.
        headingOffset = getRawHeading();
        robotHeading = 0;
    }

    private void Autonomous(){
        resetHeading();

        ///////////////////////////////
        // Cone #1 or Pre-Loaded Cone//
        ///////////////////////////////

        // Moves LEFT to align itself for the Small Junction closest to the SubStation (Where Human player places cones)
        Right_Raise(17,.8,3,.8);

        // Moves FORWARDS to Place the cone on the Small Junction
        Move(directions.FORWARDS,5,.8);

        // Drop Cone #1 on the Small Junction
        Open_Claws();

        // Move backwards to get off of the Small Junction
        Move(directions.BACKWARDS,5,.8);

        /////////////////////////////
        // Cone #2 or Stack Cone #1//
        /////////////////////////////

        // Lower Linear Slides while aligning the robot with the stack of cones
        Right_Lower(30,.8,1,.8);

        // Rotate 180 degrees so the robot is facing the cones
        driveStraight(0,0,0);
        turnToHeading( TURN_SPEED, 180.0);
        holdHeading( TURN_SPEED, 180.0, 0.5);

        // Drives FORWARDS to pick up a cone from the stack
        Move(directions.FORWARDS,24,.8);

        // Grabs the 1st stacked cone
        Close_Claws();

        // Raises the Linear Slides so it doesn't knock over the stack
        Raise(2,.8);

        // Moves BACKWARDS while Raising the Linear Slide to align itself with the High Junction Closest to the Substation (Where Human Player places cones)
        Backwards_Raise(60,.8,15,.8);

        // Rotates 90 / (270) Degrees so the robot is facing the High Junction
        driveStraight(0,0,180.0);
        turnToHeading(TURN_SPEED,270.0);
        holdHeading(TURN_SPEED,270.0,0.1);

        // Moves FORWARDS to place the cone on the High Junction
        Move(directions.FORWARDS,5,.8);

        // Drops Cone #2 on the High Junction
        Open_Claws();

        // Moves BACKWARDS to get off of the High Junction
        Move(directions.BACKWARDS,5,.8);

        /////////////////////////////
        // Cone #3 or Stack Cone #2//
        /////////////////////////////

        // Rotates 90 / (180) Degrees so the robot is facing the stack of cones
        driveStraight(0,0,270.0);
        turnToHeading(TURN_SPEED,180.0);
        holdHeading(TURN_SPEED,180.0,0.1);

        // Move FORWARDS while lowering Linear Slides to pick up a cone from the stack
        Forwards_Lower(60,.8,15,.8);

        // Grabs a cone from the stack
        Close_Claws();

        // Raises the Linear Slides so it doesn't knock over the stack
        Raise(2,.8);

        // Move FORWARDS to align itself with the Small Junction closest to the stack of cones
        Move(directions.BACKWARDS,4,.8);

        // Rotates 125 / (325) to have the robot face the Small Junction
        driveStraight(0,0,180.0);
        turnToHeading(TURN_SPEED,325.0);
        holdHeading(TURN_SPEED,325.0,0.1);

        // Move FORWARDS to place the cone on the Small Junction
        Move(directions.FORWARDS,12,.8);

        // Drops Cone #3 on Small Junction
        Open_Claws();

        // Moves Backwards to get off of the Small Junction and Align the robot with the cones
        Backwards_Lower(12,.8,1,.4);

        /////////////////////////////
        // Cone #4 or Stack Cone #3//
        /////////////////////////////

        // Rotate 125 / (180) Degrees so the robot is facing the stack of cones
        driveStraight(0,0,325.0);
        turnToHeading(turnSpeed,180.0);
        holdHeading(turnSpeed,180.0,0.1);

        // Move FORWARDS to pick up a cone from the stack
        Move(directions.FORWARDS,2,.8);

        // Grab a cone from the stack
        Close_Claws();

        // Raise the Linear Slides so we don't knock the stack of cones over
        Raise(1,0.8);

        // Move BACKWARDS to align the robot with the High Junction that is closest to the Horizontal Middle Line on the Left Side
        Backwards_Raise(46,.8,15,0.8);

        // Rotate 90 / (90) Degrees so the robot is facing the High Junction
        driveStraight(0,0,180.0);
        turnToHeading(turnSpeed,90.0);
        holdHeading(turnSpeed,90.0,0.1);

        // Move FORWARDS to place the cone on the High Junction
        Move(directions.FORWARDS,5,.8);

        // Drop Cone #4 on the High Junction
        Open_Claws();

        // Back up to align the robot for parking
        Move(directions.BACKWARDS,5,0);
    }


    private void Left_Raise(int moveTarget, double moveSpeed, int clawTarget, double clawspeed) {

        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRiseyRise.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorSlideySlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFL.setDirection(DcMotorSimple.Direction.FORWARD);
        motorFR.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBR.setDirection(DcMotorSimple.Direction.REVERSE);
        motorRiseyRise.setDirection(DcMotorSimple.Direction.FORWARD);
        motorSlideySlide.setDirection(DcMotorSimple.Direction.REVERSE);

        motorFL.setTargetPosition(moveTarget * in);
        motorFR.setTargetPosition(moveTarget * in);
        motorBL.setTargetPosition(moveTarget * in);
        motorBR.setTargetPosition(moveTarget * in);
        motorRiseyRise.setTargetPosition(clawTarget*up);
        motorSlideySlide.setTargetPosition(clawTarget*up);

        motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRiseyRise.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorSlideySlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorFL.setPower(moveSpeed);
        motorFR.setPower(moveSpeed);
        motorBL.setPower(moveSpeed);
        motorBR.setPower(moveSpeed);
        motorSlideySlide.setPower(clawspeed);
        motorRiseyRise.setPower(clawspeed);
        while (opModeIsActive() &&  motorFL.isBusy() || motorSlideySlide.isBusy()){}


        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRiseyRise.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorSlideySlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    private void Right_Raise(int moveTarget, double moveSpeed, int clawTarget, double clawspeed) {

        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRiseyRise.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorSlideySlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFR.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBL.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBR.setDirection(DcMotorSimple.Direction.FORWARD);
        motorRiseyRise.setDirection(DcMotorSimple.Direction.FORWARD);
        motorSlideySlide.setDirection(DcMotorSimple.Direction.REVERSE);

        motorFL.setTargetPosition(moveTarget * in);
        motorFR.setTargetPosition(moveTarget * in);
        motorBL.setTargetPosition(moveTarget * in);
        motorBR.setTargetPosition(moveTarget * in);
        motorRiseyRise.setTargetPosition(clawTarget*up);
        motorSlideySlide.setTargetPosition(clawTarget*up);

        motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRiseyRise.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorSlideySlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorFL.setPower(moveSpeed);
        motorFR.setPower(moveSpeed);
        motorBL.setPower(moveSpeed);
        motorBR.setPower(moveSpeed);
        motorSlideySlide.setPower(clawspeed);
        motorRiseyRise.setPower(clawspeed);
        while (opModeIsActive() &&  motorFL.isBusy() || motorSlideySlide.isBusy()){}


        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRiseyRise.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorSlideySlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    private void Forwards_Raise(int moveTarget, double moveSpeed, int clawTarget, double clawspeed) {

        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRiseyRise.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorSlideySlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFR.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBR.setDirection(DcMotorSimple.Direction.FORWARD);
        motorRiseyRise.setDirection(DcMotorSimple.Direction.FORWARD);
        motorSlideySlide.setDirection(DcMotorSimple.Direction.REVERSE);

        motorFL.setTargetPosition(moveTarget * in);
        motorFR.setTargetPosition(moveTarget * in);
        motorBL.setTargetPosition(moveTarget * in);
        motorBR.setTargetPosition(moveTarget * in);
        motorRiseyRise.setTargetPosition(clawTarget*up);
        motorSlideySlide.setTargetPosition(clawTarget*up);

        motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRiseyRise.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorSlideySlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorFL.setPower(moveSpeed);
        motorFR.setPower(moveSpeed);
        motorBL.setPower(moveSpeed);
        motorBR.setPower(moveSpeed);
        motorSlideySlide.setPower(clawspeed);
        motorRiseyRise.setPower(clawspeed);
        while (opModeIsActive() &&  motorFL.isBusy() || motorSlideySlide.isBusy()){}


        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRiseyRise.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorSlideySlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    private void Backwards_Raise(int moveTarget, double moveSpeed, int clawTarget, double clawspeed) {

        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRiseyRise.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorSlideySlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFL.setDirection(DcMotorSimple.Direction.FORWARD);
        motorFR.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBL.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBR.setDirection(DcMotorSimple.Direction.REVERSE);
        motorRiseyRise.setDirection(DcMotorSimple.Direction.FORWARD);
        motorSlideySlide.setDirection(DcMotorSimple.Direction.REVERSE);

        motorFL.setTargetPosition(moveTarget * in);
        motorFR.setTargetPosition(moveTarget * in);
        motorBL.setTargetPosition(moveTarget * in);
        motorBR.setTargetPosition(moveTarget * in);
        motorRiseyRise.setTargetPosition(clawTarget*up);
        motorSlideySlide.setTargetPosition(clawTarget*up);

        motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRiseyRise.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorSlideySlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorFL.setPower(moveSpeed);
        motorFR.setPower(moveSpeed);
        motorBL.setPower(moveSpeed);
        motorBR.setPower(moveSpeed);
        motorSlideySlide.setPower(clawspeed);
        motorRiseyRise.setPower(clawspeed);
        while (opModeIsActive() &&  motorFL.isBusy() || motorSlideySlide.isBusy()){}


        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRiseyRise.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorSlideySlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    private void Left_Lower(int moveTarget, double moveSpeed, int clawTarget, double clawspeed) {

        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRiseyRise.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorSlideySlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFL.setDirection(DcMotorSimple.Direction.FORWARD);
        motorFR.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBR.setDirection(DcMotorSimple.Direction.REVERSE);
        motorRiseyRise.setDirection(DcMotorSimple.Direction.REVERSE);
        motorSlideySlide.setDirection(DcMotorSimple.Direction.FORWARD);

        motorFL.setTargetPosition(moveTarget * in);
        motorFR.setTargetPosition(moveTarget * in);
        motorBL.setTargetPosition(moveTarget * in);
        motorBR.setTargetPosition(moveTarget * in);
        motorRiseyRise.setTargetPosition(clawTarget*up);
        motorSlideySlide.setTargetPosition(clawTarget*up);

        motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRiseyRise.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorSlideySlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorFL.setPower(moveSpeed);
        motorFR.setPower(moveSpeed);
        motorBL.setPower(moveSpeed);
        motorBR.setPower(moveSpeed);
        motorSlideySlide.setPower(clawspeed);
        motorRiseyRise.setPower(clawspeed);
        while (opModeIsActive() &&  motorFL.isBusy() || motorSlideySlide.isBusy()){}


        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRiseyRise.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorSlideySlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    private void Right_Lower(int moveTarget, double moveSpeed, int clawTarget, double clawspeed) {

        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRiseyRise.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorSlideySlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFR.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBL.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBR.setDirection(DcMotorSimple.Direction.FORWARD);
        motorRiseyRise.setDirection(DcMotorSimple.Direction.REVERSE);
        motorSlideySlide.setDirection(DcMotorSimple.Direction.FORWARD);

        motorFL.setTargetPosition(moveTarget * in);
        motorFR.setTargetPosition(moveTarget * in);
        motorBL.setTargetPosition(moveTarget * in);
        motorBR.setTargetPosition(moveTarget * in);
        motorRiseyRise.setTargetPosition(clawTarget*up);
        motorSlideySlide.setTargetPosition(clawTarget*up);

        motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRiseyRise.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorSlideySlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorFL.setPower(moveSpeed);
        motorFR.setPower(moveSpeed);
        motorBL.setPower(moveSpeed);
        motorBR.setPower(moveSpeed);
        motorSlideySlide.setPower(clawspeed);
        motorRiseyRise.setPower(clawspeed);
        while (opModeIsActive() &&  motorFL.isBusy() || motorSlideySlide.isBusy()){}


        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRiseyRise.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorSlideySlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    private void Forwards_Lower(int moveTarget, double moveSpeed, int clawTarget, double clawspeed) {

        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRiseyRise.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorSlideySlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFR.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBR.setDirection(DcMotorSimple.Direction.FORWARD);
        motorRiseyRise.setDirection(DcMotorSimple.Direction.REVERSE);
        motorSlideySlide.setDirection(DcMotorSimple.Direction.FORWARD);

        motorFL.setTargetPosition(moveTarget * in);
        motorFR.setTargetPosition(moveTarget * in);
        motorBL.setTargetPosition(moveTarget * in);
        motorBR.setTargetPosition(moveTarget * in);
        motorRiseyRise.setTargetPosition(clawTarget*up);
        motorSlideySlide.setTargetPosition(clawTarget*up);

        motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRiseyRise.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorSlideySlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorFL.setPower(moveSpeed);
        motorFR.setPower(moveSpeed);
        motorBL.setPower(moveSpeed);
        motorBR.setPower(moveSpeed);
        motorSlideySlide.setPower(clawspeed);
        motorRiseyRise.setPower(clawspeed);
        while (opModeIsActive() &&  motorFL.isBusy() || motorSlideySlide.isBusy()){}


        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRiseyRise.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorSlideySlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    private void Backwards_Lower(int moveTarget, double moveSpeed, int clawTarget, double clawspeed) {

        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRiseyRise.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorSlideySlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFL.setDirection(DcMotorSimple.Direction.FORWARD);
        motorFR.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBL.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBR.setDirection(DcMotorSimple.Direction.REVERSE);
        motorRiseyRise.setDirection(DcMotorSimple.Direction.REVERSE);
        motorSlideySlide.setDirection(DcMotorSimple.Direction.FORWARD);

        motorFL.setTargetPosition(moveTarget * in);
        motorFR.setTargetPosition(moveTarget * in);
        motorBL.setTargetPosition(moveTarget * in);
        motorBR.setTargetPosition(moveTarget * in);
        motorRiseyRise.setTargetPosition(clawTarget*up);
        motorSlideySlide.setTargetPosition(clawTarget*up);

        motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRiseyRise.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorSlideySlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorFL.setPower(moveSpeed);
        motorFR.setPower(moveSpeed);
        motorBL.setPower(moveSpeed);
        motorBR.setPower(moveSpeed);
        motorSlideySlide.setPower(clawspeed);
        motorRiseyRise.setPower(clawspeed);
        while (opModeIsActive() &&  motorFL.isBusy() || motorSlideySlide.isBusy()){}


        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRiseyRise.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorSlideySlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    private void Raise (int clawTarget, double clawspeed){
        motorRiseyRise.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorSlideySlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorRiseyRise.setDirection(DcMotorSimple.Direction.FORWARD);
        motorSlideySlide.setDirection(DcMotorSimple.Direction.REVERSE);

        motorRiseyRise.setTargetPosition(clawTarget * up);
        motorSlideySlide.setTargetPosition(clawTarget * up);

        motorRiseyRise.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorSlideySlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorRiseyRise.setPower(clawspeed);
        motorSlideySlide.setPower(clawspeed);

        while (opModeIsActive() && motorRiseyRise.isBusy() || motorSlideySlide.isBusy()){}

        motorRiseyRise.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorSlideySlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }
    private void Lower (int clawTarget, double clawspeed){
        motorRiseyRise.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorSlideySlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorRiseyRise.setDirection(DcMotorSimple.Direction.REVERSE);
        motorSlideySlide.setDirection(DcMotorSimple.Direction.FORWARD);

        motorRiseyRise.setTargetPosition(clawTarget * up);
        motorSlideySlide.setTargetPosition(clawTarget * up);

        motorRiseyRise.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorSlideySlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorRiseyRise.setPower(clawspeed);
        motorSlideySlide.setPower(clawspeed);

        while (opModeIsActive() && motorRiseyRise.isBusy() || motorSlideySlide.isBusy()){}

        motorRiseyRise.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorSlideySlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    private void Move(directions direction, int target, double speed) {
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        // This sets the direction for the motor for the wheels to drive forward
        if (direction == directions.FORWARDS) {
            motorFL.setDirection(DcMotorSimple.Direction.REVERSE);
            motorFR.setDirection(DcMotorSimple.Direction.FORWARD);
            motorBL.setDirection(DcMotorSimple.Direction.REVERSE);
            motorBR.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        // Sets the motor direction to move Backwards
        else if (direction == directions.BACKWARDS) {
            motorFL.setDirection(DcMotorSimple.Direction.FORWARD);
            motorFR.setDirection(DcMotorSimple.Direction.REVERSE);
            motorBL.setDirection(DcMotorSimple.Direction.FORWARD);
            motorBR.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        // Sets the motor direction to move to the Left ( Note * Port = Left)
        else if (direction == directions.LEFT) {
            motorFL.setDirection(DcMotorSimple.Direction.FORWARD);
            motorFR.setDirection(DcMotorSimple.Direction.FORWARD);
            motorBL.setDirection(DcMotorSimple.Direction.REVERSE);
            motorBR.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        // Sets the motor direction to move to the Right (Note * Starboard = Right)
        else if (direction == directions.RIGHT) {
            motorFL.setDirection(DcMotorSimple.Direction.REVERSE);
            motorFR.setDirection(DcMotorSimple.Direction.REVERSE);
            motorBL.setDirection(DcMotorSimple.Direction.FORWARD);
            motorBR.setDirection(DcMotorSimple.Direction.FORWARD);
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
        while (opModeIsActive() && ((motorFL.isBusy() || motorFR.isBusy()))) {
        }
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }
    enum directions {
        FORWARDS,
        BACKWARDS,
        LEFT,
        RIGHT,
    }
}
