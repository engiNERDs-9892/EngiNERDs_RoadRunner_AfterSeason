package org.firstinspires.ftc.teamcode.opmode.Auto.Testing;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.opmode.Subsystem.DriveTrain;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(name = "UnstableTemporal_Test", group = "00-Autonomous", preselectTeleOp = "Enginerds_Control")
public class Camera_With_RR_Test extends LinearOpMode{

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    public DriveTrain driveTrain;

    private DcMotor motorRiseyRise;
    private DcMotor motorSlideySlide;

    Servo servoFAL;
    Servo servoFAR;


    int Left = 11; //Detects april tag id#11 - Attached to sleeve template position one
    int Middle = 12; //Detects april tag id#12 - Attached to sleeve template position two
    int Right = 13; //Detects april tag id#13 - Attached to sleeve template position three

    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode() throws InterruptedException {
        driveTrain = new DriveTrain(hardwareMap);

        motorRiseyRise = hardwareMap.dcMotor.get("motorRiseyRise");
        motorSlideySlide = hardwareMap.dcMotor.get("motorSlideySlide");

        // Servos
        servoFAL = hardwareMap.servo.get("servoFAL");
        servoFAR = hardwareMap.servo.get("servoFAR");

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
        waitForStart();

        //Game Play Button  is pressed
        if (opModeIsActive() && !isStopRequested()) {

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
            telemetry.update();
            Close_Claws();

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
            buildParking1();

            runParking();
        }

        // The Left is Sleeve 11 (QR Code 11)
        else if (tagOfInterest.id == Left) {
        buildParking2();
        runParking();

        }

        // The Middle is Sleeve 12 (QR Code 12)
        else if (tagOfInterest.id == Middle) {

            buildParking3();
            runParking();

        }


        // The third else or in this case Right is Sleeve 13 (QR Code 13)
        else {
            buildParking4();
            runParking();
        }
    }



    // **********  HIGH Level driving functions.  ********************

    // Function to Give telemetry on which QR Code is Detected




    //Initialize any other TrajectorySequences as desired
    TrajectorySequence trajectoryParking ;

    public void buildParking1(){
        Pose2d startPose = new Pose2d(35, 60, Math.toRadians(180));

        driveTrain.setPoseEstimate(startPose);

        driveTrain.trajectorySequenceBuilder(startPose);

        trajectoryParking = driveTrain.trajectorySequenceBuilder(startPose)

                .UNSTABLE_addTemporalMarkerOffset(0,() -> {Close_Claws();})

                //.turn(90)
                //.forward(20)
                //.back(20)
                //.strafeRight(20)
                //.strafeLeft(20)
                .build();
    }

    public void buildParking2(){
        Pose2d startPose = new Pose2d(35, 60, Math.toRadians(180));

        driveTrain.setPoseEstimate(startPose);

        driveTrain.trajectorySequenceBuilder(startPose);

        trajectoryParking = driveTrain.trajectorySequenceBuilder(startPose)

                //.turn(90)
                .forward(20)
                //.back(20)
                //.strafeRight(20)
                //.strafeLeft(20)
                .build();
    }

    public void buildParking3(){
        Pose2d startPose = new Pose2d(35, 60, Math.toRadians(180));

        driveTrain.setPoseEstimate(startPose);

        driveTrain.trajectorySequenceBuilder(startPose);

        trajectoryParking = driveTrain.trajectorySequenceBuilder(startPose)

                //.turn(90)
                //.forward(20)
                .back(20)
                //.strafeRight(20)
                //.strafeLeft(20)
                .build();
    }

    public void buildParking4(){
        Pose2d startPose = new Pose2d(35, 60, Math.toRadians(180));

        driveTrain.setPoseEstimate(startPose);

        driveTrain.trajectorySequenceBuilder(startPose);

        trajectoryParking = driveTrain.trajectorySequenceBuilder(startPose)

                .turn(90)
                //.forward(20)
                //.back(20)
                //.strafeRight(20)
                //.strafeLeft(20)
                .build();
    }

    public void runParking(){

        //Run the trajectory built for Auto and Parking
        driveTrain.followTrajectorySequence(trajectoryParking);

    }




    private void Open_Claws() {
        servoFAL.setPosition(.25);
        servoFAR.setDirection(Servo.Direction.REVERSE);
        servoFAR.setPosition(.25);
    }
    private void Close_Claws() {
        servoFAL.setPosition(0);
        servoFAR.setDirection(Servo.Direction.REVERSE);
        servoFAR.setPosition(0);
    }
    private void Raise(int target, double speed) {
        int up = 360; //Used for the linear slide distance

        // The stop and reset encoders is needed to reset and start the encoders (You need one at the
        // end because it tells the robot where the drive code starts and ends, kinda like brackets)
        motorRiseyRise.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorSlideySlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Gives it a position to run to
        motorRiseyRise.setDirection(DcMotorSimple.Direction.REVERSE);
        motorSlideySlide.setDirection(DcMotorSimple.Direction.FORWARD);


        motorRiseyRise.setTargetPosition(target * up);
        motorRiseyRise.setTargetPosition(target * up);

        // tells it to go to the position that is set
        motorRiseyRise.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorSlideySlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // the motor speed
        motorRiseyRise.setPower(speed);
        motorSlideySlide.setPower(speed);
        // While loop keeps the code running until motors reach the desired position
        while (opModeIsActive() && (motorRiseyRise.isBusy() || motorSlideySlide.isBusy())) {
        }
        motorRiseyRise.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorSlideySlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    private void Lower(int target, double speed) {
        int up = 360; //Used for the linear slide distance

        // The stop and reset encoders is needed to reset and start the encoders (You need one at the
        // end because it tells the robot where the drive code starts and ends, kinda like brackets)
        motorRiseyRise.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorSlideySlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Gives it a position to run to
        motorRiseyRise.setDirection(DcMotorSimple.Direction.FORWARD);
        motorSlideySlide.setDirection(DcMotorSimple.Direction.REVERSE);


        motorRiseyRise.setTargetPosition(target * up);
        motorRiseyRise.setTargetPosition(target * up);

        // tells it to go to the position that is set
        motorRiseyRise.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorSlideySlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // the motor speed
        motorRiseyRise.setPower(speed);
        motorSlideySlide.setPower(speed);
        // While loop keeps the code running until motors reach the desired position
        while (opModeIsActive() && (motorRiseyRise.isBusy() || motorSlideySlide.isBusy())) {
        }
        motorRiseyRise.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorSlideySlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    void tagToTelemetry(AprilTagDetection detection) {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
    }
}

