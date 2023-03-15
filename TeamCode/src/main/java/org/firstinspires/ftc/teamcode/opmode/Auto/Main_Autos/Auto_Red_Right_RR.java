package org.firstinspires.ftc.teamcode.opmode.Auto.Main_Autos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.opmode.Auto.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.opmode.RoadRunner.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
@Autonomous
public class Auto_Red_Right_RR extends LinearOpMode {

    SampleMecanumDrive drivetrain = new SampleMecanumDrive(hardwareMap);

    Servo servoFAL;
    Servo servoFAR;

    private DcMotor motorRiseyRise;
    private DcMotor motorSlideySlide;


    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

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

    @Override
    public void runOpMode(){
        SampleMecanumDrive drivetrain = new SampleMecanumDrive(hardwareMap);
        motorRiseyRise = hardwareMap.dcMotor.get("motorRiseyRise");
        motorSlideySlide = hardwareMap.dcMotor.get("motorSlideySlide");

        // Servos
        servoFAL = hardwareMap.servo.get("servoFAL");
        servoFAR = hardwareMap.servo.get("servoFAR");

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


        motorRiseyRise.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorSlideySlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Close_Claws();
        waitForStart();

        ///////////////////////////////////////
        // How to write code for a trajectory//
        ///////////////////////////////////////
        //
        // Trajectory ______ = drivetrain.trajectoryBuilder(new Pose2d (Position/Heading))
        //             Name
        //
        //          .______(x)    <---- Whatever direction plus distance OR cordinates to go to for x
        //          .build();
        //      drivetrain.followTrajectory(___________);
        //                                name from up above
        ////////////////////////////////////////////////////////////////////////////
        // Splines use 90 degrees in raidians for the 3rd 0 as a heads up  /////////
        ////////////////////////////////////////////////////////////////////////////






        if (tagOfInterest == null) {
           Autonomous();

        }

        // The Left is Sleeve 10 (QR Code 10)
        else if (tagOfInterest.id == Left ) {
            Autonomous();

            Trajectory Forward = drivetrain.trajectoryBuilder(new Pose2d())
                    .back(24)
                    .build();
            drivetrain.followTrajectory(Forward);



        }

        // The Middle is Sleeve 20 (QR Code 20)
        else if (tagOfInterest.id == Middle){
            Autonomous();
        }


        // The third else or in this case Right is Sleeve 30 (QR Code 30)
        else {
            Autonomous();

            Trajectory Backward = drivetrain.trajectoryBuilder(new Pose2d())
                    .forward(24)
                    .build();
            drivetrain.followTrajectory(Backward);

        }
    }

        private void Autonomous() {
            Pose2d startPose = new Pose2d(35, -60, Math.toRadians(180));

            drivetrain.setPoseEstimate(startPose);

            drivetrain.trajectorySequenceBuilder(startPose);

            TrajectorySequence Auto_Red_Right = drivetrain.trajectorySequenceBuilder(startPose)

                    /////////////////////////////
                    // Step 1 (Pre-Loaded Cone)//
                    /////////////////////////////

                    // Strafe to the right in order to align the robot up with the High Junction
                    .strafeRight(45)

                    // Drive over to the High Junction while raising the LS
                    .lineToLinearHeading(new Pose2d(25, -2.5,Math.toRadians(115)))

                    // Raise the LS while the robot is driving
                    .UNSTABLE_addTemporalMarkerOffset(-3, () -> {Raise(20,.8);})

                    // Drop Cone #1 (Pre-Loaded Cone)
                    .addTemporalMarker(() -> Open_Claws())

                    // Drive Backwards to get off of the High Junction
                    .lineToLinearHeading(new Pose2d(35.5, -12,Math.toRadians(180)))

                    // Rotate to face the Stack of Cones
                    .turn(Math.toRadians(180))


                    //////////////////////////////////////
                    // Step 2 (Cone #2 or Stack cone #1)//
                    //////////////////////////////////////

                    // Drive Forwards to pick up from the Stack of Cones while lowering the LS
                    .forward(29)

                    // Lowers the LS system
                    .UNSTABLE_addTemporalMarkerOffset(-3, () -> {Raise(20,.8);})

                    // Grabs a Cone from the stack
                    .addTemporalMarker(() -> Close_Claws())

                    // Wait so we don't knock over the Cones
                    .waitSeconds(.5)

                    // Raise the LS so we don't knock over the Cones
                    .UNSTABLE_addTemporalMarkerOffset(-3,() -> {Raise(5,1);})

                    // Drive Backwards to a spot where the robot can rotate freely
                    .back(28.5)

                    // Rotate the robot to face the High Junction
                    .turn(Math.toRadians(90))

                    // Drive over to the High Junction while raising the LS
                    .lineToLinearHeading(new Pose2d(23.5,-6,Math.toRadians(90)))

                    // Raise the LS
                    .UNSTABLE_addTemporalMarkerOffset(-3,() -> {Raise(20,.8);})

                    // Drop Cone #2 (Stack Cone #1)
                    .addTemporalMarker(() -> Open_Claws())

                    // Drive Back to a spot where the robot can rotate freely
                    .lineToLinearHeading(new Pose2d(32.5,-12,Math.toRadians(90)))

                    // Rotate the robot to face the stack of cones
                    .turn(Math.toRadians(-90))


                    //////////////////////////////////////
                    // Step 3 (Cone #3 or Stack cone #2)//
                    //////////////////////////////////////

                    // Drive Forwards to pick up from the Stack of Cones while lowering the LS
                    .forward(29)

                    // Lowers the LS system
                    .UNSTABLE_addTemporalMarkerOffset(-3, () -> {Raise(20,.8);})

                    // Grabs a Cone from the stack
                    .addTemporalMarker(() -> Close_Claws())

                    // Wait so we don't knock over the Cones
                    .waitSeconds(.5)

                    // Raise the LS so we don't knock over the Cones
                    .UNSTABLE_addTemporalMarkerOffset(-3,() -> {Raise(5,1);})

                    // Drive Backwards to a spot where the robot can rotate freely
                    .back(28.5)

                    // Rotate the robot to face the High Junction
                    .turn(Math.toRadians(90))

                    // Drive over to the High Junction while raising the LS
                    .lineToLinearHeading(new Pose2d(23.5,-6,Math.toRadians(90)))

                    // Raise the LS
                    .UNSTABLE_addTemporalMarkerOffset(-3,() -> {Raise(20,.8);})

                    // Drop Cone #3 (Stack Cone #2)
                    .addTemporalMarker(() -> Open_Claws())

                    // Drive Back to a spot where the robot can rotate freely
                    .lineToLinearHeading(new Pose2d(32.5,-12,Math.toRadians(90)))

                    // Rotate the robot to face the stack of cones
                    .turn(Math.toRadians(-90))

                    //////////////////////////////////////
                    // Step 4 (Cone #4 or Stack cone #3)//
                    //////////////////////////////////////

                    // Drive Forwards to pick up from the Stack of Cones while lowering the LS
                    .forward(29)

                    // Lowers the LS system
                    .UNSTABLE_addTemporalMarkerOffset(-3, () -> {Raise(20,.8);})

                    // Grabs a Cone from the stack
                    .addTemporalMarker(() -> Close_Claws())

                    // Wait so we don't knock over the Cones
                    .waitSeconds(.5)

                    // Raise the LS so we don't knock over the Cones
                    .UNSTABLE_addTemporalMarkerOffset(-3,() -> {Raise(5,1);})

                    // Drive Backwards to a spot where the robot can rotate freely
                    .back(28.5)

                    // Rotate the robot to face the High Junction
                    .turn(Math.toRadians(90))

                    // Drive over to the High Junction while raising the LS
                    .lineToLinearHeading(new Pose2d(23.5,-6,Math.toRadians(90)))

                    // Raise the LS
                    .UNSTABLE_addTemporalMarkerOffset(-3,() -> {Raise(20,.8);})

                    // Drop Cone #4 (Stack Cone #3)
                    .addTemporalMarker(() -> Open_Claws())

                    // Move Backwards to Park
                    .back(6)

                    .build();

            drivetrain.followTrajectorySequence(Auto_Red_Right);
        }

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

    }


