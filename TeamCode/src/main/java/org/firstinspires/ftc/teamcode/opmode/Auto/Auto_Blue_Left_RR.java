package org.firstinspires.ftc.teamcode.opmode.Auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.opmode.Subsystem.DriveTrain;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "Auto_Blue_Left_RR", group = "00-Autonomous", preselectTeleOp = "Enginerds_Control")
public class Auto_Blue_Left_RR extends LinearOpMode{

    public DriveTrain driveTrain;

    private DcMotor motorRiseyRise;
    private DcMotor motorSlideySlide;

    Servo servoFAL;
    Servo servoFAR;




    @Override
    public void runOpMode() throws InterruptedException {
        driveTrain = new DriveTrain(hardwareMap);

        motorRiseyRise = hardwareMap.dcMotor.get("motorRiseyRise");
        motorSlideySlide = hardwareMap.dcMotor.get("motorSlideySlide");

        // Servos
        servoFAL = hardwareMap.servo.get("servoFAL");
        servoFAR = hardwareMap.servo.get("servoFAR");
        waitForStart();

        //Game Play Button  is pressed
        if (opModeIsActive() && !isStopRequested()) {

            //Build parking trajectory based on last detected target by vision
            buildParking();

            //run Autonomous trajectory
            runParking();
        }
    }

    //Initialize any other TrajectorySequences as desired
    TrajectorySequence trajectoryParking ;

    public void buildParking(){
        Pose2d startPose = new Pose2d(35, 60, Math.toRadians(180));

        driveTrain.setPoseEstimate(startPose);

        driveTrain.trajectorySequenceBuilder(startPose);

        trajectoryParking = driveTrain.trajectorySequenceBuilder(startPose)


                ////////////////////////////////////////
                // Step 1 (Cone #1 or Pre-Loaded Cone)//
                ////////////////////////////////////////

                // Strafe to the Left to align the robot up for the High Junction
                .strafeLeft(40)

                // Drive to the High Junction while raising the LS
                .lineToLinearHeading(new Pose2d(25, 2.5,Math.toRadians(245)))

                // Raises the LS
                .UNSTABLE_addTemporalMarkerOffset(-3,() -> {Raise(20,.8);})

                // Drops Cone #1 (Pre-Loaded Cone)
                .addTemporalMarker(() -> {Open_Claws();})

                // Drives back to a spot where the robot can rotate freely
                .lineToLinearHeading(new Pose2d(35, 17.5,Math.toRadians(270)))

                // Drives forwards so the robot can rotate
                .forward(5.5)

                // Rotates the robot to face the Stack of Cones
                .turn(Math.toRadians(90))


                //////////////////////////////////////
                // Step 2 (Cone #2 or Stack Cone #1)//
                //////////////////////////////////////

                // Drive Forwards to pick up a cone from the stack while lowering the LS
                .forward(29)

                // Lowers the LS
                .UNSTABLE_addTemporalMarkerOffset(-3, () -> {Lower(20,.8);})

                // Grabs a cone from the stack
                .addTemporalMarker(() -> Close_Claws())

                // Waits so we don't knock over the stack of cones
                .waitSeconds(.5)

                // Raises the LS so we don't knock over the stack of cones
                .UNSTABLE_addTemporalMarkerOffset(-.5,() -> {Raise(5,.8);})

                // Drive Backwards to a spot where the robot can rotate freely
                .back(28.5)

                // Rotate facing the High Junction
                .turn(Math.toRadians(-90))

                // Drive to the High Junction While raising the LS
                .lineToLinearHeading(new Pose2d(23.5,6,Math.toRadians(270)))

                // Raises the LS
                .UNSTABLE_addTemporalMarkerOffset(-3,() -> {Raise(20,.8);})

                // Drops Cone #2 (Stack Cone #1)
                .addTemporalMarker(() -> {Open_Claws();})

                // Drives to a spot where the robot can rotate freely
                .lineToLinearHeading(new Pose2d(35,12,Math.toRadians(270)))

                // Rotates the robot to face the stack of cones
                .turn(Math.toRadians(90))

                //////////////////////////////////////
                // Step 3 (Cone #3 or Stack Cone #2)//
                //////////////////////////////////////

                // Drive Forwards to pick up a cone from the stack while lowering the LS
                .forward(29)

                // Lowers the LS
                .UNSTABLE_addTemporalMarkerOffset(-3, () -> {Lower(20,.8);})

                // Grabs a cone from the stack
                .addTemporalMarker(() -> Close_Claws())

                // Waits so we don't knock over the stack of cones
                .waitSeconds(.5)

                // Raises the LS so we don't knock over the stack of cones
                .UNSTABLE_addTemporalMarkerOffset(-.5,() -> {Raise(5,.8);})

                // Drive Backwards to a spot where the robot can rotate freely
                .back(28.5)

                // Rotate facing the High Junction
                .turn(Math.toRadians(-90))

                // Drive to the High Junction While raising the LS
                .lineToLinearHeading(new Pose2d(23.5,6,Math.toRadians(270)))

                // Raises the LS
                .UNSTABLE_addTemporalMarkerOffset(-3,() -> {Raise(20,.8);})

                // Drops Cone #3 (Stack Cone #2)
                .addTemporalMarker(() -> {Open_Claws();})

                // Drives to a spot where the robot can rotate freely
                .lineToLinearHeading(new Pose2d(35,12,Math.toRadians(270)))

                // Rotates the robot to face the stack of cones
                .turn(Math.toRadians(90))

                //////////////////////////////////////
                // Step 4 (Cone #3 or Stack Cone #2)//
                //////////////////////////////////////

                // Drive Forwards to pick up a cone from the stack while lowering the LS
                .forward(29)

                // Lowers the LS
                .UNSTABLE_addTemporalMarkerOffset(-3, () -> {Lower(20,.8);})

                // Grabs a cone from the stack
                .addTemporalMarker(() -> Close_Claws())

                // Waits so we don't knock over the stack of cones
                .waitSeconds(.5)

                // Raises the LS so we don't knock over the stack of cones
                .UNSTABLE_addTemporalMarkerOffset(-.5,() -> {Raise(5,.8);})

                // Drive Backwards to a spot where the robot can rotate freely
                .back(28.5)

                // Rotate facing the High Junction
                .turn(Math.toRadians(-90))

                // Drive to the High Junction While raising the LS
                .lineToLinearHeading(new Pose2d(23.5,6,Math.toRadians(270)))

                // Raises the LS
                .UNSTABLE_addTemporalMarkerOffset(-3,() -> {Raise(20,.8);})

                // Drops Cone #4 (Stack Cone #3)
                .addTemporalMarker(() -> {Open_Claws();})

                // Moves Backwards to Park
                .back(5)

                .build();
    }



    //Run Auto trajectory and parking trajectory
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

}

