package org.firstinspires.ftc.teamcode.opmode.Auto.Testing;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.opmode.Subsystem.DriveTrain;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "Strafe_Test", group = "00-Autonomous", preselectTeleOp = "Enginerds_Control")
public class Strafe_Test extends LinearOpMode{

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

                //.turn(90)
                //.forward(20)
                //.back(20)
                //.strafeRight(20)
                .strafeLeft(20)
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

