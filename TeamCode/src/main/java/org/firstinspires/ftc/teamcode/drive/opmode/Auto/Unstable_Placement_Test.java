package org.firstinspires.ftc.teamcode.drive.opmode.Auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.opmode.RoadRunner.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class Unstable_Placement_Test extends LinearOpMode {

    SampleMecanumDrive drivetrain = new SampleMecanumDrive(hardwareMap);


    Servo servoFAL;
    Servo servoFAR;



    @Override
    public void runOpMode() {
        SampleMecanumDrive drivetrain = new SampleMecanumDrive(hardwareMap);

        // Servos
        servoFAL = hardwareMap.servo.get("servoFAL");
        servoFAR = hardwareMap.servo.get("servoFAR");

        waitForStart();


        Autonomous();
    }


    // Multiple Movement testing


        private void Autonomous() {
            Pose2d startPose = new Pose2d(0, 0, 0);

            drivetrain.setPoseEstimate(startPose);

            drivetrain.trajectorySequenceBuilder(startPose);

            TrajectorySequence Unstable_Placement_Test = drivetrain.trajectorySequenceBuilder(startPose)


                    .strafeLeft(20)
                    .UNSTABLE_addTemporalMarkerOffset(-2,() -> {Open_Claws();})
                    .addTemporalMarker(() -> {Close_Claws();} )

                    .build();

            drivetrain.followTrajectorySequence(Unstable_Placement_Test);
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

    }


