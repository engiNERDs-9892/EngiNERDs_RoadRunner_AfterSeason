package org.firstinspires.ftc.teamcode.drive.opmode.Auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.opmode.RoadRunner.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;

public class TrajectorySequence_Test extends LinearOpMode {

    SampleMecanumDrive drivetrain = new SampleMecanumDrive(hardwareMap);


    @Override
    public void runOpMode() {
        SampleMecanumDrive drivetrain = new SampleMecanumDrive(hardwareMap);
        waitForStart();


        Autonomous();
    }


    // Multiple Movement testing


        private void Autonomous() {
            Pose2d startPose = new Pose2d(0, 0, 0);

            drivetrain.setPoseEstimate(startPose);

            drivetrain.trajectorySequenceBuilder(startPose);

            TrajectorySequence TrajectorySequence_Test = drivetrain.trajectorySequenceBuilder(startPose)

                    .strafeRight(20)
                    .strafeLeft(20)

                    .build();

            drivetrain.followTrajectorySequence(TrajectorySequence_Test);
        }

    }


