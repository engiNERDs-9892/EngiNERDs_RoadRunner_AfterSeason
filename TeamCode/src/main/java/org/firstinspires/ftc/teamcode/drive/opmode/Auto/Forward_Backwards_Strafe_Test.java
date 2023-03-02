package org.firstinspires.ftc.teamcode.drive.opmode.Auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.opmode.RoadRunner.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class Forward_Backwards_Strafe_Test extends LinearOpMode {

    SampleMecanumDrive drivetrain = new SampleMecanumDrive(hardwareMap);


    @Override
    public void runOpMode() {
        SampleMecanumDrive drivetrain = new SampleMecanumDrive(hardwareMap);
        waitForStart();


        Autonomous();
    }


    // Singular Movement Test

        private void Autonomous() {
            Pose2d startPose = new Pose2d(0, 0, 0);

            drivetrain.setPoseEstimate(startPose);

            drivetrain.trajectorySequenceBuilder(startPose);

            TrajectorySequence Forwards_Backwards_Strafe_Test = drivetrain.trajectorySequenceBuilder(startPose)

                    //.rotate(90)
                    //.back(20)
                    //.forward(20)
                    //.strafeRight(20)
                    .strafeLeft(20)

                    .build();

            drivetrain.followTrajectorySequence(Forwards_Backwards_Strafe_Test);
        }

    }


