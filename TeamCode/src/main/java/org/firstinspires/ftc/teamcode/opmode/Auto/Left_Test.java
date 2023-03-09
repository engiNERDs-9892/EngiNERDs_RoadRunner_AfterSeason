package org.firstinspires.ftc.teamcode.opmode.Auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.opmode.Subsystem.DriveTrain;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "Left_Test", group = "00-Autonomous", preselectTeleOp = "Enginerds_Control")
public class Left_Test extends LinearOpMode{

    public DriveTrain driveTrain;



    @Override
    public void runOpMode() throws InterruptedException {
        driveTrain = new DriveTrain(hardwareMap);
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

    //Initialize any other Pose2d's as desired


    //Build parking trajectory based on target detected by vision
    public void buildParking(){

        trajectoryParking = driveTrain.trajectorySequenceBuilder(new Pose2d())
                .strafeLeft(20)
                .build();
    }



    //Run Auto trajectory and parking trajectory
    public void runParking(){

        //Run the trajectory built for Auto and Parking
        driveTrain.followTrajectorySequence(trajectoryParking);

    }

}

