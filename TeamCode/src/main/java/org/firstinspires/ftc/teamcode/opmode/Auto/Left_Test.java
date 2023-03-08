package org.firstinspires.ftc.teamcode.opmode.Auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.opmode.Subsystem.DriveTrain;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

/**
 * FTC WIRES Autonomous Example for only vision detection using tensorflow and park
 */
@Autonomous(name = "Left_Test", group = "00-Autonomous", preselectTeleOp = "Enginerds_Control")
public class Left_Test extends LinearOpMode{

    //Define and declare Robot Starting Locations

    public DriveTrain driveTrain;

    @Override
    public void runOpMode() throws InterruptedException {
        /*Create your Subsystem Objects*/
        driveTrain = new DriveTrain(hardwareMap);


        // Initiate Camera on Init.
        //Game Play Button  is pressed
        if (opModeIsActive() && !isStopRequested()) {
            //Stop Vision process
            //Build parking trajectory based on last detected target by vision
            buildParking();
            driveTrain.getLocalizer().setPoseEstimate(initPose);

            //run Autonomous trajectory
            runParking();
        }
    }

    //Initialize any other TrajectorySequences as desired
    TrajectorySequence trajectoryParking ;

    //Initialize any other Pose2d's as desired
    Pose2d initPose; // Starting Pose

    //Build parking trajectory based on target detected by vision
    public void buildParking(){



        trajectoryParking = driveTrain.trajectorySequenceBuilder(initPose)
                .strafeLeft(20)
                .build();
    }

    //Run Auto trajectory and parking trajectory
    public void runParking(){

        //Run the trajectory built for Auto and Parking
        driveTrain.followTrajectorySequence(trajectoryParking);

    }

}

