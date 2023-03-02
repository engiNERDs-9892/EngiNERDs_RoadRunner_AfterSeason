package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-35, 60, Math.toRadians(0)))

                                // Red Left

                                // Step 1 (Pre-Loaded Cone)
                                .strafeRight(45)
                                .lineToLinearHeading(new Pose2d(-25, 2.5,Math.toRadians(300)))
                                .lineToLinearHeading(new Pose2d(-35.5, 12,Math.toRadians(0)))

                                .turn(Math.toRadians(180))


                                // Step 2 (Cone #2)
                                .forward(29)
                                .back(28.5)
                                .turn(Math.toRadians(90))
                                .lineToLinearHeading(new Pose2d(-23.5,6,Math.toRadians(-90)))
                                .lineToLinearHeading(new Pose2d(-32.5,12,Math.toRadians(-90)))
                                .turn(Math.toRadians(-90))

                                // Step 3 (Cone #3)

                                .forward(29)
                                .back(29)
                                .turn(Math.toRadians(90))
                                .lineToLinearHeading(new Pose2d(-23.5,6,Math.toRadians(-90)))
                                .lineToLinearHeading(new Pose2d(-32.5,12,Math.toRadians(-90)))
                                .turn(Math.toRadians(-90))


                                // Step 4 (Cone #4)

                                .forward(29)
                                .back(29)
                                .turn(Math.toRadians(90))
                                .lineToLinearHeading(new Pose2d(-23.5,6,Math.toRadians(-90)))
                                .back(5)
                                .build()


                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_KAI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }


}
