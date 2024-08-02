package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[ ] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(70, 90, Math.toRadians(180), Math.toRadians(180), 14.79)

                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(14.783464, -58, Math.toRadians(270)))
                                       // .lineToSplineHeading(new Pose2d(8, -36, Math.toRadians(325) ))
//                                        .addDisplacementMarker(  () -> {
//                                       })
                                        .lineToSplineHeading(new Pose2d(12, -35, Math.toRadians(270)))
                                        .addTemporalMarker( 0.5, () ->{

                                        })
                                        .lineToSplineHeading(new Pose2d(45, -31, Math.toRadians(180) ))
                                        .waitSeconds(0.3)
                                        .setReversed(false)
                                        .splineTo(new Vector2d(0, -55), Math.toRadians(180))
                                        .lineToSplineHeading(new Pose2d(-25, -55, Math.toRadians(175)))
                                        .addDisplacementMarker( () -> {

                                        })
                                        .splineTo(new Vector2d(-64, -20), Math.toRadians(170))

                                        .setReversed(true)

                                        .splineTo(new Vector2d(-25, -55), Math.toRadians(0))
                                        .lineToSplineHeading(new Pose2d(10, -55, Math.toRadians(185)))
                                       // .addDisplacementMarker( this::erectie)
                                        .splineTo(new Vector2d(45, -42), Math.toRadians(0))

                                        //.waitSeconds(0.5)
                                        //.addDisplacementMarker(this::senzor)

                                        .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}