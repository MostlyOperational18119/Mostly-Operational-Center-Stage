package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.DriveShim;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.sequencesegment.SequenceSegment;

import java.util.ArrayList;

public class FrontBlueAuto {
    enum Detection {
        LEFT,
        CENTER,
        RIGHT
    }

    private static final Pose2d STARTING_POSE = new Pose2d(15.01, 62.69, Math.toRadians(90.00));

    private static ArrayList<TrajectorySequence> sequences = new ArrayList<TrajectorySequence>();

    private static TrajectorySequence mergeSequences(ArrayList<TrajectorySequence> trajectorySequences) {
        TrajectorySequence[] trajectorySequencesArr = new TrajectorySequence[trajectorySequences.size()];
        trajectorySequencesArr = trajectorySequences.toArray(trajectorySequencesArr);

        return mergeSequences(trajectorySequencesArr);
    }

    private static TrajectorySequence mergeSequences(TrajectorySequence[] trajectorySequences) {
        ArrayList<SequenceSegment> trajectorySegments = new ArrayList<SequenceSegment>();

        for (TrajectorySequence sequence : trajectorySequences) {
            for (int i = 0; i < sequence.size(); i++) {
                trajectorySegments.add(sequence.get(i));
            }
        }

        return new TrajectorySequence(trajectorySegments);
    }

    private static TrajectorySequence getCurrentTrajectorySequence(DriveShim driveShim) {
        TrajectorySequence currentTrajectory;
        if (true) {
            currentTrajectory = sequences.get(sequences.size() - 1);
        } else {
            currentTrajectory = driveShim.trajectorySequenceBuilder(STARTING_POSE)
                    .forward(0.0)
                    .build();
        }
        return currentTrajectory;
    }

    private static void followTrajectorySequence(TrajectorySequence trajectorySequence) {
        sequences.add(trajectorySequence);
    }

    public static void main(String[] args) {
        System.setProperty("sun.java2d.opengl", "true");
        MeepMeep meepMeep = new MeepMeep(800);
        Detection detection = Detection.LEFT;
        RoadRunnerBotEntity myBot;
        DriveShim driveShim;

        myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        driveShim = myBot.getDrive();

//        followTrajectorySequence(
//                driveShim.trajectorySequenceBuilder(STARTING_POSE)
////                        .forward(28.0)
//                        .build()
//        );

        assert getCurrentTrajectorySequence(driveShim) != null; // Null Pointer Protection

        switch (detection) {
            case LEFT:
                followTrajectorySequence(
                        driveShim.trajectorySequenceBuilder(STARTING_POSE/*getCurrentTrajectorySequence(driveShim).end()*/)
                                .lineToConstantHeading(new Vector2d(27.18, 36.85))
                                .splineToLinearHeading(new Pose2d(57.18, 45.07, Math.toRadians(180.00)), Math.toRadians(0.0))
                                .lineToConstantHeading(new Vector2d(41.41, 62.31))
                                .build()
                );
                break;
            case CENTER:
                followTrajectorySequence(
                        driveShim.trajectorySequenceBuilder(STARTING_POSE/*getCurrentTrajectorySequence(driveShim).end()*/)
                                .lineToConstantHeading(new Vector2d(12.35, 33.82))
                                .splineToLinearHeading(new Pose2d(50.91, 40.46, Math.toRadians(0.00)), Math.toRadians(0.00))
                                .lineToConstantHeading(new Vector2d(41.41, 62.31))
                                .build());
                break;
            case RIGHT:
                followTrajectorySequence(
                        driveShim.trajectorySequenceBuilder(STARTING_POSE/*getCurrentTrajectorySequence(driveShim).end()*/)
                                .setReversed(true)
                                .splineToLinearHeading(new Pose2d(15.75, 45.61, Math.toRadians(90.00)), Math.toRadians(-90.00))
                                .splineToLinearHeading(new Pose2d(6.95, 35.39, Math.toRadians(360.00)), Math.toRadians(180.00))
                                .setReversed(false)
                                .lineToConstantHeading(new Vector2d(51.29, 32.49))
                                .lineToConstantHeading(new Vector2d(44.45, 62.69))
                                .build());
                break;
        }

        myBot.followTrajectorySequence(mergeSequences(sequences));

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}