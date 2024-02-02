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

public class BackBlueAuto {
    enum Detection {
        LEFT,
        CENTER,
        RIGHT
    }

    private static final Pose2d STARTING_POSE = new Pose2d(-36.67, 62.45, Math.toRadians(90.00));

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
                                .setReversed(true)
                                .splineToLinearHeading(new Pose2d(-31.32, 32.85, Math.toRadians(180.00)), Math.toRadians(0))
                                .setReversed(false)
                                .splineToLinearHeading(new Pose2d(-36.09, 58.88, Math.toRadians(360.00)), Math.toRadians(360.00))
                                .splineToConstantHeading(new Vector2d(9, 58.88), Math.toRadians(0.00))
                                .splineToConstantHeading(new Vector2d(23.49, 36.49), Math.toRadians(0.00))
                                .splineToConstantHeading(new Vector2d(51.31, 36.49), Math.toRadians(0.00))
                                .build()
                );
                break;
            case CENTER:
                followTrajectorySequence(
                        driveShim.trajectorySequenceBuilder(STARTING_POSE/*getCurrentTrajectorySequence(driveShim).end()*/)
                                .lineToConstantHeading(new Vector2d(-35.34, 33.82))
                                .splineToLinearHeading(new Pose2d(-35.91, 58.88, Math.toRadians(0.00)), Math.toRadians(0.00))
                                .waitSeconds(1)
                                .splineToConstantHeading(new Vector2d(9, 58.88), Math.toRadians(0.00))
                                .splineToConstantHeading(new Vector2d(23.49, 43.46), Math.toRadians(0.00))
                                .splineToLinearHeading(new Pose2d(51.31, 43.46, Math.toRadians(0.00)), Math.toRadians(0.00))
                                .build());
                break;
            case RIGHT:
                followTrajectorySequence(
                        driveShim.trajectorySequenceBuilder(STARTING_POSE/*getCurrentTrajectorySequence(driveShim).end()*/)
                                .lineToConstantHeading(new Vector2d(-47.34, 36.82))
                                .splineToLinearHeading(new Pose2d(-35.91, 58.88, Math.toRadians(0.00)), Math.toRadians(0.00))
                                .waitSeconds(1)
                                .splineToConstantHeading(new Vector2d(9, 58.88), Math.toRadians(0.00))
                                .splineToConstantHeading(new Vector2d(23.49, 43.46), Math.toRadians(0.00))
                                .splineToLinearHeading(new Pose2d(51.31, 43.46, Math.toRadians(0.00)), Math.toRadians(0.00))
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