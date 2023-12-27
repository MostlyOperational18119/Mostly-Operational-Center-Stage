package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.DriveMethods;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.util.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.RoadRunner.util.trajectorysequence.sequencesegment.SequenceSegment;
import org.firstinspires.ftc.teamcode.Variables;

import java.util.ArrayList;


enum Detection {
    LEFT,
    CENTER,
    RIGHT
}

public abstract class AutoBoilerplate extends DriveMethods {
    @Override
    public void runOpMode() {
        drive(new Pose2d());
    }
    SampleMecanumDrive drive;

    public Pose2d STARTING_POSE = new Pose2d(-36, 61.5, Math.toRadians(-90));

    public ArrayList<TrajectorySequence> sequences = new ArrayList<TrajectorySequence>();

    public void drive(Pose2d startingPos) {
        initMotorsSecondBot();
        initVision(Variables.VisionProcessors.TFOD);

        telemetry.addLine("Motors and TFOD were ignited (same as inited)");
        telemetry.update();

        Detection detection = detect();

        while (opModeIsActive()) detection = detect();
        STARTING_POSE = startingPos;
        drive = new SampleMecanumDrive(hardwareMap);

        drive.followTrajectorySequence(getTrajectorySequence(detection, drive));
    }

    public Detection detect() {
        return Detection.RIGHT;
    }

    public abstract TrajectorySequence getTrajectorySequence(Detection detection, SampleMecanumDrive drive);

    public void print(Object o) {
        System.out.println(o);
    }

    /*
    public TrajectorySequence mergeSequences(ArrayList<TrajectorySequence> trajectorySequences) {
        TrajectorySequence[] trajectorySequencesArr = new TrajectorySequence[trajectorySequences.size()];
        trajectorySequencesArr = trajectorySequences.toArray(trajectorySequencesArr);

        return mergeSequences(trajectorySequencesArr);
    }

    public TrajectorySequence mergeSequences(TrajectorySequence[] trajectorySequences) {
        ArrayList<SequenceSegment> trajectorySegments = new ArrayList<SequenceSegment>();

        for (TrajectorySequence sequence : trajectorySequences) {
            for (int i = 0; i < sequence.size(); i++) {
                trajectorySegments.add(sequence.get(i));
            }
        }

        return new TrajectorySequence(trajectorySegments);
    }
    */

    public TrajectorySequence getCurrentTrajectorySequence(SampleMecanumDrive drive) {
         if (sequences.size() == 0) {
             return drive.trajectorySequenceBuilder(STARTING_POSE)
                    .forward(0.0)
                    .build();
        } else {
            return sequences.get(sequences.size() - 1);
        }
    }

    public Pose2d getCurrentPosition(SampleMecanumDrive drive) {
        if (sequences.size() == 0) {
            return STARTING_POSE;
        } else {
            return getCurrentTrajectorySequence(drive).end();
        }
    }

    public void followTrajectorySequence(TrajectorySequence trajectorySequence) {
//        if (!opModeIsActive()) waitForStart();
        sequences.add(trajectorySequence);
    }
}
