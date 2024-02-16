package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.DriveMethods;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.util.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Variables;

import java.util.ArrayList;


public abstract class AutoBoilerplate extends DriveMethods {
    @Override
    public void runOpMode() {
        drive(STARTING_POSE);
    }
    SampleMecanumDrive drive;

    public Pose2d STARTING_POSE = new Pose2d(-36, 61.5, Math.toRadians(-90));

    public ArrayList<TrajectorySequence> sequences = new ArrayList<TrajectorySequence>();

    public Servo passiveServo;
    public Servo autoServo;
    public DcMotor rotateMotor;

    public abstract Pose2d getSTARTING_POSE();

    public void drive(Pose2d startingPos) {
        initMotorsSecondBot();
        initVision(Variables.VisionProcessors.TFOD);

        telemetry.addLine("Motors and TFOD were ignited (same as inited)");
        telemetry.update();

        Variables.Detection detection = detect();

        STARTING_POSE = startingPos;
        drive = new SampleMecanumDrive(hardwareMap);
        initCall();

        while (opModeInInit()) {
            detection = detect();
            telemetry.addData("Detection", detection);
            telemetry.update();
        }

        drive.followTrajectorySequence(getTrajectorySequence(detection, drive));
    }

    public Variables.Detection detect() {
        return getDetectionsSingleTFOD();
    }

    public abstract TrajectorySequence getTrajectorySequence(Variables.Detection detection, SampleMecanumDrive drive);

    public void println(Object o) {
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

    public void initCall() {
        rotateMotor = hardwareMap.get(DcMotor.class, "motorSlideRotate");
        passiveServo = hardwareMap.get(Servo.class, "passiveServo");
        autoServo = hardwareMap.get(Servo.class, "autoServo");
        initVision(Variables.VisionProcessors.TFOD);
        initBlinkinSafe(getDefaultColour());
        autoServo.setPosition(0.32);
        autoServo.setPosition(0.0);
    }

    public abstract RevBlinkinLedDriver.BlinkinPattern getDefaultColour();
}
