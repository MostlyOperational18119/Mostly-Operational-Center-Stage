package org.firstinspires.ftc.teamcode.Autonomous;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.DriveMethods;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.util.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Variables;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;


public abstract class AutoBoilerplate extends AutoBoilerplateMultiSequences {

    public abstract TrajectorySequence getTrajectorySequence(Variables.Detection detection, SampleMecanumDrive drive);

    @NonNull
    @Override
    public ArrayList<TrajectorySequenceWithCallback> getTrajectorySequences(Variables.Detection detection, SampleMecanumDrive drive) {
        TrajectorySequence trajectorySequence = getTrajectorySequence(detection, drive);
        OptionalTrajectorySequence optionalTrajectorySequence = new OptionalTrajectorySequence(true, trajectorySequence);
        TrajectorySequenceWithCallback trajectorySequenceWithCallback = new TrajectorySequenceWithCallback(optionalTrajectorySequence, false, null);
        return new ArrayList<TrajectorySequenceWithCallback>(List.of(trajectorySequenceWithCallback));
    }
}

