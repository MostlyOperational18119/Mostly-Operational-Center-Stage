package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.util.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Variables.Detection;
import org.firstinspires.ftc.teamcode.Variables.VisionProcessors;

import java.util.Arrays;

@Config
@Autonomous(name = "RFJamesEXPERIMENT", group = "LinearOpmode")
public class RFExperimentByJames extends MeepMeepBoilerplate {

    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Servo passiveServo = hardwareMap.get(Servo.class, "passiveServo");
        Servo autoServo = hardwareMap.get(Servo.class, "autoServo");
        initVision(VisionProcessors.TFOD);
        Detection detection = Detection.UNKNOWN;
        TrajectoryVelocityConstraint slowConstraint = new MinVelocityConstraint(Arrays.asList(

                new TranslationalVelocityConstraint(20),

                new AngularVelocityConstraint(1)

        ));
        while (opModeInInit()) {
            detection = getDetectionsSingleTFOD();
            telemetry.addData("Detection", detection);
            telemetry.update();
        }

        switch (detection) {
            case LEFT -> drive.followTrajectorySequence(
                    drive.trajectorySequenceBuilder(new Pose2d(-36.67, -63, Math.toRadians(90)))
                            .splineToLinearHeading(new Pose2d(-47.49, -38.18, Math.toRadians(-90.00)), Math.toRadians(90.00))
                            .splineTo(new Vector2d(-49.77, -54.52), Math.toRadians(-14.42))
                            .splineTo(new Vector2d(-17.29, -59.46), Math.toRadians(356.04))
                            .splineToSplineHeading(new Pose2d(22.04, -59.65, Math.toRadians(0.00)), Math.toRadians(0.00))
                            .lineTo(new Vector2d(50.72, -36.09))
                            .setReversed(true)
                            .lineTo(new Vector2d(42.93, -61.93))
                            .build()
            );
            case CENTER -> { drive.followTrajectorySequence(
                    drive.trajectorySequenceBuilder(new Pose2d(-36.67, -63, Math.toRadians(90)))
                            .splineToSplineHeading(new Pose2d(-35.91, -25.08, Math.toRadians(-90.00)), Math.toRadians(90.00))
                            .splineToSplineHeading(new Pose2d(-29.26, -60.03, Math.toRadians(0.00)), Math.toRadians(0.00))
                            .splineToSplineHeading(new Pose2d(22.04, -59.65, Math.toRadians(0.00)), Math.toRadians(0.00))
                            .lineTo(new Vector2d(50.72, -36.09))
                            .setReversed(true)
                            .lineTo(new Vector2d(42.93, -61.93))
                            .build());
            }
            case RIGHT -> drive.followTrajectorySequence(
                    drive.trajectorySequenceBuilder(new Pose2d(-36.67, -63, Math.toRadians(90)))
                            .splineTo(new Vector2d(-50.17, -42.51), Math.toRadians(45.51))
                            .splineToLinearHeading(new Pose2d(-33.23, -33.80, Math.toRadians(0.00)), Math.toRadians(0.00))
                            .setReversed(true)
                            .splineTo(new Vector2d(-41.63, -13.75), Math.toRadians(15.98))
                            .splineTo(new Vector2d(30.37, -11.46), Math.toRadians(0.00))
                            .lineTo(new Vector2d(50.80, -38.20))
                            .lineTo(new Vector2d(50.23, -61.11))
                            .build());
            default -> {
                telemetry.addLine("Warning: Cup not detected");
                telemetry.update();
                sleep(3000);
            }
        }

//        drive.followTrajectorySequence(mergeSequences(sequences));
    }
}

