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
                    drive.trajectorySequenceBuilder(new Pose2d(-36.67, -63, Math.toRadians(-90)))
                            .lineToConstantHeading(new Vector2d(-47.30, -40.08))
                            .splineToLinearHeading(new Pose2d(-35.91, -59.46, Math.toRadians(360.00)), Math.toRadians(360.00))
                            .splineToLinearHeading(new Pose2d(18.43, -59.84, Math.toRadians(360.00)), Math.toRadians(360.00))
                            .splineToLinearHeading(new Pose2d(50.91, -26.41, Math.toRadians(0.00)), Math.toRadians(0.00))
                            .lineToConstantHeading(new Vector2d(42.93, -62.69))
                            .build()
            );
            case CENTER -> { drive.followTrajectorySequence(
                    drive.trajectorySequenceBuilder(new Pose2d(-36.67, -63, Math.toRadians(-90)))
                            .lineToConstantHeading(new Vector2d(-35.34, -33.82))
                            .splineToLinearHeading(new Pose2d(-35.91, -59.46, Math.toRadians(360.00)), Math.toRadians(360.00))
                            .splineToLinearHeading(new Pose2d(18.43, -59.84, Math.toRadians(360.00)), Math.toRadians(360.00))
                            .splineToLinearHeading(new Pose2d(51.29, -32.11, Math.toRadians(360.00)), Math.toRadians(360.00))
                            .lineToConstantHeading(new Vector2d(42.93, -62.69))
                            .build());
            }
            case RIGHT -> drive.followTrajectorySequence(
                    drive.trajectorySequenceBuilder(new Pose2d(-36.67, -63, Math.toRadians(-90)))
                            .back(28.0)
                            .turn(Math.toRadians(-90))
                            .back(4)
                            .splineToLinearHeading(new Pose2d(-36.09, -58.89, Math.toRadians(360.00)), Math.toRadians(360.00))
                            .splineToLinearHeading(new Pose2d(26.37, -58.82, Math.toRadians(0.00)), Math.toRadians(0.00))
                            .splineToLinearHeading(new Pose2d(51.31, -37.72, Math.toRadians(0.00)), Math.toRadians(0.00))
                            .lineToConstantHeading(new Vector2d(42.93, -62.69))
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

