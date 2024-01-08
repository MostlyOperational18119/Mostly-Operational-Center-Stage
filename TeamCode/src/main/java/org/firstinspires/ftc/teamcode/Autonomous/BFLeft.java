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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Variables.Detection;
import org.firstinspires.ftc.teamcode.Variables.VisionProcessors;

import java.util.Arrays;

@Config
@Disabled
@Autonomous(name = "BFLeftEXPERIMENT", group = "Linear OpMode")
public class BFLeft extends MeepMeepBoilerplate{
    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Servo passiveServo = hardwareMap.get(Servo.class, "passiveServo");
        Servo autoServo = hardwareMap.get(Servo.class, "autoServo");
        DcMotor rotateMotor = hardwareMap.get(DcMotor.class, "motorSlideRotate");

        initVision(VisionProcessors.TFOD);
        Detection detection = Detection.UNKNOWN;
        TrajectoryVelocityConstraint slowConstraint = new MinVelocityConstraint(Arrays.asList(

                new TranslationalVelocityConstraint(20),

                new AngularVelocityConstraint(1)

        ));

        TrajectoryVelocityConstraint fastConstraint = new MinVelocityConstraint(Arrays.asList(

                new TranslationalVelocityConstraint(55),

                new AngularVelocityConstraint(3)

        ));
        while (opModeInInit()) {
            detection = getDetectionsSingleTFOD();
            telemetry.addData("Detection", detection);
            telemetry.update();
        }

        autoServo.setPosition(0.78);

        rotateMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotateMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rotateMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rotateMotor.setPower(0.0);
        switch (detection) {
            case LEFT -> drive.followTrajectorySequence(
                    drive.trajectorySequenceBuilder(new Pose2d(-36.67, 62.45, Math.toRadians(90.00)))
                            .back(28.0)
                            .turn(Math.toRadians(90))
                            .back(8)
                            .splineToLinearHeading(new Pose2d(-36.09, 58.89, Math.toRadians(0.00)), Math.toRadians(0.00))
                            .splineToLinearHeading(new Pose2d(18.43, 59.84, Math.toRadians(0.00)), Math.toRadians(0.00))
                            .splineToLinearHeading(new Pose2d(50.91, 35.72, Math.toRadians(360.00)), Math.toRadians(360.00))
                            .lineToConstantHeading(new Vector2d(42.93, 62.69))
                            .build());
            case CENTER -> { drive.followTrajectorySequence(
                    drive.trajectorySequenceBuilder(new Pose2d(-36.67, 62.45, Math.toRadians(90.00)))
                            .lineToConstantHeading(new Vector2d(-35.34, 33.82))
                            .splineToLinearHeading(new Pose2d(-35.91, 59.46, Math.toRadians(0.00)), Math.toRadians(0.00))
                            .splineToLinearHeading(new Pose2d(18.43, 59.84, Math.toRadians(0.00)), Math.toRadians(0.00))
                            .splineToLinearHeading(new Pose2d(51.29, 39.89, Math.toRadians(0.00)), Math.toRadians(0.00))
                            .lineToConstantHeading(new Vector2d(42.93, 62.69))
                            .build());
            }
            case RIGHT -> drive.followTrajectorySequence(
                    drive.trajectorySequenceBuilder(new Pose2d(-36.67, 62.45, Math.toRadians(90.00)))
                            .splineToLinearHeading(new Pose2d(-47.49, 38.18, Math.toRadians(450.00)), Math.toRadians(270.00))
                            .splineTo(new Vector2d(-51.29, 54.71), Math.toRadians(-1.03))
                            .splineTo(new Vector2d(-17.29, 59.46), Math.toRadians(3.96))
                            .splineToSplineHeading(new Pose2d(22.04, 59.65, Math.toRadians(360.00)), Math.toRadians(360.00))
                            .lineTo(new Vector2d(50.72, 36.09))
                            .setReversed(true)
                            .lineTo(new Vector2d(42.93, 61.93))
                            .build()
            );
            default -> {
                telemetry.addLine("Warning: Cup not detected");
                telemetry.update();
                sleep(3000);
            }
        }



//        drive.followTrajectorySequence(mergeSequences(sequences));
    }
}
