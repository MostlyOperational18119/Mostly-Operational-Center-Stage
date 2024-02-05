package org.firstinspires.ftc.teamcode.Autonomous.SplineAuto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Autonomous.MeepMeepBoilerplate;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Variables.Detection;
import org.firstinspires.ftc.teamcode.Variables.VisionProcessors;

import java.util.Arrays;

@Config
@Autonomous(name = "RCInLeftSplines", group = "Linear OpMode")
public class RCInLeftSplines extends MeepMeepBoilerplate {
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

                new TranslationalVelocityConstraint(50),

                new AngularVelocityConstraint(3)

        ));

        while (opModeInInit()) {
            detection = getDetectionsSingleTFOD(400
            );
            telemetry.addData("Detection", detection);
            telemetry.update();
        }
        autoServo.setPosition(0.11);

        rotateMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotateMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rotateMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        drive.setPoseEstimate(new Pose2d(15.01, -62.69, Math.toRadians(270.00)));
        rotateMotor.setPower(0.0);
        switch (detection) {
            case LEFT -> drive.followTrajectorySequence(
                    drive.trajectorySequenceBuilder(new Pose2d(15.01, -62.69, Math.toRadians(270.00)))
                            .setReversed(true)
                            .splineToLinearHeading(new Pose2d(18.98, -44.59, Math.toRadians(-90.00)), Math.toRadians(90.00))
                            .splineToLinearHeading(new Pose2d(13, -34, Math.toRadians(0.00)), Math.toRadians(180.00))
                            .splineToLinearHeading(new Pose2d(7, -34, Math.toRadians(0.00)), Math.toRadians(180.00))
                            .setReversed(false)
                            .waitSeconds(.1)
                            .addTemporalMarker(() -> passiveServo.setPosition(0.2))
                            .waitSeconds(.5)
                            .lineToLinearHeading(new Pose2d(51.29, -26.60, Math.toRadians(180.00)))
                            .waitSeconds(.1)
                            .addTemporalMarker(() -> autoServo.setPosition(0.35))
                            .waitSeconds(1)
                            .addTemporalMarker(() -> autoServo.setPosition(0.3))
                            .waitSeconds(.1)
                            .lineToConstantHeading(new Vector2d(47.41, -15.3))
                            .lineToConstantHeading(new Vector2d(58, -15.3))
                            .addTemporalMarker(() -> autoServo.setPosition(0))
                            .waitSeconds(1)
                            .build());
            case CENTER -> drive.followTrajectorySequence(
                    drive.trajectorySequenceBuilder(new Pose2d(15.01, -62.69, Math.toRadians(270.00)))
                            .lineToConstantHeading(new Vector2d(12.16, -30.68))
                            .waitSeconds(.1)
                            .addTemporalMarker(() -> passiveServo.setPosition(0.2))
                            .waitSeconds(.5)
                            .splineToLinearHeading(new Pose2d(51.29, -33.09, Math.toRadians(180.00)), Math.toRadians(0.00))
                            .waitSeconds(.1)
                            .addTemporalMarker(() -> autoServo.setPosition(0.35))
                            .waitSeconds(1)
                            .addTemporalMarker(() -> autoServo.setPosition(0.3))
                            .waitSeconds(.1)
                            .lineToConstantHeading(new Vector2d(47.41, -15.3))
                            .lineToConstantHeading(new Vector2d(58, -15.3))
                            .addTemporalMarker(() -> autoServo.setPosition(0.0))
                            .waitSeconds(1)
                            .build());
            case RIGHT -> drive.followTrajectorySequence(
                    drive.trajectorySequenceBuilder(new Pose2d(15.01, -62.69, Math.toRadians(270.00)))
                            .lineToConstantHeading(new Vector2d(24.18, -37.99))
                            .waitSeconds(.1)
                            .addTemporalMarker(() -> passiveServo.setPosition(0.2))
                            .waitSeconds(.5)
                            .lineToConstantHeading(new Vector2d(24.18, -40.99))
                            .splineToLinearHeading(new Pose2d(51.29, -39.89, Math.toRadians(180.00)), Math.toRadians(360.00))
                            .waitSeconds(.1)
                            .addTemporalMarker(() -> autoServo.setPosition(0.35))
                            .waitSeconds(1.5)
                            .addTemporalMarker(() -> autoServo.setPosition(0.3))
                            .waitSeconds(.1)
                            .lineToConstantHeading(new Vector2d(47.41, -15.3))
                            .lineToConstantHeading(new Vector2d(58, -15.3))
                            .addTemporalMarker(() -> autoServo.setPosition(0.0))
                            .waitSeconds(1)
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