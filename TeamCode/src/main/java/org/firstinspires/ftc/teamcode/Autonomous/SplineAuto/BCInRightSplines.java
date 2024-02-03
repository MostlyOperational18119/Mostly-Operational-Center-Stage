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
@Autonomous(name = "BCInRightSplines", group = "Linear OpMode")
public class BCInRightSplines extends MeepMeepBoilerplate {
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

        drive.setPoseEstimate(new Pose2d(15.01, 62.69, Math.toRadians(90.00)));

        rotateMotor.setPower(0.0);
        switch (detection) {
            case LEFT -> drive.followTrajectorySequence(
                    drive.trajectorySequenceBuilder(new Pose2d(15.01, 62.69, Math.toRadians(90.00)))
                            .lineToConstantHeading(new Vector2d(27.18, 36.85))
                            .waitSeconds(.1)
                            .addTemporalMarker(() -> passiveServo.setPosition(0.2))
                            .waitSeconds(.5)
                            .splineToLinearHeading(new Pose2d(57.18, 45.07, Math.toRadians(180.00)), Math.toRadians(0.0))
                            .waitSeconds(.1)
                            .addTemporalMarker(() -> autoServo.setPosition(.35))
                            .waitSeconds(.5)
                            .addTemporalMarker(() -> autoServo.setPosition(0.3))
                            .waitSeconds(.1)
                            .lineToConstantHeading(new Vector2d(51.41, 15.83))
                            .lineToConstantHeading(new Vector2d(58, 15.83))
                            .addTemporalMarker(() -> autoServo.setPosition(0.0))
                            .waitSeconds(1)
                            .build()
            );
            case CENTER -> { drive.followTrajectorySequence(
                    drive.trajectorySequenceBuilder(new Pose2d(15.01, 62.69, Math.toRadians(90.00)))
                            .lineToConstantHeading(new Vector2d(16.35, 30))
                            .waitSeconds(.1)
                            .addTemporalMarker(() -> passiveServo.setPosition(0.2))
                            .waitSeconds(.5)
                            .splineToLinearHeading(new Pose2d(54.91, 39.46, Math.toRadians(180.00)), Math.toRadians(0.00))
                            .waitSeconds(.1)
                            .addTemporalMarker(() -> autoServo.setPosition(.35))
                            .waitSeconds(.5)
                            .addTemporalMarker(() -> autoServo.setPosition(0.3))
                            .waitSeconds(.1)
                            .lineToConstantHeading(new Vector2d(51.41, 15.83))
                            .lineToConstantHeading(new Vector2d(58, 15.83))
                            .addTemporalMarker(() -> autoServo.setPosition(0.0))
                            .waitSeconds(1)
                            .build());
            }
            case RIGHT -> drive.followTrajectorySequence(
                    drive.trajectorySequenceBuilder(new Pose2d(15.01, 62.69, Math.toRadians(90.00)))
                            .setReversed(true)
                            .splineToLinearHeading(new Pose2d(15.75, 45.61, Math.toRadians(90.00)), Math.toRadians(-90.00))
                            .splineToLinearHeading(new Pose2d(10.95, 35.39, Math.toRadians(0.00)), Math.toRadians(180.00))
                            .setReversed(false)
                            .waitSeconds(.1)
                            .addTemporalMarker(() -> passiveServo.setPosition(0.2))
                            .waitSeconds(.5)
                            .lineToLinearHeading(new Pose2d(55.29, 32.49, Math.toRadians(180.00)))
                            .addTemporalMarker(() -> autoServo.setPosition(0.35))
                            .waitSeconds(.5)
                            .addTemporalMarker(() -> autoServo.setPosition(0.3))
                            .waitSeconds(.1)
                            .lineToConstantHeading(new Vector2d(51.41, 15.83))
                            .lineToConstantHeading(new Vector2d(58, 15.83))
                            .addTemporalMarker(() -> autoServo.setPosition(0.0))
                            .waitSeconds(1)
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
