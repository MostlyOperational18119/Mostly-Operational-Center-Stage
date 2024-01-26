package org.firstinspires.ftc.teamcode.Autonomous;

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

import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Variables.Detection;
import org.firstinspires.ftc.teamcode.Variables.VisionProcessors;

import java.util.Arrays;

@Config
@Autonomous(name = "BFLeftEXPERIMENT", group = "Linear OpMode")
public class BFLeftSplines extends MeepMeepBoilerplate{
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

        drive.setPoseEstimate(new Pose2d(-36.67, 62.45, Math.toRadians(90.00)));

        switch (detection) {
            case LEFT -> drive.followTrajectorySequence(
                    drive.trajectorySequenceBuilder(new Pose2d(-36.67, 62.45, Math.toRadians(90.00)))
                            .setReversed(true)
                            .splineToLinearHeading(new Pose2d(-31.70, 34.77, Math.toRadians(180.00)), Math.toRadians(293.11))
                            .waitSeconds(.1)
                            .addTemporalMarker(() -> passiveServo.setPosition(0.2))
                            .waitSeconds(.15)
                            .setReversed(false)
                            .splineToLinearHeading(new Pose2d(-36.09, 58.89, Math.toRadians(0.00)), Math.toRadians(0.00))
                            .build());
            case CENTER -> {drive.followTrajectorySequence(
                    drive.trajectorySequenceBuilder(new Pose2d(-36.67, 62.45, Math.toRadians(90.00)))
                            .lineToConstantHeading(new Vector2d(-35.34, 33.82))
                            .waitSeconds(.1)
                            .addTemporalMarker(() -> passiveServo.setPosition(0.2))
                            .waitSeconds(.15)
                            .splineToLinearHeading(new Pose2d(-35.91, 59.46, Math.toRadians(0.00)), Math.toRadians(0.00))
                            .build());
            }
            case RIGHT -> drive.followTrajectorySequence(
                    drive.trajectorySequenceBuilder(new Pose2d(-36.67, 62.45, Math.toRadians(90.00)))
                            .back(2.0)
                            .waitSeconds(.25)
                            .strafeLeft(8)
                            .back(21)
                            .waitSeconds(.25)
                            .addTemporalMarker(() -> passiveServo.setPosition(0.2))
                            .waitSeconds(4)
                            .forward(5)
                            .build());
            default -> {
                telemetry.addLine("Warning: Cup not detected");
                telemetry.update();
                sleep(3000);
            }
        }

        while (rotateMotor.getCurrentPosition()<1000){
            rotateMotor.setPower(0.5);
        }
        rotateMotor.setPower(0.001);
        sleep(5000);

        switch (detection) {
            case LEFT -> drive.followTrajectorySequence(
                    drive.trajectorySequenceBuilder(getCurrentPosition(drive))
                            .splineToLinearHeading(new Pose2d(18.43, 59.84, Math.toRadians(0.00)), Math.toRadians(0.00))
                            .build());
            case CENTER -> drive.followTrajectorySequence(
                    drive.trajectorySequenceBuilder(getCurrentPosition(drive))
                            .waitSeconds(1)
                            .splineToLinearHeading(new Pose2d(18.43, 59.84, Math.toRadians(0.00)), Math.toRadians(0.00))
                            .build());
            case RIGHT -> drive.followTrajectorySequence(
                    drive.trajectorySequenceBuilder(getCurrentPosition(drive))
                            .forward(1)
                            .build());
            default -> {
                telemetry.addLine("Warning: Cup not detected");
                telemetry.update();
                sleep(3000);
            }
        }

        while (rotateMotor.getCurrentPosition()>50){
            rotateMotor.setPower(-0.3);
        }
        rotateMotor.setPower(0.0);
        sleep(5000);

        switch (detection) {
            case LEFT -> drive.followTrajectorySequence(
                    drive.trajectorySequenceBuilder(getCurrentPosition(drive))
                            .splineToLinearHeading(new Pose2d(50.91, 35.72, Math.toRadians(360.00)), Math.toRadians(360.00))
                            .waitSeconds(.1)
                            .addTemporalMarker(() -> autoServo.setPosition(1))
                            .waitSeconds(.5)
                            .addTemporalMarker(() -> autoServo.setPosition(0.9))
                            .waitSeconds(.1)
                            .lineToConstantHeading(new Vector2d(42.93, 62.69))
                            .build());
            case CENTER -> drive.followTrajectorySequence(
                    drive.trajectorySequenceBuilder(getCurrentPosition(drive))
                            .splineToLinearHeading(new Pose2d(51.29, 39.89, Math.toRadians(0.00)), Math.toRadians(0.00))
                            .waitSeconds(.1)
                            .addTemporalMarker(() -> autoServo.setPosition(1))
                            .waitSeconds(.5)
                            .addTemporalMarker(() -> autoServo.setPosition(0.9))
                            .waitSeconds(.1)
                            .lineToConstantHeading(new Vector2d(42.93, 62.69))
                            .build());
            case RIGHT -> drive.followTrajectorySequence(
                    drive.trajectorySequenceBuilder(getCurrentPosition(drive))
                            .forward(1)
                            .build());
            default -> {
                telemetry.addLine("Warning: Cup not detected");
                telemetry.update();
                sleep(3000);
            }
        }

        while (rotateMotor.getCurrentPosition()>50){
            rotateMotor.setPower(-0.3);
        }
        rotateMotor.setPower(0.0);
        sleep(5000);

//        drive.followTrajectorySequence(mergeSequences(sequences));
    }
}
