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
public class BFExperimentByJames extends MeepMeepBoilerplate{
    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Servo passiveServo = hardwareMap.get(Servo.class, "passiveServo");
        Servo autoServo = hardwareMap.get(Servo.class, "autoServo");
        DcMotor rotateMotor = hardwareMap.get(DcMotor.class, "motorSlideRotate");
        DcMotor slideMotor = hardwareMap.get(DcMotor.class, "motorSlideLeft");
        Servo boxServo = hardwareMap.get(Servo.class, "boxServo");

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

        drive.setPoseEstimate( new Pose2d(-34.96, -62.69, Math.toRadians(-90.00)));

        switch (detection) {
            case LEFT -> drive.followTrajectorySequence(
                    drive.trajectorySequenceBuilder(new Pose2d(-34.96, -62.69, Math.toRadians(-90.00)))
                            .lineToConstantHeading(new Vector2d(-47.30, -40.08))
                            .waitSeconds(.1)
                            .addTemporalMarker(() -> passiveServo.setPosition(0.2))
                            .waitSeconds(.15)
                            .splineToLinearHeading(new Pose2d(-35.91, -58.82, Math.toRadians(360.00)), Math.toRadians(360.00))
                            .build());
            case CENTER -> {drive.followTrajectorySequence(
                    drive.trajectorySequenceBuilder(new Pose2d(-34.96, -62.69, Math.toRadians(-90.00)))
                            .lineToConstantHeading(new Vector2d(-35.34, -30.82))
                            .waitSeconds(.1)
                            .addTemporalMarker(() -> passiveServo.setPosition(0.2))
                            .waitSeconds(.15)
                            .splineToLinearHeading(new Pose2d(-35.91, -58.82, Math.toRadians(360.00)), Math.toRadians(360.00))
                            .build());
            }
            case RIGHT -> drive.followTrajectorySequence(
                    drive.trajectorySequenceBuilder(new Pose2d(-34.96, -62.69, Math.toRadians(-90.00)))
                            .setReversed(true)
                            .splineToLinearHeading(new Pose2d(-31.32, -32.85, Math.toRadians(180.00)), Math.toRadians(0))
                            .waitSeconds(.1)
                            .addTemporalMarker(() -> passiveServo.setPosition(0.2))
                            .waitSeconds(.15)
                            .setReversed(false)
                            .splineToLinearHeading(new Pose2d(-35.91, -58.82, Math.toRadians(360.00)), Math.toRadians(360.00))
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

        drive.followTrajectorySequence(drive.trajectorySequenceBuilder(getCurrentPosition(drive))
                .splineToConstantHeading(new Vector2d(9, -58.82), Math.toRadians(0.00))
                .build());


        while (rotateMotor.getCurrentPosition()>400){
            rotateMotor.setPower(-0.3);
        }
        rotateMotor.setPower(0.0);
        sleep(5000);

        switch (detection) {
            case LEFT -> drive.followTrajectorySequence(
                    drive.trajectorySequenceBuilder(getCurrentPosition(drive))
                            .splineToConstantHeading(new Vector2d(23.49, -22.41), Math.toRadians(0.00))
                            .splineToConstantHeading(new Vector2d(51.31, -22.41), Math.toRadians(0.00))
                            .build());
            case CENTER -> drive.followTrajectorySequence(
                    drive.trajectorySequenceBuilder(getCurrentPosition(drive))
                            .splineToConstantHeading(new Vector2d(23.49, -29), Math.toRadians(0.00))
                            .splineToConstantHeading(new Vector2d(51.31, -29), Math.toRadians(0.00))
                            .build());
            case RIGHT -> drive.followTrajectorySequence(
                    drive.trajectorySequenceBuilder(getCurrentPosition(drive))
                            .splineToConstantHeading(new Vector2d(23.49, -35.89), Math.toRadians(0.00))
                            .splineToConstantHeading(new Vector2d(51.31, -35.89), Math.toRadians(0.00))
                            .build());
            default -> {
                telemetry.addLine("Warning: Cup not detected");
                telemetry.update();
                sleep(3000);
            }
        }

        while (Math.abs(slideMotor.getCurrentPosition()) < 500){
            slideMotor.setPower(-0.3);
        }

        sleep(2000);
        boxServo.setPosition(.45);
        sleep(2000);

        while (rotateMotor.getCurrentPosition()>20){
            rotateMotor.setPower(-0.3);
        }

        sleep(2000);
        rotateMotor.setPower(0.0);

        sleep(2000);

        while (Math.abs(slideMotor.getCurrentPosition()) > 20){
            slideMotor.setPower(0.3);
        }

//        drive.followTrajectorySequence(mergeSequences(sequences));
    }
}