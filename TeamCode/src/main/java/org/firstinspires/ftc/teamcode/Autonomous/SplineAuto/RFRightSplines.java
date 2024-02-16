package org.firstinspires.ftc.teamcode.Autonomous.SplineAuto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Autonomous.MeepMeepBoilerplate;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Variables.Detection;
import org.firstinspires.ftc.teamcode.Variables.VisionProcessors;

import java.util.Arrays;

@Config
@Autonomous(name = "RFRightSplines (Main)", group = "Linear OpMode")
public class RFRightSplines extends MeepMeepBoilerplate{
    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Servo passiveServo = hardwareMap.get(Servo.class, "passiveServo");
        Servo autoServo = hardwareMap.get(Servo.class, "autoServo");
        DcMotor rotateMotor = hardwareMap.get(DcMotor.class, "motorSlideRotate");
        DcMotor slideMotor = hardwareMap.get(DcMotor.class, "motorSlideLeft");
        Servo boxServo = hardwareMap.get(Servo.class, "boxServo");

        initVision(VisionProcessors.TFOD);
        initBlinkinSafe(RevBlinkinLedDriver.BlinkinPattern.RED); // Inits Blinkin with a colour corresponding to the Auto
        Detection detection = Detection.UNKNOWN;
        TrajectoryVelocityConstraint slowConstraint = new MinVelocityConstraint(Arrays.asList(

                new TranslationalVelocityConstraint(7),

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

        autoServo.setPosition(0.245);

        rotateMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotateMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rotateMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rotateMotor.setPower(0.0);

        drive.setPoseEstimate( new Pose2d(-34.96, -62.69, Math.toRadians(-90.00)));

        switch (detection) {
            case LEFT -> drive.followTrajectorySequence(
                    drive.trajectorySequenceBuilder(new Pose2d(-34.96, -62.69, Math.toRadians(-90.00)))
                            .lineToConstantHeading(new Vector2d(-47.30, -40.08))
                            .waitSeconds(.25)
                            .addTemporalMarker(() -> passiveServo.setPosition(0.2))
                            .waitSeconds(1)
                            .splineToLinearHeading(new Pose2d(-35.91, -58.5, Math.toRadians(360.00)), Math.toRadians(360.00))
                            .build());
            case CENTER -> {drive.followTrajectorySequence(
                    drive.trajectorySequenceBuilder(new Pose2d(-34.96, -62.69, Math.toRadians(-90.00)))
                            .lineToConstantHeading(new Vector2d(-37.34, -30.82))
                            .waitSeconds(.25)
                            .addTemporalMarker(() -> passiveServo.setPosition(0.2))
                            .waitSeconds(1)
                            .splineToLinearHeading(new Pose2d(-35.91, -58.5, Math.toRadians(360.00)), Math.toRadians(360.00))
                            .build());
            }
            case RIGHT -> drive.followTrajectorySequence(
                    drive.trajectorySequenceBuilder(new Pose2d(-34.96, -62.69, Math.toRadians(-90.00)))
                            .setReversed(true)
                            .lineToConstantHeading(new Vector2d(-38, -37))
                            .splineToLinearHeading(new Pose2d(-31.32, -35.5, Math.toRadians(180.00)), Math.toRadians(0))
                            .waitSeconds(.25)
                            .addTemporalMarker(() -> passiveServo.setPosition(0.2))
                            .waitSeconds(1)
                            .setReversed(false)
                            .splineToConstantHeading(new Vector2d(-36.32, -34), Math.toRadians(0))
                            .splineToLinearHeading(new Pose2d(-38.91, -58.5, Math.toRadians(360.00)), Math.toRadians(360.00))
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

        drive.followTrajectorySequence(drive.trajectorySequenceBuilder(getCurrentPosition(drive))
                        .splineToConstantHeading(new Vector2d(17, -58.5), Math.toRadians(0.00))
                        .build());

        while (rotateMotor.getCurrentPosition()>300){
            rotateMotor.setPower(-.5);
        }
        rotateMotor.setPower(0.0);
        sleep(6000);

        switch (detection) {
            case LEFT -> drive.followTrajectorySequence(
                    drive.trajectorySequenceBuilder(getCurrentPosition(drive))
                            .splineToLinearHeading(new Pose2d(43.49, -22.41, Math.toRadians(180.00)), Math.toRadians(180.00))
                            .setVelConstraint(slowConstraint)
                            .splineToConstantHeading(new Vector2d(50.5, -22.41), Math.toRadians(180.00))
                            .addTemporalMarker(() -> autoServo.setPosition(0.75))
                            .waitSeconds(2)
                            .addTemporalMarker(() -> autoServo.setPosition(1.0))
                            .waitSeconds(2)
                            .addTemporalMarker(() -> autoServo.setPosition(0.75))
                            .waitSeconds(1)
                            .build());
            case CENTER -> drive.followTrajectorySequence(
                    drive.trajectorySequenceBuilder(getCurrentPosition(drive))
                            .splineToLinearHeading(new Pose2d(43.49, -30.7, Math.toRadians(180.00)), Math.toRadians(180.00))
                            .setVelConstraint(slowConstraint)
                            .splineToConstantHeading(new Vector2d(50.5, -30.7), Math.toRadians(180.00))
                            .addTemporalMarker(() -> autoServo.setPosition(0.75))
                            .waitSeconds(2)
                            .addTemporalMarker(() -> autoServo.setPosition(1.0))
                            .waitSeconds(2)
                            .addTemporalMarker(() -> autoServo.setPosition(0.75))
                            .waitSeconds(1)
                            .build());
            case RIGHT -> drive.followTrajectorySequence(
                    drive.trajectorySequenceBuilder(getCurrentPosition(drive))
                            .splineToLinearHeading(new Pose2d(43.49, -36, Math.toRadians(180.00)), Math.toRadians(180.00))
                            .setVelConstraint(slowConstraint)
                            .splineToConstantHeading(new Vector2d(50.5, -36), Math.toRadians(180.00))
                            .addTemporalMarker(() -> autoServo.setPosition(0.75))
                            .waitSeconds(2)
                            .addTemporalMarker(() -> autoServo.setPosition(1.0))
                            .waitSeconds(2)
                            .addTemporalMarker(() -> autoServo.setPosition(0.75))
                            .waitSeconds(1)
                            .build());
            default -> {
                telemetry.addLine("Warning: Cup not detected");
                telemetry.update();
                sleep(3000);
            }
        }

//        while (Math.abs(slideMotor.getCurrentPosition()) < 500){
//            slideMotor.setPower(-0.3);
//        }
//
//        sleep(1000);
//        boxServo.setPosition(.45);
//        sleep(1000);
//
//        while (rotateMotor.getCurrentPosition()>20){
//            rotateMotor.setPower(-0.3);
//        }
//
//        sleep(1000);
//        rotateMotor.setPower(0.0);
//
//        sleep(1000);
//
//        while (Math.abs(slideMotor.getCurrentPosition()) > 20){
//            slideMotor.setPower(0.3);
//        }
//
//        boxServo.setPosition(.62);
//

//        drive.followTrajectorySequence(mergeSequences(sequences));
    }
}
