package org.firstinspires.ftc.teamcode.Autonomous;

import static org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive.ACCEL_CONSTRAINT;
import static org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive.VEL_CONSTRAINT;

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
@Autonomous(name = "BCOutLeftEXPERIMENT", group = "Linear OpMode")
public class BCOutLeftEXPERIMENT extends MeepMeepBoilerplate{
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
        autoServo.setPosition(0.73);

        rotateMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotateMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rotateMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rotateMotor.setPower(0.0);
        switch (detection) {
            case LEFT -> drive.followTrajectorySequence(
                    drive.trajectorySequenceBuilder(new Pose2d(15.01, 62.69, Math.toRadians(270.00)))
                            .splineToLinearHeading(new Pose2d(23.37, 39.51, Math.toRadians(450.00)), Math.toRadians(270.00))
                            .splineTo(new Vector2d(36.47, 49.96), Math.toRadians(0.00))
                            .lineTo(new Vector2d(50.72, 36.09))
                            .setReversed(true)
                            .lineTo(new Vector2d(42.74, 61.93))
                            .build()
            );
            case CENTER -> { drive.followTrajectorySequence(
                    drive.trajectorySequenceBuilder(new Pose2d(15.01, 62.69, Math.toRadians(270.00)))
                            .splineToLinearHeading(new Pose2d(12.73, 34.20, Math.toRadians(450.00)), Math.toRadians(270.00))
                            .splineTo(new Vector2d(31.54, 45.02), Math.toRadians(360.00))
                            .lineTo(new Vector2d(50.72, 36.09))
                            .setReversed(true)
                            .lineTo(new Vector2d(42.74, 61.93))
                            .build());
            }
            case RIGHT -> drive.followTrajectorySequence(
                    drive.trajectorySequenceBuilder(new Pose2d(15.01, 62.69, Math.toRadians(270.00)))
                            .splineToLinearHeading(new Pose2d(9.69, 34.96, Math.toRadians(0.00)), Math.toRadians(180.00))
                            .splineTo(new Vector2d(32.30, 32.11), Math.toRadians(0.00))
                            .lineTo(new Vector2d(50.72, 36.09))
                            .setReversed(true)
                            .lineTo(new Vector2d(42.74, 61.93))
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
