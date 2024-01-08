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
                    drive.trajectorySequenceBuilder(new Pose2d(15.01, 62.69, Math.toRadians(90.00)))
                            .lineToConstantHeading(new Vector2d(23.18, 36.85))
                            .splineToLinearHeading(new Pose2d(50.91, 43.69, Math.toRadians(0.00)), Math.toRadians(0.00))
                            .lineToConstantHeading(new Vector2d(41.41, 62.31))
                            .build()
            );
            case CENTER -> { drive.followTrajectorySequence(
                    drive.trajectorySequenceBuilder(new Pose2d(15.01, 62.69, Math.toRadians(90.00)))
                            .lineToConstantHeading(new Vector2d(12.35, 33.82))
                            .splineToLinearHeading(new Pose2d(50.91, 40.46, Math.toRadians(0.00)), Math.toRadians(0.00))
                            .lineToConstantHeading(new Vector2d(41.41, 62.31))
                            .build());
            }
            case RIGHT -> drive.followTrajectorySequence(
                    drive.trajectorySequenceBuilder(new Pose2d(15.01, 62.69, Math.toRadians(90.00)))
                            .setReversed(true)
                            .splineToLinearHeading(new Pose2d(15.75, 45.61, Math.toRadians(90.00)), Math.toRadians(-90.00))
                            .splineToLinearHeading(new Pose2d(6.95, 35.39, Math.toRadians(360.00)), Math.toRadians(180.00))
                            .setReversed(false)
                            .lineToConstantHeading(new Vector2d(51.29, 32.49))
                            .lineToConstantHeading(new Vector2d(44.45, 62.69))
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
