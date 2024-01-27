package org.firstinspires.ftc.teamcode.Autonomous;

import static org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive.ACCEL_CONSTRAINT;
import static org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive.VEL_CONSTRAINT;

import com.acmerobotics.dashboard.config.Config;
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
@Autonomous(name = "BCOutLeft(actual)", group = "Linear OpMode")
public class BCOutLeft extends MeepMeepBoilerplate{
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

        rotateMotor.setPower(0.0);
        switch (detection) {
            case LEFT -> drive.followTrajectorySequence(
                    drive.trajectorySequenceBuilder(getCurrentPosition(drive))
                            .splineToConstantHeading(new Vector2d(15,35),2,VEL_CONSTRAINT,ACCEL_CONSTRAINT)
                            .back(2.0)
                            .waitSeconds(.1)
                            .strafeRight(11)
                            .back(21)
                            .waitSeconds(.1)
                            .addTemporalMarker(() -> passiveServo.setPosition(0.2))
                            .waitSeconds(.1)
                            .forward(8)
                            .waitSeconds(.1)
                            .strafeRight(10)
                            .waitSeconds(.1)
                            .turn(Math.toRadians(90))
                            .waitSeconds(.1)
                            .back(20)
                            .waitSeconds(.1)
                            .addTemporalMarker(() -> autoServo.setPosition(0.35))
                            .waitSeconds(.5)
                            .addTemporalMarker(() -> autoServo.setPosition(0.30))
                            .waitSeconds(.1)
                            .forward(2)
                            .strafeRight(13)
                            .back(6)
                            .waitSeconds(1)
                            .addTemporalMarker(() -> autoServo.setPosition(0.0))
                            .waitSeconds(1)
                            .build()
            );
            case CENTER -> { drive.followTrajectorySequence(
                    drive.trajectorySequenceBuilder(getCurrentPosition(drive))
                            .back(31)
                            .waitSeconds(.25)
                            .addTemporalMarker(() -> passiveServo.setPosition(0.2))
                            .waitSeconds(.25)
                            .forward(9)
                            .turn(Math.toRadians(90))
                            .waitSeconds(.1)
                            .back(39)
                            .waitSeconds(.1)
                            .strafeLeft(1)
                            .waitSeconds(.1)
                            .back(1.5)
                            .addTemporalMarker(() -> autoServo.setPosition(1))
                            .waitSeconds(.5)
                            .addTemporalMarker(() -> autoServo.setPosition(0.9))
                            .waitSeconds(.25)
                            .forward(2)
                            .strafeRight(19.5)
                            .back(6)
                            .waitSeconds(1)
                            .addTemporalMarker(() -> autoServo.setPosition(0.65))
                            .waitSeconds(1)
                            .build());
            }
            case RIGHT -> drive.followTrajectorySequence(
                    drive.trajectorySequenceBuilder(getCurrentPosition(drive))
                            .back(28.0)
                            .turn(Math.toRadians(-90))
                            .back(4)
                            .addTemporalMarker(() -> passiveServo.setPosition(0.2))
                            .waitSeconds(.25)
                            .forward(32)
                            .waitSeconds(.25)
                            .turn(Math.toRadians(180))
                            .waitSeconds(.25)
                            .back(13)
                            .waitSeconds(.25)
                            .addTemporalMarker(() -> autoServo.setPosition(1))
                            .waitSeconds(2)
                            .addTemporalMarker(() -> autoServo.setPosition(0.9))
                            .waitSeconds(.5)
                            .forward(2)
                            .strafeRight(26)
                            .back(6)
                            .waitSeconds(1)
                            .addTemporalMarker(() -> autoServo.setPosition(0.65))
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
