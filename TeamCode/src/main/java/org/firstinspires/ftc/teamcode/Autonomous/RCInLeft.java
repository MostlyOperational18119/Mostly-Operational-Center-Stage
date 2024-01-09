package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Variables.Detection;
import org.firstinspires.ftc.teamcode.Variables.VisionProcessors;

import java.util.Arrays;

@Config
@Autonomous(name = "RCInLeft (Testing Down)", group = "Linear OpMode")
public class RCInLeft extends MeepMeepBoilerplate{
    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Servo passiveServo = hardwareMap.get(Servo.class, "passiveServo");
        Servo autoServo = hardwareMap.get(Servo.class, "autoServo");
        initVision(VisionProcessors.TFOD);
        Detection detection = Detection.UNKNOWN;
        TrajectoryVelocityConstraint fastConstraint = new MinVelocityConstraint(Arrays.asList(

                new TranslationalVelocityConstraint(50),

                new AngularVelocityConstraint(1)

        ));
        while (opModeInInit()) {
            detection = getDetectionsSingleTFOD();
            telemetry.addData("Detection", detection);
            telemetry.update();
        }
        autoServo.setPosition(0.78);
        switch (detection) {
            case LEFT -> drive.followTrajectorySequence(
                    drive.trajectorySequenceBuilder(getCurrentPosition(drive))
                            .addTemporalMarker(() -> goDownAuto())
                            .build()
            );
            case CENTER -> { drive.followTrajectorySequence(
                    drive.trajectorySequenceBuilder(getCurrentPosition(drive))
                            .back(31.5)
                            .waitSeconds(.25)
                            .addTemporalMarker(() -> passiveServo.setPosition(0.2))
                            .waitSeconds(.25)
                            .forward(10)
                            .waitSeconds(.25)
                            .turn(Math.toRadians(-90))
                            .waitSeconds(.25)
                            .back(12)
                            .strafeRight(9)
                            .back(24.5)
                            .waitSeconds(.25)
                            .addTemporalMarker(() -> autoServo.setPosition(1))
                            .waitSeconds(.5)
                            .addTemporalMarker(() -> autoServo.setPosition(0.9))
                            .waitSeconds(.25)
                            .forward(3)
                            .strafeRight(20)
                            .back(11)
                            .waitSeconds(1)
                            .addTemporalMarker(() -> autoServo.setPosition(0.65))
                            .waitSeconds(1)
                            .build());
            }
            case RIGHT -> drive.followTrajectorySequence(
                    drive.trajectorySequenceBuilder(getCurrentPosition(drive))
                            .back(2.0)
                            .waitSeconds(.25)
                            .strafeLeft(8)
                            .back(21)
                            .waitSeconds(.25)
                            .addTemporalMarker(() -> passiveServo.setPosition(0.2))
                            .waitSeconds(.25)
                            .forward(9.5)
                            .waitSeconds(.25)
                            .turn(Math.toRadians(-90))
                            .waitSeconds(.25)
                            .back(12.5)
                            .waitSeconds(.25)
                            .strafeRight(12)
                            .waitSeconds(.25)
                            .back(15.5)
                            .waitSeconds(.25)
                            .addTemporalMarker(() -> autoServo.setPosition(1))
                            .waitSeconds(.5)
                            .addTemporalMarker(() -> autoServo.setPosition(0.9))
                            .waitSeconds(.25)
                            .forward(3)
                            .strafeRight(26)
                            .back(10)
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
