package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Variables.Detection;
import org.firstinspires.ftc.teamcode.Variables.VisionProcessors;

import java.util.Arrays;

@Config
@Autonomous(name = "BlueRedLeft(actual)", group = "Linear OpMode")
public class RFLeft extends MeepMeepBoilerplate{
    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Servo passiveServo = hardwareMap.get(Servo.class, "passiveServo");
        Servo autoServo = hardwareMap.get(Servo.class, "autoServo");
        initVision(VisionProcessors.TFOD);
        Detection detection = Detection.UNKNOWN;
        TrajectoryVelocityConstraint slowConstraint = new MinVelocityConstraint(Arrays.asList(

                new TranslationalVelocityConstraint(20),

                new AngularVelocityConstraint(1)

        ));
        while (opModeInInit()) {
            detection = getDetectionsSingleTFOD();
            telemetry.addData("Detection", detection);
            telemetry.update();
        }
        autoServo.setPosition(0.35);
        switch (detection) {
            case LEFT -> drive.followTrajectorySequence(
                    drive.trajectorySequenceBuilder(getCurrentPosition(drive))
                            .forward(28.0)
                            .turn(Math.toRadians(90))
                            .forward(3)
                            .waitSeconds(1)
                            .addTemporalMarker(() -> passiveServo.setPosition(0.1))
                            .waitSeconds(1)
                            .back(6)
                            .strafeRight(25)
                            .waitSeconds(.5)
                            .back(78)
                            .waitSeconds(.25)
                            .turn(Math.toRadians(180))
                            .waitSeconds(.25)
                            .strafeRight(15)
                            .waitSeconds(.25)
                            .forward(18)
                            .waitSeconds(.25)
                            .addTemporalMarker(() -> autoServo.setPosition(0.68))
                            .waitSeconds(1.5)
                            .addTemporalMarker(() -> autoServo.setPosition(0.6))
                            .waitSeconds(.5)
                            .addTemporalMarker(() -> autoServo.setPosition(0.35))
                            .waitSeconds(.5)
                            .back(4)
                            .waitSeconds(.25)
                            .strafeLeft(20)
                            .waitSeconds(.25)
                            .forward(16)
                            .build()
            );
            case CENTER -> { drive.followTrajectorySequence(
                    drive.trajectorySequenceBuilder(getCurrentPosition(drive))
                            .forward(31.5)
                            .waitSeconds(.25)
                            .addDisplacementMarker(() -> passiveServo.setPosition(0.1))
                            .waitSeconds(.25)
                            .forward(23)
                            .turn(Math.toRadians(-90))
                            .waitSeconds(.5)
                            .back(78)
                            .waitSeconds(.25)
                            .turn(Math.toRadians(180))
                            .waitSeconds(.25)
                            .strafeRight(21)
                            .waitSeconds(.25)
                            .forward(18)
                            .waitSeconds(.25)
                            .addTemporalMarker(() -> autoServo.setPosition(0.68))
                            .waitSeconds(1.5)
                            .addTemporalMarker(() -> autoServo.setPosition(0.6))
                            .waitSeconds(.5)
                            .addTemporalMarker(() -> autoServo.setPosition(0.35))
                            .waitSeconds(.5)
                            .back(4)
                            .waitSeconds(.25)
                            .strafeLeft(26)
                            .waitSeconds(.25)
                            .forward(16)
                            .build());
            }
            case RIGHT -> drive.followTrajectorySequence(
                    drive.trajectorySequenceBuilder(getCurrentPosition(drive))
                            .forward(28.0)
                            .turn(Math.toRadians(-90))
                            .forward(6)
                            .waitSeconds(1)
                            .addTemporalMarker(() -> passiveServo.setPosition(0.1))
                            .waitSeconds(1)
                            .back(5)
//                            .strafeLeft(22)
//                            .waitSeconds(.5)
//                            .forward(18)
//                            .setVelConstraint(slowConstraint)
//                            .waitSeconds(.25)
//                            .forward(20)
//                            .resetVelConstraint()
//                            .forward(43)
//                            .waitSeconds(.25)
//                            .turn(Math.toRadians(-90))
//                            .waitSeconds(.25)
//                            .forward(21.5)
//                            .waitSeconds(.25)
//                            .strafeLeft(4)
//                            .addTemporalMarker(() -> autoServo.setPosition(0.68))
//                            .waitSeconds(1.5)
//                            .addTemporalMarker(() -> autoServo.setPosition(0.6))
//                            .waitSeconds(.5)
//                            .addTemporalMarker(() -> autoServo.setPosition(0.35))
//                            .waitSeconds(.5)
//                            .strafeRight(2)
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
