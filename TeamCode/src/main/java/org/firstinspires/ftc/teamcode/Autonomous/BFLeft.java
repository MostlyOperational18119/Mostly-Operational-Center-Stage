package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.config.Config;
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
@Autonomous(name = "BackBlueLeft(Actual)", group = "Linear OpMode")
public class BFLeft extends MeepMeepBoilerplate{
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
        switch (detection) {
            case LEFT -> drive.followTrajectorySequence(
                    drive.trajectorySequenceBuilder(getCurrentPosition(drive))
                            .back(28.0)
                            .turn(Math.toRadians(90))
                            .back(8)
                            .addTemporalMarker(() -> passiveServo.setPosition(0.2))
                            .waitSeconds(.25)
                            .forward(8)
                            .strafeRight(26)
                            .waitSeconds(.5)
                            .setVelConstraint(slowConstraint)
                            .addTemporalMarker(() -> rotateMotor.setTargetPosition(1550))
                            .addTemporalMarker(() -> rotateMotor.setPower(.3))
                            .waitSeconds(5)
                            .back(50)
                            .waitSeconds(1)
                            .resetVelConstraint()
                            .addTemporalMarker(() -> rotateMotor.setTargetPosition(0))
                            .addTemporalMarker(() -> rotateMotor.setPower(-.3))
                            .waitSeconds(4)
                            .back(30)
                            .strafeLeft(5)
                            .waitSeconds(.5)
                            .back(18)
                            .addTemporalMarker(() -> autoServo.setPosition(1))
                            .waitSeconds(2)
                            .addTemporalMarker(() -> autoServo.setPosition(0.9))
                            .waitSeconds(1)
                            .forward(2)
                            .addTemporalMarker(() -> autoServo.setPosition(0.78))
                            .build()
            );
            case CENTER -> { drive.followTrajectorySequence(
                    drive.trajectorySequenceBuilder(getCurrentPosition(drive))
                            .back(31.5)
                            .addTemporalMarker(() -> passiveServo.setPosition(0.2))
                            .forward(28)
                            .turn(Math.toRadians(90))
                            .setVelConstraint(slowConstraint)
                            .addTemporalMarker(() -> rotateMotor.setTargetPosition(1850))
                            .addTemporalMarker(() -> rotateMotor.setPower(.3))
                            .waitSeconds(5)
                            .back(50)
                            .waitSeconds(1)
                            .resetVelConstraint()
                            .addTemporalMarker(() -> rotateMotor.setTargetPosition(0))
                            .addTemporalMarker(() -> rotateMotor.setPower(-.3))
                            .waitSeconds(4)
                            .back(33)
                            .waitSeconds(.25)
                            .strafeLeft(20)
                            .waitSeconds(.25)
                            .back(18)
                            .addTemporalMarker(() -> autoServo.setPosition(1))
                            .waitSeconds(2)
                            .addTemporalMarker(() -> autoServo.setPosition(0.9))
                            .forward(2)
                            .addTemporalMarker(() -> autoServo.setPosition(0.78))
                            .waitSeconds(1)
                            .build());
            }
            case RIGHT -> drive.followTrajectorySequence(
                    drive.trajectorySequenceBuilder(getCurrentPosition(drive))
                            .back(28.0)
                            .turn(Math.toRadians(-90))
                            .back(3)
                            .addTemporalMarker(() -> passiveServo.setPosition(0.2))
                            .waitSeconds(.25)
                            .forward(3)
                            .strafeLeft(26)
                            .turn(Math.toRadians(180))
                            .setVelConstraint(slowConstraint)
                            .addTemporalMarker(() -> goDownAuto())
                            .waitSeconds(5)
                            .back(50)
                            .waitSeconds(.5)
                            .resetVelConstraint()
                            .addTemporalMarker(() -> goUpAuto())
                            .waitSeconds(4)
                            .back(33)
                            .strafeLeft(12)
                            .back(4)
                            .addTemporalMarker(() -> autoServo.setPosition(1))
                            .waitSeconds(1)
                            .addTemporalMarker(() -> autoServo.setPosition(.9))
                            .forward(2)
                            .addTemporalMarker(() -> autoServo.setPosition(.78))
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
