package org.firstinspires.ftc.teamcode.Autonomous.OtherTesting;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Autonomous.MeepMeepBoilerplate;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Variables.Detection;
import org.firstinspires.ftc.teamcode.Variables.VisionProcessors;

import java.util.Arrays;

@Config
@Disabled
@Autonomous(name = "12/3 TESTING", group = "TESTING")
public class JamesTesting123 extends MeepMeepBoilerplate {
    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Servo passiveServo = hardwareMap.get(Servo.class, "passiveServo");
        Servo autoServo = hardwareMap.get(Servo.class, "autoServo");
        DcMotor rotateMotor = hardwareMap.get(DcMotor.class, "motorSlideRotate");
        rotateMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotateMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rotateMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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
//        switch (detection) {
//            case LEFT -> drive.followTrajectorySequence(
//                    drive.trajectorySequenceBuilder(getCurrentPosition(drive))
//                            .build()
//            );
//            case CENTER -> drive.followTrajectorySequence(
//                    drive.trajectorySequenceBuilder(getCurrentPosition(drive))
//                            .build()
//            );
//            case RIGHT -> drive.followTrajectorySequence(
//                    drive.trajectorySequenceBuilder(getCurrentPosition(drive))
//                            .build()
//            );
//            default -> {
//                telemetry.addLine("Warning: Cup not detected");
//                telemetry.update();
//                sleep(3000);
//            }
//        }

        while (rotateMotor.getCurrentPosition()<1000){
            rotateMotor.setPower(0.5);
        }
        rotateMotor.setPower(0.001);
        sleep(5000);

        while (rotateMotor.getCurrentPosition()>50){
            rotateMotor.setPower(-0.3);
        }
        rotateMotor.setPower(0.0);
        sleep(20000);

//        drive.followTrajectorySequence(mergeSequences(sequences));
    }
}
