package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.util.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.RoadRunner.util.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.RoadRunner.util.trajectorysequence.sequencesegment.TrajectorySegment;
import org.firstinspires.ftc.teamcode.Variables.Detection;
import org.firstinspires.ftc.teamcode.Variables.VisionProcessors;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.Arrays;

@Config
@Disabled
@Autonomous(name = "BCOutRight", group = "Linear OpMode")
public class BCOutRight extends MeepMeepBoilerplate{

    TrajectorySequence generate_sequence(BufferedReader reader, SampleMecanumDrive drive, Servo passiveServo, Servo autoServo){
        TrajectorySequenceBuilder sequence = drive.trajectorySequenceBuilder(getCurrentPosition(drive));

        while(true){
            try {
                String line = reader.readLine();
                if (!(line!=null)) break;
               //things
               //back:parameter
               //waitSeconds:parameter
               //strafeRight:parameter
               //strafeLeft:parameter
               //forward:parameter
               //turn:parameter(will be converted to radians)
               //passiveServoSetPosition:parameter
               //autoServoSetPosition:parameter

                //Skip line if it is a comment
                if (line.indexOf("//")>=0){
                    continue;
                }
                String[] elements = line.split(":");
                if(elements.length>1){
                    continue;
                }
                String command = elements[0];
                String parameter = elements[1];

                switch (command){
                    case "back"->{
                        double param = Double.parseDouble(parameter);
                        sequence.back(param);
                    }
                    case "waitSeconds"->{
                        double param = Double.parseDouble(parameter);
                        sequence.waitSeconds(param);
                    }
                    case "strafeRight"->{
                        double param = Double.parseDouble(parameter);
                        sequence.strafeRight(param);
                    }
                    case "strafeLeft"->{
                        double param = Double.parseDouble(parameter);
                        sequence.strafeLeft(param);
                    }
                    case "forward"->{
                        double param = Double.parseDouble(parameter);
                        sequence.forward(param);
                    }
                    case "turn"->{
                        double param = Double.parseDouble(parameter);
                        sequence.turn(Math.toRadians(param));
                    }
                    case "passiveServoSetPosition"->{
                        double param = Double.parseDouble(parameter);
                        sequence.addTemporalMarker(() -> passiveServo.setPosition(param));
                    }
                    case "autoServoSetPosition"->{
                        double param = Double.parseDouble(parameter);
                        sequence.addTemporalMarker(() -> autoServo.setPosition(param));
                    }
                }
            } catch (IOException e) {
                throw new RuntimeException(e);
            }

        }
        return sequence.build();
    }

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
        TrajectoryVelocityConstraint fastConstraint = new MinVelocityConstraint(Arrays.asList(

                new TranslationalVelocityConstraint(50),

                new AngularVelocityConstraint(1)

        ));

        while (opModeInInit()) {
            detection = getDetectionsSingleTFOD();
            telemetry.addData("Detection", detection);
            telemetry.update();
        }
        autoServo.setPosition(0.35);
        switch (detection) {
            case LEFT ->{
                BufferedReader reader;
                try {
                    reader = new BufferedReader(new FileReader("/sdcard/FIRST/training/BCOutRightLeft.txt"));
                    drive.followTrajectorySequence(generate_sequence(reader,drive, passiveServo, autoServo));
                } catch (FileNotFoundException e) {
                    drive.followTrajectorySequence(
                            drive.trajectorySequenceBuilder(getCurrentPosition(drive))
                                    .back(2.0)
                                    .waitSeconds(.25)
                                    .strafeRight(9)
                                    .back(21)
                                    .waitSeconds(.25)
                                    .addTemporalMarker(() -> passiveServo.setPosition(0.1))
                                    .waitSeconds(.25)
                                    .forward(4)
                                    .waitSeconds(.25)
                                    .strafeRight(10)
                                    .waitSeconds(.25)
                                    .turn(Math.toRadians(90))
                                    .waitSeconds(.25)
                                    .back(18.5)
                                    .waitSeconds(.5)
                                    .addTemporalMarker(() -> autoServo.setPosition(1))
                                    .waitSeconds(2)
                                    .addTemporalMarker(() -> autoServo.setPosition(0.8))
                                    .waitSeconds(.5)
                                    .forward(2)
                                    .strafeRight(18)
                                    .back(10)
                                    .waitSeconds(1)
                                    .addTemporalMarker(() -> autoServo.setPosition(0.6))
                                    .waitSeconds(1)
                                    .build()
            );
                }
            }
            case CENTER -> {

                BufferedReader reader;
                try {
                    reader = new BufferedReader(new FileReader("/sdcard/FIRST/training/BCOutRightCenter.txt"));
                    drive.followTrajectorySequence(generate_sequence(reader,drive, passiveServo, autoServo));
                } catch (FileNotFoundException e) {
                    drive.followTrajectorySequence(
                            drive.trajectorySequenceBuilder(getCurrentPosition(drive))
                                    .back(31)
                                    .waitSeconds(.25)
                                    .addDisplacementMarker(() -> passiveServo.setPosition(0.1))
                                    .waitSeconds(.25)
                                    .forward(9)
                                    .turn(Math.toRadians(90))
                                    .waitSeconds(.25)
                                    .back(39.5)
                                    .waitSeconds(.25)
                                    .addTemporalMarker(() -> autoServo.setPosition(1))
                                    .waitSeconds(2)
                                    .addTemporalMarker(() -> autoServo.setPosition(0.8))
                                    .waitSeconds(.5)
                                    .forward(2)
                                    .strafeRight(22)
                                    .back(9)
                                    .waitSeconds(1)
                                    .addTemporalMarker(() -> autoServo.setPosition(0.6))
                                    .waitSeconds(1)
                                    .build()
                    );
            }
            }
            case RIGHT -> {

                BufferedReader reader;
                try {
                    reader = new BufferedReader(new FileReader("/sdcard/FIRST/training/BCOutRightRight.txt"));
                    drive.followTrajectorySequence(generate_sequence(reader,drive, passiveServo, autoServo));
                } catch (FileNotFoundException e) {
                    drive.followTrajectorySequence(
                            drive.trajectorySequenceBuilder(getCurrentPosition(drive))
                            .back(28.0)
                            .turn(Math.toRadians(-90))
                            .back(7)
                            .addTemporalMarker(() -> passiveServo.setPosition(0.1))
                            .waitSeconds(.25)
                            .forward(32)
                            .waitSeconds(.25)
                            .turn(Math.toRadians(180))
                            .waitSeconds(.25)
                            .strafeLeft(2)
                            .waitSeconds(.25)
                            .back(13)
                            .waitSeconds(.25)
                            .addTemporalMarker(() -> autoServo.setPosition(1))
                            .waitSeconds(2)
                            .addTemporalMarker(() -> autoServo.setPosition(0.8))
                            .waitSeconds(.5)
                            .forward(2)
                            .strafeRight(26)
                            .back(9)
                            .waitSeconds(1)
                            .addTemporalMarker(() -> autoServo.setPosition(0.6))
                            .waitSeconds(1)
                            .build()
                    );
                }
            }
            default -> {
                telemetry.addLine("Warning: Cup not detected");
                telemetry.update();
                sleep(3000);
            }
        }



//        drive.followTrajectorySequence(mergeSequences(sequences));
    }
}
