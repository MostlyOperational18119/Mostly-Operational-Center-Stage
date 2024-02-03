package org.firstinspires.ftc.teamcode.Autonomous.NoSplineAuto;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Autonomous.MeepMeepBoilerplate;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.util.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.RoadRunner.util.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.Variables.Detection;
import org.firstinspires.ftc.teamcode.Variables.VisionProcessors;

import java.io.BufferedReader;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.util.Arrays;

@Config
@Disabled
@Autonomous(name = "RFLeft", group = "Linear OpMode")
public class RFLeftDamienAndJesse extends MeepMeepBoilerplate {

    TrajectorySequence generate_sequence(BufferedReader reader, SampleMecanumDrive drive, Servo passiveServo, Servo autoServo){
        autoServo.setPosition(0.78);
        TrajectorySequenceBuilder sequence = drive.trajectorySequenceBuilder(getCurrentPosition(drive));
        for (String line: reader.lines().toArray(String[]::new)) {

            telemetry.addLine("Iterating! "+line);
            telemetry.update();
            if (line.indexOf("//")>=0){
                telemetry.addLine("Found comment: "+line);
                continue;
            }

            String[] elements = line.split(":");
            assert elements.length==2;
            if(elements.length<=1){
                continue;
            }
            String command = elements[0];
            String parameter = elements[1];
            String bs = ("Stuff: " + command + " more stuff: " + parameter +" one last thing: " + line);
            telemetry.addLine(bs);
            Log.d("RFLeft", bs);
            switch (command){
                case "back"->{
                    double param = Double.parseDouble(parameter);
                    sequence = sequence.back(param);
                }
                case "waitSeconds"->{
                    double param = Double.parseDouble(parameter);
                    sequence = sequence.waitSeconds(param);
                }
                case "strafeRight"->{
                    double param = Double.parseDouble(parameter);
                    sequence = sequence.strafeRight(param);
                }
                case "strafeLeft"->{
                    double param = Double.parseDouble(parameter);
                    sequence = sequence.strafeLeft(param);
                }
                case "forward"->{
                    double param = Double.parseDouble(parameter);
                    sequence = sequence.forward(param);
                }
                case "turn"->{
                    double param = Double.parseDouble(parameter);
                    sequence = sequence.turn(Math.toRadians(param));
                }
                case "passiveServoSetPosition"->{
                    double param = Double.parseDouble(parameter);
                    sequence = sequence.addTemporalMarker(() -> passiveServo.setPosition(param));
                }
                case "autoServoSetPosition"->{
                    double param = Double.parseDouble(parameter);
                    sequence = sequence.addTemporalMarker(() -> autoServo.setPosition(param));
                }
                default -> {
                    telemetry.addLine("Defaulted!!!!!!!: "+ command);
                    telemetry.update();
                }
            }

        }
        return sequence.build();
    }


    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Servo passiveServo = hardwareMap.get(Servo.class, "passiveServo");
        Servo autoServo = hardwareMap.get(Servo.class, "autoServo");
        autoServo.setPosition(0.78);
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
//        autoServo.setPosition(0.35);
        switch (detection) {
            case LEFT ->{

                BufferedReader reader;
                try {
                    reader = new BufferedReader(new FileReader("/sdcard/FIRST/training/RFLeftLeft.txt"));
                    drive.followTrajectorySequence(generate_sequence(reader,drive, passiveServo, autoServo));
                } catch (FileNotFoundException e) {
                    //Fallback --- Will not run if file exists. Change RFLeftLeft.txt on the robots sd card to change behavior.
                    drive.followTrajectorySequence(
                            drive.trajectorySequenceBuilder(getCurrentPosition(drive))
                            .back(28.0)
                            .turn(Math.toRadians(90))
                            .back(6)
                            .waitSeconds(1)
                            .addTemporalMarker(() -> passiveServo.setPosition(0.1))
                            .waitSeconds(1)
                            .forward(4)
                            .build()
            );
                }
      }
            case CENTER -> {

                BufferedReader reader;
                try {
                    reader = new BufferedReader(new FileReader("/sdcard/FIRST/training/RFLeftCenter.txt"));
                    drive.followTrajectorySequence(generate_sequence(reader,drive, passiveServo, autoServo));
                } catch (FileNotFoundException e) {
                    drive.followTrajectorySequence(
                            drive.trajectorySequenceBuilder(getCurrentPosition(drive))
                            .back(31.0)
                            .waitSeconds(1)
                            .addDisplacementMarker(() -> passiveServo.setPosition(0.1))
                            .waitSeconds(1)
                            .forward(4)
                            .build()
            );
                }
            }
            case RIGHT -> {

                BufferedReader reader;
                try {
                    reader = new BufferedReader(new FileReader("/sdcard/FIRST/training/RFLeftRight.txt"));
                    drive.followTrajectorySequence(generate_sequence(reader,drive, passiveServo, autoServo));
                } catch (FileNotFoundException e) {
                    drive.followTrajectorySequence(
                            drive.trajectorySequenceBuilder(getCurrentPosition(drive))
                            .back(28.0)
                            .turn(Math.toRadians(-90))
                            .back(3.5)
                            .waitSeconds(1)
                            .addTemporalMarker(() -> passiveServo.setPosition(0.2))
                            .waitSeconds(1)
                            .forward(5)
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
