package org.firstinspires.ftc.teamcode.Autonomous.OtherTesting;

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
import java.io.BufferedWriter;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@Config
@Autonomous(name = "auto_test", group = "Linear OpMode")
@Disabled
public class test extends MeepMeepBoilerplate {

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
            sleep(100);
        }

        boolean do_recording = true;
        boolean waiting = true;
        telemetry.addLine("Odometry Recorder Mark 1\nPress a to continue to recording\nPress b to execute last record\n\nControls for recording---\nThe robot being backwards has been taken into account\nUse arrow keys to move around/strafe\nUse right bumper for auto servo (it progresses in stages. Each time you press the bumper, it will move to the next preset position)\nUse left bumper to release active servo (This will only release; you cannot close it)\nUse b/y to turn 90/-90 degrees\nDo nothing to add wait time\nWhen recording, press a to stop recording and save to file.\n\nNote that the recording process will be a bit slower than when the record is being executed.");
        telemetry.update();
        while(waiting){
            if(gamepad1.a){
                waiting=false;

            }else if(gamepad1.b){
                waiting=false;
                do_recording=false;
            }
      sleep(100);
        }
        if(!do_recording){
            BufferedReader reader;
            try {
                reader = new BufferedReader(new FileReader("/sdcard/FIRST/training/OdometryRecordText.txt"));
                drive.followTrajectorySequence(generate_sequence(reader,drive, passiveServo, autoServo));
            } catch (FileNotFoundException e) {
                telemetry.addLine("File Read Failed");
                telemetry.update();
            }

        }
        List<String> sequence = new ArrayList<String>();
        //back:parameter
        //waitSeconds:parameter
        //strafeRight:parameter
        //strafeLeft:parameter
        //forward:parameter
        //turn:parameter(will be converted to radians)
        //passiveServoSetPosition:parameter
        //autoServoSetPosition:parameter
        int auto_servo_stage = 0;
        telemetry.addLine("Controls for recording---\nThe robot being backwards has been taken into account\nUse arrow keys to move around/strafe\nUse right bumper for auto servo (it progresses in stages. Each time you press the bumper, it will move to the next preset position)\nUse left bumper to release active servo (This will only release; you cannot close it)\nUse b/y to turn 90/-90 degrees\nDo nothing to add wait time\nWhen recording, press a to stop recording and save to file.\n\nNote that the recording process will be a bit slower than when the record is being executed.");
        telemetry.update();
        sleep(1000);
        while(do_recording){
            if(gamepad1.a){
                BufferedWriter outputWriter;
                try {
                    telemetry.addLine("Saving...");
                    telemetry.update();
                    outputWriter = new BufferedWriter( new FileWriter("/sdcard/FIRST/training/OdometryRecordTest.txt"));
                    for (int i = 0; i< sequence.size()-1; i++) {
                        outputWriter.write(sequence.get(i));
                        outputWriter.newLine();
                    }
                    outputWriter.flush();
                    outputWriter.close();
                } catch (IOException e) {
                    throw new RuntimeException(e);
                }
                break;
            }
            if(gamepad1.dpad_up){

                    drive.followTrajectorySequence(
                            drive.trajectorySequenceBuilder(getCurrentPosition(drive))
                            .back(1.0)
                            .build()
            );
                    sequence.add("back:1");
            }else if(gamepad1.dpad_down){
                drive.followTrajectorySequence(
                        drive.trajectorySequenceBuilder(getCurrentPosition(drive))
                                .forward(1.0)
                                .build()
                );
                sequence.add("forward:1");
            }else if(gamepad1.dpad_left){
                drive.followTrajectorySequence(
                        drive.trajectorySequenceBuilder(getCurrentPosition(drive))
                                .strafeRight(1.0)
                                .build()
                );
                sequence.add("strafeRight:1");
            }else if(gamepad1.dpad_right){
                drive.followTrajectorySequence(
                        drive.trajectorySequenceBuilder(getCurrentPosition(drive))
                                .strafeLeft(1.0)
                                .build()
                );
                sequence.add("strafeLeft:1");
            }else if(gamepad1.left_bumper){
                    passiveServo.setPosition(0.2);
                    sequence.add("passiveServoSetPosition:0.2");
                    sequence.add("waitSeconds:0.1");
                    sleep(100);
            }else if(gamepad1.right_bumper){
                if(auto_servo_stage==0){
                    auto_servo_stage++;
                    autoServo.setPosition(1);
                    sequence.add("autoServoSetPosition:1");
                    sequence.add("waitSeconds:0.1");
                    sleep(100);
                }else if(auto_servo_stage==1){
                    auto_servo_stage++;
                    autoServo.setPosition(0.9);
                    sequence.add("autoServoSetPosition:0.9");
                    sequence.add("waitSeconds:0.1");
                    sleep(100);
                }else if(auto_servo_stage==2){
                    auto_servo_stage++;
                    autoServo.setPosition(0.65);
                    sequence.add("autoServoSetPosition:0.65");
                    sequence.add("waitSeconds:0.1");
                    sleep(100);
                }
            }else if(gamepad1.b){
                    drive.followTrajectorySequence(
                            drive.trajectorySequenceBuilder(getCurrentPosition(drive))
                            .turn(Math.toRadians(90))
                            .build()
            );
                    sequence.add("turn:90");
            }else if(gamepad1.y){
                drive.followTrajectorySequence(
                        drive.trajectorySequenceBuilder(getCurrentPosition(drive))
                                .turn(Math.toRadians(-90))
                                .build()
                );
                sequence.add("turn:-90");
            }else{
                sequence.add("waitSeconds:0.3");
                sleep(300);
            }
            sleep(50);

        }
    }

}
