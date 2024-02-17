package org.firstinspires.ftc.teamcode.Autonomous.SplineAuto

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint
import com.qualcomm.hardware.rev.RevBlinkinLedDriver
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.Autonomous.AutoBoilerplate
import org.firstinspires.ftc.teamcode.Autonomous.AutoBoilerplateMultiSequences
import org.firstinspires.ftc.teamcode.Autonomous.MeepMeepBoilerplate
import org.firstinspires.ftc.teamcode.Autonomous.TrajectorySequenceWithCallback
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.RoadRunner.util.trajectorysequence.TrajectorySequence
import org.firstinspires.ftc.teamcode.Variables
import java.util.Arrays

@Config
@Autonomous(name = "BFRightSplines (Main)", group = "Linear OpMode")
class BFRightSplines : AutoBoilerplateMultiSequences() {
    override val startingPose = Pose2d(-36.67, 62.45, Math.toRadians(90.00))
    override val defaultColour = RevBlinkinLedDriver.BlinkinPattern.BLUE

    override fun getTrajectorySequences(
        detection: Variables.Detection?,
        drive: SampleMecanumDrive?
    ): ArrayList<TrajectorySequenceWithCallback> {
        val sequences = ArrayList<TrajectorySequenceWithCallback>()

        sequences.add(
            fromSequence(
                when (detection) {
                    Variables.Detection.LEFT -> drive!!.trajectorySequenceBuilder(startingPose)
                        .setReversed(true)
                        .splineToLinearHeading(
                            Pose2d(-30.32, 33.85, Math.toRadians(180.00)),
                            Math.toRadians(0.0)
                        )
                        .waitSeconds(.1)
                        .addTemporalMarker { passiveServo!!.position = 0.2 }
                        .waitSeconds(.5)
                        .splineToConstantHeading(Vector2d(-36.32, 33.85), Math.toRadians(0.0))
                        .setReversed(false)
                        .splineToLinearHeading(
                            Pose2d(-40.09, 58.88, Math.toRadians(360.00)),
                            Math.toRadians(360.00)
                        )
                        .build()

                    Variables.Detection.CENTER -> drive!!.trajectorySequenceBuilder(startingPose)
                        .lineToConstantHeading(Vector2d(-35.34, 30.0))
                        .waitSeconds(.1)
                        .addTemporalMarker { passiveServo!!.position = 0.2 }
                        .waitSeconds(.5)
                        .splineToLinearHeading(
                            Pose2d(-36.09, 58.88, Math.toRadians(360.00)),
                            Math.toRadians(360.00)
                        )
                        .build()

                    else -> drive!!.trajectorySequenceBuilder(startingPose) // Default to right
                        .lineToConstantHeading(Vector2d(-45.34, 34.82))
                        .waitSeconds(.1)
                        .addTemporalMarker { passiveServo!!.position = 0.2 }
                        .waitSeconds(.5)
                        .splineToLinearHeading(
                            Pose2d(-36.09, 58.88, Math.toRadians(360.00)),
                            Math.toRadians(360.00)
                        )
                        .build()
                }, true) {
                while (rotateMotor!!.currentPosition < 1000) {
                    rotateMotor!!.power = 0.5
                }
                rotateMotor!!.power = 0.001

            }
        )
        // sequences[sequences.size - 1].trajectorySequence.sequence!!.end()
        sequences.add(
            fromSequence(
                drive.trajectorySequenceBuilder(sequences[0].trajectorySequence.sequence!!.end())
                    .splineToConstantHeading(Vector2d(17.0, 58.88), Math.toRadians(0.00))
                    .build(), true
            ) {
                while (rotateMotor!!.currentPosition > 300) {
                    rotateMotor!!.power = -.5
                }
                rotateMotor!!.power = 0.0
                sleep(2000)
            }
        )

        sequences.add(
            fromSequence(
                when (detection) {
                    Variables.Detection.LEFT -> drive.trajectorySequenceBuilder(sequences[sequences.size - 1].trajectorySequence.sequence!!.end())
                            .splineToLinearHeading(
                                Pose2d(43.49, 45.49, Math.toRadians(180.00)),
                                Math.toRadians(180.00)
                            )
                            .setVelConstraint(slowConstraint)
                            .splineToConstantHeading(Vector2d(52.5, 45.49), Math.toRadians(180.00))
                            .addTemporalMarker { autoServo!!.position = 0.32 }
                            .waitSeconds(2.0)
                            .addTemporalMarker { autoServo!!.position = 0.65 }
                            .waitSeconds(2.0)
                            .addTemporalMarker { autoServo!!.position = 0.32 }
                            .waitSeconds(1.0)
                            .build()

                    Variables.Detection.CENTER -> drive.trajectorySequenceBuilder(sequences[sequences.size - 1].trajectorySequence.sequence!!.end())
                            .splineToLinearHeading(
                                Pose2d(43.49, 41.26, Math.toRadians(180.00)),
                                Math.toRadians(180.00)
                            )
                            .setVelConstraint(slowConstraint)
                            .splineToConstantHeading(Vector2d(52.5, 41.26), Math.toRadians(180.00))
                            .addTemporalMarker { autoServo!!.position = 0.32 }
                            .waitSeconds(2.0)
                            .addTemporalMarker { autoServo!!.position = 0.65 }
                            .waitSeconds(2.0)
                            .addTemporalMarker { autoServo!!.position = 0.32 }
                            .waitSeconds(1.0)
                            .build()

                    else -> drive.trajectorySequenceBuilder(sequences[sequences.size - 1].trajectorySequence.sequence!!.end())
                            .splineToLinearHeading(
                                Pose2d(43.49, 33.4, Math.toRadians(180.00)),
                                Math.toRadians(180.00)
                            )
                            .setVelConstraint(slowConstraint)
                            .splineToConstantHeading(Vector2d(52.5, 33.4), Math.toRadians(180.00))
                            .addTemporalMarker { autoServo!!.position = 0.32 }
                            .waitSeconds(2.0)
                            .addTemporalMarker { autoServo!!.position = 0.65 }
                            .waitSeconds(2.0)
                            .addTemporalMarker { autoServo!!.position = 0.32 }
                            .waitSeconds(1.0)
                            .build()
                }
            )
        )

        return sequences
    }

    override fun runOpMode() {
        val drive = SampleMecanumDrive(hardwareMap)
        val passiveServo = hardwareMap.get(
            Servo::class.java, "passiveServo"
        )
        val autoServo = hardwareMap.get(
            Servo::class.java, "autoServo"
        )
        val rotateMotor = hardwareMap.get(DcMotor::class.java, "motorSlideRotate")
        val slideMotor = hardwareMap.get(DcMotor::class.java, "motorSlideLeft")
        val boxServo = hardwareMap.get(
            Servo::class.java, "boxServo"
        )
        initVision(Variables.VisionProcessors.TFOD)
        initBlinkinSafe(RevBlinkinLedDriver.BlinkinPattern.BLUE) // Inits Blinkin with a colour corresponding to the Auto
        var detection = Variables.Detection.UNKNOWN
        val slowConstraint: TrajectoryVelocityConstraint = MinVelocityConstraint(
            Arrays.asList(
                TranslationalVelocityConstraint(7.0),
                AngularVelocityConstraint(1.0)
            )
        )
        val fastConstraint: TrajectoryVelocityConstraint = MinVelocityConstraint(
            Arrays.asList(
                TranslationalVelocityConstraint(55.0),
                AngularVelocityConstraint(3.0)
            )
        )
        while (opModeInInit()) {
            detection = getDetectionsSingleTFOD()
            telemetry.addData("Detection", detection)
            telemetry.update()
        }
        autoServo.position = 0.245
        rotateMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        rotateMotor.mode = DcMotor.RunMode.RUN_USING_ENCODER
        rotateMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        slideMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        slideMotor.mode = DcMotor.RunMode.RUN_USING_ENCODER
        slideMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        rotateMotor.power = 0.0
        drive.poseEstimate = Pose2d(-36.67, 62.45, Math.toRadians(90.00))
        when (detection) {
            Variables.Detection.LEFT -> drive.followTrajectorySequence(
                drive.trajectorySequenceBuilder(startingPose)
                    .setReversed(true)
                    .splineToLinearHeading(
                        Pose2d(-30.32, 33.85, Math.toRadians(180.00)),
                        Math.toRadians(0.0)
                    )
                    .waitSeconds(.1)
                    .addTemporalMarker { passiveServo.position = 0.2 }
                    .waitSeconds(.5)
                    .splineToConstantHeading(Vector2d(-36.32, 33.85), Math.toRadians(0.0))
                    .setReversed(false)
                    .splineToLinearHeading(
                        Pose2d(-40.09, 58.88, Math.toRadians(360.00)),
                        Math.toRadians(360.00)
                    )
                    .build())

            Variables.Detection.CENTER -> {
                drive.followTrajectorySequence(
                    drive.trajectorySequenceBuilder(startingPose)
                        .lineToConstantHeading(Vector2d(-35.34, 30.0))
                        .waitSeconds(.1)
                        .addTemporalMarker { passiveServo.position = 0.2 }
                        .waitSeconds(.5)
                        .splineToLinearHeading(
                            Pose2d(-36.09, 58.88, Math.toRadians(360.00)),
                            Math.toRadians(360.00)
                        )
                        .build())
            }

            Variables.Detection.RIGHT -> drive.followTrajectorySequence(
                drive.trajectorySequenceBuilder(startingPose)
                    .lineToConstantHeading(Vector2d(-45.34, 34.82))
                    .waitSeconds(.1)
                    .addTemporalMarker { passiveServo.position = 0.2 }
                    .waitSeconds(.5)
                    .splineToLinearHeading(
                        Pose2d(-36.09, 58.88, Math.toRadians(360.00)),
                        Math.toRadians(360.00)
                    )
                    .build())

            else -> {
                telemetry.addLine("Warning: Cup not detected")
                telemetry.update()
                sleep(3000)
            }
        }
        while (rotateMotor.currentPosition < 1000) {
            rotateMotor.power = 0.5
        }
        rotateMotor.power = 0.001
        drive.followTrajectorySequence(
            drive.trajectorySequenceBuilder(getCurrentPosition(drive))
                .splineToConstantHeading(Vector2d(17.0, 58.88), Math.toRadians(0.00))
                .build()
        )
        while (rotateMotor.currentPosition > 300) {
            rotateMotor.power = -.5
        }
        rotateMotor.power = 0.0
        sleep(2000)
        when (detection) {
            Variables.Detection.LEFT -> drive.followTrajectorySequence(
                drive.trajectorySequenceBuilder(getCurrentPosition(drive))
                    .splineToLinearHeading(
                        Pose2d(43.49, 45.49, Math.toRadians(180.00)),
                        Math.toRadians(180.00)
                    )
                    .setVelConstraint(slowConstraint)
                    .splineToConstantHeading(Vector2d(52.5, 45.49), Math.toRadians(180.00))
                    .addTemporalMarker { autoServo.position = 0.32 }
                    .waitSeconds(2.0)
                    .addTemporalMarker { autoServo.position = 0.65 }
                    .waitSeconds(2.0)
                    .addTemporalMarker { autoServo.position = 0.32 }
                    .waitSeconds(1.0)
                    .build())

            Variables.Detection.CENTER -> drive.followTrajectorySequence(
                drive.trajectorySequenceBuilder(getCurrentPosition(drive))
                    .splineToLinearHeading(
                        Pose2d(43.49, 41.26, Math.toRadians(180.00)),
                        Math.toRadians(180.00)
                    )
                    .setVelConstraint(slowConstraint)
                    .splineToConstantHeading(Vector2d(52.5, 41.26), Math.toRadians(180.00))
                    .addTemporalMarker { autoServo.position = 0.32 }
                    .waitSeconds(2.0)
                    .addTemporalMarker { autoServo.position = 0.65 }
                    .waitSeconds(2.0)
                    .addTemporalMarker { autoServo.position = 0.32 }
                    .waitSeconds(1.0)
                    .build())

            Variables.Detection.RIGHT -> drive.followTrajectorySequence(
                drive.trajectorySequenceBuilder(getCurrentPosition(drive))
                    .splineToLinearHeading(
                        Pose2d(43.49, 33.4, Math.toRadians(180.00)),
                        Math.toRadians(180.00)
                    )
                    .setVelConstraint(slowConstraint)
                    .splineToConstantHeading(Vector2d(52.5, 33.4), Math.toRadians(180.00))
                    .addTemporalMarker { autoServo.position = 0.32 }
                    .waitSeconds(2.0)
                    .addTemporalMarker { autoServo.position = 0.65 }
                    .waitSeconds(2.0)
                    .addTemporalMarker { autoServo.position = 0.32 }
                    .waitSeconds(1.0)
                    .build())

            else -> {
                telemetry.addLine("Warning: Cup not detected")
                telemetry.update()
                sleep(3000)
            }
        }


//        while (Math.abs(slideMotor.getCurrentPosition()) < 500){
//            slideMotor.setPower(-0.3);
//        }
//
//        sleep(1000);
//        boxServo.setPosition(.45);
//        sleep(1000);
//
//        while (rotateMotor.getCurrentPosition()>20){
//            rotateMotor.setPower(-0.3);
//        }
//
//        sleep(1000);
//        rotateMotor.setPower(0.0);
//
//        sleep(1000);
//
//        while (Math.abs(slideMotor.getCurrentPosition()) > 20){
//            slideMotor.setPower(0.3);
//        }
//
//        boxServo.setPosition(.62);

//        drive.followTrajectorySequence(mergeSequences(sequences));
    }
}
