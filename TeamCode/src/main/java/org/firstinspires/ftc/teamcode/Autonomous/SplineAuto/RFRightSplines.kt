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
import org.firstinspires.ftc.teamcode.Variables
import org.firstinspires.ftc.teamcode.Variables.VisionProcessors
import org.firstinspires.ftc.teamcode.Variables.servoMidPosition
import org.firstinspires.ftc.teamcode.Variables.servoPlacePosition
import java.util.Arrays

@Config
@Autonomous(name = "RFRightSplines (Main)", group = "Linear OpMode")
class RFRightSplines : AutoBoilerplateMultiSequences() {
    override val defaultColour = RevBlinkinLedDriver.BlinkinPattern.RED
    override val startingPose = Pose2d(-34.96, -62.69, Math.toRadians(-90.00))

    override fun getTrajectorySequences(
        detection: Variables.Detection?,
        drive: SampleMecanumDrive?
    ): ArrayList<TrajectorySequenceWithCallback> {
        val sequences = ArrayList<TrajectorySequenceWithCallback>()

        sequences.add(
            fromSequence(
                when (detection) {
                    Variables.Detection.LEFT -> drive!!.trajectorySequenceBuilder(startingPose)
                        .lineToConstantHeading(Vector2d(-47.30, -40.08))
                        .waitSeconds(.25)
                        .addTemporalMarker { passiveServo!!.position = 0.2 }
                        .waitSeconds(0.5)
                        .splineToLinearHeading(
                            Pose2d(-35.91, -58.5, Math.toRadians(360.00)),
                            Math.toRadians(360.00)
                        )
                        .build()

                    Variables.Detection.CENTER -> {
                        drive!!.trajectorySequenceBuilder(startingPose)
                            .lineToConstantHeading(Vector2d(-37.34, -30.82))
                            .waitSeconds(.25)
                            .addTemporalMarker { passiveServo!!.position = 0.2 }
                            .waitSeconds(0.5)
                            .splineToLinearHeading(
                                Pose2d(-35.91, -58.5, Math.toRadians(360.00)),
                                Math.toRadians(360.00)
                            )
                            .build()
                    }

                    else -> drive!!.trajectorySequenceBuilder(startingPose) // Default to right
                        .setReversed(true)
                        .lineToConstantHeading(Vector2d(-38.0, -37.0))
                        .splineToLinearHeading(
                            Pose2d(-31.32, -35.5, Math.toRadians(180.00)),
                            Math.toRadians(0.0)
                        )
                        .waitSeconds(.25)
                        .addTemporalMarker { passiveServo!!.position = 0.2 }
                        .waitSeconds(0.5)
                        .setReversed(false)
                        .splineToConstantHeading(Vector2d(-36.32, -34.0), Math.toRadians(0.0))
                        .splineToLinearHeading(
                            Pose2d(-38.91, -58.5, Math.toRadians(360.00)),
                            Math.toRadians(360.00)
                        )
                        .build()
                }
            ) {
                while (rotateMotor!!.currentPosition < 1000) {
                    rotateMotor!!.power = 0.5
                }
                rotateMotor!!.power = -0.001
            }
        )

        sequences.add(
            fromSequence(
                drive.trajectorySequenceBuilder(sequences[sequences.size-1].trajectorySequence.sequence!!.end())
                    .splineToConstantHeading(Vector2d(17.0, -58.5), Math.toRadians(0.00))
                    .build()
            ) {
                while (rotateMotor!!.currentPosition > 300) {
                    rotateMotor!!.power = -.5
                }
                rotateMotor!!.power = -0.001
                sleep(5000)
            }
        )

        sequences.add(
            fromSequence(
                when (detection) {
                    Variables.Detection.LEFT -> drive.trajectorySequenceBuilder(sequences[sequences.size-1].trajectorySequence.sequence!!.end())
                            .splineToLinearHeading(
                                Pose2d(37.0, -22.8, Math.toRadians(180.00)),
                                Math.toRadians(180.00)
                            )
                            .setVelConstraint(slowConstraint)
                            .splineToConstantHeading(Vector2d(50.5, -23.1), Math.toRadians(180.00))
                            .addTemporalMarker { autoServo!!.position = servoMidPosition }
                            .waitSeconds(2.0)
                            .addTemporalMarker { autoServo!!.position = servoPlacePosition }
                            .waitSeconds(2.0)
                            .addTemporalMarker { autoServo!!.position = servoMidPosition }
                            .waitSeconds(1.0)
                            .build()

                    Variables.Detection.CENTER -> drive.trajectorySequenceBuilder(sequences[sequences.size-1].trajectorySequence.sequence!!.end())
                            .splineToLinearHeading(
                                Pose2d(37.0, -32.2, Math.toRadians(180.00)),
                                Math.toRadians(180.00)
                            )
                            .setVelConstraint(slowConstraint)
                            .splineToConstantHeading(Vector2d(50.5, -32.2), Math.toRadians(180.00))
                            .addTemporalMarker { autoServo!!.position = servoMidPosition }
                            .waitSeconds(2.0)
                            .addTemporalMarker { autoServo!!.position = servoPlacePosition }
                            .waitSeconds(2.0)
                            .addTemporalMarker { autoServo!!.position = servoMidPosition }
                            .waitSeconds(1.0)
                            .build()

                    else -> drive.trajectorySequenceBuilder(sequences[sequences.size-1].trajectorySequence.sequence!!.end())
                        .splineToLinearHeading(
                            Pose2d(37.0, -36.0, Math.toRadians(180.00)),
                            Math.toRadians(180.00)
                        )
                        .setVelConstraint(slowConstraint)
                        .splineToConstantHeading(Vector2d(50.5, -36.0), Math.toRadians(180.00))
                        .addTemporalMarker { autoServo!!.position = servoMidPosition }
                        .waitSeconds(2.0)
                        .addTemporalMarker { autoServo!!.position = servoPlacePosition }
                        .waitSeconds(2.0)
                        .addTemporalMarker { autoServo!!.position = servoMidPosition }
                        .waitSeconds(1.0)
                        .build()
                }
            )
        )

        return sequences
    }
}
