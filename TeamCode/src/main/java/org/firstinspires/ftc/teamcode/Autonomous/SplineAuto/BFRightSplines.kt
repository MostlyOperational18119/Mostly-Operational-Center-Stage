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
import org.firstinspires.ftc.teamcode.Variables.servoPlacePosition
import org.firstinspires.ftc.teamcode.Variables.servoRestPosition
import org.firstinspires.ftc.teamcode.Variables.servoMidPosition
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
                            Pose2d(-40.09, 59.88, Math.toRadians(360.00)),
                            Math.toRadians(360.00)
                        )
                        .build()

                    Variables.Detection.CENTER -> drive!!.trajectorySequenceBuilder(startingPose)
                        .lineToConstantHeading(Vector2d(-35.34, 30.0))
                        .waitSeconds(.1)
                        .addTemporalMarker { passiveServo!!.position = 0.2 }
                        .waitSeconds(.5)
                        .splineToLinearHeading(
                            Pose2d(-36.09, 59.88, Math.toRadians(360.00)),
                            Math.toRadians(360.00)
                        )
                        .build()

                    else -> drive!!.trajectorySequenceBuilder(startingPose) // Default to right
                        .lineToConstantHeading(Vector2d(-45.34, 34.82))
                        .waitSeconds(.1)
                        .addTemporalMarker { passiveServo!!.position = 0.2 }
                        .waitSeconds(.5)
                        .splineToLinearHeading(
                            Pose2d(-36.09, 59.88, Math.toRadians(360.00)),
                            Math.toRadians(360.00)
                        )
                        .build()
                }, true) {
                while (rotateMotor!!.currentPosition < 1000) {
                    rotateMotor!!.power = 0.3
                }
                rotateMotor!!.power = -0.001

            }
        )
        // sequences[sequences.size - 1].trajectorySequence.sequence!!.end()
        sequences.add(
            fromSequence(
                drive.trajectorySequenceBuilder(sequences[0].trajectorySequence.sequence!!.end())
                    .splineToConstantHeading(Vector2d(17.0, 59.88), Math.toRadians(0.00))
                    .build(), true
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
                    Variables.Detection.LEFT -> drive.trajectorySequenceBuilder(sequences[sequences.size - 1].trajectorySequence.sequence!!.end())
                            .splineToLinearHeading(
                                Pose2d(43.49, 43.2, Math.toRadians(180.00)),
                                Math.toRadians(180.00)
                            )
                            .setVelConstraint(slowConstraint)
                            .splineToConstantHeading(Vector2d(52.5, 43.2), Math.toRadians(180.00))
                            .addTemporalMarker { autoServo!!.position = servoMidPosition }
                            .waitSeconds(2.0)
                            .addTemporalMarker { autoServo!!.position = servoPlacePosition }
                            .waitSeconds(2.0)
                            .addTemporalMarker { autoServo!!.position = servoMidPosition }
                            .waitSeconds(1.0)
                            .build()

                    Variables.Detection.CENTER -> drive.trajectorySequenceBuilder(sequences[sequences.size - 1].trajectorySequence.sequence!!.end())
                            .splineToLinearHeading(
                                Pose2d(43.49, 40.06, Math.toRadians(180.00)),
                                Math.toRadians(180.00)
                            )
                            .setVelConstraint(slowConstraint)
                            .splineToConstantHeading(Vector2d(52.5, 44.06), Math.toRadians(180.00))
                            .addTemporalMarker { autoServo!!.position = servoMidPosition}
                            .waitSeconds(2.0)
                            .addTemporalMarker { autoServo!!.position = servoPlacePosition }
                            .waitSeconds(2.0)
                            .addTemporalMarker { autoServo!!.position = servoMidPosition }
                            .waitSeconds(1.0)
                            .build()

                    else -> drive.trajectorySequenceBuilder(sequences[sequences.size - 1].trajectorySequence.sequence!!.end())
                            .splineToLinearHeading(
                                Pose2d(43.49, 33.4, Math.toRadians(180.00)),
                                Math.toRadians(180.00)
                            )
                            .setVelConstraint(slowConstraint)
                            .splineToConstantHeading(Vector2d(52.5, 35.0), Math.toRadians(180.00))
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
