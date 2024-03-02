package org.firstinspires.ftc.teamcode.Autonomous.SplineAuto

import android.util.Log
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.qualcomm.hardware.rev.RevBlinkinLedDriver
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.Autonomous.AutoBoilerplateMultiSequences
import org.firstinspires.ftc.teamcode.Autonomous.TrajectorySequenceWithCallback
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.Variables
import org.firstinspires.ftc.teamcode.Variables.servoMidPosition
import org.firstinspires.ftc.teamcode.Variables.servoPlacePosition

@Config
@Autonomous(name = "RF_Left_Splines", group = "Linear OpMode")
class RFLeftSplines : AutoBoilerplateMultiSequences() {
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
                        .splineToLinearHeading(Pose2d(-35.91, -58.5, Math.toRadians(360.00)), Math.toRadians(360.00))
                        .build()

                    Variables.Detection.CENTER -> drive!!.trajectorySequenceBuilder(startingPose)
                        .lineToConstantHeading(Vector2d(-37.34, -30.82))
                        .waitSeconds(.25)
                        .addTemporalMarker { passiveServo!!.position = 0.2 }
                        .waitSeconds(0.5)
                        .splineToLinearHeading(Pose2d(-35.91, -58.5, Math.toRadians(360.00)), Math.toRadians(360.00))
                        .build()

                    Variables.Detection.RIGHT -> drive!!.trajectorySequenceBuilder(startingPose)
                        .setReversed(true)
                        .lineToConstantHeading(Vector2d(-38.0, -37.0))
                        .splineToLinearHeading(Pose2d(-31.32, -35.5, Math.toRadians(180.00)), Math.toRadians(0.0))
                        .waitSeconds(.25)
                        .addTemporalMarker { passiveServo!!.position = 0.2 }
                        .waitSeconds(0.5)
                        .setReversed(false)
                        .splineToConstantHeading(Vector2d(-36.32, -34.0), Math.toRadians(0.0))
                        .splineToLinearHeading(Pose2d(-38.91, -58.5, Math.toRadians(360.00)), Math.toRadians(360.00))
                        .build()

                    else -> {
                        telemetry.addLine("Warning: Cup not detected")
                        telemetry.update()
//                        sleep(3000)
                        null
                    }
                }!!, true
            ) {
                while (rotateMotor!!.currentPosition < 1000) {
                    rotateMotor!!.power = 0.5
                }
                rotateMotor!!.power = -0.001
            }
        )

        sequences.add(
            fromSequence(
                drive!!.trajectorySequenceBuilder(sequences[sequences.size-1].trajectorySequence.sequence!!.end())
                    .splineToConstantHeading(Vector2d(20.0, -58.5), Math.toRadians(0.00))
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
                        .splineToLinearHeading(Pose2d(37.0, -20.41, Math.toRadians(180.00)), Math.toRadians(180.00))
                        .setVelConstraint(slowConstraint)
                        .splineToConstantHeading(Vector2d(50.5, -22.91), Math.toRadians(180.00)) // -20.41 Y originally
                        .addTemporalMarker { autoServo!!.position = servoMidPosition }
                        .waitSeconds(2.0)
                        .addTemporalMarker { autoServo!!.position = servoPlacePosition }
                        .waitSeconds(2.0)
                        .addTemporalMarker { autoServo!!.position = servoMidPosition }
                        .waitSeconds(1.0)
                        .build()

                    Variables.Detection.CENTER -> drive.trajectorySequenceBuilder(sequences[sequences.size-1].trajectorySequence.sequence!!.end())
                        .splineToLinearHeading(Pose2d(37.0, -27.7, Math.toRadians(180.00)), Math.toRadians(180.00))
                        .setVelConstraint(slowConstraint)
                        .splineToConstantHeading(Vector2d(50.5, -27.7), Math.toRadians(180.00)) //                            .addTemporalMarker(() -> autoServo.setPosition(0.12))
                        .addTemporalMarker { autoServo!!.position = servoMidPosition }
                        .waitSeconds(2.0)
                        .addTemporalMarker { autoServo!!.position = servoPlacePosition }
                        .waitSeconds(2.0)
                        .addTemporalMarker { autoServo!!.position = servoMidPosition }
                        .waitSeconds(1.0)
                        .build()

                    Variables.Detection.RIGHT -> drive.trajectorySequenceBuilder(sequences[sequences.size-1].trajectorySequence.sequence!!.end()) // Default RIGHT
                        .splineToLinearHeading(Pose2d(37.0, -32.7, Math.toRadians(180.00)), Math.toRadians(180.00))
                        .setVelConstraint(slowConstraint)
                        .splineToConstantHeading(Vector2d(50.5, -32.7), Math.toRadians(180.00)) // -32.3 Y orig                           .addTemporalMarker(() -> autoServo.setPosition(0.12))
                        .addTemporalMarker { autoServo!!.position = servoMidPosition }
                        .waitSeconds(2.0)
                        .addTemporalMarker { autoServo!!.position = servoPlacePosition }
                        .waitSeconds(2.0)
                        .addTemporalMarker { autoServo!!.position = servoMidPosition }
                        .waitSeconds(1.0)
                        .build()
                    else -> null!!
                }
            ) {
                Log.i("RFLeftSplines", "Final sequence callback called (it doesn't do sh*t)")
            }
        )


        Log.i("RFLeftSplines", "Trajectory calculated")

        return sequences
    }
}
