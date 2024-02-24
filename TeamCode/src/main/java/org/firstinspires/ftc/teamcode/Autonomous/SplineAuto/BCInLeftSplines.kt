package org.firstinspires.ftc.teamcode.Autonomous.SplineAuto

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.Autonomous.AutoBoilerplate
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.RoadRunner.util.trajectorysequence.TrajectorySequence
import org.firstinspires.ftc.teamcode.Variables
import org.firstinspires.ftc.teamcode.Variables.servoPlacePosition
import org.firstinspires.ftc.teamcode.Variables.servoRestPosition
import org.firstinspires.ftc.teamcode.Variables.servoMidPosition

@Config
@Autonomous(name = "BCInLeftSplines (Main)", group = "Linear OpMode")
class BCInLeftSplines : AutoBoilerplate() {
    override val startingPose = Pose2d(15.01, 62.69, Math.toRadians(90.00))
    override val defaultColour = BlinkinPattern.BLUE

    override fun getTrajectorySequence(
        detection: Variables.Detection?,
        drive: SampleMecanumDrive?
    ): TrajectorySequence? {
        return when (detection) {
            Variables.Detection.LEFT -> drive!!.trajectorySequenceBuilder(startingPose)
                .lineToConstantHeading(Vector2d(27.18, 36.85))
                .waitSeconds(.1)
                .addTemporalMarker { passiveServo!!.position = 0.2 }
                .waitSeconds(.5)
                .splineToLinearHeading(
                    Pose2d(55.0, 48.07, Math.toRadians(180.00)),
                    Math.toRadians(0.0)
                )
                .waitSeconds(.25)
                .addTemporalMarker { autoServo!!.position = servoPlacePosition }
                .waitSeconds(2.0)
                .addTemporalMarker { autoServo!!.position = servoMidPosition }
                .waitSeconds(.1)
                .lineToConstantHeading(Vector2d(52.41, 10.83))
                .lineToConstantHeading(Vector2d(58.0, 10.83))
                .build()

            Variables.Detection.CENTER -> drive!!.trajectorySequenceBuilder(startingPose)
                .lineToConstantHeading(Vector2d(16.35, 30.0))
                .waitSeconds(.1)
                .addTemporalMarker { passiveServo!!.position = 0.2 }
                .waitSeconds(.5)
                .splineToLinearHeading(
                    Pose2d(55.0, 42.46, Math.toRadians(180.00)),
                    Math.toRadians(0.00)
                )
                .waitSeconds(.25)
                .addTemporalMarker { autoServo!!.position = servoPlacePosition }
                .waitSeconds(2.0)
                .addTemporalMarker { autoServo!!.position = servoMidPosition }
                .waitSeconds(.1)
                .lineToConstantHeading(Vector2d(52.41, 10.83))
                .lineToConstantHeading(Vector2d(58.0, 10.83))
                .build()

            Variables.Detection.RIGHT -> drive!!.trajectorySequenceBuilder(startingPose)
                .setReversed(true)
                .splineToLinearHeading(
                    Pose2d(15.75, 45.61, Math.toRadians(90.00)),
                    Math.toRadians(-90.00)
                )
                .splineToLinearHeading(
                    Pose2d(10.95, 35.39, Math.toRadians(0.00)),
                    Math.toRadians(180.00)
                )
                .setReversed(false)
                .waitSeconds(.1)
                .addTemporalMarker { passiveServo!!.position = 0.2 }
                .waitSeconds(.5)
                .lineToLinearHeading(Pose2d(55.0, 35.49, Math.toRadians(180.00)))
                .waitSeconds(.25)
                .addTemporalMarker { autoServo!!.position = servoPlacePosition }
                .waitSeconds(2.0)
                .addTemporalMarker { autoServo!!.position = servoMidPosition }
                .waitSeconds(.1)
                .lineToConstantHeading(Vector2d(52.41, 10.83))
                .lineToConstantHeading(Vector2d(58.0, 10.83))
                .build()

            else -> {
                telemetry.addLine("Warning: Cup not detected")
                telemetry.update()
                sleep(3000)
                null
            }
        }
    }
}
