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
import org.firstinspires.ftc.teamcode.Variables.passiveServoPlacePosition
import org.firstinspires.ftc.teamcode.Variables.servoMidPosition
import org.firstinspires.ftc.teamcode.Variables.servoRestPosition
import org.firstinspires.ftc.teamcode.Variables.servoPlacePosition

@Config
@Autonomous(name = "RC_In_Left_Splines", group = "Linear OpMode")
class RCInLeftSplines : AutoBoilerplate() {
    override val defaultColour = BlinkinPattern.RED
    override val startingPose = Pose2d(15.01, -62.69, Math.toRadians(270.00))

    override fun getTrajectorySequence(
        detection: Variables.Detection?,
        drive: SampleMecanumDrive?
    ): TrajectorySequence? {
        return when (detection) {
            Variables.Detection.LEFT -> drive!!.trajectorySequenceBuilder(startingPose)
                .setReversed(true)
                .splineToLinearHeading(Pose2d(18.98, -44.59, Math.toRadians(-90.00)), Math.toRadians(90.00))
                .splineToLinearHeading(Pose2d(13.0, -34.0, Math.toRadians(0.00)), Math.toRadians(180.00))
                .splineToLinearHeading(Pose2d(7.0, -34.0, Math.toRadians(0.00)), Math.toRadians(180.00))
                .setReversed(false)
                .waitSeconds(.25)
                .addTemporalMarker { passiveServo!!.position = passiveServoPlacePosition }
                .waitSeconds(.5)
                .lineToLinearHeading(Pose2d(52.29, -23.60, Math.toRadians(180.00)))
                .waitSeconds(.5)
                .addTemporalMarker { autoServo!!.position = servoPlacePosition }
                .waitSeconds(2.0)
                .addTemporalMarker { autoServo!!.position = servoMidPosition }
                .waitSeconds(.1)
                .lineToConstantHeading(Vector2d(47.41, -12.3))
                .lineToConstantHeading(Vector2d(58.0, -12.3))
                .build()

            Variables.Detection.CENTER -> drive!!.trajectorySequenceBuilder(startingPose)
                .lineToConstantHeading(Vector2d(12.16, -30.68))
                .waitSeconds(.25)
                .addTemporalMarker { passiveServo!!.position = passiveServoPlacePosition }
                .waitSeconds(.5)
                .splineToLinearHeading(Pose2d(52.29, -30.7, Math.toRadians(180.00)), Math.toRadians(0.00))
                .waitSeconds(.5)
                .addTemporalMarker { autoServo!!.position = servoPlacePosition }
                .waitSeconds(2.0)
                .addTemporalMarker { autoServo!!.position = servoMidPosition }
                .waitSeconds(.1)
                .lineToConstantHeading(Vector2d(47.41, -12.3))
                .lineToConstantHeading(Vector2d(58.0, -12.3))
                .build()

            Variables.Detection.RIGHT -> drive!!.trajectorySequenceBuilder(startingPose)
                .lineToConstantHeading(Vector2d(24.18, -37.99))
                .waitSeconds(.25)
                .addTemporalMarker { passiveServo!!.position = passiveServoPlacePosition }
                .waitSeconds(.5)
                .lineToConstantHeading(Vector2d(24.18, -40.99))
                .splineToLinearHeading(Pose2d(52.29, -35.99, Math.toRadians(180.00)), Math.toRadians(360.00))
                .waitSeconds(0.5)
                .addTemporalMarker { autoServo!!.position = servoPlacePosition }
                .waitSeconds(2.0)
                .addTemporalMarker { autoServo!!.position = servoMidPosition }
                .waitSeconds(.1)
                .lineToConstantHeading(Vector2d(47.41, -12.3))
                .lineToConstantHeading(Vector2d(58.0, -12.3))
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
