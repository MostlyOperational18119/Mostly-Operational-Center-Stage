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
import org.firstinspires.ftc.teamcode.Variables.servoPlacePosition

@Config
@Autonomous(name = "BC_Out_Right_Splines", group = "Linear OpMode")
class BCOutRightSplines : AutoBoilerplate() {
    override val defaultColour = BlinkinPattern.BLUE
    override val startingPose = Pose2d(15.01, 62.69, Math.toRadians(90.00))


    override fun getTrajectorySequence(
        detection: Variables.Detection?,
        drive: SampleMecanumDrive?
    ): TrajectorySequence? {
        return when (detection) {
            Variables.Detection.LEFT -> drive!!.trajectorySequenceBuilder(startingPose)
                .lineToConstantHeading(Vector2d(27.18, 36.85))
                .waitSeconds(.25)
                .addTemporalMarker { passiveServo!!.position = passiveServoPlacePosition }
                .waitSeconds(.25)
                .splineToLinearHeading(Pose2d(56.0, 44.9, Math.toRadians(180.00)), Math.toRadians(0.0))
                .waitSeconds(.5)
                .addTemporalMarker { autoServo!!.position = servoPlacePosition}
                .waitSeconds(1.0)
                .addTemporalMarker { autoServo!!.position = servoMidPosition }
                .waitSeconds(.1)
                .lineToConstantHeading(Vector2d(52.41, 59.31))
                .lineToConstantHeading(Vector2d(62.0, 59.31))
                .build()

            Variables.Detection.CENTER -> drive!!.trajectorySequenceBuilder(startingPose)
                .lineToConstantHeading(Vector2d(16.35, 30.0))
                .waitSeconds(.25)
                .addTemporalMarker { passiveServo!!.position = passiveServoPlacePosition }
                .waitSeconds(.25)
                .splineToLinearHeading(Pose2d(56.0, 39.15, Math.toRadians(180.00)), Math.toRadians(0.00))
                .waitSeconds(.5)
                .addTemporalMarker { autoServo!!.position = servoPlacePosition }
                .waitSeconds(1.0)
                .addTemporalMarker { autoServo!!.position = servoMidPosition}
                .waitSeconds(.1)
                .lineToConstantHeading(Vector2d(52.41, 59.31))
                .lineToConstantHeading(Vector2d(62.0, 59.31))
                .build()

            Variables.Detection.RIGHT -> drive!!.trajectorySequenceBuilder(startingPose)
                .setReversed(true)
                .splineToLinearHeading(Pose2d(15.75, 45.61, Math.toRadians(90.00)), Math.toRadians(-90.00))
                .splineToLinearHeading(Pose2d(10.95, 35.39, Math.toRadians(0.00)), Math.toRadians(180.00))
                .setReversed(false)
                .waitSeconds(.25)
                .addTemporalMarker { passiveServo!!.position = passiveServoPlacePosition }
                .waitSeconds(.25)
                .lineToLinearHeading(Pose2d(56.0, 32.49, Math.toRadians(180.00)))
                .waitSeconds(.5)
                .addTemporalMarker { autoServo!!.position = servoPlacePosition}
                .waitSeconds(1.0)
                .addTemporalMarker { autoServo!!.position = servoMidPosition }
                .waitSeconds(.1)
                .lineToConstantHeading(Vector2d(52.41, 59.31))
                .lineToConstantHeading(Vector2d(62.0, 59.31))
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
