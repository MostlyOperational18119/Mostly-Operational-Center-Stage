package org.firstinspires.ftc.teamcode.Autonomous.SplineAuto

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint
import com.qualcomm.hardware.rev.RevBlinkinLedDriver
import com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.Autonomous.AutoBoilerplate
import org.firstinspires.ftc.teamcode.Autonomous.MeepMeepBoilerplate
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.RoadRunner.util.trajectorysequence.TrajectorySequence
import org.firstinspires.ftc.teamcode.Variables
import java.util.Arrays

@Config
@Autonomous(name = "BCOutRightSplines", group = "Linear OpMode")
class BCOutRightSplines : AutoBoilerplate() {
    override fun getDefaultColour(): RevBlinkinLedDriver.BlinkinPattern {
        return BlinkinPattern.BLUE
    }

    override fun getStartingPose(): Pose2d {
        return Pose2d(15.01, 62.69, Math.toRadians(90.00))
    }

    override fun getTrajectorySequence(
        detection: Variables.Detection?,
        drive: SampleMecanumDrive
    ): TrajectorySequence? {
        return when (detection) {
            Variables.Detection.LEFT -> drive.trajectorySequenceBuilder(startingPose)
                    .lineToConstantHeading(Vector2d(27.18, 36.85))
                    .waitSeconds(.1)
                    .addTemporalMarker { passiveServo.position = 0.2 }
                    .waitSeconds(.5)
                    .splineToLinearHeading(
                        Pose2d(57.18, 45.07, Math.toRadians(180.00)),
                        Math.toRadians(0.0)
                    )
                    .waitSeconds(.1)
                    .addTemporalMarker { autoServo.position = .65 }
                    .waitSeconds(2.0)
                    .addTemporalMarker { autoServo.position = 0.32 }
                    .waitSeconds(.1)
                    .lineToConstantHeading(Vector2d(51.41, 59.31))
                    .build()

            Variables.Detection.CENTER -> drive.trajectorySequenceBuilder(startingPose)
                        .lineToConstantHeading(Vector2d(16.35, 30.0))
                        .waitSeconds(.1)
                        .addTemporalMarker { passiveServo.position = 0.2 }
                        .waitSeconds(.5)
                        .splineToLinearHeading(
                            Pose2d(54.91, 39.46, Math.toRadians(180.00)),
                            Math.toRadians(0.00)
                        )
                        .waitSeconds(.1)
                        .addTemporalMarker { autoServo.position = .65 }
                        .waitSeconds(2.0)
                        .addTemporalMarker { autoServo.position = 0.32 }
                        .waitSeconds(.1)
                        .lineToConstantHeading(Vector2d(51.41, 59.31))
                        .build()

            Variables.Detection.RIGHT -> drive.trajectorySequenceBuilder(startingPose)
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
                    .addTemporalMarker { passiveServo.position = 0.2 }
                    .waitSeconds(.5)
                    .lineToLinearHeading(Pose2d(55.29, 32.49, Math.toRadians(180.00)))
                    .addTemporalMarker { autoServo.position = 0.65 }
                    .waitSeconds(2.0)
                    .addTemporalMarker { autoServo.position = 0.32 }
                    .waitSeconds(.1)
                    .lineToConstantHeading(Vector2d(51.41, 59.31))
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
