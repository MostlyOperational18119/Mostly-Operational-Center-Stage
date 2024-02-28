package org.firstinspires.ftc.teamcode.Autonomous.NoSplineAuto

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint
import com.qualcomm.hardware.rev.RevBlinkinLedDriver
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.Autonomous.AutoBoilerplate
import org.firstinspires.ftc.teamcode.Autonomous.MeepMeepBoilerplate
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.RoadRunner.util.trajectorysequence.TrajectorySequence
import org.firstinspires.ftc.teamcode.Variables
import org.firstinspires.ftc.teamcode.Variables.VisionProcessors
import java.util.Arrays

@Config
@Autonomous(name = "RFRightNOOOO", group = "LinearOpmode")
class RFRightNOSplines : AutoBoilerplate() {
    override val defaultColour = RevBlinkinLedDriver.BlinkinPattern.RED
    override val startingPose = Pose2d(-36.0, 61.5, Math.toRadians(-90.0))

    override fun getTrajectorySequence(
        detection: Variables.Detection?,
        drive: SampleMecanumDrive?
    ): TrajectorySequence? {
        return when (detection) {
            Variables.Detection.LEFT -> drive!!.trajectorySequenceBuilder(startingPose)
                    .back(2.0)
                    .waitSeconds(.1)
                    .strafeRight(11.0)
                    .back(21.0)
                    .waitSeconds(.1)
                    .addTemporalMarker { passiveServo!!.position = 0.2 }
                    .waitSeconds(4.0)
                    .forward(10.0)
                    .turn(Math.toRadians(180.0))
                    .build()

            Variables.Detection.CENTER -> drive!!.trajectorySequenceBuilder(startingPose)
                        .back(32.25)
                        .waitSeconds(.25)
                        .addTemporalMarker { passiveServo!!.position = 0.2 }
                        .waitSeconds(4.0)
                        .forward(10.0)
                        .turn(Math.toRadians(180.0))
                        .build()

            Variables.Detection.RIGHT -> drive!!.trajectorySequenceBuilder(startingPose)
                    .back(28.0)
                    .turn(Math.toRadians(-90.0))
                    .back(4.0)
                    .addTemporalMarker { passiveServo!!.position = 0.2 }
                    .waitSeconds(4.0)
                    .forward(10.0)
                    .turn(Math.toRadians(-90.0))
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
