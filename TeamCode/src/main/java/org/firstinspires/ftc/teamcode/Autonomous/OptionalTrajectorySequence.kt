package org.firstinspires.ftc.teamcode.Autonomous

import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.RoadRunner.util.trajectorysequence.TrajectorySequence

data class OptionalTrajectorySequence(var hasSequence: Boolean, var sequence: TrajectorySequence?) {
    fun followAsync(drive: SampleMecanumDrive): Boolean {
        if (hasSequence) drive.followTrajectorySequenceAsync(sequence)
        return hasSequence
    }

    fun followSync(drive: SampleMecanumDrive): Boolean {
        if (hasSequence) drive.followTrajectorySequence(sequence)
        return hasSequence
    }
}
