package org.firstinspires.ftc.teamcode.Autonomous

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint
import com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.DriveMethods
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.RoadRunner.util.trajectorysequence.TrajectorySequence
import org.firstinspires.ftc.teamcode.Variables
import org.firstinspires.ftc.teamcode.Variables.servoRestPosition
import java.util.Arrays


abstract class AutoBoilerplateMultiSequences : DriveMethods() {
    val slowConstraint: TrajectoryVelocityConstraint = MinVelocityConstraint(
        Arrays.asList(
            TranslationalVelocityConstraint(10.0),
            AngularVelocityConstraint(1.0)
        )
    )

    val fastConstraint: TrajectoryVelocityConstraint = MinVelocityConstraint(
        Arrays.asList(
            TranslationalVelocityConstraint(55.0),
            AngularVelocityConstraint(3.0)
        )
    )

    var tfodFirstTime = true
    override fun runOpMode() {
        drive(startingPose!!)
    }

    lateinit var drive: SampleMecanumDrive
    lateinit var STARTING_POSE: Pose2d
    var sequences = ArrayList<TrajectorySequence>()
    var passiveServo: Servo? = null
    var autoServo: Servo? = null
    var rotateMotor: DcMotor? = null
    abstract val startingPose: Pose2d?
    fun drive(startingPos: Pose2d) {
        initMotorsSecondBot()
        initVision(Variables.VisionProcessors.TFOD)
        telemetry.addLine("Motors and TFOD were ignited (same as inited)")
        telemetry.update()
        var detection = detect()
        STARTING_POSE = startingPos
        drive = SampleMecanumDrive(hardwareMap)
        drive.poseEstimate = startingPose!!
        initCall()
        while (opModeInInit()) {
            detection = detect()
            telemetry.addData("Detection", detection)
            telemetry.update()
        }

        getTrajectorySequences(detection, drive).forEach {
            it.followSync(drive)
        }
    }

    fun detect(): Variables.Detection {
        val detection = getDetectionsSingleTFOD(tfodFirstTime)
        tfodFirstTime = false
        return detection
    }

    abstract fun getTrajectorySequences(
        detection: Variables.Detection?,
        drive: SampleMecanumDrive?
    ): ArrayList<TrajectorySequenceWithCallback>

    fun println(o: Any?) {
        kotlin.io.println(o)
    }

    /*
    public TrajectorySequence mergeSequences(ArrayList<TrajectorySequence> trajectorySequences) {
        TrajectorySequence[] trajectorySequencesArr = new TrajectorySequence[trajectorySequences.size()];
        trajectorySequencesArr = trajectorySequences.toArray(trajectorySequencesArr);

        return mergeSequences(trajectorySequencesArr);
    }

    public TrajectorySequence mergeSequences(TrajectorySequence[] trajectorySequences) {
        ArrayList<SequenceSegment> trajectorySegments = new ArrayList<SequenceSegment>();

        for (TrajectorySequence sequence : trajectorySequences) {
            for (int i = 0; i < sequence.size(); i++) {
                trajectorySegments.add(sequence.get(i));
            }
        }

        return new TrajectorySequence(trajectorySegments);
    }
    */
    fun getCurrentTrajectorySequence(drive: SampleMecanumDrive): TrajectorySequence {
        return if (sequences.size == 0) {
            drive.trajectorySequenceBuilder(STARTING_POSE)
                .forward(0.0)
                .build()
        } else {
            sequences[sequences.size - 1]
        }
    }

    fun getCurrentPosition(drive: SampleMecanumDrive): Pose2d {
        return if (sequences.size == 0) {
            STARTING_POSE
        } else {
            getCurrentTrajectorySequence(drive).end()
        }
    }

    fun followTrajectorySequence(trajectorySequence: TrajectorySequence) {
//        if (!opModeIsActive()) waitForStart();
        sequences.add(trajectorySequence)
    }

    fun initCall() {
        rotateMotor = hardwareMap.get(DcMotor::class.java, "motorSlideRotate")
        passiveServo = hardwareMap.get(Servo::class.java, "passiveServo")
        autoServo = hardwareMap.get(Servo::class.java, "autoServo")
        initVision(Variables.VisionProcessors.TFOD)
        initBlinkinSafe(defaultColour)
        autoServo!!.position = servoRestPosition
    }

    abstract val defaultColour: BlinkinPattern

    open fun fromSequence(trajectorySequence: TrajectorySequence, hasCallback: Boolean, callback: (() -> Unit)?): TrajectorySequenceWithCallback {
        val optionalTrajectorySequence = OptionalTrajectorySequence(true, trajectorySequence)
        val trajectorySequenceWithCallback = if (hasCallback) {
            TrajectorySequenceWithCallback(optionalTrajectorySequence, true, callback)
        } else TrajectorySequenceWithCallback(optionalTrajectorySequence, false, null)

        return trajectorySequenceWithCallback
    }

    open fun fromSequence(trajectorySequence: TrajectorySequence, callback: (() -> Unit)?): TrajectorySequenceWithCallback {
        val hasCallback = callback != null

        return fromSequence(trajectorySequence, hasCallback, callback)
    }

    open fun fromSequence(trajectorySequence: TrajectorySequence): TrajectorySequenceWithCallback {
        return fromSequence(trajectorySequence, false, null)
    }
}
