package org.firstinspires.ftc.teamcode.Teleop

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.DriveMethods
import org.firstinspires.ftc.teamcode.Variables

@TeleOp(name="TensorFlowTest")
class TensorFlowTest : DriveMethods() {
    override fun runOpMode() {
        initVision(Variables.VisionProcessors.TFOD)
        telemetry.addLine("Init")
        telemetry.update()

        while (opModeInInit() || opModeIsActive()) {
            val detection = getDetectionsSingleTFOD()
            telemetry.addData("Detection", detection)
            telemetry.update()
        }
    }
}