package org.firstinspires.ftc.teamcode.Teleop.Testing

import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.teamcode.DriveMethods

@TeleOp(name = "SlideAndRotate", group = "Testing")
@Disabled
class TestingRotateAndSlide: DriveMethods() {
    override fun runOpMode() {
        val rotate = hardwareMap.get<DcMotor>(DcMotor::class.java, "motorSlideRotate")
        val slide = hardwareMap.get<DcMotor>(DcMotor::class.java, "motorSlideLeft")

        rotate.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        rotate.mode = DcMotor.RunMode.RUN_USING_ENCODER
        rotate.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        slide.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        slide.mode = DcMotor.RunMode.RUN_USING_ENCODER
        slide.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        waitForStart()

        while (opModeIsActive()) {
            if (gamepad2.right_stick_y.toDouble() !=0.0) {
                rotate!!.power = 1.0*gamepad2.right_stick_y
            }
            else {
                rotate!!.power = 0.0
            }

            if (gamepad2.left_stick_y.toDouble() !=0.0) {
                slide!!.power = 1.0*gamepad2.left_stick_y
            }
            else {
                slide!!.power = 0.0
            }

            telemetry.addData("Right Stick: ", gamepad2.right_stick_y.toDouble())
            telemetry.addData("Left Stick: ", gamepad2.left_stick_y.toDouble())
            telemetry.addData("Rotate Motor Value: ", rotate.currentPosition)
            telemetry.addData("Slide Motor Value: ", slide.currentPosition)
            telemetry.update()
        }
    }
}