package org.firstinspires.ftc.teamcode.Teleop

import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.teamcode.DriveMethods
import org.firstinspires.ftc.teamcode.Variables
import org.firstinspires.ftc.teamcode.Variables.slideRotationMotor

@TeleOp(name = "RotationMotor", group = "Testing")
class RotationTest: DriveMethods() {
    override fun runOpMode() {
        //var motorBeingTested = hardwareMap.get<DcMotor>(DcMotor::class.java, "slideRotationMotor")
        val motorBeingTested = hardwareMap.get<DcMotor>(DcMotor::class.java, "motorSlideRotate")
        val secondmotorBeingTested = hardwareMap.get<DcMotor>(DcMotor::class.java, "motorSlideLeft")
        var holdingpower = 0.001
        var toggle1 = false
        var toggle2 = false
        var rotatePower = -.5

        motorBeingTested!!.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        motorBeingTested.mode = DcMotor.RunMode.RUN_USING_ENCODER
        motorBeingTested.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        secondmotorBeingTested!!.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        secondmotorBeingTested.mode = DcMotor.RunMode.RUN_USING_ENCODER
        secondmotorBeingTested.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        waitForStart()

        while (opModeIsActive()) {
//            motorBeingTested.targetPosition = ((motorBeingTested!!.currentPosition) + gamepad2.left_stick_y.toDouble()*10).toInt()
            if (secondmotorBeingTested.currentPosition<500){
                holdingpower =0.0
            }
            else {
                holdingpower = .001
            }
            if (motorBeingTested.currentPosition>=(motorBeingTested.targetPosition-20) && motorBeingTested.targetPosition == 1850){
                toggle1=false
            }
            if (secondmotorBeingTested.currentPosition<=50 && secondmotorBeingTested.targetPosition == 0){
                toggle2=false
            }
            if (gamepad2.b){
                toggle1= true
                motorBeingTested.targetPosition = 1850
                motorBeingTested.power = 0.3
                sleep(500)
            }
            else if (gamepad2.left_stick_y.toDouble() !=0.0) {
                motorBeingTested.power = 1.0*gamepad2.left_stick_y
                //motorBeingTested.power = .05
            }
            else if (!toggle1){
                motorBeingTested.power = 0.0
            }

            if (gamepad2.left_bumper){
                toggle2=true
                secondmotorBeingTested.targetPosition = 0
                secondmotorBeingTested.power = -0.3
                sleep(500)
            }
            else if (gamepad2.right_stick_y.toDouble() != 0.0) {
                secondmotorBeingTested.power = -1.0*gamepad2.right_stick_y
                //motorBeingTested.power = .05
            }
            else if (!toggle2){
                secondmotorBeingTested.power = holdingpower
            }
            telemetry.addData("Rotate Motor Value: ", motorBeingTested.currentPosition)
            telemetry.addData("Slide Motor Value: ", secondmotorBeingTested.currentPosition)
            telemetry.addData("Holding Power: ", holdingpower)
            telemetry.addData("Toggle1: ", toggle1)
            telemetry.addData("Toggle2: ", toggle2)
            telemetry.update()
        }
    }
}