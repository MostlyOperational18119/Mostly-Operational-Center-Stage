package org.firstinspires.ftc.teamcode.Teleop

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.DriveMethods
import org.firstinspires.ftc.teamcode.Variables

@TeleOp(name = "STEMNightTeleOp")
class STEMNightTeleOp: DriveMethods() {
    override fun runOpMode() {
        initMotorsSecondSimple()
        waitForStart()
        var leftY: Double
        var leftYGPadTwo: Double
        var leftX: Double
        var rightX: Double
        val pomPomServo = hardwareMap.get(Servo::class.java, )
        var pomPomToggle = false

        while (opModeIsActive()) {
            val reverseThing = 1
            val speedDiv = 2.0
            leftY = (-gamepad1.left_stick_y).toDouble()* reverseThing
            leftYGPadTwo = (gamepad2.left_stick_y).toDouble()
            leftX = -gamepad1.left_stick_x.toDouble() * reverseThing
            rightX = -gamepad1.right_stick_x.toDouble()

            Variables.motorFL?.power = -(leftY - leftX - rightX) / speedDiv
            Variables.motorBL?.power = -(leftY + leftX - rightX) / speedDiv
            Variables.motorFR?.power = (leftY + leftX + rightX) / speedDiv
            Variables.motorBR?.power = (leftY - leftX + rightX) / speedDiv

            if (!pomPomToggle && gamepad1.a) {
                pomPomToggle = true

            } else if (pomPomToggle && gamepad1.a) {
                pomPomToggle = false
            }
        }
    }
}