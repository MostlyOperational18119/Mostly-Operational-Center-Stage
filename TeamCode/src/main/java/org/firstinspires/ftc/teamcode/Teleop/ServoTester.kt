package org.firstinspires.ftc.teamcode.Teleop

import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.DriveMethods

@TeleOp(name = "ServoTester", group = "Testing")
class ServoTester: DriveMethods() {
    override fun runOpMode() {
        initMotorsSecondBot()
        initSlideMotors()
        val servoBeingTested: Servo = hardwareMap.get(Servo::class.java, "PLANE!")!!

        waitForStart()

        servoBeingTested.position = 0.0
        while (opModeIsActive()) {
            if (gamepad2.y) {
                servoBeingTested.position+=0.1
                sleep(150)
            }

            if (gamepad2.x) {
                servoBeingTested.position-=.1
                sleep(150)
            }
            telemetry.addData("Servo Value: ", servoBeingTested.position)
            telemetry.update()
        }
    }
}