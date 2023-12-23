package org.firstinspires.ftc.teamcode.Teleop

import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.CRServo
import org.firstinspires.ftc.teamcode.DriveMethods
import org.firstinspires.ftc.teamcode.Variables.boxServo
import org.firstinspires.ftc.teamcode.Variables.intakeServo
import org.firstinspires.ftc.teamcode.Variables.leftFlyWheel
import org.firstinspires.ftc.teamcode.Variables.rightFlyWheel

@TeleOp(name = "CRServoTest", group = "Flywheel")
class FlyWheelTest: DriveMethods() {
    override fun runOpMode() {


        intakeServo = hardwareMap.get(CRServo::class.java, "intakeServo")
//        boxServo = hardwareMap.get(CRServo::class.java, "boxServo")
//        rightFlyWheel = hardwareMap.get(CRServo::class.java, "rightFlyWheel")
        waitForStart()

        while (opModeIsActive()) {
            if (gamepad2.b) {
                intakeServo?.power = -10.0
//                rightFlyWheel?.setPower(10.0)
            }
            if (gamepad2.a){
                intakeServo?.power = 0.0
            }
//            if (gamepad2.x) {
//                boxServo?.power = 10.0
////                rightFlyWheel?.setPower(10.0)
//            }
//            if (gamepad2.y){
//                boxServo?.power = 0.0
//            }
        }
    }
}