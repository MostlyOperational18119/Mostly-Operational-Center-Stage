package org.firstinspires.ftc.teamcode.Autonomous.NoSplineAuto

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.DriveMethods
import org.firstinspires.ftc.teamcode.Variables

@Autonomous(name="Just Drive Left, we need an Auto")
@Disabled
class JustDriveDuckingLeft: DriveMethods() {
    override fun runOpMode() {
        initMotorsSecondBot()
        var passiveServo = hardwareMap.get(Servo::class.java, "passiveServo")
        waitForStart()

        val speed: Double = 0.5

        Variables.motorBL!!.power = -speed
        Variables.motorBR!!.power = speed
        Variables.motorFL!!.power = -speed
        Variables.motorFR!!.power = speed
        sleep(1650)
        Variables.motorBL!!.power = 0.0
        Variables.motorBR!!.power = 0.0
        Variables.motorFL!!.power = 0.0
        Variables.motorFR!!.power = 0.0
    }
}