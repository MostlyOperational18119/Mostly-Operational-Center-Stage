package org.firstinspires.ftc.teamcode.Teleop

import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.teamcode.DriveMethods
import org.firstinspires.ftc.teamcode.Variables.motorBL
import org.firstinspires.ftc.teamcode.Variables.motorBR
import org.firstinspires.ftc.teamcode.Variables.motorFL
import org.firstinspires.ftc.teamcode.Variables.motorFR
import org.firstinspires.ftc.teamcode.Variables.slideRotationMotor

@Disabled
@TeleOp(name = "MotorTester", group = "Testing")
class MotorTester: DriveMethods() {
    override fun runOpMode() {
        initMotorsSecondBot()
        initSlideMotors()
        val motorBeingTested1: DcMotor = motorBR!!

        motorBeingTested1.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        motorBeingTested1.mode = DcMotor.RunMode.RUN_USING_ENCODER

        val motorBeingTested2: DcMotor = motorBL!!

        motorBeingTested2.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        motorBeingTested2.mode = DcMotor.RunMode.RUN_USING_ENCODER

        val motorBeingTested3: DcMotor = motorFR!!

        motorBeingTested3.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        motorBeingTested3.mode = DcMotor.RunMode.RUN_USING_ENCODER

        val motorBeingTested4: DcMotor = motorFL!!

        motorBeingTested4.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        motorBeingTested4.mode = DcMotor.RunMode.RUN_USING_ENCODER
        waitForStart()

        while (opModeIsActive()) {
            if (gamepad1.a){
                motorBeingTested1.power = 1.0;
            }
            if (gamepad1.b){
                motorBeingTested2.power = 1.0;
            }
            if (gamepad1.x){
                motorBeingTested3.power = 1.0;
            }
            if (gamepad1.y){
                motorBeingTested4.power = 1.0;
            }

            telemetry.addData("Motor Value1: ", motorBeingTested1.currentPosition)
            telemetry.update()
        }
    }
}