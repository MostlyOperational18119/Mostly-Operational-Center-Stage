package org.firstinspires.ftc.teamcode.Teleop.Testing

import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.teamcode.DriveMethods
import org.firstinspires.ftc.teamcode.Variables.motorBL
import org.firstinspires.ftc.teamcode.Variables.motorBR
import org.firstinspires.ftc.teamcode.Variables.motorFL
import org.firstinspires.ftc.teamcode.Variables.motorFR

@TeleOp(name = "MotorTester", group = "Testing")
@Disabled
class MotorTester: DriveMethods() {
    override fun runOpMode() {
        initMotorsSecondBot()
        initSlideMotors()
        val BR: DcMotor = motorBR!!

        BR.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        BR.mode = DcMotor.RunMode.RUN_USING_ENCODER

        val BL: DcMotor = motorBL!!

        BL.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        BL.mode = DcMotor.RunMode.RUN_USING_ENCODER

        val FR: DcMotor = motorFR!!

        FR.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        FR.mode = DcMotor.RunMode.RUN_USING_ENCODER

        val FL: DcMotor = motorFL!!

        FL.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        FL.mode = DcMotor.RunMode.RUN_USING_ENCODER
        waitForStart()

        while (opModeIsActive()) {
            if (gamepad1.a){
                BR.power = 1.0;
            }
            if (gamepad1.b){
                BL.power = 1.0;
            }
            if (gamepad1.x){
                FR.power = 1.0;
            }
            if (gamepad1.y){
                FL.power = 1.0;
            }

            telemetry.addData("BR", BR.currentPosition)
            telemetry.addData("FR: ", FR.currentPosition)
            telemetry.addData("FL: ", FL.currentPosition)
            telemetry.addData("BL: ", BL.currentPosition)
            telemetry.update()
        }
    }
}