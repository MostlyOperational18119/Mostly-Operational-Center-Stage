package org.firstinspires.ftc.teamcode.Teleop.Testing

import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.teamcode.DriveMethods
import org.firstinspires.ftc.teamcode.Variables

class testSlide: DriveMethods() {
    override fun runOpMode() {
        initMotorsSecondBot()

        Variables.slideMotor?.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        Variables.slideMotor?.mode = DcMotor.RunMode.RUN_USING_ENCODER
        Variables.slideMotor?.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        while (opModeIsActive()) {
            Variables.slideMotor?.power = 1.0
        }
    }
}