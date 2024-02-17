package org.firstinspires.ftc.teamcode.Teleop

import com.qualcomm.hardware.rev.RevBlinkinLedDriver
import com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.teamcode.DriveMethods
import org.firstinspires.ftc.teamcode.Variables.AEROPLANE_CLOSE
import org.firstinspires.ftc.teamcode.Variables.AEROPLANE_LAUNCH
import org.firstinspires.ftc.teamcode.Variables.actualintakeServo
import org.firstinspires.ftc.teamcode.Variables.aeroplaneLauncherServo
import org.firstinspires.ftc.teamcode.Variables.autoServo
import org.firstinspires.ftc.teamcode.Variables.blinkinWorks
import org.firstinspires.ftc.teamcode.Variables.bottom
import org.firstinspires.ftc.teamcode.Variables.boxServo
import org.firstinspires.ftc.teamcode.Variables.lMax
import org.firstinspires.ftc.teamcode.Variables.lMin
import org.firstinspires.ftc.teamcode.Variables.lPower
import org.firstinspires.ftc.teamcode.Variables.lPowerSlow
import org.firstinspires.ftc.teamcode.Variables.lSpeedMax
import org.firstinspires.ftc.teamcode.Variables.lSpeedMin
import org.firstinspires.ftc.teamcode.Variables.motorBL
import org.firstinspires.ftc.teamcode.Variables.motorBR
import org.firstinspires.ftc.teamcode.Variables.motorFL
import org.firstinspires.ftc.teamcode.Variables.motorFR
import org.firstinspires.ftc.teamcode.Variables.passiveServo
import org.firstinspires.ftc.teamcode.Variables.rMax
import org.firstinspires.ftc.teamcode.Variables.rMin
import org.firstinspires.ftc.teamcode.Variables.rMotorL
import org.firstinspires.ftc.teamcode.Variables.rMotorR
import org.firstinspires.ftc.teamcode.Variables.rPower
import org.firstinspires.ftc.teamcode.Variables.rPowerSlow
import org.firstinspires.ftc.teamcode.Variables.rSpeedMax
import org.firstinspires.ftc.teamcode.Variables.rSpeedMin
import org.firstinspires.ftc.teamcode.Variables.rotateMotor
import org.firstinspires.ftc.teamcode.Variables.slideMotor
import org.firstinspires.ftc.teamcode.Variables.slideTouch
import org.firstinspires.ftc.teamcode.Variables.touchyL
import org.firstinspires.ftc.teamcode.Variables.touchyR
import com.qualcomm.robotcore.hardware.TouchSensor
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.CRServo
import kotlin.math.abs

@TeleOp(name = "ProtoICBM", group = "TeleopTest")
class SigmaBusterTeleop: DriveMethods() {
//A Teleop with the goal of automating large amounts of movements
    override fun runOpMode() {
        var motorFL = hardwareMap.get<DcMotor>(DcMotor::class.java, "motorFL")
        motorFL?.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        var motorBL = hardwareMap.get<DcMotor>(DcMotor::class.java, "motorBL")
        motorBL?.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        var motorFR = hardwareMap.get<DcMotor>(DcMotor::class.java, "motorFR")
        motorFR?.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        var motorBR = hardwareMap.get<DcMotor>(DcMotor::class.java, "motorBR")
        motorBR?.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        var rMotorR = hardwareMap.get<DcMotor>(DcMotor::class.java, "rMotorR")

        

        var rMotorL = hardwareMap.get<DcMotor>(DcMotor::class.java, "rMotorL")
        var touchyR = hardwareMap.get<TouchSensor>(TouchSensor::class.java, "touchyR")
        var touchyL = hardwareMap.get<TouchSensor>(TouchSensor::class.java, "touchyL")
        var passiveServo = hardwareMap.get(Servo::class.java, "passiveServo")
        var aeroplaneLauncherServo = hardwareMap.get(Servo::class.java, "PLANE!")
        var rotateMotor = hardwareMap.get<DcMotor>(DcMotor::class.java, "motorSlideRotate")
        var slideMotor = hardwareMap.get<DcMotor>(DcMotor::class.java, "motorSlideLeft")
        var actualintakeServo = hardwareMap.get(CRServo::class.java, "intakeServo")
        var boxServo = hardwareMap.get(Servo::class.java, "boxServo")
        var autoServo = hardwareMap.get(Servo::class.java, "autoServo")
        //What is this name???????
        var slideTouch = hardwareMap.get<TouchSensor>(TouchSensor::class.java, "GreenCreamsImTouchingYou")
        
      
      


        //read last position from autonomous
        var location_data = grab_location_data();
        //determine what side to be on - Red or Blue
        while (opModeIsActive()) {
        
        //read from encoders
        var encoder1 = motorFL.getCurrentPosition();
        var encoder2 = motorBL.getCurrentPosition();
        var encoder3 = motorBR.getCurrentPosition();
        var converted_encoder_data = convert_encoder_data_to_position(location_data, encoder1, encoder2, encoder3);
        var x = converted_encoder_data[0];
        var y = converted_encoder_data[1];
        var heading = converted_encoder_data[2];
        //decide on target
        

        //Run to target


        //Stop if needed
        //Splines?
        //Dodging?
        }
        telemetry.addLine("Sigma Buster Init")
        telemetry.addLine("Encoder positions")


    }
    //returns a list containing the following data: x, y, heading, starting color and position by reading from a file
    //This means that the autonomous must be run before this
    //This can be converted to a x, y, and heading coordinate eventually
    fun grab_location_data(): List<String>{
    //todo
        var commandList = ArrayList<String>();
        return commandList;
    }
    fun convert_encoder_data_to_position(location_data:List<String>, encoder1:Int, encoder2:Int, encoder3:Int): Array<Double>{
      //do magic 
      return arrayOf(1.0, 1.0, 1.0)
    }
}


