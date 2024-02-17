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
        var truss_crossing_len = 20;
        var truss_crossing_points_close = arrayOf(arrayOf(0.0,0.0), arrayOf(5.0, 0.0));
        var truss_crossing_points_far = arrayOf(arrayOf(0.0,0.0), arrayOf(5.0, 0.0));

        //coordinates are in inches and align to the close blue corner
        //The truss runs from the y direction and the x direction is perpendicular to the truss

        //TODO: Change when tuning
        var robotHitboxLength = 10.0;
        var robotHitboxWidth = 10.0;
        //robot will start lowering the slide in the buffer before going under the truss
        var trussZoneBuffer = 20.0;
        //Align to left side of truss, X+Buffer zone
        var trussZoneRedX = 50.0;
        var trussZoneRedY = 50.0;
        var trussZoneBlueX = 150.0;
        var trussZoneBlueY = 150.0;
        // var slide_hitbox = 10;
        //Seconds
        var slideLoweringSpeedEstimate = 5.0;
      
      


        //read last position from autonomous
        var location_data = grab_location_data();
        var targetX = 0;
        var targetY = 0;
        var targetHeading = 0;
        var obstacle_map = ArrayList<Obstacle>();
        var slideDown = false;
        var running_to_position = false;
        var target_encoder_positions = arrayOf(0,0,0)
        //determine what side to be on - Red or Blue
        while (opModeIsActive()) {
        
        //read from encoders
        var encoder1 = motorFL.getCurrentPosition();
        var encoder2 = motorBL.getCurrentPosition();
        var encoder3 = motorBR.getCurrentPosition();
        var converted_encoder_data = convert_encoder_data_to_position(location_data, encoder1, encoder2, encoder3);
        var currX = converted_encoder_data[0];
        var currY = converted_encoder_data[1];
        var currHeading = converted_encoder_data[2];

        //decide on target
        if( gamepad1.dpad_up){
          var target_coordinates = fastest_coordinate_to_success(144.0, 144.0, 135.0, currX, currY, currHeading, obstacle_map, slideLoweringSpeedEstimate, slideDown);
          target_encoder_positions = convert_to_target_encoder_position(currX, currY, currHeading, target_coordinates[0], target_coordinates[1], target_coordinates[2], encoder1, encoder2, encoder3);
          running_to_position = true;
        }
        if (running_to_position){
          if(encoder1<target_encoder_positions[0]){
            //add horizontal accell
          }
          if(encoder2<target_encoder_positions[1]){
            //add vertical accell
          }
          if(encoder3<target_encoder_positions[3]){
            //add rotational accell 
          }
          if(is_in_truss_buffer_zone(currX, currY, robotHitboxWidth, robotHitboxLength, trussZoneRedX, trussZoneRedY, trussZoneBlueX, trussZoneBlueY, trussZoneBuffer)){
            if (!slideDown){
              //lower slide
              //Slow down if neccessary
            }
          }
        }
        
    
        //find fastest path
        //Begin execution
        //Route blocked options - 
        //  Stop when told to
        //  atempt re-route
        // Temporary driver control
        // Spline in direction
        //Multi-Track route system
        //Route realignment - realign to path when drifting?
        //Lower slide when in zone
        //Run to target

        }
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
    fun is_in_truss_buffer_zone(currX:Double, currY:Double, robotHitboxWidth:Double, robotHitboxLength:Double, trussZoneRedX:Double, trussZoneRedY:Double, trussZoneBlueX:Double, trussZoneBlueY:Double, trussZoneBuffer:Double):Boolean{
      //hitbox intersection check
      var robot_top_right_edge = currX+(robotHitboxLength/2)+(robotHitboxWidth/2);
      var robot_bottom_right_edge = currX-(robotHitboxLength/2)-(robotHitboxWidth/2);
      return false;
    }
    fun convert_encoder_data_to_position(location_data:List<String>, encoder1:Int, encoder2:Int, encoder3:Int): Array<Double>{
      //do magic 
      return arrayOf(1.0, 1.0, 1.0)
    }
    //returns a target coordinate that exists on the route that the robot should move on
    fun fastest_coordinate_to_success(targetX:Double, targetY:Double, targetHeading:Double, currX:Double, currY:Double, currHeading:Double, obstacle_map:List<Obstacle>, slideLoweringSpeedEstimate:Double, slideDown:Boolean):Array<Double>{
      
      return arrayOf(1.0, 1.0, 1.0)
    }
    //return a target encoder position
    fun convert_to_target_encoder_position(currX:Double, currY:Double, currHeading:Double, targetX:Double, targetY:Double, targetHeading:Double, currEncoderPosition1:Int, currEncoderPosition2:Int, currEncoderPosition3:Int):Array<Int>{
      return arrayOf(1,1,1)
    }
}

class Obstacle{
  x:Double,
  y:Double,
  width:Double,
  length:Double,
}
