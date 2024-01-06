package org.firstinspires.ftc.teamcode

import com.qualcomm.hardware.rev.RevBlinkinLedDriver
import com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.TouchSensor
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection


object Variables {
    // Redundant thanks to odometry.
    var rotationsPerMeter = 3.3
    var encoders = 537.6

    // Farmers Market Servo Values
    var servoRelease = 0.0
    var servoClamp = 0.0

    var speedDiv: Double = 3.0
    var leftY: Double = 0.0
    var leftX: Double = 0.0
    var rightX: Double = 0.0

    var motorFL: DcMotor? = null
    var motorBL: DcMotor? = null
    var motorFR: DcMotor? = null
    var motorBR: DcMotor? = null
    var motorFLPower: Double = 0.0
    var motorBLPower: Double = 0.0
    var motorFRPower: Double = 0.0
    var motorBRPower: Double = 0.0
    var slideGate: Servo? = null
    var pomPomServo: CRServo? = null
    var leftFlyWheel: CRServo? = null
    var intakeServo: CRServo? = null
    var rightFlyWheel: CRServo? = null
    var rMotorR: DcMotor? = null;
    var rMotorL: DcMotor? = null;
    var touchyR: TouchSensor? = null
    var touchyL: TouchSensor? = null
    var passiveServo: Servo? = null
    var aeroplaneLauncherServo: Servo? = null
    var rotateMotor: DcMotor? = null
    var slideMotor: DcMotor? = null
    var actualintakeServo: CRServo? = null
    var boxServo: Servo? = null
    var slideRotationMotor: DcMotor? = null
    var motorSlideRotate: DcMotor? = null
    var motorSlideLeft: DcMotor? = null
    var motorSlideRight: DcMotor? = null
    var clawRotation: Servo? = null
    var clawMotor: Servo? = null

    var closedClaw = 0.87
    var openClaw = 0.8
    var bottom = 0
    var low = 100 //unknown
    var mid = 500 //unknown
    var high = 1000
    var speed = 400
    var lPower = 0.8
    var lPowerSlow = lPower/3
    var rPower = -0.89122741664
    var rPowerSlow = rPower/4
    var lMax = 9850
    var lSpeedMax = 9500
    var rMax = -10000
    var rSpeedMax = -9500
    var lMin = 200
    var lSpeedMin = 900
    var rMin = -300
    var rSpeedMin = -800
    var slideRotMin = -200
    var slideRotMax = 428
    var slideGateClosed = 0.59
    var slideGateOpen = 0.55
    var length = (13.5 + (35.3 * Math.PI))/encoders
    var click2Degree = 0.154265
    var degree2Click = encoders/360
    var blinkinLedDriver: RevBlinkinLedDriver? = null
    var pattern: BlinkinPattern? = null
    var blinkinWorks = true

    var desiredTag: AprilTagDetection? = null
    var targetFound = false

    var DESIRED_DISTANCE = 12.0 //how close the camera should get to the target (inches)
    var SPEED_GAIN = 0.02 //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    var STRAFE_GAIN = 0.015 //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    var TURN_GAIN = 0.01 //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)
    var MAX_AUTO_SPEED = 0.5 //  Clip the approach speed to this max value (adjust for your robot)
    var MAX_AUTO_STRAFE = 0.5 //  Clip the approach speed to this max value (adjust for your robot)
    var MAX_AUTO_TURN = 0.3  //  Clip the turn speed to this max value (adjust for your robot)

    var drive = 0.0 //x
    var strafe = 0.0 //y
    var turn = 0.0 //yaw
    val AEROPLANE_LAUNCH = 0.9
    val AEROPLANE_CLOSE = 0.65

    enum class Direction {
        FORWARD, BACKWARD, RIGHT, LEFT, ROTATE_LEFT, ROTATE_RIGHT, ROTATE
    }

    enum class BlinkinColor {
        RAINBOW, RED, RED_PULSE, ORANGE, ORANGE_PULSE, BLUE, GREEN, GREEN_PULSE, YELLOW, PURPLE, PINK
    }

    enum class Detection {
        LEFT, CENTER, RIGHT, UNKNOWN
    }

    enum class CameraSide {
        LEFT, RIGHT
    }

    var t = 0.0;
    var slideToBoard = 0.0;
    var clawToBoard = 0.1;
    var x =  0.0;
    var y = 0.0;
    var slideLength = 0.0;
    var slideAngle = 0.0;
    var clawAngle = 0.0;

    enum class VisionProcessors {
        TFOD, APRILTAG, BOTH
    }
}