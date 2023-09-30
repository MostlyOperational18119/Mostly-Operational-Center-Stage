package org.firstinspires.ftc.teamcode

import com.qualcomm.hardware.rev.RevBlinkinLedDriver
import com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern
import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection


object Variables {
    // Redundant thanks to odometry.
    var rotationsPerMeter = 3.3
    var encoders = 537.6

    // Farmers Market Servo Values
    var servoRelease = 0.0
    var servoClamp = 0.0

    var t = 0.0;
    var slideToBoard = 0.0;
    var clawToBoard = 0.0;
    var x =  0.0;
    var y = 0.0;
    var slideLength = 0.0;
    var slideAngle = 0.0;
    var clawAngle = 0.0;

    enum class VisionProcessors {
        TFOD, APRILTAG, BOTH
    }
}