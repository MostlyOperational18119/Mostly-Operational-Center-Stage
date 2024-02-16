package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.Optional;


public class VariablesButJava {
    // Redundant thanks to odometry.
    public static double servoRestPosition = 0.245;
    public static double servoMidPosition = 0.32;
    public static double servoPlacePosition = 0.65;
    static final double rotationsPerMeter = 3.3;
    static double encoders = 537.6;
    // Farmers Market Servo Values
    static double servoRelease = 0.0;
    static double servoClamp = 0.0;
    static double speedDiv = 3.0;
    static double leftY = 0.0;
    static double leftX = 0.0;
    static double rightX = 0.0;
    static DcMotor motorFL = null;
    static DcMotor motorBL = null;
    static DcMotor motorFR = null;
    static DcMotor motorBR = null;
    static double motorFLPower = 0.0;
    static double motorBLPower = 0.0;
    static double motorFRPower = 0.0;
    static double motorBRPower = 0.0;
    static Servo slideGate = null;
    static CRServo pomPomServo = null;
    static CRServo leftFlyWheel = null;
    static CRServo intakeServo = null;
    static CRServo rightFlyWheel = null;
    static DcMotor rMotorR = null;
    static DcMotor rMotorL = null;
    static TouchSensor touchyR = null;
    static TouchSensor touchyL = null;
    static Servo passiveServo = null;
    static Servo aeroplaneLauncherServo = null;
    static DcMotor rotateMotor = null;
    static DcMotor slideMotor = null;
    static CRServo actualintakeServo = null;
    static Servo boxServo = null;
    static DcMotor slideRotationMotor = null;
    static DcMotor motorSlideRotate = null;
    static DcMotor motorSlideLeft = null;
    static DcMotor motorSlideRight = null;
    static Servo clawRotation = null;
    static Servo clawMotor = null;
    static Servo autoServo = null;
    static TouchSensor slideTouch = null;
    static final double closedClaw = 0.87;
    static final double openClaw = 0.8;
    static final int bottom = 0;
    static final int low = 100; //unknown
    static final int mid = 500; //unknown
    static final int high = 1000;
    static final int speed = 400;
    static double lPower = 0.8;
    static final double lPowerSlow = lPower / 3;
    static double rPower = -0.89122741664;
    static final double rPowerSlow = rPower / 4;
    static final int lMax = 9850;
    static final int lSpeedMax = 9500;
    static final int rMax = -10000;
    static final int rSpeedMax = -9500;
    static final int lMin = 200;
    static final int lSpeedMin = 900;
    static final int rMin = -300;
    static final int rSpeedMin = -800;
    static final int slideRotMin = -200;
    static final int slideRotMax = 428;
    static final double slideGateClosed = 0.59;
    static final double slideGateOpen = 0.55;
    static final double length = (13.5 + (35.3 * Math.PI)) / encoders;
    static final double click2Degree = 0.154265;
    static final double degree2Click = encoders / 360;
    public static final double DESIRED_DISTANCE = 12.0; //how close the camera should get to the target (inches)
    public static final double SPEED_GAIN = 0.02; // Forward Speed Control "Gain".
    public static final double STRAFE_GAIN = 0.015; // Strafe Speed Control "Gain".
    public static final double TURN_GAIN = 0.01; // Turn Control "Gain".
    public static final double MAX_AUTO_SPEED = 0.5; // Clip the approach speed to this max value (adjust for your robot)
    public static final double MAX_AUTO_STRAFE = 0.5; // Clip the approach speed to this max value (adjust for your robot)
    public static final double MAX_AUTO_TURN = 0.3; // Clip the turn speed to this max value (adjust for your robot)
    public static final double AEROPLANE_LAUNCH = 0.9;
    public static final double AEROPLANE_CLOSE = 0.65;
    public Optional<RevBlinkinLedDriver> blinkinLedDriver = Optional.empty();
    public Optional<BlinkinPattern> pattern = Optional.empty();
    public boolean blinkinWorks = true;
    public double setAutoServoDown = 0.0;
    public double setAutoServoPlace = 0.35;
    public Optional<AprilTagDetection> desiredTag = Optional.empty();
    public boolean targetFound = false;
    public double drive = 0.0; //x
    public double strafe = 0.0; //y
    public double turn = 0.0; //yaw
    public double t = 0.0;
    public double slideToBoard = 0.0;
    public double clawToBoard = 0.1;
    public double x = 0.0;
    public double y = 0.0;
    public double slideLength = 0.0;
    public double slideAngle = 0.0;
    public double clawAngle = 0.0;

    public enum Direction {
        FORWARD, BACKWARD, RIGHT, LEFT, ROTATE_LEFT, ROTATE_RIGHT, ROTATE
    }

    public enum BlinkinColor {
        RAINBOW, RED, RED_PULSE, ORANGE, ORANGE_PULSE, BLUE, GREEN, GREEN_PULSE, YELLOW, PURPLE, PINK
    }

    public enum Detection {
        LEFT, CENTER, RIGHT, UNKNOWN
    }

    public enum CameraSide {
        LEFT, RIGHT
    }

    public enum VisionProcessors {
        TFOD, APRILTAG, BOTH
    }

// Assuming RevBlinkinLedDriver and AprilTagDetection are classes you will need to define in Java.
// Replace 'RevBlinkinLedDriver' and 'AprilTagDetection' with the correct Java class names.

}