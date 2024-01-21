package org.firstinspires.ftc.teamcode.Teleop

import com.qualcomm.hardware.rev.RevBlinkinLedDriver
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
import org.firstinspires.ftc.teamcode.Variables.touchyL
import org.firstinspires.ftc.teamcode.Variables.touchyR
import kotlin.math.abs

@TeleOp(name = "TeleopFromHell", group = "TeleopFinal")
class TeleopFromHell: DriveMethods() {
    override fun runOpMode() {
        initMotorsSecondBot() //init rack and pinion & wheel motors

        telemetry.addLine(when ((0..38).random()) {
            1 -> "good luck buddy"
            2 -> "\"what spectrum?\""
            3 -> "MostlyOp >>> AHoT"
            4 -> "01101011 01101001 01101100 01101100 00100000 01111001 01101111 01110101 01110010 01110011 01100101 01101100 01100110"
            5 -> "I LOVE ULTRAKILL!!!!!!!!!!!!"
            6 -> "\"just hit clean build\""
            7 -> "this match is gonna be ghoulish green"
            8 -> "we are so back"
            9 -> "ok so would you rather have a 1% chance of becoming a turkey everyday or..."
            10 -> "RIP damien mode 2022-2023"
            11 -> "build freeze at 3 AM challenge (GONE WRONG)"
            12 -> "\"who unqueued my song?\""
            13 -> "at least we don't have a pushbot! (not confirmed, high likelyhood of pushbot)"
            14 -> "whoever set continuous rotation as the default is my #1 opp"
            15 -> "shoutout to Huy for being our insider <3"
            16 -> "why does jack always come to TR3? Is he stupid?"
            17 -> "Nick, I need you to sand this."
            18 -> "I wish fame and good fortune upon Sachal's bloodline"
            19 -> "-((2 / (1 + (exp(-(target - Pos) / speed)))) - 1) * max"
            20 -> "\"the grid system is stupid.\" *starts pointing at poles*"
            21 -> "James, how many orange cups have you eaten today?"
            22 -> "Tennisball is the newest sport sweeping across the nation!"
            23 -> "our robot has been too big for the bounding box on 3 different occasions."
            24 -> "cord control is not real"
            25 -> "in Raytheon we trust"
            26 -> "drive practice is for nerds."
            27 -> "Sebastian (yum)"
            28 -> "this is the abyss of our hero's journey."
            29 -> "beware the FTC to Raytheon pipeline"
            30 -> "when build says 15 minutes, expect 30. When programming says 15 minutes, expect 2-60."
            31 -> "99% of programmers quit right before the working push"
            32 -> "Tiger Woods PGA tour 2005 has always been there"
            33 -> "THIS SENT ME \n sent you where? \n TO FINLAND \u1F1E"
            34 -> "How purple?"
            35 -> "That is fragrantly upside down"
            36 -> "I literally just stand here and look at you guys and think god when is this done"
            37 -> "They should be singing in the closet"
            else -> "Why did we add these?"
        })
        telemetry.update()

        //slideGate?.position = 0.59
        aeroplaneLauncherServo!!.position = AEROPLANE_CLOSE
        waitForStart()
        //slideGate?.position = 0.55
        //set claw position into bounds
        //clawRotation!!.position = 0.3
        //clawMotor!!.position = closedClaw it broke womp womp
        //reset motors
       // motorSlideLeft!!.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        //motorSlideLeft!!.mode = DcMotor.RunMode.RUN_USING_ENCODER;
        //slideRotationMotor!!.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        //slideRotationMotor!!.mode = DcMotor.RunMode.RUN_USING_ENCODER;
        //set up variables
        var leftY: Double
        var leftYGPadTwo: Double
        var leftX: Double
        var rightX: Double
        var placeOrCollect = true
        var upOrDown = true
        var slideTarget = bottom
        var targetHeight = 0
        var slidePos = 0
        var slideRotPos = 0
        var speedDiv = 1.5
        var slideDeg = 0.0
        var angleFromSlideToClaw = 0.0
        var slideRottarget = 25.0
        var magicHoldNumber = 0
        var clawClamp = false
        var aeroplaneHasBeenLaunched = false
        var holdingpower = 0.001
        var rackAndPainNotAtTopToggle = false
        var rackAndPainNotAtBottomToggle = false
        var moveTheServoNuggetAtTheTopToggle = true
        var intakeServoOffToggle = true
        var pixelDropperToggle = false
        var rotatePower = -.5
        var reverseToggle = false
        var reverseThing = 1
        var rackAndPainUpRightToggle = true
        var rackAndPainUpLeftToggle = true
        var touchSensorLeftHit = false
        var touchSensorRightHit = false
        var rackAndPainAutoLeftToggle = false
        var rackAndPainAutoRightToggle = false
        var gamepadYToggle = false
        var gamepadBToggle = false

        //rotateMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        rotateMotor?.mode = DcMotor.RunMode.RUN_USING_ENCODER
        rotateMotor?.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        //slideMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        slideMotor?.mode = DcMotor.RunMode.RUN_USING_ENCODER
        slideMotor?.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        while (opModeIsActive()) {
            //set gamepad inputs
            leftY = (-gamepad1.left_stick_y).toDouble()* reverseThing
            leftYGPadTwo = (gamepad2.left_stick_y).toDouble()
            leftX = gamepad1.left_stick_x.toDouble() * reverseThing
            rightX = -gamepad1.right_stick_x.toDouble()

            //set motor speeds

            motorFL?.power = -(leftY - leftX - rightX) / speedDiv
            motorBL?.power = -(leftY + leftX - rightX) / speedDiv
            motorFR?.power = (leftY + leftX + rightX) / speedDiv
            motorBR?.power = (leftY - leftX + rightX) / speedDiv
            /*
            //open/close claw
            if (gamepad2.b) {
                //clawMotor!!.position = closedClaw
            }
            if (gamepad2.a) {
                //clawMotor!!.position = openClaw
            }
            */
            if (gamepad1.left_bumper) {
                motorFL?.power = 1.0/speedDiv
                motorBL?.power = -1.0/speedDiv
                motorFR?.power = 1.0/speedDiv
                motorBR?.power = -1.0/speedDiv
            }

            if (gamepad1.right_bumper) {
                motorFL?.power = -1.0/speedDiv
                motorBL?.power = 1.0/speedDiv
                motorFR?.power = -1.0/speedDiv
                motorBR?.power = 1.0/speedDiv
            }
            if (gamepad1.y){
                gamepadYToggle = true
            }
            else {
                gamepadYToggle = false
            }
            if (gamepad1.b){
                gamepadBToggle = true
            }
            else {
                gamepadBToggle = false
            }
            if (gamepad1.a) {
                sleep(500)
                if (reverseToggle) {
                    reverseThing = 1
                    reverseToggle = false
                }
                else {
                    reverseThing = -1
                    reverseToggle = true
                }
            }

            if (gamepad1.dpad_down) {
                autoServo!!.position = 0.63
            }

            if (!blinkinWorks) telemetry.addLine("Blinkin is not currently working")

            //raise/lower slide
            //if (gamepad2.left_trigger >= 0.8) {
            //    if (targetHeight < 3) {
               //     targetHeight++
           //     }
            //    sleep(150)
           // }
            //if (gamepad2.left_bumper) {
           //     if (targetHeight > 0) {
           //         targetHeight--
            //    }
           //     sleep(150)
          //  }

            //when (targetHeight) {
          //      0 -> {slideTarget = bottom}
          //      1 -> {slideTarget = low}
          //      2 -> {slideTarget = mid}
         //       3 -> {slideTarget = high}
          //  }

         //   slidePos = motorSlideLeft?.let { -(it.currentPosition) }!!
        //    speed = if (slideTarget < slidePos!!) {
         //       50
         //   } else {
         //       300
         //   }
            //motorSlideLeft?.power = -((2 / (1 + (exp((-(slideTarget - slidePos) / speed).toDouble())))) - 1)

            //rack & pinion control
            telemetry.addData("Left Motor:", rMotorR?.currentPosition)
            telemetry.addData("Right Motor:", rMotorL?.currentPosition)

            if (rackAndPainUpLeftToggle && (rMotorL?.currentPosition!!>6000)){
                rMotorL!!.power = 0.0
                rackAndPainUpLeftToggle = false
                rackAndPainAutoLeftToggle = false
            }
            if (!rackAndPainUpLeftToggle && (rMotorL?.currentPosition!!>9000)){
                rMotorL!!.power = 0.0
                rackAndPainUpLeftToggle = true
                rackAndPainAutoLeftToggle = false
            }
            if (rackAndPainUpRightToggle && (rMotorR?.currentPosition!!<-6000)){
                rMotorR!!.power = 0.0
                rackAndPainUpRightToggle = false
                rackAndPainAutoRightToggle = false
            }
            if (!rackAndPainUpRightToggle && (rMotorR?.currentPosition!!<-9000)){
                rMotorR!!.power = 0.0
                rackAndPainUpRightToggle = true
                rackAndPainAutoRightToggle = false
            }



            if (gamepadYToggle){
                rackAndPainAutoLeftToggle = true
                sleep(1000)
                if (rackAndPainUpLeftToggle){
                    rMotorL!!.power = lPower
                }
                else {
                    rMotorL!!.power = lPowerSlow
                }
            }
            else if (gamepadBToggle){
                rackAndPainAutoLeftToggle = true
                rMotorL!!.power = -.4
            }
            else if (gamepad1.left_trigger >= 0.5) {
                if (upOrDown) {
                    if (rMotorL?.currentPosition!! <= lMax) {
                        if (rMotorL?.currentPosition!! in lSpeedMin..lSpeedMax) {
                            telemetry.addLine("LUp")
                            rMotorL!!.power = lPower
                        } else {
                            rMotorL!!.power = lPowerSlow
                        }
                    }
                } else {
                    if (rMotorL?.currentPosition!! >= lMin) {
                        if (rMotorL?.currentPosition!! in lSpeedMin..lSpeedMax) {
                            telemetry.addLine("LUp")
                            rMotorL!!.power = -lPower
                        } else {
                            rMotorL!!.power = -lPowerSlow
                        }
                    }
                }
            }
            else if (touchyL!!.isPressed){
                rMotorL!!.power = 0.0
                rackAndPainAutoLeftToggle = false
            }
            else if (!rackAndPainAutoLeftToggle){
                rMotorL!!.power = 0.0
            }



            if (gamepadYToggle){
                rackAndPainAutoRightToggle = true
                sleep(1000)
                if (rackAndPainUpRightToggle){
                    rMotorR!!.power = rPower
                }
                else {
                    rMotorR!!.power = rPowerSlow
                }
            }
            else if (gamepadBToggle){
                rackAndPainAutoRightToggle = true
                rMotorR!!.power = .4
            }
            else if (gamepad1.right_trigger >= 0.5) {
                // telemetry.addLine("edrgthgj")
                if (upOrDown) {
                    if (rMotorR?.currentPosition!! >= rMax) {
                        if (rMotorR?.currentPosition!! in rSpeedMax..rSpeedMin) {
                            rMotorR!!.power = rPower
                        } else {
                            rMotorR!!.power = rPowerSlow
                        }
                        telemetry.addLine("RMAX")
                    }
                } else {
                    if (rMotorR?.currentPosition!! <= rMin) {
                        if (rMotorR?.currentPosition!! in rSpeedMax..rSpeedMin) {
                            rMotorR!!.power = -rPower
                        } else {
                            rMotorR!!.power = -rPowerSlow
                        }
                        telemetry.addLine("RMIN")
                    }
                }
            }
            else if (touchyR!!.isPressed){
                rMotorR!!.power = 0.0
                rackAndPainAutoRightToggle = false
            }
            else if (!rackAndPainAutoRightToggle){
                rMotorR!!.power = 0.0
            }


            if (gamepad1.x) {
                telemetry.addData("upOrDown", upOrDown)
                upOrDown = !upOrDown
                sleep(500)
            }
            if (gamepad2.y && !aeroplaneHasBeenLaunched)  {
                if (magicHoldNumber >= 25) {
                    // Launch Aeroplane
                    aeroplaneLauncherServo!!.position = AEROPLANE_LAUNCH
                    magicHoldNumber = 0
                    aeroplaneHasBeenLaunched = true
                   // sleep(1000)
               } else {
                    magicHoldNumber++
                    sleep(25)
                    telemetry.addData("Magic Hold Number", magicHoldNumber)
               }
            } else if (!gamepad2.y && !aeroplaneHasBeenLaunched) {
            telemetry.addLine("reseting Magic Hold Number");
                sleep(25)
                magicHoldNumber = 0
            }

            if (abs(slideMotor!!.currentPosition) < 500){
                holdingpower = 0.0
            }
            else if (slideMotor!!.currentPosition > 500){
                holdingpower = .001
            }
            else if (slideMotor!!.currentPosition < -500){
                holdingpower = -.001
            }
            if (rotateMotor!!.currentPosition >= (rotateMotor!!.targetPosition - 20) && rotateMotor!!.targetPosition == 1850){
                rackAndPainNotAtTopToggle = false
            }
            if (abs(slideMotor!!.currentPosition)<=50 && slideMotor!!.targetPosition == 0){
                rackAndPainNotAtBottomToggle = false
            }
            if (gamepad2.left_stick_y.toDouble() >0.0) {
                rotateMotor!!.power = -gamepad2.left_stick_y.toDouble()
            }
            else if (gamepad2.left_stick_y.toDouble() < 0.0 && rotateMotor!!.currentPosition >0){
                rotateMotor!!.power = gamepad2.left_stick_y.toDouble()
            }
            else if (!rackAndPainNotAtTopToggle){
                rotateMotor!!.power = 0.0
            }
            if (gamepad2.right_bumper  && moveTheServoNuggetAtTheTopToggle){
                actualintakeServo?.power = -10.0
                moveTheServoNuggetAtTheTopToggle = false
                sleep(500)
            }
            else if (gamepad2.right_bumper  && !moveTheServoNuggetAtTheTopToggle){
                actualintakeServo?.power = 0.0
                moveTheServoNuggetAtTheTopToggle = true
                intakeServoOffToggle = true
                sleep(500)
            }
            if (gamepad2.right_trigger >= 0.5 && intakeServoOffToggle){
                actualintakeServo?.power = 10.0
                intakeServoOffToggle = false
                sleep(500)
            }
            else if (gamepad2.right_trigger >= 0.5 && !intakeServoOffToggle){
                actualintakeServo?.power = 0.0
                intakeServoOffToggle = true
                moveTheServoNuggetAtTheTopToggle = true
                sleep(500)
            }
            if (gamepad2.x) {
                if (!pixelDropperToggle) {
                    setBlinkinColour(RevBlinkinLedDriver.BlinkinPattern.GREEN) // Green means that it's open
                    boxServo!!.position = .45
                    pixelDropperToggle = true
                    sleep(500)
                } else {
                    setBlinkinColour(RevBlinkinLedDriver.BlinkinPattern.RED) // Red means that it's closed
                    boxServo!!.position = .62
                    pixelDropperToggle = false
                    sleep(500)
                }
            }
//            if (gamepad2.dpad_left){
//                boxServo.position = .62
//            }
//            if (gamepad2.dpad_right){
//                boxServo.position = .48
//            }
            telemetry.addData("Gamepad2 Right Y", gamepad2.right_stick_y)
            if (gamepad2.left_bumper){
                rackAndPainNotAtBottomToggle=true
                slideMotor!!.targetPosition = 0
                if (slideMotor!!.currentPosition >50){
                    slideMotor!!.power = -0.3
                }
                else if (slideMotor!!.currentPosition <50){
                    slideMotor!!.power = 0.3
                }
                sleep(500)
            }
            else if (gamepad2.right_stick_y.toDouble() > 0.0 && slideMotor!!.currentPosition >-1100) {
                slideMotor!!.power = -1.0 * gamepad2.right_stick_y
                //motorBeingTested.power = .05
            }
            else if (gamepad2.right_stick_y.toDouble() < 0.0 && slideMotor!!.currentPosition<1100) {
                slideMotor!!.power = -1.0 * gamepad2.right_stick_y
                //motorBeingTested.power = .05
            }
            else if (!rackAndPainNotAtBottomToggle){
                slideMotor!!.power = holdingpower
            }
            //claw stuff
           // if (gamepad2.y) {
           //     if (placeOrCollect) {
           //         placeOrCollect = !placeOrCollect
          //      }
//
         //       sleep(500)
         //   }

            //rotation
        //    slideRotationMotor?.currentPosition?.times(length)


         //   slideRotPos = slideRotationMotor?.let { -(it.currentPosition) }!!
         //   slideDeg = abs(slideRotPos - slideRotMax) * click2Degree
         //   angleFromSlideToClaw = 90.0 - (slideRotationMotor?.currentPosition!! + 113.0) * 0.157116

          //  if (placeOrCollect) {
          //      slideRottarget = 0.0
          //  } else {
          //      slideRottarget = 176.0
          //  }
         //   slideRotationMotor?.power = -(slideRotPos - abs(slideRottarget))/1000
          //  telemetry.addData("pos", slideRotationMotor!!.currentPosition)
         //   telemetry.addData("Tpos", slideRottarget)


//            All Stuff for Temp Passive intake claw

            if(gamepad2.a) {
                if (clawClamp) {
                    passiveServo!!.position = 0.2;
                }
                else {
                    passiveServo!!.position = .37
                }
                clawClamp = !clawClamp
                sleep(200)
            }

            telemetry.addData("Claw Clamped: ", clawClamp)
            telemetry.addData("Right rack: ", rMotorR?.currentPosition)
            telemetry.addData("Left rack: ", rMotorL?.currentPosition)
            telemetry.addData("Right rack Power: ", rMotorR?.power)
            telemetry.addData("Left rack Power: ", rMotorL?.power)
            telemetry.addData("Magic num: ", magicHoldNumber)
            telemetry.addData("Rotate Motor Value: ", rotateMotor!!.currentPosition)
            telemetry.addData("Slide Motor Value: ", slideMotor!!.currentPosition)
            telemetry.addData("Holding Power: ", holdingpower)
            telemetry.addData("ToggleRightRack: ", rackAndPainUpRightToggle)
            telemetry.addData("ToggleLeftRack: ", rackAndPainUpLeftToggle)
//            telemetry.addData("FR: ", motorFR?.power)
//            telemetry.addData("FL: ", motorFL?.power)
//            telemetry.addData("BR: ", motorBR?.power)
//            telemetry.addData("BL: ", motorBL?.power)
            telemetry.update()
        }
        aeroplaneLauncherServo!!.position = AEROPLANE_CLOSE
    }
}


