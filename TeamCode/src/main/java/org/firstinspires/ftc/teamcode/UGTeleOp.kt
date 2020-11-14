package org.firstinspires.ftc.teamcode
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference
import org.firstinspires.ftc.teamcode.UGMechRobot
//import org.openftc.revextensions2.ExpansionHubEx
/*
    Created by S_Suchyy on 11/13/20
 */

@TeleOp(name = "UGTeleOp",group = "TeleOp")
//@Disabled
class UGTeleOp : OpMode(){
        //using robot class for motors, servos etc
        val robot = UGMechRobot()
        val zero = 0.0.toFloat()


        var leftPower: Float = 0.0.toFloat()
        var rightPower: Float = 0.0.toFloat()

        var drive = 0.0


        override fun init() {
            telemetry.addData("Status: ", "TeleOp Initialized")
            telemetry.update()
            //initializes all parts
            robot.init(hardwareMap)
            //robot.vSlide?.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER //Use encoders for linear slide motor

        }

        override fun start() { //runs once when play button is pushed
            //robot.vSlide?.mode = DcMotor.RunMode.RUN_USING_ENCODER
            //robot.leftHook?.position = 0.0
            //robot.rightHook?.position = 0.0
        }

        override fun loop() {

            /**
             * Gamepad1: Tank Drive-Left Stick y=Left Motor; right stick y=Right Motor
             * Gamepad2: Crane: Right Stick Y = Y Slide; Left Bumper = pinch claw; Left Stick Y= X Slide; a = foundation hooks
             */

            robot.mechanumPOV(gamepad1) //Drive Power Calculation
//            robot.vSlide?.power = robot.vSlideCalc(gamepad2)
//            robot.hSlide?.position = robot.hSlideCalc(gamepad2)
//            robot.pinch(gamepad2) //operates claw
//            robot.dropHook(gamepad2) //operates hooks
//            robot.nyoomPark(gamepad2) //operates the tape measure


//            if (!robot.touch!!.state) telemetry.addData("Touch Sensor:", "Activated")
//
//            if (robot.tooHigh) telemetry.addData("Linear Slide Y Error:", "MAX HEIGHT REACHED")
//
//            if (robot.tooLow) telemetry.addData("Linear Slide Y Error:", "MIN HEIGHT REACHED")

            if (gamepad1.left_trigger > 0.0) telemetry.addData("Slowdown:", "Engaged!")

//            telemetry.addData("Linear Slides", "V: ${robot.vSlide?.power}  ;  H: ${robot.hSlide?.position}") //kotlin string templates
//
//            telemetry.addData("Drive Motors:","front left: ${robot.fLDrive?.power}, front right: ${robot.fRDrive?.power}, " +
//                    "back left: ${robot.bLDrive?.power}, back right: ${robot.bRDrive?.power}")
//
//            telemetry.addData("Attachments:", "Claw = ${robot.claw?.position?.toFloat()}, " +
//                    "VSlide = ${robot.curPos.toFloat()}" +
//                    "Hooks: ${robot.leftHook?.position},${robot.rightHook?.position}")

            telemetry.addData("Gamepad Stick Vals (x,y):","Left Stick= ${gamepad1.left_stick_x}, ${gamepad1.left_stick_y}; " +
                    "Right Stick = ${gamepad1.right_stick_x}, ${gamepad1.right_stick_y}")
        }

        override fun stop() {
            robot.brake()
            telemetry.addData("Status: ", "TeleOp Terminated")
            telemetry.update()
        }


        fun tankMode()
        {
            //slowDown = gamepad1.left_trigger + 2.0
            robot.slowDown = if(gamepad1.left_bumper) 2.35 else 1.00 //condensed if else


            //Tank Drive-sets power equal to numerical value of joystick positions
            leftPower = gamepad1.left_stick_y
            rightPower = gamepad1.right_stick_y
            robot.fLDrive?.power = leftPower.toDouble() / robot.slowDown
            robot.bLDrive?.power = leftPower.toDouble() / robot.slowDown
            robot.fRDrive?.power = rightPower.toDouble() / robot.slowDown
            robot.bRDrive?.power = rightPower.toDouble() / robot.slowDown
            telemetry.addData("front left: ${robot.fLDrive?.power}, front right: ${robot.fRDrive?.power}, " +
                    "back left: ${robot.bLDrive?.power}, back right: ${robot.bRDrive?.power}", "")

        }


        /*
            Tank Mode 1: DPad control strafe, right joystick control power and turn
         */

        /*
            Tank Mode 2: Old Tank mode with triggers controlling strafe
         */

        /*
            Heli Mode: Left Joystick control power, right joystick controls turning, triggers control strafe
         */
}

