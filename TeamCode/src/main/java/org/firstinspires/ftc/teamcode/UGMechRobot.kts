package org.firstinspires.ftc.teamcode

import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator
import com.qualcomm.robotcore.hardware.*  
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference
import org.firstinspires.ftc.robotcore.external.navigation.Orientation
import org.openftc.revextensions2.ExpansionHubEx
import kotlin.math.abs
import kotlin.math.max
import kotlin.math.pow

/* 
    TODO: Clean up the Code to match this years season


 */


/**
 * Created by S_Suchyy on 10/14/2020. --
 */
class SSMechRobot {

    var hwdMap: HardwareMap? = null
    var bLDrive: DcMotor? = null
    var bRDrive: DcMotor? = null
    var fLDrive: DcMotor? = null
    var fRDrive: DcMotor? = null
//    var vSlide: DcMotorEx? = null
//    var hSlide: Servo? = null
//    var touch: DigitalChannel? = null
    var hub2: ExpansionHubEx? = null
    var imu: BNO055IMU? = null


    var slowDown = 1.85//default

    // Set up the parameters with which we will use our IMU. Note that integration
    // algorithm here just reports accelerations to the logcat log; it doesn't actually
    // provide positional information.
    var parameters = BNO055IMU.Parameters()




    //val cvFirstPercent = 23.0
    //val cvPercentSpace = 27.0
    //val cvStoneWidth = 50.0
    //val cvStoneHeight = 90.0

    var motF = DcMotorSimple.Direction.FORWARD
    var motR = DcMotorSimple.Direction.REVERSE
    var serR = Servo.Direction.REVERSE
    var serF = Servo.Direction.FORWARD

    fun init(ahwdMap: HardwareMap) {
        //hardware maping motors, servos, and sensors
        var dcList = mutableListOf<DcMotor>() //creating empty list for dc motor
        //increment for each var above
        //map for each dc
        //similar for servo and sensor
        hwdMap = ahwdMap

        bLDrive = ahwdMap.dcMotor.get("bLDrive")
        bRDrive = ahwdMap.dcMotor.get("bRDrive")
        fLDrive = ahwdMap.dcMotor.get("fLDrive")
        fRDrive = ahwdMap.dcMotor.get("fRDrive")
//        vSlide = ahwdMap.dcMotor.get("vSlide") as DcMotorEx
//        hSlide = ahwdMap.servo.get("hSlide")
//        touch = ahwdMap.digitalChannel.get("touch")
        hub2 = ahwdMap.get(ExpansionHubEx::class.java, "Expansion Hub 2")
        imu = ahwdMap.get(BNO055IMU::class.java, "imu")

        //Setting direction
        bLDrive?.direction = motF
        bRDrive?.direction = motR
        fLDrive?.direction = motF
        fRDrive?.direction = motR
        vSlide?.direction = motR
//        hSlide?.direction = serR
//        claw?.direction = serF

        initIMU()
        imu?.initialize(parameters)

        bLDrive?.power = 0.0
        bRDrive?.power = 0.0
        fLDrive?.power = 0.0
        fRDrive?.power = 0.0
        bLDrive?.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        bRDrive?.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
//        vSlide?.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
//        vSlide?.mode = DcMotor.RunMode.RUN_USING_ENCODER //Use encoders for linear slide motor
       // curPos = this.vSlide!!.currentPosition
        //vSlide?.targetPosition = vSlide!!.currentPosition
//        vSlide?.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
//        vSlide?.setVelocityPIDFCoefficients(kP, kI, kD, kF)
//        this.capstoneGate?.position = 0.8
    }


    //METHODS
    /**
     * Controls the left 2 motors at the same power
     *
     * @param pow the power assigned to both motors, flipped due to the joystick readings
     */
    fun leftPow(pow: Double) {
        bLDrive?.power = -pow
        fLDrive?.power = -pow
    }

    /**
     * Controls the right 2 motors at the same power
     *
     * @param pow the power assigned to both motors, flipped due to the joystick readings
     */
    fun rightPow(pow: Double) {
        bRDrive?.power = -pow
        fRDrive?.power = -pow
    }

    /**
     * Causes the robot to strafe left or right, by having the back left motor and top right motor spin one way,
     * and the other two spin the other way. Slows down the back motors to compensate for differences in hardware
     *
     * @param pow the power assigned to all motors before compensation and flipping the direction
     */
    fun strafe(pow: Double) //Positive Value = Left Strafe || Negative Value = Right Strafe
    {
        bLDrive?.power = -pow
        fLDrive?.power = pow
        bRDrive?.power = pow
        fRDrive?.power = -pow
    }

    /**
     * Controls all 4 motors with each side having it's own power.
     * Right side needs compensation due to the unaligned center of mass
     *
     * @param leftM the power assigned to the left 2 motors
     *
     * @param rightM the power assigned to the right 2 motors
     */
    fun drive(leftM: Double, rightM: Double) { //used for turning
        leftPow(leftM)
        rightPow(rightM)
    }

    /**
     * Overloads the previous method by having both the side set to the same power
     *
     * @param pow the power assigned to all motors
     */
    fun drive(pow: Double) {
        drive(pow, pow)
    }

    fun mechanumPOV(gp: Gamepad) {
        //POV Mode-left joystick=power(y power and strafing), right joystick=turning
        // Put powers in the range of -1 to 1 only if they aren't already (not
        // checking would cause us to always drive at full speed)

        slowDown = (gp.left_trigger * 3) + 1.0  //Dynamic Slowdown

        var drive = (gp.left_stick_y).toDouble()
        var turn = -gp.left_stick_x.toDouble() * 1.5
        var strafe = -gp.right_stick_x.toDouble()
        var nor = 0.0

        var frontLeftPower = (drive + turn + strafe)
        var backLeftPower = (drive - turn + strafe)
        var frontRightPower = (drive - turn - strafe)
        var backRightPower = (drive + turn - strafe)

        if (abs(frontLeftPower) > 1 || abs(backLeftPower) > 1 ||
                abs(frontRightPower) > 1 || abs(backRightPower) > 1) { //normalizing values to [-1.0,1.0]
            // Find the largest power
            nor = max(abs(frontLeftPower), abs(backLeftPower))
            nor = max(abs(frontRightPower), nor)
            nor = max(abs(backRightPower), nor)
        }
        // Divide everything by nor (it's positive so we don't need to worry
        // about signs)
        this.fLDrive?.power = (frontLeftPower / slowDown)
        this.bLDrive?.power = (backLeftPower / slowDown)
        this.fRDrive?.power = (frontRightPower / slowDown)
        this.bRDrive?.power = (backRightPower / slowDown)

    }

    /**
     * Sets all the motors' power to zero
     */
    fun brake() {
        this.drive(0.0)
    }

    fun stop() {
        this.brake()
        //this.vSlide?.power = 0.0
        //this.hSlide?.position = 0.5
        //this.tapeMeasure?.power = 0.0
    }

//

    /**
     * Controls the foundation hooks. By holding a, the foundation hooks drop to a set position
     *
     * @param gp the gamepad used to control the hooks
     */





    fun pause() {
        this.brake()
        Thread.sleep(500)
    }

    /**
     * Rotates the claw used for grabbing stones. Currently NOT in use.
     *
     * @param gp the gamepad used to control the hooks
     */


    /**
     *  Horizontal slide Power Calculation
     */

    fun initIMU()
    {
        this.parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES
        this.parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC
        this.parameters.calibrationDataFile = "BNO055IMUCalibration.json" // see the calibration sample opmode
        this.parameters.loggingEnabled = true
        this.parameters.loggingTag = "IMU"
        this.parameters.accelerationIntegrationAlgorithm = JustLoggingAccelerationIntegrator()
    }

    fun setDriveMode(mode: DcMotor.RunMode)
    {
        fLDrive?.mode = mode
        fRDrive?.mode = mode
        bLDrive?.mode = mode
        bRDrive?.mode = mode
    }

    fun turnAround()
    {/*
        var curAngle = imu?.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES)
        while(curAngle!!.firstAngle < imu?.angularOrientation!!.firstAngle + 180)
        {
            this.drive(-0.5, 0.5)
        }*/
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER)
        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER)
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER)
        while(fLDrive!!.currentPosition < 225) {
            fLDrive?.targetPosition = -224
            fRDrive?.targetPosition = 224
            bLDrive?.targetPosition = -224
            bRDrive?.targetPosition = 224
        }
        setDriveMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)

    }
}