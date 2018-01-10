package org.firstinspires.ftc.teamcode.robots

import RelicRecoveryRobotOpModeManager
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.*
import org.corningrobotics.enderbots.endercv.CameraViewDisplay
import org.firstinspires.ftc.teamcode.libraries.components.lift.RelicRecoveryRobotLift
import org.firstinspires.ftc.teamcode.libraries.sensors.RangeSensor
import org.firstinspires.ftc.teamcode.libraries.vision.JewelConfigurationDetector
import org.firstinspires.ftc.teamcode.opmodes.competition.RelicRecoveryTeleOp
import kotlin.math.abs

/**
 * A class for our robot in the 2017-2018 FTC game, Relic Recovery.
 *
 * @author FTC Team 5037 gotrobot?
 */
class RelicRecoveryRobot : MecanumRobot() {

    companion object {
        val CRYPTO_BOX_SPACING = 50.0

        val LEADING_FRONT_CRYPTO_BOX_DISTANCE = 46.0
        val CENTER_FRONT_CRYPTO_BOX_DISTANCE = LEADING_FRONT_CRYPTO_BOX_DISTANCE + 17.0
        val TRAILING_FRONT_CRYPTO_BOX_DISTANCE = CENTER_FRONT_CRYPTO_BOX_DISTANCE + 17.0

        val LEADING_SIDE_CRYPTO_BOX_DISTANCE = 100.0
        val CENTER_SIDE_CRYPTO_BOX_DISTANCE = LEADING_SIDE_CRYPTO_BOX_DISTANCE + 17.0
        val TRAILING_SIDE_CRYPTO_BOX_DISTANCE = CENTER_SIDE_CRYPTO_BOX_DISTANCE + 17.0

        private val DEFAULT_OBJECT_DISTANCE_TOLERANCE = 3.0

        private val BALANCING_STONE_ANGLE_THRESHOLD = 7.5
        private val BALANCING_STONE_GROUND_ANGLE_THRESHOLD = 3.0

        private val GLYPH_GRABBER_OPEN_POSITION = 0.50
        private val GLYPH_GRABBER_SMALL_OPEN_POSITION = 0.66
        private val GLYPH_GRABBER_RELEASE_POSITION = 0.75
        private val GLYPH_GRABBER_CLOSED_POSITION = 0.95

        private val GLYPH_DEPLOYER_EXTENDED_POSITION = 0.25
        private val GLYPH_DEPLOYER_RETRACTED_POSITION = 0.90
        private val GLYPH_DEPLOYER_UP = 0.01

        private val JEWEL_STICK_UP_POSITION = 0.0
        private val JEWEL_STICK_DOWN_POSITION = 0.875

        /**
         * Determines the current setup position of the robot.
         * @param hardwareMap The instance of HardwareMap from the OpMode that the robot is running on.
         * @return The setup position of the robot.
         */
        fun getRobotSetupPosition(hardwareMap: HardwareMap): SetupPosition {
            val colorSensor = hardwareMap.colorSensor.get("floor color sensor")
            val backRangeSensor = hardwareMap.analogInput.get("back range sensor")

            val red = colorSensor.red()
            val blue = colorSensor.blue()
            val backObjectDistance = (backRangeSensor.voltage / (5.0 / 512.0)) * 2.54

            return when {
                red < blue && backObjectDistance > 60 -> SetupPosition.FRONT_BLUE
                red < blue && backObjectDistance < 60 -> SetupPosition.BACK_BLUE
                red > blue && backObjectDistance < 60 -> SetupPosition.FRONT_RED
                red > blue && backObjectDistance > 60 -> SetupPosition.BACK_RED
                else -> SetupPosition.UNKNOWN
            }
        }
    }

    /**
     * A position that the robot can be setup on on the field.
     */
    enum class SetupPosition {
        FRONT_RED,
        FRONT_BLUE,
        BACK_BLUE,
        BACK_RED,
        UNKNOWN
    }

    lateinit var lift: RelicRecoveryRobotLift
    private lateinit var jewelStick: Servo
    private lateinit var leftGlyphGrabber: Servo
    private lateinit var rightGlyphGrabber: Servo
    private lateinit var glyphDeployer: Servo
    lateinit var leftRangeSensor: RangeSensor
    lateinit var rightRangeSensor: RangeSensor
    lateinit var frontLeftRangeSensor: RangeSensor
    lateinit var frontRightRangeSensor: RangeSensor
    lateinit var backRangeSensor: RangeSensor
    private lateinit var floorColorSensor: ColorSensor
    lateinit var jewelConfigurationDetector: JewelConfigurationDetector
    private var startingPitch = 0.0

    /**
     * Sets up the hardware needed in order to use the robot.
     * @param hardwareMap A HardwareMap object. Usually provided by linearOpMode.hardwareMap.
     */
    override fun setup(hardwareMap: HardwareMap) {
        linearOpMode.telemetry.log().add("Setting up the robot.")

        // Lift
        lift = RelicRecoveryRobotLift(linearOpMode, "winch motor", DcMotorSimple.Direction.REVERSE, "limit switch")

        // Attachments
        jewelStick = hardwareMap.servo.get("jewel stick")
        leftGlyphGrabber = hardwareMap.servo.get("left grabber")
        rightGlyphGrabber = hardwareMap.servo.get("right grabber")
        glyphDeployer = hardwareMap.servo.get("glyph deployer")

        // Sensors
        leftRangeSensor = RangeSensor(linearOpMode, "left range sensor", I2cAddr.create8bit(0x32))
        frontLeftRangeSensor = RangeSensor(linearOpMode, "front left range sensor")
        frontRightRangeSensor = RangeSensor(linearOpMode, "front right range sensor")
        rightRangeSensor = RangeSensor(linearOpMode, "right range sensor")
        backRangeSensor = RangeSensor(linearOpMode, "back range sensor")
        floorColorSensor = hardwareMap.colorSensor.get("floor color sensor")

        // Computer Vision
        jewelConfigurationDetector = JewelConfigurationDetector()

        super.setup(hardwareMap)
    }

    /**
     * Starts processes that are critical for the robot to operate properly in autonomous.
     */
    fun start() {
        startUpdatingRangeSensors()
        startUpdatingDriveMotorPowers()
        startingPitch = this.pitch
    }

    /**
     * Runs everything that is needed in order to run autonomous.
     */
    fun prepareForAutonomous(linearOpMode: LinearOpMode) {
        linearOpMode.telemetry.log().add("Preparing the robot for autonomous.")

        this.linearOpMode = linearOpMode
        setup(linearOpMode.hardwareMap)

        jewelConfigurationDetector.init(linearOpMode.hardwareMap.appContext, CameraViewDisplay.getInstance())
        jewelConfigurationDetector.enable()

        RelicRecoveryRobotOpModeManager.queueOpMode(linearOpMode, RelicRecoveryTeleOp.OPMODE_NAME)
        RelicRecoveryRobotOpModeManager.robotInUse = this

        waitForGyroCalibration()
    }

    // Driving

    /**
     * Drives the robot forward or backwards for a certain amount of time.
     * @param milliseconds The length of time in seconds to drive.
     * @param power The power to run the motors at when driving.
     */
    fun timeDrive(milliseconds: Long, power: Double = 0.25) {

        if (!linearOpMode.isStopRequested) {
            setDrivePower(power)
            linearOpMode.sleep(milliseconds)
        }

        stopAllDriveMotors()
    }

    /**
     * Drives backward at max speed for a very short period of time to slow down. Only use in cases
     * where the robot is moving very fast forward.
     */
    fun powerBreak() {
        timeDrive(100, -1.0)
    }

    /**
     * Drives the robot forwards or backwards until the wall is a certain distance away from the front.
     * @param distance The distance the robot should be from a front object.
     * @param power The power to run the motors at when driving.
     */
    fun driveToDistanceFromForwardObject(distance: Double, power: Double = 0.2) {

        linearOpMode.telemetry.log().add("Driving to forward object.")

        // Check to see if the opmode is still active.
        if (!linearOpMode.isStopRequested) {
            // Get the current distance for future reference.
            val currentDistance = frontRightRangeSensor.distanceDetected

            when {
                currentDistance > distance -> {
                    setDrivePower(Math.abs(power))
                    while (frontRightRangeSensor.distanceDetected > distance && linearOpMode.opModeIsActive()) {
                        linearOpMode.sleep(10)
                    }
                }

                currentDistance < distance -> {
                    setDrivePower(-Math.abs(power))
                    while (frontRightRangeSensor.distanceDetected < distance && linearOpMode.opModeIsActive()) {
                        linearOpMode.sleep(10)
                    }
                }
            }
        }

        stopAllDriveMotors()
    }

    /**
     * Drives the robot left or right until the wall is a certain distance away from the left.
     * @param distance The distance the robot should be from a left object.
     * @param power The power to run the motors at when driving.
     */
    fun driveToDistanceFromLeftObject(distance: Double, power: Double = 0.325) {

        linearOpMode.telemetry.log().add("Driving to distance from left object.")

        val currentDistance = leftRangeSensor.distanceDetected

        when {
            currentDistance > distance -> {
                linearOpMode.telemetry.log().add("Current distance is greater than the target distance.")
                setStrafePower(-Math.abs(power))
                while (leftRangeSensor.distanceDetected > distance && !linearOpMode.isStopRequested) {
                    linearOpMode.telemetry.addLine("LRS: ${leftRangeSensor.distanceDetected}")
                    linearOpMode.telemetry.update()
                    linearOpMode.sleep(10)
                }
            }

            currentDistance < distance -> {
                linearOpMode.telemetry.log().add("Current distance is greater than the target distance.")
                setStrafePower(Math.abs(power))
                while (leftRangeSensor.distanceDetected < distance && !linearOpMode.isStopRequested) {
                    linearOpMode.telemetry.addLine("LRS: ${leftRangeSensor.distanceDetected}")
                    linearOpMode.telemetry.update()
                    linearOpMode.sleep(10)
                }
            }

            else -> return
        }

        stopAllDriveMotors()
        linearOpMode.sleep(1000)

        val leftDistance = leftRangeSensor.distanceDetected
        if ((distance + DEFAULT_OBJECT_DISTANCE_TOLERANCE < leftDistance || distance - DEFAULT_OBJECT_DISTANCE_TOLERANCE > leftDistance)
                && linearOpMode.opModeIsActive()) {
            driveToDistanceFromLeftObject(distance, power)
        }

        stopAllDriveMotors()
    }

    /**
     * Drives the robot left or right until the wall is a certain distance away from the right.
     * @param distance The distance the robot should be from a right object.
     * @param power The power to run the motors at when driving.
     */
    fun driveToDistanceFromRightObject(distance: Double, power: Double = 0.325) {

        linearOpMode.telemetry.log().add("Driving to distance from right object.")

        val currentDistance = rightRangeSensor.distanceDetected

        when {
            currentDistance > distance -> {
                setStrafePower(Math.abs(power))
                while (rightRangeSensor.distanceDetected > distance && !linearOpMode.isStopRequested) {
                    linearOpMode.sleep(10)
                }
            }

            currentDistance < distance -> {
                setStrafePower(-Math.abs(power))
                while (rightRangeSensor.distanceDetected < distance && !linearOpMode.isStopRequested) {
                    linearOpMode.sleep(10)
                }
            }

            else -> return
        }

        linearOpMode.telemetry.update()

        stopAllDriveMotors()
        linearOpMode.sleep(1000)

        val rightDistance = rightRangeSensor.distanceDetected
        if ((distance + DEFAULT_OBJECT_DISTANCE_TOLERANCE < rightDistance || distance - DEFAULT_OBJECT_DISTANCE_TOLERANCE > rightDistance)
                && !linearOpMode.isStopRequested) {
            driveToDistanceFromRightObject(distance, 0.45)
        }

        stopAllDriveMotors()
    }

    /**
     * Drives the robot off of the balancing stone, using the angle of the robot as an indication of completion.
     * @param power The power ot run the motors at when driving.
     */
    fun driveOffBalancingStone(power: Double = 0.175) {
        if (!linearOpMode.isStopRequested) {
            setDrivePower(power)

            when {
                power > 0 -> {
                    linearOpMode.telemetry.log().add("Driving until offset pitch.")
                    while (startingPitch - pitch < BALANCING_STONE_ANGLE_THRESHOLD && !linearOpMode.isStopRequested) {
                        linearOpMode.sleep(10)
                    }
                    linearOpMode.telemetry.log().add("Detected a pitch of $pitch.")

                    this.raiseJewelStick(0)

                    linearOpMode.telemetry.log().add("Driving until level pitch.")
                    while (startingPitch - pitch >= BALANCING_STONE_GROUND_ANGLE_THRESHOLD && !linearOpMode.isStopRequested) {
                        linearOpMode.sleep(10)
                    }
                    linearOpMode.telemetry.log().add("Detected a pitch of $pitch.")
                }

                power < 0 -> {
                    linearOpMode.telemetry.log().add("Driving until offset pitch.")
                    while (startingPitch - pitch > -BALANCING_STONE_ANGLE_THRESHOLD && !linearOpMode.isStopRequested) {
                        linearOpMode.sleep(10)
                    }
                    linearOpMode.telemetry.log().add("Detected a pitch of $pitch.")

                    this.raiseJewelStick(0)

                    linearOpMode.telemetry.log().add("Driving until level pitch.")
                    while (startingPitch - pitch <= -BALANCING_STONE_GROUND_ANGLE_THRESHOLD && !linearOpMode.isStopRequested) {
                        linearOpMode.sleep(10)
                    }
                    linearOpMode.telemetry.log().add("Detected a pitch of $pitch.")
                }

                else -> return
            }
        }

        stopAllDriveMotors()
    }

    /**
     * Drives the robot on of the balancing stone, using the angle of the robot as an indication of completion.
     * @param power The power ot run the motors at when driving.
     */
    fun driveOnBalancingStone(power: Double = 0.45) {
        if (!linearOpMode.isStopRequested) {
            setDrivePower(power)

            while (abs(startingPitch - pitch) >= BALANCING_STONE_GROUND_ANGLE_THRESHOLD && !linearOpMode.isStopRequested) {
                linearOpMode.sleep(10)
            }
        }

        stopAllDriveMotors()
    }

    // Glyph Grabbers

    /**
     * Sets the position of the glyph grabbers.
     * @param position The position to set the grabbers at.
     */
    private fun setGlyphGrabbersPosition(position: Double) {
        leftGlyphGrabber.position = position
        rightGlyphGrabber.position = 1 - position
    }

    /**
     * Sets the glyph grabbers to the open position.
     * @param delay The milliseconds to wait after opening the glyph grabbers.
     */
    fun openGlyphGrabbers(delay: Long = 0) {
        setGlyphGrabbersPosition(GLYPH_GRABBER_OPEN_POSITION)
        linearOpMode.sleep(delay)
    }

    /**
     * Sets the glyph grabbers to the open position.
     * @param delay The milliseconds to wait after opening the glyph grabbers.
     */
    fun smallOpenGlyphGrabbers(delay: Long = 0) {
        setGlyphGrabbersPosition(GLYPH_GRABBER_SMALL_OPEN_POSITION)
        linearOpMode.sleep(delay)
    }

    /**
     * Sets the glyph grabbers to the release position.
     * @param delay The milliseconds to wait after releasing the glyph grabbers.
     */
    fun releaseGlyphGrabbers(delay: Long = 0) {
        setGlyphGrabbersPosition(GLYPH_GRABBER_RELEASE_POSITION)
        linearOpMode.sleep(delay)
    }

    /**
     * Sets the glyph grabbers to the closed position.
     * @param delay The milliseconds to wait after closing the glyph grabbers.
     */
    fun closeGlyphGrabbers(delay: Long = 0) {
        setGlyphGrabbersPosition(GLYPH_GRABBER_CLOSED_POSITION)
        linearOpMode.sleep(delay)
    }

    // Glyph Deployer

    /**
     * Sets the glyph deployer to the extended position.
     * @param delay The milliseconds to wait after extending the glyph deployer.
     */
    fun extendGlyphDeployer(delay: Long = 0) {
        glyphDeployer.position = GLYPH_DEPLOYER_EXTENDED_POSITION
        linearOpMode.sleep(delay)
    }

    /**
     * Sets the glyph deployer to the retracted position.
     * @param delay The milliseconds to wait after retracting the glyph deployer.
     */
    fun retractGlyphDeployer(delay: Long = 0) {
        glyphDeployer.position = GLYPH_DEPLOYER_RETRACTED_POSITION
        linearOpMode.sleep(delay)
    }

    /**
     * Sets the glyph deployer to the lifted position.
     * @param delay The milliseconds to wait after lifting the glyph deployer.
     */
    fun liftGlyphDeployer(delay: Long = 0) {
        glyphDeployer.position = GLYPH_DEPLOYER_UP
        linearOpMode.sleep(delay)
    }

    // Jewel Stick

    /**
     * Sets the jewel stick position.
     * @param position The position to set the jewel stick in.
     */
    private fun setJewelStickPosition(position: Double) {
        jewelStick.position = position
    }

    /**
     * Sets the position of the jewel stick to the raised position.
     * @param delay The milliseconds to wait after raising the jewel stick.
     */
    fun raiseJewelStick(delay: Long = 0) {
        setJewelStickPosition(JEWEL_STICK_UP_POSITION)
        linearOpMode.sleep(delay)
    }

    /**
     * Sets the position of the jewel stick to the lowered position.
     * @param delay The milliseconds to wait after lowering the jewel stick.
     */
    fun lowerJewelStick(delay: Long = 500) {
        setJewelStickPosition(JEWEL_STICK_DOWN_POSITION)
        linearOpMode.sleep(delay)
    }

    // Range Sensors

    /**
     * Begins updating the values of the range sensors.
     */
    fun startUpdatingRangeSensors() {
        frontLeftRangeSensor.startUpdatingDetectedDistance()
        frontRightRangeSensor.startUpdatingDetectedDistance()
        leftRangeSensor.startUpdatingDetectedDistance()
        rightRangeSensor.startUpdatingDetectedDistance()
        backRangeSensor.startUpdatingDetectedDistance()
    }

    // Gamepads

    fun firstGamepadIsIdle(): Boolean {
        val gp = linearOpMode.gamepad1
        return !(gp.atRest() || gp.a || gp.b || gp.x || gp.y || gp.left_bumper || gp.right_bumper
                || gp.dpad_up || gp.dpad_down || gp.dpad_right || gp.dpad_left || gp.back || gp.start)
    }

    fun secondGamepadIsIdle(): Boolean {
        val gp = linearOpMode.gamepad2
        return !(gp.atRest() || gp.a || gp.b || gp.x || gp.y || gp.left_bumper || gp.right_bumper
                || gp.dpad_up || gp.dpad_down || gp.dpad_right || gp.dpad_left || gp.back || gp.start)
    }
}