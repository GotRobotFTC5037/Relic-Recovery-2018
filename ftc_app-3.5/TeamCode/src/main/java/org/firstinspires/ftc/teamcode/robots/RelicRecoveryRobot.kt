package org.firstinspires.ftc.teamcode.robots

/**
 * A class for our robot in the 2017-2018 FTC game, Relic Recovery.
 *
 * @author FTC Team 5037 gotrobot?
 */
class RelicRecoveryRobot/* : MecanumRobot() */{
    /*

    companion object {
        val CRYPTO_BOX_SPACING = 50.0

        val LEADING_FRONT_CRYPTO_BOX_DISTANCE = 46.0
        val CENTER_FRONT_CRYPTO_BOX_DISTANCE = LEADING_FRONT_CRYPTO_BOX_DISTANCE + 18.0
        val TRAILING_FRONT_CRYPTO_BOX_DISTANCE = CENTER_FRONT_CRYPTO_BOX_DISTANCE + 18.0

        val LEADING_SIDE_CRYPTO_BOX_DISTANCE = 100.0
        val CENTER_SIDE_CRYPTO_BOX_DISTANCE = LEADING_SIDE_CRYPTO_BOX_DISTANCE + 18.0
        val TRAILING_SIDE_CRYPTO_BOX_DISTANCE = CENTER_SIDE_CRYPTO_BOX_DISTANCE + 18.0

        private val DEFAULT_OBJECT_DISTANCE_TOLERANCE = 3.0

        private val BALANCING_STONE_ANGLE_THRESHOLD = 6.0
        private val BALANCING_STONE_GROUND_ANGLE_THRESHOLD = 4.5

        /**
         * Determines the current setup position of the robot.
         * @return The setup position of the robot.
         */
        fun getRobotSetupPosition(linearOpMode: LinearOpMode): SetupPosition {
            val colorSensor = linearOpMode.hardwareMap.colorSensor.get("floor color sensor")
            val backRangeSensor = linearOpMode.hardwareMap.get(ModernRoboticsI2cRangeSensor::class.java, "back range sensor")

            val red = colorSensor.red()
            val blue = colorSensor.blue()
            val backObjectDistance = backRangeSensor.cmUltrasonic()

            return when {
                blue > red && backObjectDistance >= 35 -> SetupPosition.FRONT_BLUE
                blue > red && backObjectDistance < 35 -> SetupPosition.BACK_BLUE

                red > blue && backObjectDistance < 100 -> SetupPosition.BACK_RED
                red > blue && backObjectDistance >= 100 -> SetupPosition.FRONT_RED

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
    private lateinit var jewelStick: JewelStick
    private lateinit var glyphGrabber: GlyphGrabber

    private lateinit var leftRangeSensor: RangeSensor
    private lateinit var rightRangeSensor: RangeSensor
    private lateinit var frontLeftRangeSensor: RangeSensor
    private lateinit var frontRightRangeSensor: RangeSensor
    private lateinit var backRangeSensor: RangeSensor
    private lateinit var floorColorSensor: ColorSensor

    lateinit var jewelConfigurationDetector: JewelConfigurationDetector

    private var startingPitch = 0.0

    /**
     * Sets up the hardware needed in order to use the robot.
     * @param hardwareMap A HardwareMap object. Usually provided by opMode.hardwareMap.
     */
    override fun setup(hardwareMap: HardwareMap) {
        opMode.telemetry.log().add("Setting up the robot.")

        // Lift
        lift = RelicRecoveryRobotLift(opMode, "winch motor", DcMotorSimple.Direction.REVERSE, "limit switch")

        // Attachments
        jewelStick = hardwareMap.servo.get("jewel stick")
        leftGlyphGrabber = hardwareMap.servo.get("left grabber")
        rightGlyphGrabber = hardwareMap.servo.get("right grabber")
        glyphDeployer = hardwareMap.servo.get("glyph deployer")

        // Sensors
        leftRangeSensor = RangeSensor(opMode, "left range sensor", I2cAddr.create8bit(0x32))
        frontLeftRangeSensor = RangeSensor(opMode, "front left range sensor")
        frontRightRangeSensor = RangeSensor(opMode, "front right range sensor")
        rightRangeSensor = RangeSensor(opMode, "right range sensor")
        backRangeSensor = RangeSensor(opMode, "back range sensor")
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
     * Starts processes that are critical for the robot to operate properly in autonomous.
     */
    fun startAuto() {
        startUpdatingRangeSensors()
        startUpdatingDriveMotorPowersAuto()
        startingPitch = this.pitch
    }

    /**
     * Runs everything that is needed in order to run autonomous.
     */
    fun prepareForAutonomous(linearOpMode: LinearOpMode) {
        linearOpMode.telemetry.log().add("Preparing the robot for autonomous.")

        this.opMode = linearOpMode
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
    fun timeDrive(milliseconds: Long, power: Double = 0.5) {

        if (!opMode.isStopRequested) {
            setDrivePower(power)
            opMode.sleep(milliseconds)
        }

        stopAllDriveMotors()
    }

    /**
     * Drives backward at max speed for a very short period of time to slow down. Only use in cases
     * where the robot is moving very fast forward.
     */
    fun powerBreak(forward: Boolean = true) {
        if (forward) {
            timeDrive(25, -1.0)
        } else {
            timeDrive(25, 1.0)
        }
    }

    /**
     * Drives the robot forwards or backwards until the wall is a certain distance away from the front.
     * @param distance The distance the robot should be from a front object.
     * @param power The power to run the motors at when driving.
     */
    fun driveToDistanceFromForwardObject(distance: Double, power: Double = 0.315) {

        opMode.telemetry.log().add("Driving to forward object.")

        // Check to see if the opmode is still active.
        if (!opMode.isStopRequested) {
            // Get the current distance for future reference.
            val currentDistance = frontRightRangeSensor.distanceDetected

            when {
                currentDistance > distance -> {
                    setDrivePower(Math.abs(power))
                    while (frontRightRangeSensor.distanceDetected > distance && opMode.opModeIsActive()) {
                        opMode.sleep(10)
                    }
                }

                currentDistance < distance -> {
                    setDrivePower(-Math.abs(power))
                    while (frontRightRangeSensor.distanceDetected < distance && opMode.opModeIsActive()) {
                        opMode.sleep(10)
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
    fun driveToDistanceFromLeftObject(distance: Double, power: Double = 0.315, shouldCorrect: Boolean = true) {

        opMode.telemetry.log().add("Driving to distance from left object.")

        val currentDistance = leftRangeSensor.distanceDetected

        when {
            currentDistance > distance -> {
                opMode.telemetry.log().add("Current distance is greater than the target distance.")
                setStrafePower(-Math.abs(power))
                while (leftRangeSensor.distanceDetected > distance && !opMode.isStopRequested) {
                    opMode.sleep(10)
                }
            }

            currentDistance < distance -> {
                opMode.telemetry.log().add("Current distance is greater than the target distance.")
                setStrafePower(Math.abs(power))
                while (leftRangeSensor.distanceDetected < distance && !opMode.isStopRequested) {
                    opMode.sleep(10)
                }
            }

            else -> return
        }

        stopAllDriveMotors()

        if (shouldCorrect) {
            opMode.sleep(1000)

            val leftDistance = leftRangeSensor.distanceDetected
            if ((distance + DEFAULT_OBJECT_DISTANCE_TOLERANCE < leftDistance || distance - DEFAULT_OBJECT_DISTANCE_TOLERANCE > leftDistance)
                    && opMode.opModeIsActive()) {
                driveToDistanceFromLeftObject(distance, power)
            }

            stopAllDriveMotors()
        }
    }

    /**
     * Drives the robot left or right until the wall is a certain distance away from the right.
     * @param distance The distance the robot should be from a right object.
     * @param power The power to run the motors at when driving.
     */
    fun driveToDistanceFromRightObject(distance: Double, power: Double = 0.325, shouldCorrect: Boolean = true) {

        opMode.telemetry.log().add("Driving to distance from right object.")

        val currentDistance = rightRangeSensor.distanceDetected

        when {
            currentDistance > distance -> {
                setStrafePower(Math.abs(power))
                while (rightRangeSensor.distanceDetected > distance && !opMode.isStopRequested) {
                    opMode.sleep(10)
                }
            }

            currentDistance < distance -> {
                setStrafePower(-Math.abs(power))
                while (rightRangeSensor.distanceDetected < distance && !opMode.isStopRequested) {
                    opMode.sleep(10)
                }
            }

            else -> return
        }

        opMode.telemetry.update()

        stopAllDriveMotors()

        if (shouldCorrect) {
            opMode.sleep(1000)

            val rightDistance = rightRangeSensor.distanceDetected
            if ((distance + DEFAULT_OBJECT_DISTANCE_TOLERANCE < rightDistance || distance - DEFAULT_OBJECT_DISTANCE_TOLERANCE > rightDistance)
                    && !opMode.isStopRequested) {
                driveToDistanceFromRightObject(distance, 0.45)
            }

            stopAllDriveMotors()
        }
    }

    /**
     * Drives the robot off of the balancing stone, using the angle of the robot as an indication of completion.
     * @param power The power ot run the motors at when driving.
     */
    fun driveOffBalancingStone(power: Double = 0.175) {
        if (!LinearOpMode.isStopRequested) {
            setDrivePower(power)

            when {
                power > 0 -> {
                    opMode.telemetry.log().add("Driving until offset pitch.")
                    while (startingPitch - pitch < BALANCING_STONE_ANGLE_THRESHOLD && !opMode.isStopRequested) {
                        opMode.sleep(10)
                    }
                    opMode.telemetry.log().add("Detected a pitch of $pitch.")

                    this.raiseJewelStick(0)

                    opMode.telemetry.log().add("Driving until level pitch.")
                    while (startingPitch - pitch >= BALANCING_STONE_GROUND_ANGLE_THRESHOLD && !opMode.isStopRequested) {
                        opMode.sleep(10)
                    }
                    opMode.telemetry.log().add("Detected a pitch of $pitch.")
                }

                power < 0 -> {
                    opMode.telemetry.log().add("Driving until offset pitch.")
                    while (startingPitch - pitch > -BALANCING_STONE_ANGLE_THRESHOLD && !opMode.isStopRequested) {
                        opMode.sleep(10)
                    }
                    opMode.telemetry.log().add("Detected a pitch of $pitch.")

                    this.raiseJewelStick(0)

                    opMode.telemetry.log().add("Driving until level pitch.")
                    while (startingPitch - pitch <= -BALANCING_STONE_GROUND_ANGLE_THRESHOLD && !opMode.isStopRequested) {
                        opMode.sleep(10)
                    }
                    opMode.telemetry.log().add("Detected a pitch of $pitch.")
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
    fun driveOnBalancingStone(power: Double = 0.40) {
        if (!opMode.isStopRequested) {
            setDrivePower(power)

            while (abs(startingPitch - pitch) >= BALANCING_STONE_GROUND_ANGLE_THRESHOLD && !opMode.isStopRequested) {
                opMode.sleep(10)
            }
        }

        stopAllDriveMotors()
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
    */
}