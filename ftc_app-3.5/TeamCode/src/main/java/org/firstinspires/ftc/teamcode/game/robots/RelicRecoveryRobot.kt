package org.firstinspires.ftc.teamcode.game.robots

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.I2cAddr
import org.firstinspires.ftc.teamcode.game.components.GlyphGrabber
import org.firstinspires.ftc.teamcode.game.components.JewelStick
import org.firstinspires.ftc.teamcode.game.components.Lift
import org.firstinspires.ftc.teamcode.libraries.robot.Robot
import org.firstinspires.ftc.teamcode.libraries.robot.drivetrains.MecanumDriveTrain
import org.firstinspires.ftc.teamcode.libraries.robot.sensors.RangeSensor
import kotlin.math.abs

class RelicRecoveryRobot(linearOpMode: LinearOpMode): Robot(linearOpMode) {

    companion object {
        private val LIFT = "lift"
        private val GLYPH_GRABBER = "glyph_grabber"
        private val JEWEL_STICK = "jewel_stick"
        private val FRONT_LEFT_RANGE_SENSOR = "front_left_range_sensor"
        private val FRONT_RIGHT_RANGE_SENSOR = "front_right_range_sensor"
        private val LEFT_RANGE_SENSOR = "left range sensor"
        private val RIGHT_RANGE_SENSOR = "right_range_sensor"
        private val BACK_RANGE_SENSOR = "back_range_sensor"

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
    }

    private var startingPitch: Double = 0.0

    /*
     * Adds the necessary components to the robot.
     */
    fun setup() {
        this.addComponent(MecanumDriveTrain(linearOpMode))
        this.addComponent(Lift(linearOpMode), LIFT)
        this.addComponent(GlyphGrabber(linearOpMode), GLYPH_GRABBER)
        this.addComponent(JewelStick(linearOpMode), JEWEL_STICK)

        this.addComponent(RangeSensor(linearOpMode, "front left range sensor"), FRONT_LEFT_RANGE_SENSOR)
        this.addComponent(RangeSensor(linearOpMode, "front right range sensor"), FRONT_RIGHT_RANGE_SENSOR)
        this.addComponent(RangeSensor(linearOpMode, "left range sensor", I2cAddr.create8bit(0x32)), LEFT_RANGE_SENSOR)
        this.addComponent(RangeSensor(linearOpMode, "right range sensor"), RIGHT_RANGE_SENSOR)
        this.addComponent(RangeSensor(linearOpMode, "back range sensor"), BACK_RANGE_SENSOR)

        mecanumDriveTrain.waitForGyroCalibration()
        startingPitch = mecanumDriveTrain.pitch
    }

    val mecanumDriveTrain: MecanumDriveTrain
        get() = driveTrain as MecanumDriveTrain

    val lift: Lift
        get() = lifts[LIFT] as Lift

    val glyphGrabber: GlyphGrabber
        get() = attachments[GLYPH_GRABBER] as GlyphGrabber

    val jewelStick: JewelStick
        get() = attachments[JEWEL_STICK] as JewelStick

    enum class ObjectDirection {
        FRONT_LEFT,
        FRONT_RIGHT,
        LEFT,
        RIGHT,
        BACK
    }

    fun driveToDistanceFromObject(direction: ObjectDirection, targetDistance: Double, drivePower: Double, shouldCorrect: Boolean) {

        if (!linearOpMode.isStopRequested) {

            val rangeSensor = when (direction) {
                ObjectDirection.LEFT -> sensors[LEFT_RANGE_SENSOR] as RangeSensor
                ObjectDirection.RIGHT -> sensors[RIGHT_RANGE_SENSOR] as RangeSensor
                else -> return // For now, do nothing
            }

            val currentDistance = rangeSensor.distanceDetected
            when {
                targetDistance > currentDistance -> {
                    mecanumDriveTrain.setStrafePower(abs(drivePower))
                    while (targetDistance > rangeSensor.distanceDetected) {
                        linearOpMode.sleep(10)
                    }
                }

                currentDistance > targetDistance -> {
                    mecanumDriveTrain.setDrivePower(-abs(drivePower))
                    while (rangeSensor.distanceDetected > targetDistance) {
                        linearOpMode.sleep(10)
                    }
                }
            }

            mecanumDriveTrain.stopAllDriveMotors()

            if (shouldCorrect) {
                linearOpMode.sleep(1000)

                val distance = rangeSensor.distanceDetected
                if ((targetDistance + DEFAULT_OBJECT_DISTANCE_TOLERANCE < distance
                        || targetDistance - DEFAULT_OBJECT_DISTANCE_TOLERANCE > distance)
                        && linearOpMode.opModeIsActive()) {
                    driveToDistanceFromObject(direction, targetDistance, drivePower, shouldCorrect)
                }

                mecanumDriveTrain.stopAllDriveMotors()
            }

        }

    }

    /**
     * Drives the robot off of the balancing stone, using the angle of the robot as an indication of completion.
     * @param power The power ot run the motors at when driving.
     */
    fun driveOffBalancingStone(power: Double = 0.175) {
        if (!linearOpMode.isStopRequested) {

            mecanumDriveTrain.setDrivePower(power)

            when {
                power > 0 -> {
                    while (startingPitch - mecanumDriveTrain.pitch < BALANCING_STONE_ANGLE_THRESHOLD && !linearOpMode.isStopRequested) {
                        linearOpMode.sleep(10)
                    }

                    jewelStick.setPosition(JewelStick.Position.UP)

                    while (startingPitch - mecanumDriveTrain.pitch >= BALANCING_STONE_GROUND_ANGLE_THRESHOLD && !linearOpMode.isStopRequested) {
                        linearOpMode.sleep(10)
                    }
                }

                power < 0 -> {
                    while (startingPitch - mecanumDriveTrain.pitch > -BALANCING_STONE_ANGLE_THRESHOLD && !linearOpMode.isStopRequested) {
                        linearOpMode.sleep(10)
                    }

                    jewelStick.setPosition(JewelStick.Position.UP)

                    while (startingPitch - mecanumDriveTrain.pitch <= -BALANCING_STONE_GROUND_ANGLE_THRESHOLD && !linearOpMode.isStopRequested) {
                        linearOpMode.sleep(10)
                    }
                }

                else -> return
            }

            mecanumDriveTrain.stopAllDriveMotors()
        }
    }

    /**
     * Drives the robot on of the balancing stone, using the angle of the robot as an indication of completion.
     * @param power The power ot run the motors at when driving.
     */
    fun driveOnBalancingStone(power: Double = 0.40) {

        if (!linearOpMode.isStopRequested) {

            mecanumDriveTrain.setDrivePower(power)

            while (abs(startingPitch - mecanumDriveTrain.pitch) >= BALANCING_STONE_GROUND_ANGLE_THRESHOLD && !linearOpMode.isStopRequested) {
                linearOpMode.sleep(10)
            }

            mecanumDriveTrain.stopAllDriveMotors()
        }

    }

}