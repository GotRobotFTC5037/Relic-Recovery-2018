package org.firstinspires.ftc.teamcode.game.robots

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.I2cAddr
import org.firstinspires.ftc.teamcode.game.components.DriveTrain
import org.firstinspires.ftc.teamcode.game.components.GlyphGrabber
import org.firstinspires.ftc.teamcode.game.components.JewelDisplacmentBar
import org.firstinspires.ftc.teamcode.game.components.Lift
import org.firstinspires.ftc.teamcode.libraries.robot.Robot
import org.firstinspires.ftc.teamcode.libraries.robot.sensor.RangeSensor
import kotlin.concurrent.thread
import kotlin.math.abs

class Coda(linearOpMode: LinearOpMode) : Robot(linearOpMode) {

    val driveTrain: DriveTrain by lazy { components[DRIVE_TRAIN] as DriveTrain }
    val lift: Lift by lazy { components[LIFT] as Lift }
    val glyphGrabber: GlyphGrabber by lazy { components[GLYPH_GRABBER] as GlyphGrabber }
    val jewelDisplacementBar: JewelDisplacmentBar by lazy { components[JEWEL_STICK] as JewelDisplacmentBar }

    private var startingPitch: Double = 0.0

    /*
     * Adds the necessary components to the robot.
     */
    fun setup() {
        addComponent(DriveTrain(linearOpMode), DRIVE_TRAIN)
        addComponent(Lift(linearOpMode), LIFT)
        addComponent(GlyphGrabber(linearOpMode), GLYPH_GRABBER)
        addComponent(JewelDisplacmentBar(linearOpMode), JEWEL_STICK)

        addComponent(
            RangeSensor(linearOpMode, "front left range sensor"),
            FRONT_LEFT_RANGE_SENSOR
        )
        addComponent(
            RangeSensor(linearOpMode, "front right range sensor"),
            FRONT_RIGHT_RANGE_SENSOR
        )
        addComponent(
            RangeSensor(linearOpMode, "left range sensor", I2cAddr.create8bit(0x32)),
            LEFT_RANGE_SENSOR
        )
        addComponent(
            RangeSensor(linearOpMode, "right range sensor"),
            RIGHT_RANGE_SENSOR
        )
        addComponent(
            RangeSensor(linearOpMode, "back range sensor"),
            BACK_RANGE_SENSOR
        )

        driveTrain.waitForGyroCalibration()
        startingPitch = driveTrain.currentPitch

        linearOpMode.onStart {
            driveTrain.startUpdatingDrivePowers()
        }
    }

    enum class ObjectDirection {
        FRONT_LEFT,
        FRONT_RIGHT,
        LEFT,
        RIGHT,
        BACK
    }

    fun driveToDistanceFromObject(
        direction: ObjectDirection,
        targetDistance: Double,
        drivePower: Double = 0.325,
        shouldCorrect: Boolean = true
    ) {

        if (!linearOpMode.isStopRequested) {

            val rangeSensor = when (direction) {
                ObjectDirection.LEFT -> components[LEFT_RANGE_SENSOR] as RangeSensor
                ObjectDirection.RIGHT -> components[RIGHT_RANGE_SENSOR] as RangeSensor
                else -> return // For now, do nothing
            }

            val currentDistance = rangeSensor.distanceDetected
            when {
                targetDistance > currentDistance -> {
                    driveTrain.strafeDrive(abs(drivePower))
                    while (targetDistance > rangeSensor.distanceDetected) {
                        linearOpMode.sleep(10)
                    }
                }

                currentDistance > targetDistance -> {
                    driveTrain.linearDrive(-abs(drivePower))
                    while (rangeSensor.distanceDetected > targetDistance) {
                        linearOpMode.sleep(10)
                    }
                }
            }

            driveTrain.stop()

            if (shouldCorrect) {
                linearOpMode.sleep(1000)

                val distance = rangeSensor.distanceDetected
                if ((targetDistance + WALL_DISTANCE_TOLERANCE < distance
                            || targetDistance - WALL_DISTANCE_TOLERANCE > distance)
                    && linearOpMode.opModeIsActive()) {
                    driveToDistanceFromObject(direction, targetDistance, drivePower, shouldCorrect)
                }

                driveTrain.stop()
            }

        }

    }

    /**
     * Drives the robot off of the balancing stone, using
     * the angle of the robot as an indication of completion.
     */
    fun driveOffBalancingStone(power: Double) {
        if (!linearOpMode.isStopRequested) {

            driveTrain.linearDrive(power)

            when {
                power > 0 -> {
                    while (startingPitch - driveTrain.currentPitch < BALANCING_STONE_ANGLE_THRESHOLD && !linearOpMode.isStopRequested) {
                        linearOpMode.sleep(10)
                    }

                    jewelDisplacementBar.setPosition(JewelDisplacmentBar.Position.UP)

                    while (startingPitch - driveTrain.currentPitch >= BALANCING_STONE_GROUND_ANGLE_THRESHOLD && !linearOpMode.isStopRequested) {
                        linearOpMode.sleep(10)
                    }
                }

                power < 0 -> {
                    while (startingPitch - driveTrain.currentPitch > -BALANCING_STONE_ANGLE_THRESHOLD && !linearOpMode.isStopRequested) {
                        linearOpMode.sleep(10)
                    }

                    jewelDisplacementBar.setPosition(JewelDisplacmentBar.Position.UP)

                    while (startingPitch - driveTrain.currentPitch <= -BALANCING_STONE_GROUND_ANGLE_THRESHOLD && !linearOpMode.isStopRequested) {
                        linearOpMode.sleep(10)
                    }
                }

                else -> return
            }

            driveTrain.stop()
        }
    }

    /**
     * Drives the robot on of the balancing stone, using
     * the angle of the robot as an indication of completion.
     */
    fun driveOnBalancingStone(power: Double) {

        if (!linearOpMode.isStopRequested) {

            driveTrain.linearDrive(power)

            while (abs(startingPitch - driveTrain.currentPitch) >= BALANCING_STONE_GROUND_ANGLE_THRESHOLD && !linearOpMode.isStopRequested) {
                linearOpMode.sleep(10)
            }

            driveTrain.stop()
        }

    }

    /**
     * Calls the handler once the opmode starts.
     */
    private fun LinearOpMode.onStart(startHandler: () -> Unit) {
        thread(start = true) {
            waitForStart()
            startHandler()
        }
    }

    companion object {
        private const val DRIVE_TRAIN = "drive_train"
        private const val LIFT = "lift"
        private const val GLYPH_GRABBER = "glyph_grabber"
        private const val JEWEL_STICK = "jewel_stick"
        private const val FRONT_LEFT_RANGE_SENSOR = "front_left_range_sensor"
        private const val FRONT_RIGHT_RANGE_SENSOR = "front_right_range_sensor"
        private const val LEFT_RANGE_SENSOR = "left range sensor"
        private const val RIGHT_RANGE_SENSOR = "right_range_sensor"
        private const val BACK_RANGE_SENSOR = "back_range_sensor"

        private const val WALL_DISTANCE_TOLERANCE = 3.0
        private const val BALANCING_STONE_ANGLE_THRESHOLD = 6.0
        private const val BALANCING_STONE_GROUND_ANGLE_THRESHOLD = 4.5
    }

}