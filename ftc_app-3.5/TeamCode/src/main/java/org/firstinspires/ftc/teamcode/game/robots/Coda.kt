package org.firstinspires.ftc.teamcode.game.robots

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.game.components.*
import org.firstinspires.ftc.teamcode.lib.powercontroller.PowerController
import org.firstinspires.ftc.teamcode.lib.robot.Robot
import org.firstinspires.ftc.teamcode.lib.robot.sensor.RangeSensor
import kotlin.concurrent.thread
import kotlin.math.abs

class Coda(linearOpMode: LinearOpMode) : Robot(linearOpMode) {

    val driveTrain by lazy {
        components[DRIVE_TRAIN] as CodaDriveTrain
    }

    val lift by lazy {
        components[LIFT] as CodaLift
    }

    val glyphGrabber by lazy {
        components[GLYPH_GRABBER] as CodaGlyphGrabbers
    }

    val jewelDisplacementBar by lazy {
        components[JEWEL_STICK] as CodaJewelDisplacementBar
    }

    val balancingStoneHolder by lazy {
        components[BALANCING_STONE_HOLDER] as CodaBalancingStoneHolder
    }

    val relicGrabber by lazy {
        components[RELIC_GRABBER] as CodaRelicGrabber
    }

    private var startingPitch: Double = 0.0

    var isSetup = false
        private set

    /*
     * Adds the necessary components to the robot.
     */
    fun setup() {
        if (!isSetup) {
            addComponent(CodaDriveTrain(linearOpMode), DRIVE_TRAIN)
            addComponent(CodaGlyphGrabbers(linearOpMode), GLYPH_GRABBER)
            addComponent(CodaLift(linearOpMode), LIFT)
            addComponent(CodaRelicGrabber(linearOpMode), RELIC_GRABBER)
            addComponent(CodaJewelDisplacementBar(linearOpMode), JEWEL_STICK)
            addComponent(CodaBalancingStoneHolder(linearOpMode), BALANCING_STONE_HOLDER)

            addComponent(
                RangeSensor(linearOpMode, "front left range sensor"),
                FRONT_LEFT_RANGE_SENSOR
            )
            addComponent(
                RangeSensor(linearOpMode, "front right range sensor"),
                FRONT_RIGHT_RANGE_SENSOR
            )
            addComponent(
                RangeSensor(linearOpMode, "left range sensor"),
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

            glyphGrabber.setState(CodaGlyphGrabbers.GlyphGrabberState.OPEN)
            balancingStoneHolder.setState(CodaBalancingStoneHolder.BalancingStoneHolderState.UP)

            linearOpMode.onStart {
                driveTrain.startUpdatingDrivePowers()
                lift.resetEncoder()
                lift.startSettingMotorPowers()
            }
        }

        isSetup = true
    }

    enum class RangeSensorDirection {
        FRONT_LEFT,
        FRONT_RIGHT,
        LEFT,
        RIGHT,
        BACK
    }

    fun driveToDistanceFromObject(
        rangeSensorDirection: RangeSensorDirection,
        targetDistance: Double,
        controller: PowerController,
        shouldCorrect: Boolean = true
    ) {
        if (!linearOpMode.isStopRequested) {

            val rangeSensor: RangeSensor = when (rangeSensorDirection) {
                RangeSensorDirection.LEFT -> components[LEFT_RANGE_SENSOR] as RangeSensor
                RangeSensorDirection.RIGHT -> components[RIGHT_RANGE_SENSOR] as RangeSensor
                RangeSensorDirection.FRONT_LEFT -> components[FRONT_LEFT_RANGE_SENSOR] as RangeSensor
                RangeSensorDirection.FRONT_RIGHT -> components[FRONT_RIGHT_RANGE_SENSOR] as RangeSensor
                RangeSensorDirection.BACK -> TODO()
            }

            rangeSensor.startUpdatingDetectedDistance()
            controller.errorValueHandler = {
                targetDistance - rangeSensor.distanceDetected
            }

            val currentDistance = rangeSensor.distanceDetected

            when {
                targetDistance > currentDistance ->
                    while (
                        targetDistance > rangeSensor.distanceDetected &&
                        linearOpMode.opModeIsActive()
                    ) {
                        when (rangeSensorDirection) {
                            RangeSensorDirection.LEFT ->
                                driveTrain.strafeDriveAtPower(-abs(controller.outputPower))

                            RangeSensorDirection.RIGHT ->
                                driveTrain.strafeDriveAtPower(abs(controller.outputPower))

                            RangeSensorDirection.FRONT_LEFT ->
                                driveTrain.linearDriveAtPower(-controller.outputPower)

                            RangeSensorDirection.FRONT_RIGHT ->
                                driveTrain.linearDriveAtPower(-controller.outputPower)

                            RangeSensorDirection.BACK ->
                                driveTrain.linearDriveAtPower(controller.outputPower)
                        }
                    }


                targetDistance < currentDistance ->
                    while (targetDistance < rangeSensor.distanceDetected &&
                        linearOpMode.opModeIsActive()
                    ) {
                        when (rangeSensorDirection) {
                            RangeSensorDirection.LEFT ->
                                driveTrain.strafeDriveAtPower(abs(controller.outputPower))

                            RangeSensorDirection.RIGHT ->
                                driveTrain.strafeDriveAtPower(-abs(controller.outputPower))

                            RangeSensorDirection.FRONT_LEFT ->
                                driveTrain.linearDriveAtPower(controller.outputPower)

                            RangeSensorDirection.FRONT_RIGHT ->
                                driveTrain.linearDriveAtPower(controller.outputPower)

                            RangeSensorDirection.BACK ->
                                driveTrain.linearDriveAtPower(-controller.outputPower)
                        }
                    }
            }

            driveTrain.stop()

            if (shouldCorrect) {
                linearOpMode.sleep(500)
                val distance = rangeSensor.distanceDetected
                if (
                    distance > targetDistance + WALL_DISTANCE_TOLERANCE ||
                    distance < targetDistance - WALL_DISTANCE_TOLERANCE
                ) {
                    linearOpMode.telemetry.log().add("Correcting distance from object.")
                    driveToDistanceFromObject(
                        rangeSensorDirection,
                        targetDistance,
                        controller,
                        shouldCorrect
                    )
                }

            }

            controller.stopUpdatingOutput()
            rangeSensor.stopUpdatingDetectedDistance()

        }

    }

    /**
     * Drives the robot off of the balancing stone, using
     * the angle of the robot as an indication of completion.
     */
    fun driveOffBalancingStone(power: Double) {
        if (!linearOpMode.isStopRequested) {

            driveTrain.linearDriveAtPower(power)

            when {
                power > 0 -> {
                    while (startingPitch - driveTrain.currentPitch < BALANCING_STONE_ANGLE_THRESHOLD && !linearOpMode.isStopRequested) {
                        linearOpMode.sleep(10)
                    }

                    jewelDisplacementBar.setPosition(CodaJewelDisplacementBar.Position.UP)

                    while (startingPitch - driveTrain.currentPitch >= BALANCING_STONE_GROUND_ANGLE_THRESHOLD && !linearOpMode.isStopRequested) {
                        linearOpMode.sleep(10)
                    }
                }

                power < 0 -> {
                    while (startingPitch - driveTrain.currentPitch > -BALANCING_STONE_ANGLE_THRESHOLD && !linearOpMode.isStopRequested) {
                        linearOpMode.sleep(10)
                    }

                    jewelDisplacementBar.setPosition(CodaJewelDisplacementBar.Position.UP)

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
    fun driveOnBalancingStone(controller: PowerController) {
        if (!linearOpMode.isStopRequested) {

            controller.errorValueHandler = {
                driveTrain.currentPitch - startingPitch - BALANCING_STONE_GROUND_ANGLE_THRESHOLD
            }

            while (abs(startingPitch - driveTrain.currentPitch) >= BALANCING_STONE_GROUND_ANGLE_THRESHOLD && !linearOpMode.isStopRequested) {
                driveTrain.linearDriveAtPower(controller.outputPower)
                linearOpMode.sleep(10)
            }

            controller.stopUpdatingOutput()
            driveTrain.stop()
        }
    }

    companion object {
        private const val DRIVE_TRAIN = "drive_train"
        private const val GLYPH_GRABBER = "glyph_grabber"
        private const val LIFT = "lift"
        private const val RELIC_GRABBER = "relic_grabber"
        private const val JEWEL_STICK = "jewel_stick"
        private const val BALANCING_STONE_HOLDER = "balancing_stone_holder"
        const val FRONT_LEFT_RANGE_SENSOR = "front_left_range_sensor"
        const val FRONT_RIGHT_RANGE_SENSOR = "front_right_range_sensor"
        const val LEFT_RANGE_SENSOR = "left range sensor"
        const val RIGHT_RANGE_SENSOR = "right_range_sensor"
        const val BACK_RANGE_SENSOR = "back_range_sensor"

        private const val WALL_DISTANCE_TOLERANCE = 1.5
        private const val BALANCING_STONE_ANGLE_THRESHOLD = 6.0
        private const val BALANCING_STONE_GROUND_ANGLE_THRESHOLD = 3.0
    }

}

fun LinearOpMode.onStart(startHandler: () -> Unit) {
    thread(start = true) {
        waitForStart()
        startHandler()
    }
}