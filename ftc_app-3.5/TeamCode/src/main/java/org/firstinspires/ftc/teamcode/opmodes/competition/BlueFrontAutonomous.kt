package org.firstinspires.ftc.teamcode.opmodes.competition

import RelicRecoveryRobotOpModeManager
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.corningrobotics.enderbots.endercv.CameraViewDisplay
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark
import org.firstinspires.ftc.teamcode.libraries.PictographIdentifier
import org.firstinspires.ftc.teamcode.libraries.vision.JewelPipeline
import org.firstinspires.ftc.teamcode.robots.RelicRecoveryRobot
import java.util.*

@Autonomous(name = "Blue Front Autonomous", group = "Manual Selection Autonomous")
class BlueFrontAutonomous : LinearOpMode() {

    companion object {
        val OPMODE_NAME = "Blue Front Autonomous"

        private val FRONT_WALL_DISTANCE = 45.0
        private val LEFT_CRYPTO_BOX_DISTANCE = 46.0
        private val CENTER_CRYPTO_BOX_DISTANCE = 64.0
        private val RIGHT_CRYPTO_BOX_DISTANCE = 82.0
    }

    @Throws(InterruptedException::class)
    override fun runOpMode() {
        val robot = RelicRecoveryRobot()
        robot.linearOpMode = this
        robot.setup(hardwareMap)

        RelicRecoveryRobotOpModeManager.queueOpMode(this, RelicRecoveryTeleOp.OPMODE_NAME)

        val jewelDetector = JewelPipeline()
        jewelDetector.init(hardwareMap.appContext, CameraViewDisplay.getInstance())
        jewelDetector.enable()

        val pictographIdentifier = PictographIdentifier(hardwareMap)

        robot.waitForGyroCalibration()
        robot.waitForStart()
        robot.start()

        val jewelPosition = jewelDetector.jewelPositions
        jewelDetector.disable()

        pictographIdentifier.activate()
        sleep(2000)
        val pictograph = pictographIdentifier.getIdentifiedPictograph()
        pictographIdentifier.deactivate()

        robot.closeGlyphGrabbers(); sleep(500)
        robot.setLiftPosition(RelicRecoveryRobot.AUTO_LIFT_FIRST_LEVEL)

        when(jewelPosition) {
            JewelPipeline.JewelPositions.RED_BLUE -> {
                robot.lowerJewelStick()
                robot.timeDrive(500, -0.25)
                robot.raiseJewelStick()
                robot.timeDrive(500, 0.50)
                sleep(1000)
            }

            JewelPipeline.JewelPositions.BLUE_RED -> {
                robot.lowerJewelStick()
                robot.timeDrive(500, 0.25)
                robot.raiseJewelStick()
                sleep(1000)
            }

            JewelPipeline.JewelPositions.UNKNOWN -> {
                // Do nothing
            }

            else -> {
                // Shouldn't happen
            }
        }

        robot.driveOffBalancingStone()
        robot.driveToDistanceFromForwardObject(FRONT_WALL_DISTANCE); sleep(1000)

        var leftWallDistance = 0.0
        when(pictograph) {
            RelicRecoveryVuMark.LEFT -> {
                leftWallDistance = LEFT_CRYPTO_BOX_DISTANCE
                telemetry.addLine("Left Crypto Box")
            }

            RelicRecoveryVuMark.CENTER -> {
                leftWallDistance = CENTER_CRYPTO_BOX_DISTANCE
                telemetry.addLine("Center Crypto Box")
            }

            RelicRecoveryVuMark.RIGHT -> { leftWallDistance = RIGHT_CRYPTO_BOX_DISTANCE
                telemetry.addLine("Right Crypto Box")
            }

            // Pick a random crypto box if the pictograph is unknown.
            RelicRecoveryVuMark.UNKNOWN -> {
                leftWallDistance = listOf(LEFT_CRYPTO_BOX_DISTANCE, CENTER_CRYPTO_BOX_DISTANCE,
                        RIGHT_CRYPTO_BOX_DISTANCE)[Random().nextInt(2)]
                telemetry.addLine("Unknown Crypto Box")
            }
        }

        telemetry.update()

        robot.driveToDistanceFromLeftObject(leftWallDistance)

        robot.timeDrive(1000)
        robot.dropLift()
        robot.extendGlyphDeployer()
        robot.openGlyphGrabbers(); sleep(500)
        robot.timeDrive(850, -0.15)
        sleep(1000)
        robot.liftGlyphDeployer()
        sleep(500)
    }

}
