package org.firstinspires.ftc.teamcode.opmodes.competition

import RelicRecoveryRobotOpModeManager
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark
import org.firstinspires.ftc.teamcode.libraries.PictographIdentifier
import org.firstinspires.ftc.teamcode.robots.RelicRecoveryRobot
import java.util.*

@Autonomous(name = "Blue Back Autonomous", group = "Manual Selection Autonomous")
class BlueBackAutonomous : LinearOpMode() {

    companion object {
        val OPMODE_NAME = "Blue Back Autonomous"

        private val FRONT_WALL_DISTANCE = 45.0
        private val LEFT_CRYPTO_BOX_DISTANCE = 98.0
        private val CENTER_CRYPTO_BOX_DISTANCE = 116.0
        private val RIGHT_CRYPTO_BOX_DISTANCE = 134.0
    }

    @Throws(InterruptedException::class)
    override fun runOpMode() {
        val robot = RelicRecoveryRobot()
        robot.linearOpMode = this
        robot.setup(hardwareMap)

        val pictographIdentifier = PictographIdentifier()
        pictographIdentifier.activate()

        RelicRecoveryRobotOpModeManager.queueOpMode(this, RelicRecoveryTeleOp.OPMODE_NAME)

        robot.waitForGyroCalibration()
        robot.waitForStart()
        robot.start()

        val pictograph = pictographIdentifier.getIdentifiedPictograph()
        pictographIdentifier.deactivate()

        robot.closeGlyphGrabbers(); sleep(500)
        robot.setLiftPosition(RelicRecoveryRobot.LIFT_FIRST_LEVEL)

        robot.driveOffBalancingStone(); sleep(500)
        robot.turn(0.50, 90.0)
        robot.driveToDistanceFromForwardObject(FRONT_WALL_DISTANCE); sleep(1000)

        val leftWallDistance = when(pictograph) {
            RelicRecoveryVuMark.LEFT -> LEFT_CRYPTO_BOX_DISTANCE
            RelicRecoveryVuMark.CENTER -> CENTER_CRYPTO_BOX_DISTANCE
            RelicRecoveryVuMark.RIGHT -> RIGHT_CRYPTO_BOX_DISTANCE

            // Pick a random crypto box if the pictograph is unknown.
            RelicRecoveryVuMark.UNKNOWN ->
                listOf(LEFT_CRYPTO_BOX_DISTANCE,
                        CENTER_CRYPTO_BOX_DISTANCE,
                        RIGHT_CRYPTO_BOX_DISTANCE)[Random().nextInt(2)]
        }

        robot.driveToDistanceFromLeftObject(leftWallDistance)

        robot.timeDrive(1000)
        robot.dropLift()
        robot.extendGlyphDeployer()
        robot.openGlyphGrabbers(); sleep(500)
        robot.timeDrive(750, -0.15)

        robot.timeDrive(500)
        robot.timeDrive(250, -0.15)
    }

}