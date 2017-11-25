package org.firstinspires.ftc.teamcode.opmodes.competition

import RelicRecoveryRobotOpModeManager
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark
import org.firstinspires.ftc.teamcode.libraries.PictographIdentifier
import org.firstinspires.ftc.teamcode.robots.RelicRecoveryRobot
import java.util.*

@Autonomous(name = "Red Back Autonomous", group = "Manual Selection Autonomous")
class RedBackAutonomous : LinearOpMode() {

    companion object {
        val OPMODE_NAME = "Red Back Autonomous"

        private val FRONT_WALL_DISTANCE = 45.0
        private val RIGHT_CRYPTO_BOX_DISTANCE = 98.0
        private val CENTER_CRYPTO_BOX_DISTANCE = 116.0
        private val LEFT_CRYPTO_BOX_DISTANCE = 134.0
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
        robot.turn(0.50, -90.0)
        robot.driveToDistanceFromForwardObject(RedBackAutonomous.FRONT_WALL_DISTANCE); sleep(1000)

        val rightWallDistance = when(pictograph) {
            RelicRecoveryVuMark.LEFT -> RedBackAutonomous.LEFT_CRYPTO_BOX_DISTANCE
            RelicRecoveryVuMark.CENTER -> RedBackAutonomous.CENTER_CRYPTO_BOX_DISTANCE
            RelicRecoveryVuMark.RIGHT -> RedBackAutonomous.RIGHT_CRYPTO_BOX_DISTANCE

        // Pick a random crypto box if the pictograph is unknown.
            RelicRecoveryVuMark.UNKNOWN ->
                listOf(RedBackAutonomous.LEFT_CRYPTO_BOX_DISTANCE,
                        RedBackAutonomous.CENTER_CRYPTO_BOX_DISTANCE,
                        RedBackAutonomous.RIGHT_CRYPTO_BOX_DISTANCE)[Random().nextInt(2)]
        }

        robot.driveToDistanceFromRightObject(rightWallDistance)

        robot.dropLift()
        robot.openGlyphGrabbers(); sleep(500)

        robot.timeDrive(1000)
        robot.timeDrive(350, -0.15)
    }

}