package org.firstinspires.ftc.teamcode.opmodes.competition

import RelicRecoveryRobotOpModeManager
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark
import org.firstinspires.ftc.teamcode.libraries.PictographIdentifier
import org.firstinspires.ftc.teamcode.robots.RelicRecoveryRobot
import java.util.*

@Autonomous(name = "Red Front Autonomous", group = "Manual Selection Autonomous")
class RedFrontAutonomous : LinearOpMode() {

    companion object {
        val OPMODE_NAME = "Red Front Autonomous"

        private val FRONT_WALL_DISTANCE = 45.0
        private val RIGHT_CRYPTO_BOX_DISTANCE = 46.0
        private val CENTER_CRYPTO_BOX_DISTANCE = 64.0
        private val LEFT_CRYPTO_BOX_DISTANCE = 82.0
    }

    @Throws(InterruptedException::class)
    override fun runOpMode() {
        val robot = RelicRecoveryRobot()
        robot.linearOpMode = this
        robot.setup(hardwareMap)

        val pictographIdentifier = PictographIdentifier(hardwareMap)
        pictographIdentifier.activate()

        RelicRecoveryRobotOpModeManager.queueOpMode(this, RelicRecoveryTeleOp.OPMODE_NAME)

        robot.waitForGyroCalibration()
        robot.waitForStart()
        robot.start()

        val pictograph = pictographIdentifier.getIdentifiedPictograph()
        pictographIdentifier.deactivate()

        robot.closeGlyphGrabbers(); sleep(500)
        robot.setLiftPosition(RelicRecoveryRobot.AUTO_LIFT_FIRST_LEVEL)

        robot.driveOffBalancingStone()
        robot.driveToDistanceFromForwardObject(RedFrontAutonomous.FRONT_WALL_DISTANCE); sleep(1000)

        val rightWallDistance = when(pictograph) {
            RelicRecoveryVuMark.LEFT -> RedFrontAutonomous.LEFT_CRYPTO_BOX_DISTANCE
            RelicRecoveryVuMark.CENTER -> RedFrontAutonomous.CENTER_CRYPTO_BOX_DISTANCE
            RelicRecoveryVuMark.RIGHT -> RedFrontAutonomous.RIGHT_CRYPTO_BOX_DISTANCE

        // Pick a random crypto box if the pictograph is unknown.
            RelicRecoveryVuMark.UNKNOWN ->
                listOf(RedFrontAutonomous.LEFT_CRYPTO_BOX_DISTANCE,
                        RedFrontAutonomous.CENTER_CRYPTO_BOX_DISTANCE,
                        RedFrontAutonomous.RIGHT_CRYPTO_BOX_DISTANCE)[Random().nextInt(2)]
        }

        robot.driveToDistanceFromRightObject(rightWallDistance)

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