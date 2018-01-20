package org.firstinspires.ftc.teamcode.game.opmodes

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark
import org.firstinspires.ftc.teamcode.game.CodaActions
import org.firstinspires.ftc.teamcode.game.components.Lift
import org.firstinspires.ftc.teamcode.game.robots.Coda
import org.firstinspires.ftc.teamcode.libraries.RobotOpMode

@Autonomous(name = "1: Blue Front", group = "Blue Manual Selection Autonomous")
class BlueFrontAutonomous : RobotOpMode() {

    companion object {
        val OPMODE_NAME = "1: Blue Front"
    }

    override val type = OpModeType.AUTONOMOUS

    @Throws(InterruptedException::class)
    override fun runOpMode() {

        val robot = Coda(this)
        val actions = CodaActions(this, robot)

        actions.setupCameras()
        actions.waitForStart()
        actions.elevateGlyph()
        actions.performCameraIdentificationActions()
        actions.waitForGlyphElevation()
        actions.displaceJewel(CodaActions.AllianceColor.BLUE)
        actions.setLiftPositionToFirstLevel()

        val wallDistance = when (actions.detectedPictograph) {
            RelicRecoveryVuMark.LEFT -> Coda.LEADING_FRONT_CRYPTO_BOX_DISTANCE
            RelicRecoveryVuMark.CENTER -> Coda.CENTER_FRONT_CRYPTO_BOX_DISTANCE
            RelicRecoveryVuMark.RIGHT -> Coda.TRAILING_FRONT_CRYPTO_BOX_DISTANCE
            RelicRecoveryVuMark.UNKNOWN -> Coda.LEADING_FRONT_CRYPTO_BOX_DISTANCE
        }

        actions.placeGlyph(Coda.ObjectDirection.LEFT, wallDistance)

        if (actions.detectedPictograph != RelicRecoveryVuMark.RIGHT) {
            actions.driveToTrailingCryptoBox()
        }

        robot.driveTrain.turnTo(-135.0, 0.20)
        actions.grabSecondGlyph(CodaActions.DistanceFromCenter.LONG)
        robot.driveTrain.turnTo(0.0, 0.35)
        actions.driveBackToCryptoBoxFromCenter()

        if (actions.detectedPictograph == RelicRecoveryVuMark.RIGHT) {
            robot.lift.setPosition(Lift.LiftPosition.SECOND_LEVEL)
        } else {
            robot.lift.drop()
        }

        actions.placeGlyph()

        robot.driveTrain.turnTo(-135.0, 0.35)
    }

}
