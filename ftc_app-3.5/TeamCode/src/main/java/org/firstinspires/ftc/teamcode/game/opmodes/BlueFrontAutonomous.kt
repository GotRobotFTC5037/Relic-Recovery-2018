package org.firstinspires.ftc.teamcode.game.opmodes

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark
import org.firstinspires.ftc.teamcode.game.CodaActions
import org.firstinspires.ftc.teamcode.game.RelicRecoveryConstants
import org.firstinspires.ftc.teamcode.game.components.Lift
import org.firstinspires.ftc.teamcode.game.robots.Coda

@Autonomous(name = "1: Blue Front", group = "Blue Manual Selection Autonomous")
class BlueFrontAutonomous : LinearOpMode() {

    companion object {
        val OPMODE_NAME = "1: Blue Front"
    }

    @Throws(InterruptedException::class)
    override fun runOpMode() {

        val robot = Coda(this)
        val actions = CodaActions(this, robot)

        robot.setup()
        actions.setupCameras()
        actions.waitForStart()
        actions.elevateGlyph()
        actions.performCameraIdentificationActions()
        actions.waitForGlyphElevation()
        actions.displaceJewel(CodaActions.AllianceColor.BLUE)
        actions.setLiftPositionToFirstLevel()

        val wallDistance = when (actions.detectedPictograph) {
            RelicRecoveryVuMark.LEFT -> RelicRecoveryConstants.LEADING_FRONT_CRYPTO_BOX_DISTANCE
            RelicRecoveryVuMark.CENTER -> RelicRecoveryConstants.CENTER_FRONT_CRYPTO_BOX_DISTANCE
            RelicRecoveryVuMark.RIGHT -> RelicRecoveryConstants.TRAILING_FRONT_CRYPTO_BOX_DISTANCE
            RelicRecoveryVuMark.UNKNOWN -> RelicRecoveryConstants.LEADING_FRONT_CRYPTO_BOX_DISTANCE
        }

        actions.placeGlyph(Coda.ObjectDirection.LEFT, wallDistance)

        if (actions.detectedPictograph != RelicRecoveryVuMark.RIGHT) {
            actions.driveToTrailingCryptoBox()
        }

        robot.driveTrain.turnToHeading(-135.0, 0.20)
        actions.grabSecondGlyph(CodaActions.DistanceFromCenter.LONG)
        robot.driveTrain.turnToHeading(0.0, 0.35)
        actions.driveBackToCryptoBoxFromCenter()

        if (actions.detectedPictograph == RelicRecoveryVuMark.RIGHT) {
            robot.lift.setPosition(Lift.LiftPosition.SECOND_LEVEL)
        } else {
            robot.lift.drop()
        }

        actions.placeGlyph()

        robot.driveTrain.turnToHeading(-135.0, 0.35)
    }

}
