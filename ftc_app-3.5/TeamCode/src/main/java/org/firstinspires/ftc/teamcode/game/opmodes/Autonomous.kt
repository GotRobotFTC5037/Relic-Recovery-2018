package org.firstinspires.ftc.teamcode.game.opmodes

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.game.components.CodaLift
import org.firstinspires.ftc.teamcode.game.components.GlyphGrabber
import org.firstinspires.ftc.teamcode.game.components.JewelDisplacementBar
import org.firstinspires.ftc.teamcode.game.robots.Coda
import org.firstinspires.ftc.teamcode.game.vision.JewelConfigurationDetector
import org.firstinspires.ftc.teamcode.game.vision.PictographIdentifier
import kotlin.concurrent.thread

private object AutonomousActions {

    lateinit var linearOpMode: LinearOpMode

    fun getRobot(): Coda = Coda(linearOpMode)

    fun waitForStart() {
        linearOpMode.waitForStart()
    }

}

@Autonomous
class BlueFront : LinearOpMode() {

    @Throws(InterruptedException::class)
    override fun runOpMode() {
        AutonomousActions.linearOpMode = this
        val robot = AutonomousActions.getRobot()
        robot.setup()
        val jewelConfigurationDetector = JewelConfigurationDetector()
        val pictographIdentifier = PictographIdentifier(hardwareMap)
        waitForStart()

        val glyphGrabbingThread = thread(true) {
            robot.glyphGrabber.setState(GlyphGrabber.GlyphGrabberState.CLOSED)
            thread(true) {
                robot.lift.setPosition(CodaLift.LiftPosition.SECOND_LEVEL)
            }
        }

        val elapsedTime = ElapsedTime(ElapsedTime.Resolution.MILLISECONDS)
        val jewelPosition = jewelConfigurationDetector.waitForJewelIdentification(elapsedTime, this)
        jewelConfigurationDetector.disable()

        if (jewelPosition != JewelConfigurationDetector.JewelConfiguration.UNKNOWN) {
            robot.jewelDisplacementBar.setPosition(JewelDisplacementBar.Position.DOWN)
        }

        pictographIdentifier.activate()
        val pictograph = pictographIdentifier.waitForPictographIdentification(elapsedTime, this)
        pictographIdentifier.deactivate()
        glyphGrabbingThread.join()

        /*

        // Knock off the correct jewel.
        when (jewelPosition) {
            JewelConfigurationDetector.JewelConfiguration.RED_BLUE -> {
                robot.linearTimeDrive(1000, 0.175)
                robot.raiseJewelStick()
                robot.driveOnBalancingStone(-0.40)
                robot.driveOffBalancingStone(-0.175)
                robot.powerBreak(false)
            }

            JewelConfigurationDetector.JewelConfiguration.BLUE_RED -> {
                robot.driveOffBalancingStone(-0.175)
                robot.powerBreak(false)
                robot.raiseJewelStick()
            }

            JewelConfigurationDetector.JewelConfiguration.UNKNOWN -> {
                robot.driveOffBalancingStone(-0.175)
                robot.powerBreak(false)
            }
        }

        thread(true) {
            robot.lift.setPosition(Lift.LiftPosition.FIRST_LEVEL, 0.3)
        }

        // Determine the distance from the wall to the correct crypto box.
        val rightWallDistance = when (pictograph) {
            RelicRecoveryVuMark.LEFT -> Coda.TRAILING_SIDE_CRYPTO_BOX_DISTANCE
            RelicRecoveryVuMark.CENTER -> Coda.CENTER_SIDE_CRYPTO_BOX_DISTANCE
            RelicRecoveryVuMark.RIGHT -> Coda.LEADING_SIDE_CRYPTO_BOX_DISTANCE
            RelicRecoveryVuMark.UNKNOWN -> Coda.LEADING_SIDE_CRYPTO_BOX_DISTANCE
        }

        // Drive to the correct crypto box.
        robot.turnAtPower(0.3, 90.0)
        robot.driveToDistanceFromRightObject(rightWallDistance)

        // Place the glyph in the correct crypto box.
        robot.lift.drop()
        robot.linearTimeDrive(1000)
        robot.extendGlyphDeployer()
        robot.openGlyphGrabbers(250)

        // Back away from the crypto box.
        robot.linearTimeDrive(1250, -0.225)
        robot.liftGlyphDeployer(500)

        // Open the glyph grabbers.
        robot.releaseGlyphGrabbers()
        robot.retractGlyphDeployer()

        robot.turnAtPower(0.20, -90.0)

        // Drive to the glyphs in the center and grab one.
        sleep(1000)
        robot.linearTimeDrive(475, 1.0)
        robot.closeGlyphGrabbers(1250)

        // Drive back the crypto boxes.
        robot.linearTimeDrive(650, -0.25)
        thread(true) { robot.lift.setPosition(Lift.LiftPosition.FIRST_LEVEL) }
        robot.turnAtPower(0.35, 90.0)
        robot.linearTimeDrive(500)
        robot.driveToDistanceFromForwardObject(Coda.CRYPTO_BOX_SPACING)

        // Line up with the center crypto box.
        robot.driveToDistanceFromRightObject(Coda.CENTER_SIDE_CRYPTO_BOX_DISTANCE)

        // Place the glyph in the correct crypto box.
        if (pictograph == RelicRecoveryVuMark.CENTER) {
            robot.lift.setPosition(Lift.LiftPosition.SECOND_LEVEL)
        } else {
            robot.lift.drop()
        }

        robot.linearTimeDrive(1000)
        robot.extendGlyphDeployer()
        robot.openGlyphGrabbers(250)

        // Back away from the crypto box.
        robot.linearTimeDrive(450, -0.50)
        robot.liftGlyphDeployer(500)

        // Turn towards the center glyphs.
        robot.turnAtPower(0.35, -90.0)
        */
    }

}

@Autonomous
@Disabled
class RedFront : LinearOpMode() {

    @Throws(InterruptedException::class)
    override fun runOpMode() {
        AutonomousActions.linearOpMode = this
        val robot = AutonomousActions.getRobot()
        robot.setup()
        AutonomousActions.waitForStart()
    }

}

@Autonomous
@Disabled
class BlueBack : LinearOpMode() {

    @Throws(InterruptedException::class)
    override fun runOpMode() {
        AutonomousActions.linearOpMode = this
        val robot = AutonomousActions.getRobot()
        robot.setup()
        AutonomousActions.waitForStart()
    }

}

@Autonomous
@Disabled
class RedBack : LinearOpMode() {

    @Throws(InterruptedException::class)
    override fun runOpMode() {
        AutonomousActions.linearOpMode = this
        val robot = AutonomousActions.getRobot()
        robot.setup()
        AutonomousActions.waitForStart()
    }

}