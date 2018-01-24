package org.firstinspires.ftc.teamcode.game.opmodes

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark
import org.firstinspires.ftc.teamcode.game.components.CodaLift
import org.firstinspires.ftc.teamcode.game.components.GlyphGrabber
import org.firstinspires.ftc.teamcode.game.components.JewelDisplacementBar
import org.firstinspires.ftc.teamcode.game.robots.Coda
import org.firstinspires.ftc.teamcode.game.vision.JewelConfigurationDetector
import org.firstinspires.ftc.teamcode.game.vision.PictographIdentifier
import kotlin.concurrent.thread

private object AutonomousActions {

    lateinit var linearOpMode: LinearOpMode
    var detectedJewelConfiguration = JewelConfigurationDetector.JewelConfiguration.UNKNOWN
    var detectedPictograph = RelicRecoveryVuMark.UNKNOWN

    fun getRobot() = Coda(linearOpMode)

    fun waitForStart() {
        linearOpMode.waitForStart()
    }

    fun getJewelConfigurationDetector() = JewelConfigurationDetector()
    fun getPictographIdentifier() = PictographIdentifier(linearOpMode.hardwareMap)

    fun performSharedActionGroup1() {
        val robot = AutonomousActions.getRobot()
        robot.setup()
        val jewelConfigurationDetector = getJewelConfigurationDetector()
        val pictographIdentifier = getPictographIdentifier()

        waitForStart()

        val glyphGrabbingThread = thread(start = true) {
            robot.glyphGrabber.setState(GlyphGrabber.GlyphGrabberState.CLOSED)
            thread(start = true) {
                robot.lift.setPosition(CodaLift.LiftPosition.SECOND_LEVEL)
            }
        }

        val elapsedTime = ElapsedTime(ElapsedTime.Resolution.MILLISECONDS)

        detectedJewelConfiguration = jewelConfigurationDetector.waitForJewelIdentification(elapsedTime, linearOpMode)
        jewelConfigurationDetector.disable()
        if (detectedJewelConfiguration != JewelConfigurationDetector.JewelConfiguration.UNKNOWN) {
            robot.jewelDisplacementBar.setPosition(JewelDisplacementBar.Position.DOWN)
        }

        pictographIdentifier.activate()
        detectedPictograph = pictographIdentifier.waitForPictographIdentification(elapsedTime, linearOpMode)
        pictographIdentifier.deactivate()
        glyphGrabbingThread.join()
    }

}

@Autonomous
class BlueFront : LinearOpMode() {

    @Throws(InterruptedException::class)
    override fun runOpMode() {
        AutonomousActions.linearOpMode = this
        AutonomousActions.performSharedActionGroup1()
    }

}

@Autonomous
class RedFront : LinearOpMode() {

    @Throws(InterruptedException::class)
    override fun runOpMode() {
        AutonomousActions.linearOpMode = this
        AutonomousActions.performSharedActionGroup1()
    }

}

@Autonomous
class BlueBack : LinearOpMode() {

    @Throws(InterruptedException::class)
    override fun runOpMode() {
        AutonomousActions.linearOpMode = this
        AutonomousActions.performSharedActionGroup1()
    }

}

@Autonomous
class RedBack : LinearOpMode() {

    @Throws(InterruptedException::class)
    override fun runOpMode() {
        AutonomousActions.linearOpMode = this
        AutonomousActions.performSharedActionGroup1()
    }

}