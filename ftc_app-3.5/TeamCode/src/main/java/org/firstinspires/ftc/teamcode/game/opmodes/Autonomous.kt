package org.firstinspires.ftc.teamcode.game.opmodes

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark
import org.firstinspires.ftc.teamcode.game.RelicRecoveryConstants
import org.firstinspires.ftc.teamcode.game.components.CodaLift
import org.firstinspires.ftc.teamcode.game.components.GlyphGrabber
import org.firstinspires.ftc.teamcode.game.components.JewelDisplacementBar
import org.firstinspires.ftc.teamcode.game.robots.Coda
import org.firstinspires.ftc.teamcode.game.vision.JewelConfigurationDetector
import org.firstinspires.ftc.teamcode.game.vision.PictographIdentifier
import kotlin.concurrent.thread

private object AutonomousActions {

    lateinit var robot: Coda
    lateinit var linearOpMode: LinearOpMode
    var detectedJewelConfiguration = JewelConfigurationDetector.JewelConfiguration.UNKNOWN
    var detectedPictograph = RelicRecoveryVuMark.UNKNOWN

    fun waitForStart() {
        linearOpMode.waitForStart()
    }

    fun getJewelConfigurationDetector() = JewelConfigurationDetector()
    fun getPictographIdentifier() = PictographIdentifier(linearOpMode.hardwareMap)

    fun performSharedActionGroup1() {
        robot = Coda(linearOpMode)
        robot.setup()
        val jewelConfigurationDetector = getJewelConfigurationDetector()
        val pictographIdentifier = getPictographIdentifier()

        waitForStart()

        val glyphGrabbingThread = thread(start = true) {
            robot.glyphGrabber.setState(GlyphGrabber.GlyphGrabberState.CLOSED)
            robot.lift.setPosition(CodaLift.LiftPosition.SECOND_LEVEL)
        }

        val elapsedTime = ElapsedTime(ElapsedTime.Resolution.MILLISECONDS)

        detectedJewelConfiguration =
                jewelConfigurationDetector.waitForJewelIdentification(elapsedTime, linearOpMode)
        jewelConfigurationDetector.disable()
        if (detectedJewelConfiguration != JewelConfigurationDetector.JewelConfiguration.UNKNOWN) {
            robot.jewelDisplacementBar.setPosition(JewelDisplacementBar.Position.DOWN)
        }

        pictographIdentifier.activate()
        detectedPictograph =
                pictographIdentifier.waitForPictographIdentification(elapsedTime, linearOpMode)
        pictographIdentifier.deactivate()
        glyphGrabbingThread.join()
    }

    enum class AllianceColor {
        RED, BLUE
    }

    fun performSharedActionGroup2(allianceColor: AllianceColor) {
        when (allianceColor) {
            AllianceColor.RED ->
                when (detectedJewelConfiguration) {
                    JewelConfigurationDetector.JewelConfiguration.RED_BLUE -> {
                        robot.driveTrain.linearTimeDrive(1000, 0.175)
                        robot.jewelDisplacementBar.setPosition(JewelDisplacementBar.Position.UP)
                        robot.driveOnBalancingStone(-0.40)
                        robot.driveOffBalancingStone(-0.175)
                    }

                    JewelConfigurationDetector.JewelConfiguration.BLUE_RED -> {
                        robot.driveOffBalancingStone(-0.175)
                        robot.jewelDisplacementBar.setPosition(JewelDisplacementBar.Position.UP)
                    }

                    JewelConfigurationDetector.JewelConfiguration.UNKNOWN -> {
                        robot.driveOffBalancingStone(-0.175)
                    }
                }

            AllianceColor.BLUE ->
                when (detectedJewelConfiguration) {
                    JewelConfigurationDetector.JewelConfiguration.RED_BLUE -> {
                        robot.driveTrain.linearTimeDrive(1000, -0.175)
                        robot.jewelDisplacementBar.setPosition(JewelDisplacementBar.Position.UP)
                        robot.driveOnBalancingStone(0.40)
                        robot.driveOffBalancingStone(0.175)
                    }

                    JewelConfigurationDetector.JewelConfiguration.BLUE_RED -> {
                        robot.driveOffBalancingStone(0.175)
                        robot.jewelDisplacementBar.setPosition(JewelDisplacementBar.Position.UP)
                    }

                    JewelConfigurationDetector.JewelConfiguration.UNKNOWN -> {
                        robot.driveOffBalancingStone(0.175)
                    }
                }
        }
    }

    enum class CryptoBoxPosition {
        FRONT, SIDE
    }

    fun performSharedActionGroup3(
        direction: Coda.ObjectDirection,
        cryptoBoxPosition: CryptoBoxPosition
    ) {

        val wallDistance = when (cryptoBoxPosition) {
            CryptoBoxPosition.FRONT ->
                when (detectedPictograph) {
                    RelicRecoveryVuMark.LEFT -> RelicRecoveryConstants.LEADING_FRONT_CRYPTO_BOX_DISTANCE
                    RelicRecoveryVuMark.CENTER -> RelicRecoveryConstants.CENTER_FRONT_CRYPTO_BOX_DISTANCE
                    RelicRecoveryVuMark.RIGHT -> RelicRecoveryConstants.TRAILING_FRONT_CRYPTO_BOX_DISTANCE
                    RelicRecoveryVuMark.UNKNOWN -> RelicRecoveryConstants.LEADING_FRONT_CRYPTO_BOX_DISTANCE
                }

            CryptoBoxPosition.SIDE ->
                when (detectedPictograph) {
                    RelicRecoveryVuMark.LEFT -> RelicRecoveryConstants.LEADING_SIDE_CRYPTO_BOX_DISTANCE
                    RelicRecoveryVuMark.CENTER -> RelicRecoveryConstants.CENTER_SIDE_CRYPTO_BOX_DISTANCE
                    RelicRecoveryVuMark.RIGHT -> RelicRecoveryConstants.TRAILING_SIDE_CRYPTO_BOX_DISTANCE
                    RelicRecoveryVuMark.UNKNOWN -> RelicRecoveryConstants.LEADING_SIDE_CRYPTO_BOX_DISTANCE
                }
        }

        robot.driveToDistanceFromObject(direction, wallDistance)
    }

    fun performSharedActionGroup4() {
        robot.lift.drop()
        robot.driveTrain.linearTimeDrive(1000, 0.25)
        robot.glyphGrabber.setState(GlyphGrabber.GlyphGrabberState.RELEASE)
        robot.driveTrain.linearTimeDrive(1250, -0.25)
        robot.glyphGrabber.setState(GlyphGrabber.GlyphGrabberState.SMALL_OPEN)
    }

}

@Autonomous(group = "Auto")
class BlueFront : LinearOpMode() {

    @Throws(InterruptedException::class)
    override fun runOpMode() {
        AutonomousActions.linearOpMode = this

        AutonomousActions.performSharedActionGroup1()
        AutonomousActions.performSharedActionGroup2(AutonomousActions.AllianceColor.BLUE)

        // All other autonomous opmodes have a turn here. It's unnecessary for ths one.

        AutonomousActions.performSharedActionGroup3(
            Coda.ObjectDirection.LEFT,
            AutonomousActions.CryptoBoxPosition.FRONT
        )

        AutonomousActions.performSharedActionGroup4()

        AutonomousActions.robot.driveTrain.turnToHeading(-135.0, 0.50)
    }


}

@Autonomous(group = "Auto")
class RedFront : LinearOpMode() {

    @Throws(InterruptedException::class)
    override fun runOpMode() {
        AutonomousActions.linearOpMode = this

        AutonomousActions.performSharedActionGroup1()
        AutonomousActions.performSharedActionGroup2(AutonomousActions.AllianceColor.BLUE)

        AutonomousActions.robot.driveTrain.turnToHeading(180.0, 0.75)

        AutonomousActions.performSharedActionGroup3(
            Coda.ObjectDirection.RIGHT,
            AutonomousActions.CryptoBoxPosition.FRONT
        )

        AutonomousActions.performSharedActionGroup4()

        AutonomousActions.robot.driveTrain.turnToHeading(-45.0, 0.50)
    }

}

@Autonomous(group = "Auto")
class BlueBack : LinearOpMode() {

    @Throws(InterruptedException::class)
    override fun runOpMode() {
        AutonomousActions.linearOpMode = this

        AutonomousActions.performSharedActionGroup1()
        AutonomousActions.performSharedActionGroup2(AutonomousActions.AllianceColor.RED)

        AutonomousActions.robot.driveTrain.turnToHeading(90.0, 0.75)

        AutonomousActions.performSharedActionGroup3(
            Coda.ObjectDirection.LEFT,
            AutonomousActions.CryptoBoxPosition.SIDE
        )

        AutonomousActions.performSharedActionGroup4()

        AutonomousActions.robot.driveTrain.turnToHeading(-90.0, 0.50)
    }

}

@Autonomous(group = "Auto")
class RedBack : LinearOpMode() {

    @Throws(InterruptedException::class)
    override fun runOpMode() {
        AutonomousActions.linearOpMode = this

        AutonomousActions.performSharedActionGroup1()
        AutonomousActions.performSharedActionGroup2(AutonomousActions.AllianceColor.RED)

        AutonomousActions.robot.driveTrain.turnToHeading(90.0, 0.75)

        AutonomousActions.performSharedActionGroup3(
            Coda.ObjectDirection.RIGHT,
            AutonomousActions.CryptoBoxPosition.SIDE
        )

        AutonomousActions.performSharedActionGroup4()

        AutonomousActions.robot.driveTrain.turnToHeading(-90.0, 0.50)
    }

}