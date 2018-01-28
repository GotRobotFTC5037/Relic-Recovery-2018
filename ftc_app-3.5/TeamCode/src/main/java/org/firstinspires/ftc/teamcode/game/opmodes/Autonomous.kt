package org.firstinspires.ftc.teamcode.game.opmodes

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.util.ElapsedTime
import org.corningrobotics.enderbots.endercv.CameraViewDisplay
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark
import org.firstinspires.ftc.teamcode.game.RelicRecoveryConstants
import org.firstinspires.ftc.teamcode.game.components.CodaGlyphGrabber
import org.firstinspires.ftc.teamcode.game.components.CodaJewelDisplacementBar
import org.firstinspires.ftc.teamcode.game.components.CodaLift
import org.firstinspires.ftc.teamcode.game.robots.Coda
import org.firstinspires.ftc.teamcode.game.vision.JewelConfigurationDetector
import org.firstinspires.ftc.teamcode.game.vision.PictographIdentifier
import kotlin.concurrent.thread

private object AutonomousActions {

    const val DEFAULT_TURN_SPEED = 0.75

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

        // Setup the robot.
        robot = Coda(linearOpMode)
        robot.setup()

        // Setup the camera.
        val jewelConfigurationDetector = getJewelConfigurationDetector()
        jewelConfigurationDetector.init(
            linearOpMode.hardwareMap.appContext, CameraViewDisplay.getInstance()
        )
        jewelConfigurationDetector.enable()

        val pictographIdentifier = getPictographIdentifier()

        // Wait for start.
        waitForStart()
        robot.lift.startSettingMotorPowers()

        // Grab the glyph.
        val glyphGrabbingThread = thread(start = true) {
            robot.glyphGrabber.setState(CodaGlyphGrabber.GlyphGrabberState.CLOSED)
            linearOpMode.sleep(1300)
            robot.lift.setPosition(CodaLift.LiftPosition.FIRST_LEVEL)
        }

        // Start the camera detection timer.
        val elapsedTime = ElapsedTime(ElapsedTime.Resolution.MILLISECONDS)

        // Detect the jewel configuration.
        detectedJewelConfiguration = jewelConfigurationDetector.waitForJewelIdentification(
            elapsedTime, linearOpMode
        )
        jewelConfigurationDetector.disable()

        // Move the jewel displacement bar if applicable.
        if (detectedJewelConfiguration != JewelConfigurationDetector.JewelConfiguration.UNKNOWN) {
            robot.jewelDisplacementBar.setPosition(CodaJewelDisplacementBar.Position.DOWN)
        }

        // Detect the pictograph.
        pictographIdentifier.activate()
        detectedPictograph = pictographIdentifier.waitForPictographIdentification(
            elapsedTime, linearOpMode
        )
        pictographIdentifier.deactivate()

        // Wait for the grabber to grab the glyph.
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
                        robot.jewelDisplacementBar.setPosition(CodaJewelDisplacementBar.Position.UP)
                        robot.driveOnBalancingStone(-0.40)
                        robot.driveOffBalancingStone(-0.175)
                    }

                    JewelConfigurationDetector.JewelConfiguration.BLUE_RED -> {
                        robot.driveOffBalancingStone(-0.175)
                        robot.jewelDisplacementBar.setPosition(CodaJewelDisplacementBar.Position.UP)
                    }

                    JewelConfigurationDetector.JewelConfiguration.UNKNOWN -> {
                        robot.driveOffBalancingStone(-0.175)
                    }
                }

            AllianceColor.BLUE ->
                when (detectedJewelConfiguration) {
                    JewelConfigurationDetector.JewelConfiguration.RED_BLUE -> {
                        robot.driveTrain.linearTimeDrive(1000, -0.175)
                        robot.jewelDisplacementBar.setPosition(CodaJewelDisplacementBar.Position.UP)
                        robot.driveOnBalancingStone(0.40)
                        robot.driveOffBalancingStone(0.175)
                    }

                    JewelConfigurationDetector.JewelConfiguration.BLUE_RED -> {
                        robot.driveOffBalancingStone(0.175)
                        robot.jewelDisplacementBar.setPosition(CodaJewelDisplacementBar.Position.UP)
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

    private fun deliverGlyph() {
        robot.driveTrain.linearTimeDrive(1000, 0.25)
        robot.glyphGrabber.setState(CodaGlyphGrabber.GlyphGrabberState.RELEASE)
        robot.driveTrain.linearTimeDrive(750, -0.25)
        robot.glyphGrabber.setState(CodaGlyphGrabber.GlyphGrabberState.SMALL_OPEN)
    }

    fun performSharedActionGroup4() {
        robot.lift.setPosition(CodaLift.LiftPosition.BOTTOM)
        linearOpMode.sleep(1000)
        deliverGlyph()
    }

    fun performSharedActionGroup5() {
        robot.driveTrain.linearTimeDrive(850, 1.0)
        robot.glyphGrabber.setState(CodaGlyphGrabber.GlyphGrabberState.CLOSED)
        linearOpMode.sleep(2000)
        robot.driveTrain.linearTimeDrive(500, -0.50)
    }

    fun performSharedActionGroup6() {
        robot.driveTrain.linearTimeDrive(200, 0.25)
        robot.driveToDistanceFromObject(
            Coda.ObjectDirection.LEFT,
            RelicRecoveryConstants.TRAILING_FRONT_CRYPTO_BOX_DISTANCE
        )
        deliverGlyph()
    }

}

@Autonomous(group = "Front")
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
        AutonomousActions.robot.driveToDistanceFromObject(
            Coda.ObjectDirection.LEFT,
            RelicRecoveryConstants.CENTER_FRONT_CRYPTO_BOX_DISTANCE,
            0.75, false
        )
        AutonomousActions.robot.driveTrain.turnToHeading(
            -145.0, AutonomousActions.DEFAULT_TURN_SPEED
        )
        AutonomousActions.performSharedActionGroup5()
        AutonomousActions.robot.driveTrain.turnToHeading(
            0.0, AutonomousActions.DEFAULT_TURN_SPEED
        )
        AutonomousActions.performSharedActionGroup6()
    }

}

@Autonomous(group = "Front")
class RedFront : LinearOpMode() {

    @Throws(InterruptedException::class)
    override fun runOpMode() {
        AutonomousActions.linearOpMode = this
        AutonomousActions.performSharedActionGroup1()
        AutonomousActions.performSharedActionGroup2(AutonomousActions.AllianceColor.RED)
        AutonomousActions.robot.driveTrain.turnToHeading(
            180.0, AutonomousActions.DEFAULT_TURN_SPEED
        )
        AutonomousActions.performSharedActionGroup3(
            Coda.ObjectDirection.RIGHT,
            AutonomousActions.CryptoBoxPosition.FRONT
        )
        AutonomousActions.performSharedActionGroup4()
        AutonomousActions.robot.driveToDistanceFromObject(
            Coda.ObjectDirection.RIGHT,
            RelicRecoveryConstants.TRAILING_FRONT_CRYPTO_BOX_DISTANCE,
            0.75, false
        )
        AutonomousActions.robot.driveTrain.turnToHeading(
            -45.0, AutonomousActions.DEFAULT_TURN_SPEED
        )
        AutonomousActions.performSharedActionGroup5()
        AutonomousActions.robot.driveTrain.turnToHeading(
            0.0, AutonomousActions.DEFAULT_TURN_SPEED
        )
        AutonomousActions.performSharedActionGroup6()
    }

}

@Autonomous(group = "Rear")
class BlueBack : LinearOpMode() {

    @Throws(InterruptedException::class)
    override fun runOpMode() {
        AutonomousActions.linearOpMode = this
        AutonomousActions.performSharedActionGroup1()
        AutonomousActions.performSharedActionGroup2(AutonomousActions.AllianceColor.BLUE)
        AutonomousActions.robot.driveTrain.turnToHeading(
            90.0, AutonomousActions.DEFAULT_TURN_SPEED
        )
        AutonomousActions.performSharedActionGroup3(
            Coda.ObjectDirection.LEFT,
            AutonomousActions.CryptoBoxPosition.SIDE
        )
        AutonomousActions.performSharedActionGroup4()
        // The front antonymous opmodes have a drive here. It is not necessary for the back ones.
        AutonomousActions.robot.driveTrain.turnToHeading(
            -90.0, AutonomousActions.DEFAULT_TURN_SPEED
        )
        AutonomousActions.performSharedActionGroup5()
        AutonomousActions.robot.driveTrain.turnToHeading(
            0.0, AutonomousActions.DEFAULT_TURN_SPEED
        )
        AutonomousActions.performSharedActionGroup6()
    }

}

@Autonomous(group = "Rear")
class RedBack : LinearOpMode() {

    @Throws(InterruptedException::class)
    override fun runOpMode() {
        AutonomousActions.linearOpMode = this
        AutonomousActions.performSharedActionGroup1()
        AutonomousActions.performSharedActionGroup2(AutonomousActions.AllianceColor.RED)
        AutonomousActions.robot.driveTrain.turnToHeading(
            90.0, AutonomousActions.DEFAULT_TURN_SPEED
        )
        AutonomousActions.performSharedActionGroup3(
            Coda.ObjectDirection.RIGHT,
            AutonomousActions.CryptoBoxPosition.SIDE
        )
        AutonomousActions.performSharedActionGroup4()
        // The front antonymous opmodes have a drive here. It is not necessary for the back ones.
        AutonomousActions.robot.driveTrain.turnToHeading(
            -90.0, AutonomousActions.DEFAULT_TURN_SPEED
        )
        AutonomousActions.performSharedActionGroup5()
        AutonomousActions.robot.driveTrain.turnToHeading(
            0.0, AutonomousActions.DEFAULT_TURN_SPEED
        )
        AutonomousActions.performSharedActionGroup6()
    }

}
