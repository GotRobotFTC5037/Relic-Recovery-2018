package org.firstinspires.ftc.teamcode.game.opmodes

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.game.OpModeContinuity
import org.firstinspires.ftc.teamcode.game.actions.CodaAutonomousActions

/**
 * The autonomous opmode that is run on the blue alliance with the front crypto box.
 */
@Autonomous(name = "Blue Front", group = "Gameplay")
class BlueFront : LinearOpMode() {

    @Throws(InterruptedException::class)
    override fun runOpMode() {
        OpModeContinuity.lastAllianceColor = OpModeContinuity.AllianceColor.BLUE
        CodaAutonomousActions(linearOpMode = this)
            .apply {
                allianceColor = CodaAutonomousActions.AllianceColor.BLUE
                cryptoBoxPosition = CodaAutonomousActions.CryptoBoxPosition.FRONT
                cryptoBoxHeading = CRYPTO_BOX_HEADING
                glyphPitHeading = GLYPH_PIT_HEADING
            }
            .performForAutonomousOpMode()
    }

    companion object {
        private const val CRYPTO_BOX_HEADING = 0.0
        private const val GLYPH_PIT_HEADING = -155.0
    }

}

/**
 * The autonomous opmode that is run on the red alliance with the front crypto box.
 */
@Autonomous(name = "Red Front", group = "Gameplay")
class RedFront : LinearOpMode() {

    @Throws(InterruptedException::class)
    override fun runOpMode() {
        OpModeContinuity.lastAllianceColor = OpModeContinuity.AllianceColor.RED
        CodaAutonomousActions(linearOpMode = this)
            .apply {
                allianceColor = CodaAutonomousActions.AllianceColor.RED
                cryptoBoxPosition = CodaAutonomousActions.CryptoBoxPosition.FRONT
                cryptoBoxHeading = CRYPTO_BOX_HEADING
                glyphPitHeading = GLYPH_PIT_HEADING
            }
            .performForAutonomousOpMode()
    }

    companion object {
        private const val CRYPTO_BOX_HEADING = 180.0
        private const val GLYPH_PIT_HEADING = -25.0
    }

}

/**
 * The autonomous opmode that is run on the blue alliance with the side crypto box.
 */
@Autonomous(name = "Blue Side", group = "Gameplay")
class BlueRear : LinearOpMode() {

    @Throws(InterruptedException::class)
    override fun runOpMode() {
        OpModeContinuity.lastAllianceColor = OpModeContinuity.AllianceColor.BLUE
        CodaAutonomousActions(linearOpMode = this)
            .apply {
                allianceColor = CodaAutonomousActions.AllianceColor.BLUE
                cryptoBoxPosition = CodaAutonomousActions.CryptoBoxPosition.SIDE
                cryptoBoxHeading = CRYPTO_BOX_HEADING
                glyphPitHeading = GLYPH_PIT_HEADING
            }
            .performForAutonomousOpMode()
    }

    companion object {
        private const val CRYPTO_BOX_HEADING = 90.0
        private const val GLYPH_PIT_HEADING = -90.0
    }

}

/**
 * The autonomous opmode that is run on the red alliance with the side crypto box.
 */
@Autonomous(name = "Red Side", group = "Gameplay")
class RedRear : LinearOpMode() {

    @Throws(InterruptedException::class)
    override fun runOpMode() {
        OpModeContinuity.lastAllianceColor = OpModeContinuity.AllianceColor.RED
        CodaAutonomousActions(linearOpMode = this)
            .apply {
                allianceColor = CodaAutonomousActions.AllianceColor.RED
                cryptoBoxPosition = CodaAutonomousActions.CryptoBoxPosition.SIDE
                cryptoBoxHeading = CRYPTO_BOX_HEADING
                glyphPitHeading = GLYPH_PIT_HEADING
            }
            .performForAutonomousOpMode()
    }

    companion object {
        private const val CRYPTO_BOX_HEADING = 90.0
        private const val GLYPH_PIT_HEADING = -90.0
    }

}
