package org.firstinspires.ftc.teamcode.game

import org.firstinspires.ftc.teamcode.game.elements.CryptoBox

object OpModeContinuity {

    enum class AllianceColor {
        BLUE,
        RED,
        UNKNOWN,
        UNDETERMINED
    }

    var lastAllianceColor = AllianceColor.UNDETERMINED

    lateinit var lastCryptoBox: CryptoBox

}