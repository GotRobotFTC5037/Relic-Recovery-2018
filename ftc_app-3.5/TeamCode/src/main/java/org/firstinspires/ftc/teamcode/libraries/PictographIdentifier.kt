package org.firstinspires.ftc.teamcode.libraries

import org.firstinspires.ftc.robotcore.external.ClassFactory
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables

class PictographIdentifier {

    private val relicTrackables: VuforiaTrackables
    private val relicTemplate: VuforiaTrackable

    init {
        val parameters = VuforiaLocalizer.Parameters()
        parameters.vuforiaLicenseKey = VUFORIA_LICENSE_KEY
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK

        val vuforia = ClassFactory.createVuforiaLocalizer(parameters)
        this.relicTrackables = vuforia.loadTrackablesFromAsset("RelicVuMark")
        this.relicTemplate = relicTrackables[0]
    }

    fun activate() {
        relicTrackables.activate()
    }

    fun identifyPictograph(): RelicRecoveryVuMark {
        return RelicRecoveryVuMark.from(relicTemplate)
    }

    companion object {
        private val VUFORIA_LICENSE_KEY = "ARai2kL/////AAAAGbtywJzFuEENhljmjUYXMOxYSnK" +
                "PHlrkYr0UAA2AiMQwru6hVJxv0+XBwi8d7KGOU90Gku8ubGjJNWdQwPMsvyPOivcjD" +
                "JABZuxgkVr6p8CrwnYIuFTt7v0YDG/9zFztVLkn2jaZnh8p7PkRoPeKFaixcUaDlWz" +
                "U4eydNHAfn0ufnFNm1i5FDsI5pf4UAtwRS2jGHHjyp3/o7sPZpT6Mu1yVT7D6YStzl" +
                "9I/7jMeKHn9lu6zhbnMlOi2D/iY14mOYUvQg1eMiHstkPzUY7IeBngPmCnEjOHWmv7" +
                "XRboufL3JsCTrC8pnnEn5n3pZHei++FW9ovS6Aub89z//Yxq6OhPQ6+WaRNc3VSwFH/KJImw0"
    }

}