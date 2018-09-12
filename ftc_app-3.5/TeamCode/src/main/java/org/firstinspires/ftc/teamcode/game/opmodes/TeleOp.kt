package org.firstinspires.ftc.teamcode.game.opmodes

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.game.components.CodaBalancingStoneHolder
import org.firstinspires.ftc.teamcode.game.components.CodaGlyphGrabbers
import org.firstinspires.ftc.teamcode.game.components.CodaRelicGrabber
import org.firstinspires.ftc.teamcode.game.robots.Coda
import kotlin.math.abs

@TeleOp(name = "One-Man TeleOp")
class CodaOneManTeleOp : LinearOpMode() {

    private val robot: Coda by lazy { Coda(this) }

    private val lastManualHeadingUpdate: ElapsedTime by lazy { ElapsedTime() }

    @Throws(InterruptedException::class)
    override fun runOpMode() {
        robot.setup()
        waitForStart()

        while (opModeIsActive()) {
            manuallySetDriveDirection()
            performGunnerActions()
        }

        robot.driveTrain.stop()
    }

    private fun performGunnerActions() {
        val gamepad = gamepad1

        when {
            gamepad.a -> robot.glyphGrabber.setState(CodaGlyphGrabbers.GlyphGrabberState.CLOSED)
            gamepad.b -> robot.glyphGrabber.setState(CodaGlyphGrabbers.GlyphGrabberState.RELEASE)
            gamepad.x -> robot.glyphGrabber.setState(CodaGlyphGrabbers.GlyphGrabberState.SMALL_OPEN)
        }

        val liftPower = if (gamepad.dpad_up) { 1.0 } else if (gamepad.dpad_down) { -1.0 } else { 0.0}
        if (liftPower != 0.0) {
            robot.lift.shouldHoldLiftPosition = false
            robot.lift.setPower(liftPower)
            val position = robot.lift.position
            position.value = robot.lift.motor.currentPosition
            robot.lift.position = position
        } else {
            robot.lift.shouldHoldLiftPosition = true
        }

    }

    private fun manuallySetDriveDirection() {
        var linearPower = -gamepad1.left_stick_y.toDouble()
        var strafePower = gamepad1.left_stick_x.toDouble()
        var turnPower = -gamepad1.right_stick_x.toDouble()

        if (gamepad1.right_trigger > 0.1) {
            linearPower *= 0.75
            strafePower *= 0.75
            turnPower *= 0.65
        } else {
            linearPower *= 1.00
            strafePower *= 1.00
            turnPower *= 0.85
        }

        linearPower = abs(Math.pow(linearPower, 2.0)) * Math.signum(linearPower)
        strafePower = abs(Math.pow(strafePower, 2.0)) * Math.signum(strafePower)
        turnPower = abs(Math.pow(turnPower, 2.0)) * Math.signum(turnPower)

        robot.driveTrain.setMovementPowers(linearPower, strafePower, turnPower)

        if (
            robot.glyphGrabber.currentState == CodaGlyphGrabbers.GlyphGrabberState.RELEASE &&
            linearPower > 0.65
        ) {
            robot.glyphGrabber.setState(CodaGlyphGrabbers.GlyphGrabberState.SMALL_OPEN)
        }

        if (turnPower != 0.0) {
            lastManualHeadingUpdate.reset()
            robot.driveTrain.shouldCorrectHeading = false
        } else if (
            lastManualHeadingUpdate.milliseconds() >= 750 &&
            !robot.driveTrain.shouldCorrectHeading
        ) {
            robot.driveTrain.targetHeading = robot.driveTrain.currentHeading
            robot.driveTrain.shouldCorrectHeading = true
        }
    }
}

@TeleOp(name = "TeleOp")
class CodaTeleOp : LinearOpMode() {

    private val robot: Coda by lazy { Coda(this) }

    private val lastManualHeadingUpdate: ElapsedTime by lazy { ElapsedTime() }

    @Throws(InterruptedException::class)
    override fun runOpMode() {
        robot.setup()
        waitForStart()

        while (opModeIsActive()) {
            performDriverActions()
            performGunnerActions()
        }

        robot.driveTrain.stop()
    }

    private fun performDriverActions() {
        val gamepad = gamepad1

        when {
            gamepad.a -> robot.relicGrabber.setArmPosition(CodaRelicGrabber.ArmPosition.DOWN)
            gamepad.b -> robot.relicGrabber.setGrabberState(CodaRelicGrabber.GrabberState.CLOSED)
            gamepad.x -> robot.relicGrabber.setGrabberState(CodaRelicGrabber.GrabberState.OPEN)
            gamepad.y -> robot.relicGrabber.setArmPosition(CodaRelicGrabber.ArmPosition.IN)

            gamepad.dpad_left -> {
                if (gamepad.left_trigger > 0.1) {
                    robot.driveTrain.turnAtPower(0.30)
                    lastManualHeadingUpdate.reset()
                    robot.driveTrain.shouldCorrectHeading = false
                } else {
                    robot.driveTrain.strafeDriveAtPower(0.40)
                }
            }

            gamepad.dpad_right -> {
                if (gamepad.left_trigger > 0.1) {
                    robot.driveTrain.turnAtPower(-0.30)
                    lastManualHeadingUpdate.reset()
                    robot.driveTrain.shouldCorrectHeading = false
                } else {
                    robot.driveTrain.strafeDriveAtPower(-0.40)
                }
            }

            gamepad.dpad_up -> robot.driveTrain.linearDriveAtPower(0.20)
            gamepad.dpad_down -> robot.driveTrain.linearDriveAtPower(-0.20)
            else -> manuallySetDriveDirection()
        }

        when {
            gamepad.left_bumper ->
                robot.balancingStoneHolder.setState(
                    CodaBalancingStoneHolder.BalancingStoneHolderState.UP
                )

            gamepad.right_bumper ->
                robot.balancingStoneHolder.setState(
                    CodaBalancingStoneHolder.BalancingStoneHolderState.DOWN
                )
        }
    }

    private fun performGunnerActions() {
        when {
            gamepad2.a -> robot.glyphGrabber.setState(CodaGlyphGrabbers.GlyphGrabberState.CLOSED)
            gamepad2.b -> robot.glyphGrabber.setState(CodaGlyphGrabbers.GlyphGrabberState.RELEASE)
            gamepad2.x -> robot.glyphGrabber.setState(CodaGlyphGrabbers.GlyphGrabberState.SMALL_OPEN)

            gamepad2.left_trigger > 0.1 -> {
                robot.glyphGrabber.bottomLeftGlyphGrabber.position =
                    CodaGlyphGrabbers.BOTTOM_GRABBER_CLOSED_POSITION

                robot.glyphGrabber.bottomRightGlyphGrabber.position =
                    CodaGlyphGrabbers.BOTTOM_GRABBER_CLOSED_POSITION
            }

            gamepad2.right_trigger > 0.1 -> {
                robot.glyphGrabber.topLeftGlyphGrabber.position =
                    CodaGlyphGrabbers.TOP_GRABBER_CLOSED_POSITION

                robot.glyphGrabber.topRightGlyphGrabber.position =
                    CodaGlyphGrabbers.TOP_GRABBER_CLOSED_POSITION
            }
        }

        val liftPower = -gamepad2.left_stick_y.toDouble()
        if (liftPower != 0.0) {
            robot.lift.shouldHoldLiftPosition = false
            robot.lift.setPower(liftPower)
            val position = robot.lift.position
            position.value = robot.lift.motor.currentPosition
            robot.lift.position = position
        } else {
            robot.lift.shouldHoldLiftPosition = true
        }

        when {
            gamepad2.dpad_up -> {
                while (gamepad2.dpad_up) {
                    idle()
                }
                robot.lift.elevate()
            }

            gamepad2.dpad_down -> {
                while (gamepad2.dpad_down) {
                    idle()
                }
                robot.lift.lower()
            }
        }

        val relicGrabberPower = -gamepad2.right_stick_y.toDouble()
        robot.relicGrabber.setPower(relicGrabberPower)
    }

    private fun manuallySetDriveDirection() {
        var linearPower = -gamepad1.left_stick_y.toDouble()
        var strafePower = gamepad1.left_stick_x.toDouble()
        var turnPower = -gamepad1.right_stick_x.toDouble()

        if (gamepad1.right_trigger > 0.1) {
            linearPower *= 0.75
            strafePower *= 0.75
            turnPower *= 0.65
        } else {
            linearPower *= 1.00
            strafePower *= 1.00
            turnPower *= 0.85
        }

        linearPower = abs(Math.pow(linearPower, 2.0)) * Math.signum(linearPower)
        strafePower = abs(Math.pow(strafePower, 2.0)) * Math.signum(strafePower)
        turnPower = abs(Math.pow(turnPower, 2.0)) * Math.signum(turnPower)

        robot.driveTrain.setMovementPowers(linearPower, strafePower, turnPower)

        if (
            robot.glyphGrabber.currentState == CodaGlyphGrabbers.GlyphGrabberState.RELEASE &&
            linearPower > 0.65
        ) {
            robot.glyphGrabber.setState(CodaGlyphGrabbers.GlyphGrabberState.SMALL_OPEN)
        }

        if (turnPower != 0.0) {
            lastManualHeadingUpdate.reset()
            robot.driveTrain.shouldCorrectHeading = false
        } else if (
            lastManualHeadingUpdate.milliseconds() >= 750 &&
            !robot.driveTrain.shouldCorrectHeading
        ) {
            robot.driveTrain.targetHeading = robot.driveTrain.currentHeading
            robot.driveTrain.shouldCorrectHeading = true
        }
    }
}


