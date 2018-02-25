package org.firstinspires.ftc.teamcode.game.components

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.lib.powercontroller.PowerController
import org.firstinspires.ftc.teamcode.lib.powercontroller.StaticPowerController
import org.firstinspires.ftc.teamcode.lib.robot.attachment.RobotAttachment

class CodaRelicGrabber(linearOpMode: LinearOpMode) : RobotAttachment(linearOpMode) {

    private val winch by lazy {
        val motor = hardwareMap.dcMotor.get("relic grabber winch")
        motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        motor
    }

    private val armServo by lazy {
        val servo = hardwareMap.servo.get("relic arm")
        servo.direction = Servo.Direction.REVERSE
        servo
    }

    private val grabberServo by lazy {
        val servo = hardwareMap.servo.get("relic grabber")
        servo.direction = Servo.Direction.REVERSE
        servo
    }

    fun setPower(power: Double) {
        winch.power = power
    }

    private fun setPosition(position: Int, controller: PowerController) {

        controller.errorValueHandler = {
            position.toDouble() - winch.currentPosition
        }

        if (position > winch.currentPosition) {
            while (position > winch.currentPosition) {
                setPower(controller.outputPower)
            }
        } else if (position < winch.currentPosition) {
            while (position < winch.currentPosition) {
                setPower(controller.outputPower)
            }
        }

        winch.power = 0.0

        controller.stopUpdatingOutput()
    }
  
    enum class ArmPosition(val position: Double) { IN(0.0), DOWN(1.0) }

    fun setArmPosition(position: ArmPosition) {
        armServo.position = position.position
    }

    enum class GrabberState(val position: Double) { OPEN(1.0), CLOSED(0.0) }

    fun setGrabberState(state: GrabberState) {
        grabberServo.position = state.position
    }

    fun deliverRelic() {
        setPosition(WINCH_EXTENDED_POSITION, StaticPowerController(1.0))
        setArmPosition(ArmPosition.DOWN)
        linearOpMode.sleep(1000)
        setGrabberState(GrabberState.OPEN)
        linearOpMode.sleep(1000)
        setArmPosition(ArmPosition.IN)
        setPosition(5, StaticPowerController(1.0))
    }

    companion object {
        private const val WINCH_EXTENDED_POSITION = 3800
    }

}