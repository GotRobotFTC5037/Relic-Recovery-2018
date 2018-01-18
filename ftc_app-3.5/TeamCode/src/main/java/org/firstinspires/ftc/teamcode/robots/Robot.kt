package org.firstinspires.ftc.teamcode.robots

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.HardwareMap

/**
 * A base class for creating a robot of any type.
 *
 * @author FTC Team 5037 gotrobot?
 */
abstract class Robot {

    lateinit var opMode: OpMode

    /**
     * Sets up the hardware needed in order to use the robot.
     * @param hardwareMap A HardwareMap object. Usually provided by opMode.hardwareMap.
     */
    abstract fun setup(hardwareMap: HardwareMap)

    /**
     * Sets the power that the robot should drive forward or backwards.
     * @param power The power that the robot should drive at.
     */
    abstract fun setDrivePower(power: Double)

    /**
     * Sets the power that the robot should turn.
     * @param power The power that the robot should turn at.
     */
    abstract fun setTurnPower(power: Double)

    /**
     * Sets the power of the drive motors to 0.
     */
    abstract fun stopAllDriveMotors()

    /**
     * Turns the robot to the specified angle.
     * @param power The power that the robot should turn at.
     * @param degrees the degrees the robot should move at.
     */
    abstract fun turn(power: Double, degrees: Double)
}
