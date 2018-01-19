package org.firstinspires.ftc.teamcode.libraries.robot

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.libraries.robot.attachments.RobotAttachment
import org.firstinspires.ftc.teamcode.libraries.robot.drivetrains.RobotDriveTrain
import org.firstinspires.ftc.teamcode.libraries.robot.lift.RobotLift
import org.firstinspires.ftc.teamcode.libraries.robot.sensors.RobotSensor

open class Robot(val linearOpMode: LinearOpMode) {

    protected var driveTrains = HashMap<String, RobotDriveTrain>()
    protected val attachments = HashMap<String, RobotAttachment>()
    protected val lifts = HashMap<String, RobotLift>()
    protected val sensors = HashMap<String, RobotSensor>()

    fun addComponent(component: RobotComponent, key: String = "") {
        when (component) {
            is RobotDriveTrain -> driveTrains[key] = component
            is RobotAttachment -> attachments[key] = component
            is RobotLift -> lifts[key] = component
            is RobotSensor -> sensors[key] = component
        }
    }

}