package org.firstinspires.ftc.teamcode.libraries.components

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.libraries.components.attachments.RobotAttachment
import org.firstinspires.ftc.teamcode.libraries.components.drivetrains.RobotDriveTrain
import org.firstinspires.ftc.teamcode.libraries.components.lift.RobotLift
import org.firstinspires.ftc.teamcode.libraries.components.sensors.RobotSensor

open class Robot(val linearOpMode: LinearOpMode) {

    protected var driveTrain: RobotDriveTrain? = null
    protected val attachments = HashMap<String, RobotAttachment>()
    protected val lifts = HashMap<String, RobotLift>()
    protected val sensors = HashMap<String, RobotSensor>()

    fun addComponent(component: RobotComponent, key: String = "") {
        when (component) {
            is RobotDriveTrain -> driveTrain = component
            is RobotAttachment -> attachments[key] = component
            is RobotLift -> lifts[key] = component
            is RobotSensor -> sensors[key] = component
        }
    }

}