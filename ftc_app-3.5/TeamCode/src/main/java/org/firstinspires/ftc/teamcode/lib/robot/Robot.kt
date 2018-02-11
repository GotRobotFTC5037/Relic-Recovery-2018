package org.firstinspires.ftc.teamcode.lib.robot

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode

open class Robot(val linearOpMode: LinearOpMode) {

    val components = hashMapOf<String, Component>()

    fun addComponent(component: Component, key: String) {
        components[key] = component
    }

}