package org.firstinspires.ftc.teamcode.libraries.robot

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode

open class Robot(val linearOpMode: LinearOpMode) {

    protected val components = hashMapOf<String, Component>()

    fun addComponent(component: Component, key: String) {
        components[key] = component
    }

}