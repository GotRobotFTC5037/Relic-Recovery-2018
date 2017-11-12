package org.firstinspires.ftc.teamcode.libraries

import com.qualcomm.robotcore.hardware.*

class PixyCam : I2cDeviceSynch {
    override fun isEngaged(): Boolean {
        TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
    }

    override fun resetDeviceConfigurationForOpMode() {
        TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
    }

    override fun enableWriteCoalescing(enable: Boolean) {
        TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
    }

    override fun isArmed(): Boolean {
        TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
    }

    override fun setLoggingTag(loggingTag: String?) {
        TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
    }

    override fun setLogging(enabled: Boolean) {
        TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
    }

    override fun isWriteCoalescingEnabled(): Boolean {
        TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
    }

    override fun write(ireg: Int, data: ByteArray?) {
        TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
    }

    override fun write(ireg: Int, data: ByteArray?, waitControl: I2cWaitControl?) {
        TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
    }

    override fun setI2cAddress(newAddress: I2cAddr?) {
        TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
    }

    override fun getDeviceName(): String {
        TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
    }

    override fun getHeartbeatInterval(): Int {
        TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
    }

    override fun setHeartbeatAction(action: I2cDeviceSynch.HeartbeatAction?) {
        TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
    }

    override fun getConnectionInfo(): String {
        TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
    }

    override fun getVersion(): Int {
        TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
    }

    override fun waitForWriteCompletions(waitControl: I2cWaitControl?) {
        TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
    }

    override fun getReadWindow(): I2cDeviceSynch.ReadWindow {
        TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
    }

    override fun setHealthStatus(status: HardwareDeviceHealth.HealthStatus?) {
        TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
    }

    override fun read8(ireg: Int): Byte {
        TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
    }

    override fun engage() {
        TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
    }

    override fun close() {
        TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
    }

    override fun getManufacturer(): HardwareDevice.Manufacturer {
        TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
    }

    override fun read(ireg: Int, creg: Int): ByteArray {
        TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
    }

    override fun getUserConfiguredName(): String? {
        TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
    }

    override fun getHeartbeatAction(): I2cDeviceSynch.HeartbeatAction {
        TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
    }

    override fun setHeartbeatInterval(ms: Int) {
        TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
    }

    override fun readTimeStamped(ireg: Int, creg: Int, readWindowNeeded: I2cDeviceSynch.ReadWindow?, readWindowSet: I2cDeviceSynch.ReadWindow?): TimestampedData {
        TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
    }

    override fun readTimeStamped(ireg: Int, creg: Int): TimestampedData {
        TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
    }

    override fun getHealthStatus(): HardwareDeviceHealth.HealthStatus {
        TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
    }

    override fun ensureReadWindow(windowNeeded: I2cDeviceSynch.ReadWindow?, windowToSet: I2cDeviceSynch.ReadWindow?) {
        TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
    }

    override fun write8(ireg: Int, bVal: Int) {
        TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
    }

    override fun write8(ireg: Int, bVal: Int, waitControl: I2cWaitControl?) {
        TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
    }

    override fun getLoggingTag(): String {
        TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
    }

    override fun setI2cAddr(i2cAddr: I2cAddr?) {
        TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
    }

    override fun setUserConfiguredName(name: String?) {
        TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
    }

    override fun getI2cAddr(): I2cAddr {
        TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
    }

    override fun getI2cAddress(): I2cAddr {
        TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
    }

    override fun getLogging(): Boolean {
        TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
    }

    override fun disengage() {
        TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
    }

    override fun setReadWindow(window: I2cDeviceSynch.ReadWindow?) {
        TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
    }

    val foo = 0

}