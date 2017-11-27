import android.util.Log
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeManagerImpl

object RelicRecoveryRobotOpModeManager : Thread() {

    public var currentOpMode: OpMode? = null
    private set

    private var queuedOpModeName: String? = null

    init { this.start() }

    override fun run() {
        try {
            while (true) {
                synchronized(this) {
                    if (currentOpMode != null) {
                        val opModeManager = currentOpMode!!.internalOpModeServices as OpModeManagerImpl
                        if (opModeManager.activeOpMode !== currentOpMode) {
                            Thread.sleep(500)
                            opModeManager.initActiveOpMode(queuedOpModeName!!)
                            reset()
                        }
                    }
                }

                Thread.sleep(100)
            }
        } catch (ex: InterruptedException) {
            Log.e(FtcRobotControllerActivity.TAG, "RelicRecoveryRobotOpModeManager shutdown, thread interrupted")
        }

    }

    private fun setNewTransition(currentOpMode: OpMode, name: String) {
        synchronized(this) {
            this.currentOpMode = currentOpMode
            this.queuedOpModeName = name
        }
    }

    private fun reset() {
        this.currentOpMode = null
        this.queuedOpModeName = null
    }

    fun queueOpMode(currentOpMode: OpMode, name: String) {
        setNewTransition(currentOpMode, name)
    }
}
