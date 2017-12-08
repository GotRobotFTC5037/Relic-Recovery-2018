import android.util.Log
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeManagerImpl

/**
 * An object that manages the running of OpModes.
 *
 * @author FTC Team 5037 gotrobot?
 */
object RelicRecoveryRobotOpModeManager : Thread() {

    private var currentOpMode: OpMode? = null
    private var queuedOpModeName: String? = null

    init {
        this.start()
    }

    /**
     * Watches for the current OpMode to end in order to being the next one.
     */
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

    /**
     * Queues an opmode to run after the current one ends.
     * @param currentOpMode The OpMode that is currently running.
     * @param name The name of the opmode that should run when the current one ends.
     */
    private fun setNewTransition(currentOpMode: OpMode, name: String) {
        synchronized(this) {
            this.currentOpMode = currentOpMode
            this.queuedOpModeName = name
        }
    }

    /**
     * Clears the currently queued opmode.
     */
    private fun reset() {
        this.currentOpMode = null
        this.queuedOpModeName = null
    }

    /**
     * Queues an opmode to run after the current one ends.
     * @param currentOpMode The OpMode that is currently running.
     * @param name The name of the opmode that should run when the current one ends.
     */
    fun queueOpMode(currentOpMode: OpMode, name: String) {
        setNewTransition(currentOpMode, name)
    }
}
