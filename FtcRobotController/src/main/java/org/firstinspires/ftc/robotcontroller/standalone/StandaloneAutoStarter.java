package org.firstinspires.ftc.robotcontroller.standalone;

import android.app.Activity;

import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerImpl;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.internal.network.NetworkConnectionHandler;

/**
 * Starts a configured autonomous OpMode from Control Hub digital inputs.
 *
 * This class deliberately uses the normal SDK OpModeManager lifecycle:
 * initOpMode() first, then startActiveOpMode() after the physical button.
 */
public final class StandaloneAutoStarter {
    private static final String TAG = "StandaloneAuto";

    private final Activity activity;
    private volatile boolean running = false;
    private Thread workerThread;

    public StandaloneAutoStarter(Activity activity) {
        this.activity = activity;
    }

    public synchronized void start() {
        if (!StandaloneAutoConfig.ENABLED) return;
        if (running) return;

        running = true;
        workerThread = new Thread(new Runnable() {
            @Override public void run() {
                runStarterLoop();
            }
        }, "StandaloneAutoStarter");
        workerThread.setDaemon(true);
        workerThread.start();
    }

    public synchronized void stop() {
        running = false;
        if (workerThread != null) {
            workerThread.interrupt();
            workerThread = null;
        }
        StandaloneAutoRuntime.reset();
    }

    private void runStarterLoop() {
        try {
            OpModeManagerImpl opModeManager = waitForOpModeManager();
            if (opModeManager == null) return;

            HardwareMap hardwareMap = waitForHardwareMap(opModeManager);
            if (hardwareMap == null) return;

            DigitalChannel startButton = getDigitalInput(hardwareMap, StandaloneAutoConfig.START_BUTTON_NAME);
            DigitalChannel sideSwitch = getDigitalInput(hardwareMap, StandaloneAutoConfig.SIDE_SWITCH_NAME);
            if (startButton == null || sideSwitch == null) return;

            boolean blueSelected = sideSwitch.getState() == StandaloneAutoConfig.BLUE_SWITCH_ON_STATE;
            StandaloneAutoRuntime.Side side = blueSelected
                    ? StandaloneAutoRuntime.Side.BLUE
                    : StandaloneAutoRuntime.Side.RED;
            String opModeName = blueSelected
                    ? StandaloneAutoConfig.BLUE_OP_MODE_NAME
                    : StandaloneAutoConfig.RED_OP_MODE_NAME;

            StandaloneAutoRuntime.setSelected(side, opModeName);
            RobotLog.ii(TAG, "Standalone side=%s opMode=%s", side, opModeName);

            opModeManager.initOpMode(opModeName);
            if (!waitForInitTransition(opModeManager, opModeName)) {
                RobotLog.ee(TAG, "Timed out waiting for OpMode INIT: %s", opModeName);
                return;
            }

            RobotLog.ii(TAG, "Waiting for physical start button: %s", StandaloneAutoConfig.START_BUTTON_NAME);
            waitForStartButton(startButton);
            if (!running) return;

            RobotLog.ii(TAG, "Physical START received; starting OpMode: %s", opModeName);
            opModeManager.startActiveOpMode();
            StandaloneAutoRuntime.markStarted();

            if (StandaloneAutoConfig.DISABLE_NETWORK_AFTER_START) {
                disableNetworkAfterStart();
            }
        } catch (Throwable t) {
            RobotLog.ee(TAG, t, "Standalone autonomous startup failed");
        }
    }

    private OpModeManagerImpl waitForOpModeManager() {
        while (running) {
            OpModeManagerImpl manager = OpModeManagerImpl.getOpModeManagerOfActivity(activity);
            if (manager != null) return manager;
            sleepPollInterval();
        }
        return null;
    }

    private HardwareMap waitForHardwareMap(OpModeManagerImpl opModeManager) {
        while (running) {
            HardwareMap hardwareMap = opModeManager.getHardwareMap();
            if (hardwareMap != null) return hardwareMap;
            sleepPollInterval();
        }
        return null;
    }

    private DigitalChannel getDigitalInput(HardwareMap hardwareMap, String deviceName) {
        try {
            DigitalChannel channel = hardwareMap.get(DigitalChannel.class, deviceName);
            channel.setMode(DigitalChannel.Mode.INPUT);
            return channel;
        } catch (RuntimeException e) {
            RobotLog.ee(TAG, e, "Missing Digital Input in Robot Config: %s", deviceName);
            return null;
        }
    }

    private boolean waitForInitTransition(OpModeManagerImpl opModeManager, String opModeName) {
        long deadline = System.currentTimeMillis() + StandaloneAutoConfig.INIT_TIMEOUT_MS;
        while (running && System.currentTimeMillis() < deadline) {
            if (opModeName.equals(opModeManager.getActiveOpModeName())) {
                return true;
            }
            sleepPollInterval();
        }
        return false;
    }

    private void waitForStartButton(DigitalChannel startButton) {
        long pressedSince = -1;
        while (running) {
            boolean pressed = startButton.getState() == StandaloneAutoConfig.START_BUTTON_PRESSED_STATE;
            long now = System.currentTimeMillis();

            if (pressed) {
                if (pressedSince < 0) pressedSince = now;
                if (now - pressedSince >= StandaloneAutoConfig.START_DEBOUNCE_MS) return;
            } else {
                pressedSince = -1;
            }

            sleepPollInterval();
        }
    }

    private void disableNetworkAfterStart() {
        try {
            if (NetworkConnectionHandler.getInstance().getNetworkConnection() != null) {
                NetworkConnectionHandler.getInstance().getNetworkConnection().disable();
                RobotLog.ii(TAG, "SDK network connection disabled after standalone START");
            }
        } catch (RuntimeException e) {
            RobotLog.ww(TAG, e, "Unable to disable SDK network connection after START");
        }
    }

    private void sleepPollInterval() {
        try {
            Thread.sleep(StandaloneAutoConfig.POLL_INTERVAL_MS);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
            running = false;
        }
    }
}
