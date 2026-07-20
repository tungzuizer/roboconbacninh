package org.firstinspires.ftc.robotcontroller.standalone;

/**
 * Configuration for standalone autonomous startup on a REV Control Hub.
 *
 * This is intended for non-FTC autonomous competitions where the robot must
 * start from physical controls instead of Driver Station INIT/START commands.
 */
public final class StandaloneAutoConfig {
    private StandaloneAutoConfig() { }

    /** Master switch for the standalone startup feature. */
    public static final boolean ENABLED = true;

    /** Robot Configuration name for the physical start button Digital Input. */
    public static final String START_BUTTON_NAME = "standaloneStart";

    /** Robot Configuration name for the Red/Blue selector Digital Input. */
    public static final String SIDE_SWITCH_NAME = "sideSwitch";

    /**
     * Expected DigitalChannel state when the start button is pressed.
     *
     * REV Digital Inputs commonly read true when pulled up and false when
     * connected to ground, so false is a common active-low button value.
     */
    public static final boolean START_BUTTON_PRESSED_STATE = false;

    /** Digital Switch 1: OFF = Red, ON = Blue. */
    public static final boolean BLUE_SWITCH_ON_STATE = true;

    /** OpMode selected when sideSwitch is OFF / Red. */
    public static final String RED_OP_MODE_NAME = "12 Box Auto - Ivy";

    /** OpMode selected when sideSwitch is ON / Blue. */
    public static final String BLUE_OP_MODE_NAME = "12 Box Auto - Ivy";

    /** Poll interval while waiting for Robot Controller setup and button press. */
    public static final long POLL_INTERVAL_MS = 20;

    /** Start button debounce time before START is accepted. */
    public static final long START_DEBOUNCE_MS = 100;

    /** Timeout for waiting for the selected OpMode to finish INIT transition. */
    public static final long INIT_TIMEOUT_MS = 10_000;

    /**
     * Optional hardening for competitions that disallow wireless communication.
     *
     * Default is false to preserve normal FTC SDK behavior. If set true, the
     * SDK network connection is disabled immediately after physical START.
     */
    public static final boolean DISABLE_NETWORK_AFTER_START = false;
}
