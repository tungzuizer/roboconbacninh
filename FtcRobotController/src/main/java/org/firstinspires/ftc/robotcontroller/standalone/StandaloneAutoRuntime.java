package org.firstinspires.ftc.robotcontroller.standalone;

/** Runtime state exposed to TeamCode for mirrored autonomous logic. */
public final class StandaloneAutoRuntime {
    public enum Side {
        RED,
        BLUE,
        UNKNOWN
    }

    private static volatile Side selectedSide = Side.UNKNOWN;
    private static volatile String selectedOpModeName = "";
    private static volatile boolean standaloneStarted = false;

    private StandaloneAutoRuntime() { }

    public static Side getSelectedSide() {
        return selectedSide;
    }

    public static String getSelectedOpModeName() {
        return selectedOpModeName;
    }

    public static boolean hasStandaloneStarted() {
        return standaloneStarted;
    }

    static void setSelected(Side side, String opModeName) {
        selectedSide = side;
        selectedOpModeName = opModeName == null ? "" : opModeName;
        standaloneStarted = false;
    }

    static void markStarted() {
        standaloneStarted = true;
    }

    static void reset() {
        selectedSide = Side.UNKNOWN;
        selectedOpModeName = "";
        standaloneStarted = false;
    }
}
