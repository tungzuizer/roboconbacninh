package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

/**
 * =========================================================================
 *                   HƯỚNG DẪN TUNING TRÊN FTCONTROL PANELS
 * =========================================================================
 * PHẦN 1: TUNE TỌA ĐỘ DI CHUYỂN (P0 -> P9)
 *   1. Chạy OpMode "Pedro 10-Position Tuner".
 *   2. Chọn vị trí qua TUNER_TARGET_POSITION_INDEX và bật TUNER_DRIVE_TO_SELECTED = true.
 *   3. Điều chỉnh tọa độ X, Y, H_DEG của vị trí đó trực tiếp bằng slider trên Panels.
 *
 * PHẦN 2: TUNE SERVO GẮP HỘP (Hành trình gắp vươn - hạ - kẹp - lùi - cất)
 *   1. Chạy OpMode "Servo Panels Tuner".
 *   2. Chọn vị trí mục tiêu di chuyển qua Panels (ví dụ: Shelf 3) và cho robot lái tới đó.
 *   3. Bật TUNER_APPLY = true.
 *   4. Lần lượt chạy qua các TUNER_MODE từ 1 đến 8 đại diện cho từng bước cơ học:
 *      Mode 1: S4 xoay ra trước gắp, S5 quẹo trái/phải theo kệ.
 *      Mode 2: S1 hạ xuống tầng 1 hoặc tầng 2.
 *      Mode 3: S2 vươn ra tiếp cận hộp.
 *      Mode 4: S3 kẹp chốt (0.1).
 *      Mode 5: S1 nâng lên một chút (Lift Up Offset).
 *      Mode 6: S2 thu tay về (S2 HOME).
 *      Mode 7: S1 nâng lên vị trí cao để chuẩn bị thả (HIGH STORE), S4 xoay ngược ra sau.
 *      Mode 8: S5 quẹo đến miệng khay cất lựa chọn (1..4), S1/S2 đặt vị trí xả, S3 mở nhả hộp (0.0).
 * =========================================================================
 */
@Configurable
public class BoxAutoPanels {

    public static void refresh() {
        PanelsConfigurables.INSTANCE.refreshClass(BoxAutoPanels.class);
    }

    // ═══════════════════════════════════════════════════════════
    //  FEATURE TOGGLES
    // ═══════════════════════════════════════════════════════════
    public static boolean ENABLE_DROP_MECHANISM = false;

    // ═══════════════════════════════════════════════════════════
    //  TÊN THIẾT BỊ (Robot Config — hardwareMap)
    // ═══════════════════════════════════════════════════════════
    public static String NAME_LIMELIGHT = "limelight";

    public static String NAME_S1 = "s1";
    public static String NAME_S2 = "s2";
    public static String NAME_S3 = "s3";
    public static String NAME_S4 = "s4";
    public static String NAME_S5 = "s5";

    public static String NAME_DROP1 = "drop1";
    public static String NAME_DROP2 = "drop2";
    public static String NAME_DROP3 = "drop3";
    public static String NAME_PCA9685 = "pca9685";

    // ═══════════════════════════════════════════════════════════
    //  P0–P9: 10 TUNABLE POSITIONS (Mặc định reset về 0)
    // ═══════════════════════════════════════════════════════════
    public static double START_X     = 0.0;
    public static double START_Y     = 0.0;
    public static double START_H_DEG = 0.0;

    public static double SHELF1_X     = 0.0;
    public static double SHELF1_Y     = 20.0;
    public static double SHELF1_H_DEG = 0.0;

    public static double SHELF2_X     = 25.5;
    public static double SHELF2_Y     = 20;
    public static double SHELF2_H_DEG = 0.0;

    public static double SHELF3_X     = 51;
    public static double SHELF3_Y     = 20;
    public static double SHELF3_H_DEG = 0.0;

    public static double DROP1_X     = 7.0;
    public static double DROP1_Y     = -21.0;
    public static double DROP1_H_DEG = 0.0;

    public static double DROP2_X     = 16.0;
    public static double DROP2_Y     = -21.0;
    public static double DROP2_H_DEG = 0.0;

    public static double DROP3_X     = 38.0;
    public static double DROP3_Y     = -21.0;
    public static double DROP3_H_DEG = 0.0;
    public static double DROP4_X     = 48.0;
    public static double DROP4_Y     = -21.0;
    public static double DROP4_H_DEG = 0.0;

    public static double EXTRA_PICK_X     = 0.0;
    public static double EXTRA_PICK_Y     = 0.0;
    public static double EXTRA_PICK_H_DEG = 0.0;

    public static double EXTRA_DROP_X     = 0.0;
    public static double EXTRA_DROP_Y     = 0.0;
    public static double EXTRA_DROP_H_DEG = 0.0;

    // ═══════════════════════════════════════════════════════════
    //  PICK SERVOS — THÔNG SỐ TUNE CHU TRÌNH GẤP 3 BƯỚC + 2 BƯỚC RÚT
    // ═══════════════════════════════════════════════════════════

    // --- HOME (Cánh tay thu gọn / di chuyển an toàn) ---
    public static double S1_HOME = 0.5;
    public static double S2_HOME = 0.75; // S2 phải >= 0.75
    public static double S3_HOME = 0;  // S3 chỉ 0.0 hoặc 0.1
    public static double S4_HOME = 0.375;
    public static double S5_HOME = 0.0;

    // --- SERVO 4: WRIST ROTATE (Hướng đầu quay gắp / thả) ---
    public static double S4_GRAB  = 0.0; // Quay ra trước gắp hộp
    public static double S4_STORE = 0.75; // Quay ra sau để cất khay thả

    // --- SERVO 5: COLUMN SELECTION (Quay sang kệ trái / kệ phải) ---
    public static double S5_LEFT  = 0.3; // Xoay quẹo sang khay gắp Trái
    public static double S5_RIGHT = 0.65; // Xoay quẹo sang khay gắp Phải

    // --- SERVO 1: ELEVATOR LIFT (Cao độ nâng hạ tầng 1 & tầng 2) ---
    public static double S1_ROW1           = 0.9; // Tầng 1 (Bottom)
    public static double S1_ROW2           = 0.35; // Tầng 2 (Top)
    public static double S1_LIFT_UP_OFFSET = 0.1; // Khoảng nhích nâng lên thêm sau gắp (ví dụ: 0.05)
    public static double S1_HIGH_STORE     = 0.2; // Cao độ thật cao để ko vướng khi xoay S4 ra sau
    public static double S1_STORE          = 0.0; // Cao độ xả hộp trùng vị trí cất khay robot

    // --- SERVO 2: TELESCOPIC EXTENSION (Khoảng vươn tay gắp) ---
    public static double S2_EXTEND       = 0.75;  // Độ vươn khi gắp (chung cả 2 tầng, mặc định vươn hẳn ra 1.0)
    public static double S2_STORE       = 0.75; // Độ vươn khi thả vào khay robot

    // --- SERVO 3: GRIPPER CLAMP (Chỉ hoạt động 0.0 và 0.1) ---
    public static double S3_OPEN   = 0.0; // Nhả kẹp
    public static double S3_CLOSED = 0.1; // Bóp kẹp giữ hộp

    // --- SERVO 5 STORE PRESETS: Swings to align with compartments 1-4 ---
    public static double S5_STORE1 = 0.0; // Khay cất 1
    public static double S5_STORE2 = 0.0; // Khay cất 2
    public static double S5_STORE3 = 0.0; // Khay cất 3
    public static double S5_STORE4 = 0.0; // Khay cất 4

    // --- EXTRA PRESETS FOR BONUS ---
    public static double S1_EXTRA_PICK = 0.0;
    public static double S2_EXTRA_PICK = 0.75;
    public static double S3_EXTRA_PICK = 0.0;
    public static double S4_EXTRA_PICK = 0.0;
    public static double S5_EXTRA_PICK = 0.0;

    public static double S1_EXTRA_DROP = 0.0;
    public static double S2_EXTRA_DROP = 0.75;
    public static double S3_EXTRA_DROP = 0.0;
    public static double S4_EXTRA_DROP = 0.0;
    public static double S5_EXTRA_DROP = 0.0;

    // ═══════════════════════════════════════════════════════════
    //  DROP SERVOS (3 servo nắp xả)
    // ═══════════════════════════════════════════════════════════
    public static double D1_CLOSED = 0.0;
    public static double D2_CLOSED = 0.0;
    public static double D3_CLOSED = 0.0;

    public static double D1_OPEN1 = 0.0;
    public static double D2_OPEN1 = 0.0;
    public static double D3_OPEN1 = 0.0;

    public static double D1_OPEN2 = 0.0;
    public static double D2_OPEN2 = 0.0;
    public static double D3_OPEN2 = 0.0;

    public static double D1_OPEN3 = 0.0;
    public static double D2_OPEN3 = 0.0;
    public static double D3_OPEN3 = 0.0;

    public static double D1_OPEN4 = 0.0;
    public static double D2_OPEN4 = 0.0;
    public static double D3_OPEN4 = 0.0;

    // ═══════════════════════════════════════════════════════════
    //  TIMING & CONSTANTS
    // ═══════════════════════════════════════════════════════════
    public static int PICK_SERVO_STEP_DELAY = 400;
    public static int DROP_SERVO_STEP_DELAY = 400;
    public static int GRIPPER_SETTLE_MS = 300;

    public static int    LL_PIPELINE       = 8;
    public static double TX_THRESHOLD      = 8.0;
    public static double TY_THRESHOLD      = 7.0;
    public static int    SAMPLE_COUNT      = 13;
    public static int    SAMPLE_DELAY      = 80;
    public static long   LL_STARTUP_DELAY  = 700;
    public static double MIN_CONFIDENCE    = 35.0;
    public static double MIN_TARGET_AREA   = 0.05;
    public static int    MIN_SLOT_VOTES    = 3;
    public static int    MIN_WIN_MARGIN    = 1;

    public static double POSE_XY_TOLERANCE_IN     = 0.45;
    public static double POSE_HEADING_TOLERANCE_DEG = 1.5;
    public static long   POSE_STABLE_TIME_MS      = 350;
    public static long   POSE_TIMEOUT_MS          = 2500;

    public static int DROP2_PCA_CHANNEL = 0;
    public static int DROP3_PCA_CHANNEL = 1;

    // Preview
    public static boolean TUNER_APPLY = false;
    /**
     * 0 = Pick: Individual Servo
     * 1 = Step 1: Prep Wrist swing S4 / S5
     * 2 = Step 2: Set Lift height S1
     * 3 = Step 3: Extend S2
     * 4 = Step 4: Grip S3 (0.1)
     * 5 = Step 5: S1 Lift slightly
     * 6 = Step 6: S2 Retract (HOME)
     * 7 = Step 7: HIGH_STORE raise + S4 rotate store
     * 8 = Step 8: S5 quẹo Compartment & S1/S2 store config (S3 open)
     * 9 = Drop: Individual Drop Servo
     * 10 = Drop: Closed
     * 11 = Drop: Open Compartment
     */
    public static int TUNER_MODE = 0;
    public static int TUNER_SELECTED_SERVO = 1;
    public static int TUNER_SELECTED_DROP_SERVO = 1;
    public static double TUNER_SERVO_POS = 0.0;
    /** 1=Left column, 2=Right column */
    public static int TUNER_SELECTED_COLUMN = 1;
    /** 1=Bottom row, 2=Top row */
    public static int TUNER_SELECTED_ROW = 1;
    /** 1–4 compartment */
    public static int TUNER_SELECTED_COMPARTMENT = 1;

    public static boolean TUNER_DRIVE_TO_SELECTED = false;
    public static int TUNER_TARGET_POSITION_INDEX = 0;

    // ═══════════════════════════════════════════════════════════
    //  HELPER CLASSES
    // ═══════════════════════════════════════════════════════════

    public static final class PickServoSet {
        public final double s1, s2, s3, s4, s5;

        public PickServoSet(double s1, double s2, double s3, double s4, double s5) {
            this.s1 = clamp(s1);
            this.s2 = clampS2(s2);
            this.s3 = clampS3(s3);
            this.s4 = clamp(s4);
            this.s5 = clamp(s5);
        }

        public double[] toArray() {
            return new double[]{s1, s2, s3, s4, s5};
        }
    }

    public static final class DropServoSet {
        public final double d1, d2, d3;

        public DropServoSet(double d1, double d2, double d3) {
            this.d1 = clamp(d1);
            this.d2 = clamp(d2);
            this.d3 = clamp(d3);
        }

        public double[] toArray() {
            return new double[]{d1, d2, d3};
        }
    }

    // ═══════════════════════════════════════════════════════════
    //  CENTRAL HARDWARE CLAMPS FOR SAFETY / LIMITS
    // ═══════════════════════════════════════════════════════════

    public static double clampS2(double v) {
        if (v < 0.75) return 0.75;
        if (v > 1.0)  return 1.0;
        return v;
    }

    public static double clampS3(double v) {
        if (v < 0.05) return 0.0;
        return 0.1;
    }

    // ═══════════════════════════════════════════════════════════
    //  PICK SERVO HELPERS (Backward-compatible / unused references)
    // ═══════════════════════════════════════════════════════════

    public static PickServoSet pickHome() {
        return new PickServoSet(S1_HOME, S2_HOME, S3_HOME, S4_HOME, S5_HOME);
    }

    public static PickServoSet pickReady(int slot) {
        // Backwards compatibility for compiled references
        return pickHome();
    }

    public static PickServoSet pickDown(int slot) {
        return pickHome();
    }

    public static PickServoSet pickGrab(int slot) {
        return pickHome();
    }

    public static PickServoSet pickRetract(int slot) {
        return pickHome();
    }

    public static PickServoSet storeCompartment(int compartment) {
        return pickHome();
    }

    public static PickServoSet pickExtra() {
        return new PickServoSet(S1_EXTRA_PICK, S2_EXTRA_PICK, S3_EXTRA_PICK, S4_EXTRA_PICK, S5_EXTRA_PICK);
    }

    public static PickServoSet dropExtra() {
        return new PickServoSet(S1_EXTRA_DROP, S2_EXTRA_DROP, S3_EXTRA_DROP, S4_EXTRA_DROP, S5_EXTRA_DROP);
    }

    // ═══════════════════════════════════════════════════════════
    //  DROP SERVO HELPERS
    // ═══════════════════════════════════════════════════════════

    public static DropServoSet dropClosed() {
        return new DropServoSet(D1_CLOSED, D2_CLOSED, D3_CLOSED);
    }

    public static DropServoSet dropOpen(int compartment) {
        switch (compartment) {
            case 1: return new DropServoSet(D1_OPEN1, D2_OPEN1, D3_OPEN1);
            case 2: return new DropServoSet(D1_OPEN2, D2_OPEN2, D3_OPEN2);
            case 3: return new DropServoSet(D1_OPEN3, D2_OPEN3, D3_OPEN3);
            case 4: return new DropServoSet(D1_OPEN4, D2_OPEN4, D3_OPEN4);
            default: return dropClosed();
        }
    }

    // ═══════════════════════════════════════════════════════════
    //  POSE HELPERS (10 positions)
    // ═══════════════════════════════════════════════════════════

    public static Pose poseStart() {
        return toPose(START_X, START_Y, START_H_DEG);
    }

    public static Pose poseShelf1() {
        return toPose(SHELF1_X, SHELF1_Y, SHELF1_H_DEG);
    }

    public static Pose poseShelf2() {
        return toPose(SHELF2_X, SHELF2_Y, SHELF2_H_DEG);
    }

    public static Pose poseShelf3() {
        return toPose(SHELF3_X, SHELF3_Y, SHELF3_H_DEG);
    }

    public static Pose poseDrop1() {
        return toPose(DROP1_X, DROP1_Y, DROP1_H_DEG);
    }

    public static Pose poseDrop2() {
        return toPose(DROP2_X, DROP2_Y, DROP2_H_DEG);
    }

    public static Pose poseDrop3() {
        return toPose(DROP3_X, DROP3_Y, DROP3_H_DEG);
    }

    public static Pose poseDrop4() {
        return toPose(DROP4_X, DROP4_Y, DROP4_H_DEG);
    }

    public static Pose poseExtraPick() {
        return toPose(EXTRA_PICK_X, EXTRA_PICK_Y, EXTRA_PICK_H_DEG);
    }

    public static Pose poseExtraDrop() {
        return toPose(EXTRA_DROP_X, EXTRA_DROP_Y, EXTRA_DROP_H_DEG);
    }

    public static Pose poseByIndex(int index) {
        switch (index) {
            case 0: return poseStart();
            case 1: return poseShelf1();
            case 2: return poseShelf2();
            case 3: return poseShelf3();
            case 4: return poseDrop1();
            case 5: return poseDrop2();
            case 6: return poseDrop3();
            case 7: return poseDrop4();
            case 8: return poseExtraPick();
            case 9: return poseExtraDrop();
            default: return poseStart();
        }
    }

    public static String poseName(int index) {
        switch (index) {
            case 0: return "P0 START";
            case 1: return "P1 KHO HQ 1";
            case 2: return "P2 KHO HQ 2";
            case 3: return "P3 KHO HQ 3";
            case 4: return "P4 FACTORY SS";
            case 5: return "P5 FACTORY HM";
            case 6: return "P6 FACTORY AM";
            case 7: return "P7 FACTORY FOX";
            case 8: return "P8 EX PICK";
            case 9: return "P9 EX DROP";
            default: return "P? UNKNOWN";
        }
    }

    public static final int POSITION_COUNT = 10;

    public static Pose poseDropForBox(String boxType) {
        if ("box01".equals(boxType)) return poseDrop1();
        if ("box02".equals(boxType)) return poseDrop2();
        if ("box03".equals(boxType)) return poseDrop3();
        if ("box04".equals(boxType)) return poseDrop4();
        return poseDrop1();
    }

    public static boolean isDropEnabled() {
        return ENABLE_DROP_MECHANISM;
    }

    private static Pose toPose(double x, double y, double headingDeg) {
        return new Pose(x, y, Math.toRadians(headingDeg));
    }

    private static double clamp(double v) {
        if (v < 0.0) return 0.0;
        if (v > 1.0) return 1.0;
        return v;
    }
}
