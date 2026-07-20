package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

/**
 * Tất cả thông số tune cho "12 Box Auto" — chỉnh trên FTControl Panels.
 *
 * Cách dùng:
 * 1. Mở Panels trên điện thoại / máy tính.
 * 2. Chọn class {@code BoxAutoPanels}.
 * 3. Chỉnh giá trị → lưu.
 * 4. Chạy Autonomous "12 Box Auto - Ivy" (đọc tự động).
 *
 * Tune servo trực tiếp trên robot: chạy TeleOp "Servo Panels Tuner".
 */
@Configurable
public class BoxAutoPanels {

    public static void refresh() {
        PanelsConfigurables.INSTANCE.refreshClass(BoxAutoPanels.class);
    }

    /**
     * Bật {@code true} khi đã lắp cơ cấu thả (3 servo).
     * Tạm {@code false}: auto chỉ gắp + bỏ vào robot, không lái thả / không dùng drop1–3.
     */
    public static boolean ENABLE_DROP_MECHANISM = false;

    // ═══════════════════════════════════════════════════════════
    //  TÊN THIẾT BỊ (Robot Config)
    // ═══════════════════════════════════════════════════════════
    public static String NAME_LIMELIGHT = "limelight";
    public static String NAME_S1 = "s1";
    public static String NAME_S2 = "s2";
    public static String NAME_S3 = "s3";
    public static String NAME_S4 = "s4";
    public static String NAME_S5 = "s5";

    // Cơ cấu THẢ — chỉ dùng khi ENABLE_DROP_MECHANISM = true
    public static String NAME_DROP1 = "drop1";
    public static String NAME_DROP2 = "drop2";
    public static String NAME_DROP3 = "drop3";
    public static String NAME_PCA9685 = "pca9685";

    // ═══════════════════════════════════════════════════════════
    //  CƠ CẤP GẮP (5 servo) — HOME
    // ═══════════════════════════════════════════════════════════
    public static double HOME_S1 = 0.50;
    public static double HOME_S2 = 0.50;
    public static double HOME_S3 = 0.50;
    public static double HOME_S4 = 0.50;
    public static double HOME_S5 = 0.50;

    // ═══════════════════════════════════════════════════════════
    //  CƠ CẤP GẮP (5 servo) — 4 Ô KỆ + BỎ VÀO ROBOT
    // ═══════════════════════════════════════════════════════════
    // Trên-Trái
    public static double TL_S1 = 0.10;
    public static double TL_S2 = 0.20;
    public static double TL_S3 = 0.30;
    public static double TL_S4 = 0.40;
    public static double TL_S5 = 0.50;
    // Trên-Phải
    public static double TR_S1 = 0.20;
    public static double TR_S2 = 0.30;
    public static double TR_S3 = 0.40;
    public static double TR_S4 = 0.50;
    public static double TR_S5 = 0.60;
    // Dưới-Trái
    public static double BL_S1 = 0.70;
    public static double BL_S2 = 0.80;
    public static double BL_S3 = 0.90;
    public static double BL_S4 = 1.00;
    public static double BL_S5 = 0.10;
    // Dưới-Phải
    public static double BR_S1 = 0.50;
    public static double BR_S2 = 0.60;
    public static double BR_S3 = 0.70;
    public static double BR_S4 = 0.80;
    public static double BR_S5 = 0.90;

    // 4 điểm bỏ vào trong robot (cơ cấu gắp tự phân loại theo loại hộp)
    // DEP1 = ngăn box01, DEP2 = ngăn box02, DEP3 = ngăn box03, DEP4 = ngăn box04
    public static double DEP1_S1 = 0.45;
    public static double DEP1_S2 = 0.55;
    public static double DEP1_S3 = 0.50;
    public static double DEP1_S4 = 0.50;
    public static double DEP1_S5 = 0.50;

    public static double DEP2_S1 = 0.45;
    public static double DEP2_S2 = 0.55;
    public static double DEP2_S3 = 0.50;
    public static double DEP2_S4 = 0.50;
    public static double DEP2_S5 = 0.50;

    public static double DEP3_S1 = 0.45;
    public static double DEP3_S2 = 0.55;
    public static double DEP3_S3 = 0.50;
    public static double DEP3_S4 = 0.50;
    public static double DEP3_S5 = 0.50;

    public static double DEP4_S1 = 0.45;
    public static double DEP4_S2 = 0.55;
    public static double DEP4_S3 = 0.50;
    public static double DEP4_S4 = 0.50;
    public static double DEP4_S5 = 0.50;

    /** ms chờ giữa mỗi khớp — cơ cấu gắp */
    public static int PICK_SERVO_STEP_DELAY = 400;

    // ═══════════════════════════════════════════════════════════
    //  CƠ CẤU THẢ — bỏ qua khi ENABLE_DROP_MECHANISM = false
    // ═══════════════════════════════════════════════════════════
    public static double DROP_HOME_D1 = 0.50;
    public static double DROP_HOME_D2 = 0.50;
    public static double DROP_HOME_D3 = 0.50;

    /** Tư thế thả hộp khi robot đứng tại vùng thả trên sân */
    public static double DROP_RELEASE_D1 = 0.35;
    public static double DROP_RELEASE_D2 = 0.15;
    public static double DROP_RELEASE_D3 = 0.50;

    /** ms chờ giữa mỗi khớp — cơ cấu thả */
    public static int DROP_SERVO_STEP_DELAY = 400;

    // ═══════════════════════════════════════════════════════════
    //  LIMELIGHT (pipeline 8, detector box01–box04)
    // ═══════════════════════════════════════════════════════════
    public static int LL_PIPELINE = 8;
    public static double TX_THRESHOLD = 8.0;
    public static double TY_THRESHOLD = 7.0;
    public static int SAMPLE_COUNT = 13;
    public static int SAMPLE_DELAY = 80;
    public static long LL_STARTUP_DELAY = 700;
    public static double MIN_CONFIDENCE = 35.0;
    public static double MIN_TARGET_AREA = 0.05;
    public static int MIN_SLOT_VOTES = 3;
    public static int MIN_WIN_MARGIN = 1;

    // ═══════════════════════════════════════════════════════════
    //  DI CHUYỂN — độ chính xác khi dừng
    // ═══════════════════════════════════════════════════════════
    public static double POSE_XY_TOLERANCE_IN = 0.45;
    public static double POSE_HEADING_TOLERANCE_DEG = 1.5;
    public static long POSE_STABLE_TIME_MS = 350;
    public static long POSE_TIMEOUT_MS = 2500;

    // ═══════════════════════════════════════════════════════════
    //  8 ĐIỂM — x, y (inch), heading (độ)
    //  (0,0) = góc dưới-trái sân Pedro
    // ═══════════════════════════════════════════════════════════
    // Điểm 1: xuất phát
    public static double START_X = 0;
    public static double START_Y = 0;
    public static double START_H_DEG = 0;

    // Điểm 2–4: 3 kệ
    public static double SHELF1_X = 48;
    public static double SHELF1_Y = 48;
    public static double SHELF1_H_DEG = 90;

    public static double SHELF2_X = 48;
    public static double SHELF2_Y = 96;
    public static double SHELF2_H_DEG = 180;

    public static double SHELF3_X = 0;
    public static double SHELF3_Y = 96;
    public static double SHELF3_H_DEG = 270;

    // Điểm 5–8: 4 vùng thả — chỉ khi ENABLE_DROP_MECHANISM = true
    public static double DROP01_X = 120;
    public static double DROP01_Y = 24;
    public static double DROP01_H_DEG = 0;

    public static double DROP02_X = 120;
    public static double DROP02_Y = 72;
    public static double DROP02_H_DEG = 0;

    public static double DROP03_X = 120;
    public static double DROP03_Y = 120;
    public static double DROP03_H_DEG = 0;

    public static double DROP04_X = 72;
    public static double DROP04_Y = 120;
    public static double DROP04_H_DEG = 90;

    // ═══════════════════════════════════════════════════════════
    //  SERVO PREVIEW — dùng với TeleOp "Servo Panels Tuner"
    // ═══════════════════════════════════════════════════════════
    public static boolean TUNER_APPLY = false;
    /**
     * 0=gắp: 1 servo (s1–s5)
     * 1=gắp: ô kệ
     * 2=gắp: home
     * 3=gắp: bỏ vào robot
     * 4=thả: 1 servo | 5=thả: home | 6=thả: ra sân (cần ENABLE_DROP_MECHANISM)
     */
    public static int TUNER_MODE = 0;
    /** Cơ cấp gắp: servo 1–5 */
    public static int TUNER_SELECTED_SERVO = 1;
    /** Cơ cấu thả: servo 1–3 */
    public static int TUNER_SELECTED_DROP_SERVO = 1;
    public static double TUNER_SERVO_POS = 0.50;
    /** 0=TL, 1=TR, 2=BL, 3=BR */
    public static int TUNER_SELECTED_SLOT = 0;

    // ═══════════════════════════════════════════════════════════
    //  Helpers — test.java gọi các hàm này (đọc giá trị Panels mới nhất)
    // ═══════════════════════════════════════════════════════════

    public static boolean isDropEnabled() {
        return ENABLE_DROP_MECHANISM;
    }

    public static final class PickServoSet {
        public final double s1, s2, s3, s4, s5;

        public PickServoSet(double s1, double s2, double s3, double s4, double s5) {
            this.s1 = clamp(s1);
            this.s2 = clamp(s2);
            this.s3 = clamp(s3);
            this.s4 = clamp(s4);
            this.s5 = clamp(s5);
        }
    }

    public static final class DropServoSet {
        public final double d1, d2, d3;

        public DropServoSet(double d1, double d2, double d3) {
            this.d1 = clamp(d1);
            this.d2 = clamp(d2);
            this.d3 = clamp(d3);
        }
    }

    public static PickServoSet pickHome() {
        return new PickServoSet(HOME_S1, HOME_S2, HOME_S3, HOME_S4, HOME_S5);
    }

    /** slot: 0=TL, 1=TR, 2=BL, 3=BR */
    public static PickServoSet pickSlot(int slot) {
        switch (slot) {
            case 0: return new PickServoSet(TL_S1, TL_S2, TL_S3, TL_S4, TL_S5);
            case 1: return new PickServoSet(TR_S1, TR_S2, TR_S3, TR_S4, TR_S5);
            case 2: return new PickServoSet(BL_S1, BL_S2, BL_S3, BL_S4, BL_S5);
            case 3: return new PickServoSet(BR_S1, BR_S2, BR_S3, BR_S4, BR_S5);
            default: return pickHome();
        }
    }

    public static PickServoSet depositZone(int zone) {
        switch (zone) {
            case 1: return new PickServoSet(DEP1_S1, DEP1_S2, DEP1_S3, DEP1_S4, DEP1_S5);
            case 2: return new PickServoSet(DEP2_S1, DEP2_S2, DEP2_S3, DEP2_S4, DEP2_S5);
            case 3: return new PickServoSet(DEP3_S1, DEP3_S2, DEP3_S3, DEP3_S4, DEP3_S5);
            case 4: return new PickServoSet(DEP4_S1, DEP4_S2, DEP4_S3, DEP4_S4, DEP4_S5);
            default: return pickHome();
        }
    }

    /** Backward-compatible helper: defaults to zone 1 (box01). */
    public static PickServoSet depositInRobot() {
        return depositZone(1);
    }

    public static DropServoSet dropHome() {
        return new DropServoSet(DROP_HOME_D1, DROP_HOME_D2, DROP_HOME_D3);
    }

    public static DropServoSet dropReleaseOnField() {
        return new DropServoSet(DROP_RELEASE_D1, DROP_RELEASE_D2, DROP_RELEASE_D3);
    }

    /** PCA9685 channel mapping for drop servos. */
    public static int DROP2_PCA_CHANNEL = 0;
    public static int DROP3_PCA_CHANNEL = 1;

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

    public static Pose poseDropBox01() {
        return toPose(DROP01_X, DROP01_Y, DROP01_H_DEG);
    }

    public static Pose poseDropBox02() {
        return toPose(DROP02_X, DROP02_Y, DROP02_H_DEG);
    }

    public static Pose poseDropBox03() {
        return toPose(DROP03_X, DROP03_Y, DROP03_H_DEG);
    }

    public static Pose poseDropBox04() {
        return toPose(DROP04_X, DROP04_Y, DROP04_H_DEG);
    }

    public static Pose poseDropForBox(String boxType) {
        if ("box01".equals(boxType)) return poseDropBox01();
        if ("box02".equals(boxType)) return poseDropBox02();
        if ("box03".equals(boxType)) return poseDropBox03();
        if ("box04".equals(boxType)) return poseDropBox04();
        return poseDropBox01();
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
