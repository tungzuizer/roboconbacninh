package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

/**
 * =========================================================================
 *                   HƯỚNG DẪN TUNING TRÊN FTCONTROL PANELS
 * =========================================================================
 * PHẦN 1: TUNE TỌA ĐỘ DI CHUYỂN (P0 -> P14 bao gồm P8, P13, P14 cho Viewpoint YOLO)
 *   1. Chạy OpMode "Servo Panels Tuner" hoặc "Pedro 15-Position Tuner".
 *   2. Chọn vị trí qua TUNER_TARGET_POSITION_INDEX và bật TUNER_DRIVE_TO_SELECTED = true.
 *   3. Điều chỉnh tọa độ X, Y, H_DEG của vị trí đó trực tiếp bằng slider trên Panels.
 *
 * PHẦN 2: TUNE SERVO GẮP HỘP (Chu trình gắp & thả trực tiếp 2 ngàm S3/S5)
 *   1. Chạy OpMode "Servo Panels Tuner" hoặc "Manual Tuner".
 *   2. Chọn vị trí mục tiêu di chuyển qua Panels (ví dụ: Shelf 3) và cho robot lái tới đó.
 *   3. Lần lượt bấm Mode để chạy qua từng bước cơ học:
 *      Mode 1: S4 xoay ra trước gắp.
 *      Mode 2: S1 hạ xuống Row 1 hoặc Row 2 & S2 vươn tay tiếp cận hộp trực tiếp (Đã bỏ qua bước vươn an toàn dư thừa).
 *      Mode 3: Kẹp cả 2 ngàm S3 (trái) & S5 (phải).
 *      Mode 4: S2 thu tay về HOME ngay lập tức kéo 2 hộp ra khỏi kệ.
 *      Mode 5: S1 nâng lên một chút (Lift Up Offset).
 *      Mode 6: S1 nâng lên vị trí cao nhất (S1_HIGHEST).
 *      Mode 7: S4 xoay ngược ra sau, S2 vẫn thu gọn HOME.
 *      Mode 8: S2 vươn ra sau khi S4 đã lật xong.
 *      Mode 9: S1 hạ xuống vị trí thả trực tiếp (S1_DROP_LOW).
 *      Mode 10: S3 mở nhả hộp bên trái.
 *      Mode 11: S5 mở nhả hộp bên phải.
 *      Mode 12: Tất cả Servo về vị trí HOME an toàn.
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
    public static String NAME_S1 = "s1";
    public static String NAME_S2 = "s2";
    public static String NAME_S3 = "s3";
    public static String NAME_S4 = "s4";
    public static String NAME_S5 = "s5"; // Mở rộng bụng chứa hộp

    public static String NAME_DROP1 = "drop1";  // Nắp xả Trụ 1
    public static String NAME_PCA9685 = "pca9685";  // Nắp xả Trụ 2 dùng PCA

    // ═══════════════════════════════════════════════════════════
    //  P0–P14: 15 TUNABLE POSITIONS TRÊN PANELS UI
    // ═══════════════════════════════════════════════════════════
    public static double P0_START_X     = 0.0;
    public static double P0_START_Y     = 0.0;
    public static double P0_START_H_DEG = 0.0;
    public static double START_X = 0.0; public static double START_Y = 0.0; public static double START_H_DEG = 0.0;

    public static double P1_SHELF1_X     = 1.0;
    public static double P1_SHELF1_Y     = 20.5;
    public static double P1_SHELF1_H_DEG = 0.0;
    public static double SHELF1_X = 0.0; public static double SHELF1_Y = 20.0; public static double SHELF1_H_DEG = 0.0;

    public static double P2_SHELF2_X     = 26.5;
    public static double P2_SHELF2_Y     = 20.5;
    public static double P2_SHELF2_H_DEG = 0.0;
    public static double SHELF2_X = 25.5; public static double SHELF2_Y = 20.0; public static double SHELF2_H_DEG = 0.0;

    public static double P3_SHELF3_X     = 52.0;
    public static double P3_SHELF3_Y     = 20.5;
    public static double P3_SHELF3_H_DEG = 0.0;
    public static double SHELF3_X = 51.0; public static double SHELF3_Y = 20.0; public static double SHELF3_H_DEG = 0.0;

    public static double P4_DROP1_X     = 1.0;
    public static double P4_DROP1_Y     = -25.0;
    public static double P4_DROP1_H_DEG = 0.0;
    public static double DROP1_X = 7.0;
    public static double DROP1_Y = -21.0;
    public static double DROP1_H_DEG = 0.0;

    public static double P5_DROP2_X     = 12.0;
    public static double P5_DROP2_Y     = -24.0;
    public static double P5_DROP2_H_DEG = 0.0;
    public static double DROP2_X = 16.0; public static double DROP2_Y = -21.0; public static double DROP2_H_DEG = 0.0;

    public static double P6_DROP3_X     = 38.0;
    public static double P6_DROP3_Y     = -23.0;
    public static double P6_DROP3_H_DEG = 0.0;
    public static double DROP3_X = 38.0; public static double DROP3_Y = -21.0; public static double DROP3_H_DEG = 0.0;

    public static double P7_DROP4_X     = 50.0;
    public static double P7_DROP4_Y     = -21.0;
    public static double P7_DROP4_H_DEG = 0.0;
    public static double DROP4_X = 48.0; public static double DROP4_Y = -21.0; public static double DROP4_H_DEG = 0.0;
    
    // ── 3 ĐIỂM VIEWPOINT QUÉT YOLO KỆ 1, KỆ 2, KỆ 3 ──
    public static double P8_VIEWPOINT1_X     = 0.0; // P8 (Viewpoint Kệ 1)
    public static double P8_VIEWPOINT1_Y     = 10.0; // Cách điểm gắp Kệ 1 (Y=20) qua bên phải 10 (Y=10)
    public static double P8_VIEWPOINT1_H_DEG = 0.0;
    public static double EXTRA_PICK_X = 0.0; public static double EXTRA_PICK_Y = 0.0; public static double EXTRA_PICK_H_DEG = 0.0;

    // Dodge Drop 1..4 (P9..P12)
    public static double DODGE1_X     = 1.0;
    public static double DODGE1_Y     = -10.0;
    public static double DODGE1_H_DEG = 0.0;

    public static double DODGE2_X     = 12.0;
    public static double DODGE2_Y     = -10.0;
    public static double DODGE2_H_DEG = 0.0;

    public static double DODGE3_X     = 38.0;
    public static double DODGE3_Y     = -10.0;
    public static double DODGE3_H_DEG = 0.0;
    
    public static double DODGE4_X     = 50.0;
    public static double DODGE4_Y     = -10.0;
    public static double DODGE4_H_DEG = 0.0;

    public static double P13_VIEWPOINT2_X     = 25.5; // P13 (Viewpoint Kệ 2)
    public static double P13_VIEWPOINT2_Y     = 10.0; // Cách điểm gắp Kệ 2 (Y=20) qua bên phải 10 (Y=10)
    public static double P13_VIEWPOINT2_H_DEG = 0.0;
    public static double VIEWPOINT2_X = 25.5; public static double VIEWPOINT2_Y = 10.0; public static double VIEWPOINT2_H_DEG = 0.0;

    public static double P14_VIEWPOINT3_X     = 51.0; // P14 (Viewpoint Kệ 3)
    public static double P14_VIEWPOINT3_Y     = 10.0; // Cách điểm gắp Kệ 3 (Y=20) qua bên phải 10 (Y=10)
    public static double P14_VIEWPOINT3_H_DEG = 0.0;
    public static double VIEWPOINT3_X = 51.0; public static double VIEWPOINT3_Y = 10.0; public static double VIEWPOINT3_H_DEG = 0.0;

    public static final int POSITION_COUNT = 15;

    public static Pose poseStart() {
        return getTunablePose(0);
    }

    public static Pose poseByIndex(int index) {
        return getTunablePose(index);
    }

    public static String poseName(int index) {
        return getTunablePoseName(index);
    }

    // ═══════════════════════════════════════════════════════════
    //  PICK SERVOS — THÔNG SỐ TUNE CHU TRÌNH GẮP (S1, S2, S3, S4)
    // ═══════════════════════════════════════════════════════════

    // --- HOME (Cánh tay thu gọn / di chuyển an toàn) ---
    public static double S1_HOME = 0.5;
    public static double S2_HOME = 0.51; // S2 phải >= 0.75
    public static double S3_HOME = 0.2;  // S3 chỉ 0.0 hoặc 0.1
    public static double S4_HOME = 0.35;

    // --- SERVO 4: WRIST ROTATE (Hướng đầu quay gắp / thả) ---
    public static double S4_GRAB  = 0;  // Quay ra trước gắp hộp
    public static double S4_STORE = 0.65; // Quay ra sau để cất khay thả
    /** Tốc độ tối đa S4 di chuyển mỗi frame (~50ms). 0.003 → xoay hoàn toàn ~2.0 giây (rất chậm, mượt) */
    public static double S4_SPEED_STEP = 0.02;

    // --- SERVO 1: ELEVATOR LIFT (Cao độ nâng hạ tầng 1 & tầng 2) ---
    public static double S1_ROW1           = 0.88;  // Tầng 1 (Bottom)
    public static double S1_ROW2           = 0.32; // Tầng 2 (Top shelf - Gắp chính)
    public static double S1_LIFT_UP_OFFSET = 0.2;  // Khoảng nhích nâng lên thêm sau gắp (Bước 6)
    public static double S1_HIGHEST        = 0.2;  // Vị trí nâng cao nhất để lật S4 an toàn (Bước 8)
    public static double S1_DROP_LOW       = 1;  // Cao độ hạ xuống để thả trực tiếp bằng ngàm S3/S5

    // --- SERVO 2: TELESCOPIC EXTENSION (Khoảng vươn tay gắp) ---
    public static double S2_EXTEND         = 0.88;  // Độ vươn khi gắp phía trước (Bước 4)
    public static double S2_STORE_EXTEND   = 0.92;  // Độ vươn khi thả hất ra phía sau lưng (Bước 10)

    // --- SERVO 3: GRIPPER CLAMP (Chỉ hoạt động 0.0 và 0.1) ---
    public static double S3_OPEN   = 0.17; // Nhả kẹp (Bước 12)
    public static double S3_CLOSED = 0.08; // Bóp kẹp giữ hộp (Bước 5)

    // --- EXTRA PRESETS FOR BONUS ---
    public static double S1_EXTRA_PICK = 0.0;
    public static double S2_EXTRA_PICK = 0.75;
    public static double S3_EXTRA_PICK = 0.0;
    public static double S4_EXTRA_PICK = 0.0;

    public static double S1_EXTRA_DROP = 0.0;
    public static double S2_EXTRA_DROP = 0.75;
    public static double S3_EXTRA_DROP = 0.0;
    public static double S4_EXTRA_DROP = 0.0;

    // ═══════════════════════════════════════════════════════════
    //  DROP SERVOS (Nắp xả & Cơ cấu mỡ) 
    // ═══════════════════════════════════════════════════════════
    // S5: Mở rộng cỡ cấu bụng đầu trận
    public static double S5_CLOSED = 0.22;
    public static double S5_OPEN   = 0.12;

    // Các nắp thả gầm
    public static double D1_CLOSED = 0.0;
    public static double D2_CLOSED = 0.0; // PCA Channel 0

    // Khi lệnh mở được gọi
    public static double D1_OPEN1 = 1.0;
    public static double D2_OPEN1 = 1.0; 

    // Các biến phụ không dùng tạm xoá hoặc để không
    public static double D3_CLOSED = 0.0;
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
    //  TIMING & CONSTANTS (Đảm bảo hành trình cơ học hoàn chỉnh)
    // ═══════════════════════════════════════════════════════════
    public static int PICK_SERVO_STEP_DELAY = 450; // 450ms cho servo vươn/thu/nâng hạ đủ thời gian chạy kịch hành trình
    public static int DROP_SERVO_STEP_DELAY = 400;
    public static int GRIPPER_SETTLE_MS = 350;
    public static double MIN_TARGET_AREA   = 0.05;
    public static int    MIN_SLOT_VOTES    = 3;
    public static int    MIN_WIN_MARGIN    = 1;

    public static double POSE_XY_TOLERANCE_IN     = 0.45;
    public static double POSE_HEADING_TOLERANCE_DEG = 1.5;
    public static long   POSE_STABLE_TIME_MS      = 350;
    public static long   POSE_TIMEOUT_MS          = 2500;

    // Kênh cắm Drop 2 trên board PCA9685
    public static int DROP2_PCA_CHANNEL = 0;
    public static int DROP3_PCA_CHANNEL = 1;

    // Preview
    public static boolean TUNER_APPLY = false;
    /**
     * 0 = Tune lẻ Pick Servo (S1-S5)
     * 1 = S4 quay ra trước để gắp (S4_GRAB)
     * 2 = S1 hạ tầng & S2 vươn tiếp cận trực tiếp (setElevatorLevel)
     * 3 = Kẹp đôi S3 & S5 (clampBothS3S5)
     * 4 = S2 thu về HOME ngay lập tức (retractS2)
     * 5 = S1 nhích nâng lên (liftUpS1)
     * 6 = S1 nâng HIGHEST (s1Highest)
     * 7 = S4 lật ra sau, S2 vẫn HOME (s4FlipBackOnlyHigh)
     * 8 = S2 vươn ra sau khi S4 đã lật xong (s2ExtendHighForDrop)
     * 9 = S1 hạ thả trực tiếp (prepareS3/S5LowDrop)
     * 10 = Mở S3 thả hộp trái (openS3Drop)
     * 11 = Mở S5 thả hộp phải (openS5Drop)
     * 12 = All HOME (allHome)
     * 13 = Tune lẻ Drop Servo (D1-D3)
     */
    public static int TUNER_MODE = 0;
    public static int TUNER_SELECTED_SERVO = 1;
    public static int TUNER_SELECTED_DROP_SERVO = 1;
    public static double TUNER_SERVO_POS = 0.0;
    /** 1=Bottom row, 2=Top row (Default: 2) */
    public static int TUNER_SELECTED_ROW = 2;
    /** 1=Post 1, 2=Post 2 */
    public static int TUNER_SELECTED_POST = 1;

    public static boolean TUNER_DRIVE_TO_SELECTED = false;
    public static int TUNER_TARGET_POSITION_INDEX = 0;

    // ═══════════════════════════════════════════════════════════
    //  HELPER CLASSES
    // ═══════════════════════════════════════════════════════════

    public static final class PickServoSet {
        public final double s1, s2, s3, s4;

        public double s5;

        public PickServoSet(double s1, double s2, double s3, double s4, double s5) {
            this.s1 = clamp(s1);
            this.s2 = clampS2(s2);
            this.s3 = clampS3(s3);
            this.s4 = clamp(s4);
            this.s5 = clamp(s5); // Assumes generic clamping for s5
        }

        public double[] toArray() {
            return new double[]{s1, s2, s3, s4};
        }
    }

    public static double clamp(double v) {
        return Math.max(0.0, Math.min(1.0, v));
    }

    public static double clampS2(double s2) {
        return Math.max(0.0, Math.min(1.0, s2));
    }

    public static double clampS3(double s3) {
        return Math.max(0.0, Math.min(1.0, s3));
    }

    public static double clampS4(double s4) {
        return Math.max(0.0, Math.min(1.0, s4));
    }

    public static Pose getTunablePose(int index) {
        switch (index) {
            case 0: return new Pose(P0_START_X, P0_START_Y, Math.toRadians(P0_START_H_DEG));
            case 1: return new Pose(P1_SHELF1_X, P1_SHELF1_Y, Math.toRadians(P1_SHELF1_H_DEG));
            case 2: return new Pose(P2_SHELF2_X, P2_SHELF2_Y, Math.toRadians(P2_SHELF2_H_DEG));
            case 3: return new Pose(P3_SHELF3_X, P3_SHELF3_Y, Math.toRadians(P3_SHELF3_H_DEG));
            case 4: return new Pose(P4_DROP1_X, P4_DROP1_Y, Math.toRadians(P4_DROP1_H_DEG));
            case 5: return new Pose(P5_DROP2_X, P5_DROP2_Y, Math.toRadians(P5_DROP2_H_DEG));
            case 6: return new Pose(P6_DROP3_X, P6_DROP3_Y, Math.toRadians(P6_DROP3_H_DEG));
            case 7: return new Pose(P7_DROP4_X, P7_DROP4_Y, Math.toRadians(P7_DROP4_H_DEG));
            case 8: return new Pose(P8_VIEWPOINT1_X, P8_VIEWPOINT1_Y, Math.toRadians(P8_VIEWPOINT1_H_DEG));
            case 9: return new Pose(DODGE1_X, DODGE1_Y, Math.toRadians(DODGE1_H_DEG));
            case 10: return new Pose(DODGE2_X, DODGE2_Y, Math.toRadians(DODGE2_H_DEG));
            case 11: return new Pose(DODGE3_X, DODGE3_Y, Math.toRadians(DODGE3_H_DEG));
            case 12: return new Pose(DODGE4_X, DODGE4_Y, Math.toRadians(DODGE4_H_DEG));
            case 13: return new Pose(P13_VIEWPOINT2_X, P13_VIEWPOINT2_Y, Math.toRadians(P13_VIEWPOINT2_H_DEG));
            case 14: return new Pose(P14_VIEWPOINT3_X, P14_VIEWPOINT3_Y, Math.toRadians(P14_VIEWPOINT3_H_DEG));
            default: return new Pose(P0_START_X, P0_START_Y, Math.toRadians(P0_START_H_DEG));
        }
    }

    public static String getTunablePoseName(int index) {
        switch (index) {
            case 0: return "P0 (Start)";
            case 1: return "P1 (Shelf 1)";
            case 2: return "P2 (Shelf 2)";
            case 3: return "P3 (Shelf 3)";
            case 4: return "P4 (Drop 1)";
            case 5: return "P5 (Drop 2)";
            case 6: return "P6 (Drop 3)";
            case 7: return "P7 (Drop 4)";
            case 8: return "P8 (Viewpoint 1 - YOLO Kệ 1)";
            case 9: return "P9 (Dodge Drop 1)";
            case 10: return "P10 (Dodge Drop 2)";
            case 11: return "P11 (Dodge Drop 3)";
            case 12: return "P12 (Dodge Drop 4)";
            case 13: return "P13 (Viewpoint 2 - YOLO Kệ 2)";
            case 14: return "P14 (Viewpoint 3 - YOLO Kệ 3)";
            default: return "P0 (Start)";
        }
    }

    // ── STEP BY STEP EXACT CHOREOGRAPHY ──
    public static PickServoSet moveS4Grab() {
        return new PickServoSet(S1_HOME, S2_HOME, S3_OPEN, S4_GRAB, S5_OPEN);
    }

    public static PickServoSet extendS2SafeBeforeLower() {
        return new PickServoSet(S1_HOME, S2_EXTEND, S3_OPEN, S4_GRAB, S5_OPEN);
    }
    
    /** Step 2: Hạ S1 xuống cao độ Pallet / Kệ (S2 VẪN Ở HOME HOÀN TOÀN ĐỂ KHÔNG ĐÂM VÁCH) */
    public static PickServoSet setElevatorLevel(int row) {
        double s1 = (row == 1) ? S1_ROW1 : S1_ROW2;
        return new PickServoSet(s1, S2_HOME, S3_OPEN, S4_GRAB, S5_OPEN);
    }

    /** Step 3: Vươn S2 ra tiếp cận hộp SAU KHI S1 đã hạ xong tầng kệ */
    public static PickServoSet extendS2(int row) {
        double s1 = (row == 1) ? S1_ROW1 : S1_ROW2;
        return new PickServoSet(s1, S2_EXTEND, S3_OPEN, S4_GRAB, S5_OPEN);
    }

    public static PickServoSet clampS3(int row) {
        double s1 = (row == 1) ? S1_ROW1 : S1_ROW2;
        return new PickServoSet(s1, S2_EXTEND, S3_CLOSED, S4_GRAB, S5_OPEN);
    }

    public static PickServoSet liftUpS1(int row) {
        double s1 = (row == 1) ? S1_ROW1 : S1_ROW2;
        return new PickServoSet(s1 - S1_LIFT_UP_OFFSET, S2_HOME, S3_CLOSED, S4_GRAB, S5_CLOSED);
    }

    public static PickServoSet retractS2(int row) {
        double s1 = (row == 1) ? S1_ROW1 : S1_ROW2;
        return new PickServoSet(s1 - S1_LIFT_UP_OFFSET, S2_HOME, S3_CLOSED, S4_GRAB, S5_CLOSED);
    }

    public static PickServoSet s1Highest() {
        return new PickServoSet(S1_HIGHEST, S2_HOME, S3_CLOSED, S4_GRAB, S5_CLOSED);
    }
    
    public static PickServoSet clampBothS3S5(int row) {
        double s1 = (row == 1) ? S1_ROW1 : S1_ROW2;
        return new PickServoSet(s1, S2_EXTEND, S3_CLOSED, S4_GRAB, S5_CLOSED);
    }

    /** S4 lật ra sau trước (khi S2 vẫn đang thu gọn S2_HOME ở cao độ HIGHEST) */
    public static PickServoSet s4FlipBackOnlyHigh() {
        return new PickServoSet(S1_HIGHEST, S2_HOME, S3_CLOSED, S4_STORE, S5_CLOSED);
    }

    /** S2 vươn ra sau khi S4 đã lật xong ra sau ở cao độ HIGHEST */
    public static PickServoSet s2ExtendHighForDrop() {
        return new PickServoSet(S1_HIGHEST, S2_STORE_EXTEND, S3_CLOSED, S4_STORE, S5_CLOSED);
    }

    public static PickServoSet prepareS3LowDrop() {
        return new PickServoSet(S1_DROP_LOW, S2_STORE_EXTEND, S3_CLOSED, S4_STORE, S5_CLOSED);
    }

    public static PickServoSet prepareS5LowDrop() {
        return new PickServoSet(S1_DROP_LOW, S2_STORE_EXTEND, S3_CLOSED, S4_STORE, S5_CLOSED);
    }

    public static PickServoSet openS3Drop() {
        // Thả hộp bên trái bằng ngàm S3.
        return new PickServoSet(S1_DROP_LOW, S2_STORE_EXTEND, S3_OPEN, S4_STORE, S5_CLOSED);
    }

    public static PickServoSet openS5Drop() {
        // Thả hộp bên phải bằng ngàm S5.
        return new PickServoSet(S1_DROP_LOW, S2_STORE_EXTEND, S3_CLOSED, S4_STORE, S5_OPEN);
    }

    public static PickServoSet openBothDrop() {
        return new PickServoSet(S1_DROP_LOW, S2_STORE_EXTEND, S3_OPEN, S4_STORE, S5_OPEN);
    }
    
    public static PickServoSet s1LiftUpAfterDrop() {
        return new PickServoSet(S1_HIGHEST, S2_STORE_EXTEND, S3_OPEN, S4_STORE, S5_OPEN);
    }

    public static PickServoSet s1LiftUpAfterDrop1() {
        // Nâng S1 lên cao sau khi thả Hộp 1, S5 vẫn giữ Hộp 2
        return new PickServoSet(S1_HIGHEST, S2_STORE_EXTEND, S3_OPEN, S4_STORE, S5_CLOSED);
    }

    public static PickServoSet s2RetractHomeAfterDrop() {
        return new PickServoSet(S1_HIGHEST, S2_HOME, S3_OPEN, S4_STORE, S5_OPEN);
    }

    public static PickServoSet s2RetractHomeAfterDrop1() {
        // Rút S2 về HOME sau khi thả Hộp 1, S5 vẫn giữ Hộp 2
        return new PickServoSet(S1_HIGHEST, S2_HOME, S3_OPEN, S4_STORE, S5_CLOSED);
    }
    
    public static PickServoSet allHome() {
        return new PickServoSet(S1_HOME, S2_HOME, S3_OPEN, S4_HOME, S5_OPEN);
    }
}
