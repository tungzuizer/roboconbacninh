package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Locale;

/**
 * =========================================================================
 *             HƯỚNG DẪN SỬ DỤNG PEDRO MANUAL & CYCLE TUNER
 * =========================================================================
 * 🚗 GAMEPAD 1: ĐIỀU KHIỂN LOGISTICS & DI CHUYỂN
 *   - Trực trái / phải: Lái Mecanum & xoay robot thủ công.
 *   - DPAD Up / Down  : Tăng/giảm vị trí target chạy thử (P0 - P9).
 *   - Nút [A]         : Bật/tắt tự động lái xe đến điểm target đã chọn.
 *   - Nút [B]         : Dừng khẩn cấp chạy tự động, quay về lái tay.
 *   - Nút [Y]         : Ghi đè tọa độ thực tế của robot hiện tại vào điểm P(index).
 *
 * 🦾 GAMEPAD 2: CĂN CHỈNH BẰNG TAY & CHU TRÌNH (TỐC ĐỘ SERVO GIẢM 50%)
 *   - Nút [LB] / [RB] : Giảm / Tăng chế độ TUNER_MODE trực tiếp (0 đến 11).
 *   - Trục Left Stick Y: Đẩy lên / kéo xuống để JOG tăng giảm góc servo mịn (0.005).
 *   - Nút [Y]         : LƯU góc đang căn chỉnh trực tiếp vào file cấu hình BoxAutoPanels.
 *   - Nút [X]         : KHẨN CẤP đưa toàn bộ cơ cấu gắp về HOME và Mode 0.
 *
 * 📝 CÁC CHẾ ĐỘ TUNER_MODE (0 - 11) - CHU TRÌNH 8 BƯỚC THỰC TẾ:
 *   - MODE 0: TUNE LẺ CÁNH TAY BẰNG TAY (S1-S5). Dpad U/D chọn, Stick Y chỉnh. A/B kẹp/mở S3.
 *   - MODE 1: STEP 1 - READY. S1 hạ hàng gắp, S4 xoay trước, S5 quay sang cột. (Cột: Dpad L/R | Hàng: Dpad U/D)
 *             * Left Stick Y: Jog S5 Swing  |  Giữ LT + Left Stick Y: Jog S1 Lift gắp.
 *   - MODE 2: STEP 2 - EXTEND (S2 vươn). Chỉ có S2 vươn ra gắp hộp, các trục khác giữ nguyên Step 1.
 *             * Left Stick Y: Jog S2 Telescope vươn ra gắp.
 *   - MODE 3: STEP 3 - GRAB (Gripper S3 kẹp). S3 kẹp chặt.
 *             * Left Stick Y: Jog S3 Gripper kẹp hộp.
 *   - MODE 4: STEP 4 - LIFT UP. S1 nhấc nhẹ lên để tránh cọ sát với kệ.
 *             * Left Stick Y: Jog S1 Lift nâng lên nhích (S1_LIFT_UP_OFFSET).
 *   - MODE 5: STEP 5 - RETRACT. S2 co tay gắp về HOME.
 *             * Left Stick Y: Jog S2 Telescope co về (S2_HOME).
 *   - MODE 6: STEP 6 - READY STORE/LẬT SAU. S1 cực cao, S4 ngửa ra sau, S5 swing đến ngăn. (Khay cất 1-4: Dpad L/R)
 *             * Left Stick Y: Jog S5 Store | Giữ LT + LS Y: Jog S1 Lift cao | Giữ RT + LS Y: Jog S4 ngửa sau.
 *   - MODE 7: STEP 7 - STORE/THẢ. S1 hạ khay robot, S2 vươn nhẹ, S3 mở nhả hộp.
 *             * Left Stick Y: Jog S1 Lift xả thả robot | Giữ LT + LS Y: Jog S2 Telescope vươn cất.
 *   - MODE 8: STEP 8 - RESET HOME. Cất tay gắp an toàn, S1/S2/S4/S5 về HOME.
 *             * Bấm [Y] lưu các góc hiện tại làm góc HOME mặc định.
 *   - MODE 9: TUNE LẺ DROP SERVO CỬA XẢ DƯỚI GẦM (D1-D3). Dpad U/D chọn, Stick Y chỉnh góc.
 *   - MODE 10: ĐÓNG NẮP GẦM (Tất cả drop đóng).
 *   - MODE 11: MỞ NẮP GẦM. Mở nắp khay chọn (Khay xả 1-4: Dpad L/R). Stick Y chỉnh góc mở nắp xả.
 * =========================================================================
 */
@TeleOp(name = "Pedro Manual & Cycle Tuner", group = "Pedro Pathing")
public class ManualTuner extends LinearOpMode {

    private Follower follower;
    private Servo s1, s2, s3, s4, s5;
    private Servo drop1;
    private com.qualcomm.robotcore.hardware.HardwareDevice pcaRaw;

    // State Variables for Auto Drive
    private boolean autoDriving = false;
    private int lastTargetIndex = -1;
    private Pose lastTargetPose = new Pose(0, 0, 0);

    // Jogging and Target variables
    private int selectedServoIndex = 0; // 0=S1, 1=S2, 2=S3, 3=S4, 4=S5 for pick, or 0=D1, 1=D2, 2=D3 for drop
    
    // Target angles (Góc mong muốn)
    private double targetS1 = 0.5;
    private double targetS2 = 0.75;
    private double targetS3 = 0.0;
    private double targetS4 = 0.375;
    private double targetS5 = 0.0;
    private double targetD1 = 0.0;
    private double targetD2 = 0.0;
    private double targetD3 = 0.0;

    // Current interpolation angles (Góc hiện tại đang chạy từ từ)
    private double currentS1 = 0.5;
    private double currentS2 = 0.75;
    private double currentS3 = 0.0;
    private double currentS4 = 0.375;
    private double currentS5 = 0.0;
    private double currentD1 = 0.0;
    private double currentD2 = 0.0;
    private double currentD3 = 0.0;

    // Cache angles to avoid redundant writing and I2C dropouts
    private double lastWrittenS1 = -1;
    private double lastWrittenS2 = -1;
    private double lastWrittenS3 = -1;
    private double lastWrittenS4 = -1;
    private double lastWrittenS5 = -1;
    private double lastWrittenD1 = -1;
    private double lastWrittenD2 = -1;
    private double lastWrittenD3 = -1;

    // Rate Limiting Constants for 50% Speed (Đơn vị vị trí / giây, 0.45 tức xoay hành trình 0-1 mất khoảng 2.2 giây)
    private static final double SERVO_POS_PER_SEC = 0.45;
    private static final double GRIPPER_POS_PER_SEC = 3.0; // Gripper kẹp giữ tốc độ nhanh nhạy

    private static final double JOG_STEP = 0.005; // Bước dịch servo mịn
    private static final double DEADZONE = 0.05;

    // Button memory to catch transitions (Debounce)
    private boolean prevGp1A = false;
    private boolean prevGp1B = false;
    private boolean prevGp1Y = false;
    private boolean prevGp1DpadUp = false;
    private boolean prevGp1DpadDown = false;

    private boolean prevGp2LB = false;
    private boolean prevGp2RB = false;
    private boolean prevGp2A = false;
    private boolean prevGp2B = false;
    private boolean prevGp2X = false;
    private boolean prevGp2Y = false;
    private boolean prevGp2DpadUp = false;
    private boolean prevGp2DpadDown = false;
    private boolean prevGp2DpadLeft = false;
    private boolean prevGp2DpadRight = false;

    private String savedPoseStr = "(chưa lưu)";
    private String lastActionFeedback = "Sẵn sàng!";
    private long lastLoopTime = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(BoxAutoPanels.poseStart());

        try {
            s1 = hardwareMap.get(Servo.class, BoxAutoPanels.NAME_S1);
        } catch (Exception e) {}
        try {
            s2 = hardwareMap.get(Servo.class, BoxAutoPanels.NAME_S2);
        } catch (Exception e) {}
        try {
            s3 = hardwareMap.get(Servo.class, BoxAutoPanels.NAME_S3);
        } catch (Exception e) {}
        try {
            s4 = hardwareMap.get(Servo.class, BoxAutoPanels.NAME_S4);
        } catch (Exception e) {}
        try {
            s5 = hardwareMap.get(Servo.class, BoxAutoPanels.NAME_S5);
        } catch (Exception e) {}

        try {
            drop1 = hardwareMap.get(Servo.class, BoxAutoPanels.NAME_DROP1);
        } catch (Exception e) {}

        try {
            pcaRaw = hardwareMap.get(com.qualcomm.robotcore.hardware.HardwareDevice.class, BoxAutoPanels.NAME_PCA9685);
        } catch (Exception e) {}

        // Lấy góc hiện tại từ hardware
        readCurrentAnglesFromHardware();
        syncTargetWithCurrent();

        telemetry.addData("Trạng thái", "Đã khởi tạo. Sẵn sàng!");
        telemetry.update();

        waitForStart();
        follower.startTeleopDrive();

        lastLoopTime = System.currentTimeMillis();

        while (opModeIsActive()) {
            BoxAutoPanels.refresh();
            follower.update();

            // Tính Delta Time
            long currentLoopTime = System.currentTimeMillis();
            double deltaTimeSec = (currentLoopTime - lastLoopTime) / 1000.0;
            if (deltaTimeSec > 0.2) deltaTimeSec = 0.05; 
            lastLoopTime = currentLoopTime;

            // --- GAMEPAD 1: DI CHUYỂN & CHỌN VỊ TRÍ SÂN ---
            handleDriving();

            // --- GAMEPAD 2: CONFIG / TUNE ---
            handleMechanismTuning();

            // --- NỘI SUY MỊN TỪ GÓC HIỆN TẠI ĐẾN GÓC TARGET (CHẬM 50%) ---
            interpolateServos(deltaTimeSec);

            // --- ĐỒNG BỘ GÓC SERVO RA PHẦN CỨNG ---
            writeAnglesToHardware(false);

            // --- Telemetry ---
            showTelemetry();
        }
    }

    private void readCurrentAnglesFromHardware() {
        if (s1 != null) currentS1 = s1.getPosition();
        if (s2 != null) currentS2 = s2.getPosition();
        if (s3 != null) currentS3 = s3.getPosition();
        if (s4 != null) currentS4 = s4.getPosition();
        if (s5 != null) currentS5 = s5.getPosition();
        if (drop1 != null) currentD1 = drop1.getPosition();
    }

    private void syncTargetWithCurrent() {
        targetS1 = currentS1;
        targetS2 = currentS2;
        targetS3 = currentS3;
        targetS4 = currentS4;
        targetS5 = currentS5;
        targetD1 = currentD1;
        targetD2 = currentD2;
        targetD3 = currentD3;
    }

    private void interpolateServos(double dt) {
        double pickChangeLimit = SERVO_POS_PER_SEC * dt;
        double gridChangeLimit = GRIPPER_POS_PER_SEC * dt;

        currentS1 = moveToward(currentS1, targetS1, pickChangeLimit);
        currentS2 = moveToward(currentS2, targetS2, pickChangeLimit);
        currentS3 = moveToward(currentS3, targetS3, gridChangeLimit);
        currentS4 = moveToward(currentS4, targetS4, pickChangeLimit);
        currentS5 = moveToward(currentS5, targetS5, pickChangeLimit);

        currentD1 = moveToward(currentD1, targetD1, pickChangeLimit);
        currentD2 = moveToward(currentD2, targetD2, pickChangeLimit);
        currentD3 = moveToward(currentD3, targetD3, pickChangeLimit);
    }

    private double moveToward(double current, double target, double maxStep) {
        double diff = target - current;
        if (Math.abs(diff) <= maxStep) {
            return target;
        }
        return current + Math.signum(diff) * maxStep;
    }

    private void writeAnglesToHardware(boolean force) {
        if (force || Math.abs(currentS1 - lastWrittenS1) > 0.0005) {
            if (s1 != null) s1.setPosition(currentS1);
            lastWrittenS1 = currentS1;
        }
        if (force || Math.abs(currentS2 - lastWrittenS2) > 0.0005) {
            if (s2 != null) s2.setPosition(currentS2);
            lastWrittenS2 = currentS2;
        }
        if (force || Math.abs(currentS3 - lastWrittenS3) > 0.0005) {
            if (s3 != null) s3.setPosition(currentS3);
            lastWrittenS3 = currentS3;
        }
        if (force || Math.abs(currentS4 - lastWrittenS4) > 0.0005) {
            if (s4 != null) s4.setPosition(currentS4);
            lastWrittenS4 = currentS4;
        }
        if (force || Math.abs(currentS5 - lastWrittenS5) > 0.0005) {
            if (s5 != null) s5.setPosition(currentS5);
            lastWrittenS5 = currentS5;
        }
        if (force || Math.abs(currentD1 - lastWrittenD1) > 0.0005) {
            if (drop1 != null) drop1.setPosition(currentD1);
            lastWrittenD1 = currentD1;
        }
        if (force || Math.abs(currentD2 - lastWrittenD2) > 0.0005) {
            pcaSetServo(BoxAutoPanels.DROP2_PCA_CHANNEL, currentD2);
            lastWrittenD2 = currentD2;
        }
        if (force || Math.abs(currentD3 - lastWrittenD3) > 0.0005) {
            pcaSetServo(BoxAutoPanels.DROP3_PCA_CHANNEL, currentD3);
            lastWrittenD3 = currentD3;
        }
    }

    // ═══════════════════════════════════════════════════════════
    //  LOGIC GAMEPAD 1: LÁI XE & LƯU TỌA ĐỘ
    // ═══════════════════════════════════════════════════════════
    private void handleDriving() {
        boolean gp1A = gamepad1.a;
        if (gp1A && !prevGp1A) {
            autoDriving = !autoDriving;
            if (!autoDriving) {
                follower.breakFollowing();
            }
        }
        prevGp1A = gp1A;

        boolean gp1B = gamepad1.b;
        if (gp1B && !prevGp1B) {
            autoDriving = false;
            follower.breakFollowing();
        }
        prevGp1B = gp1B;

        boolean gp1Y = gamepad1.y;
        if (gp1Y && !prevGp1Y) {
            Pose p = follower.getPose();
            savedPoseStr = String.format(Locale.US, "X=%.2f  Y=%.2f  H=%.1f°", p.getX(), p.getY(), Math.toDegrees(p.getHeading()));
            saveCurrentPoseToConfig(BoxAutoPanels.TUNER_TARGET_POSITION_INDEX, p);
            lastActionFeedback = "Đã lưu tọa độ P" + BoxAutoPanels.TUNER_TARGET_POSITION_INDEX;
        }
        prevGp1Y = gp1Y;

        boolean gp1DU = gamepad1.dpad_up;
        boolean gp1DD = gamepad1.dpad_down;
        if (gp1DU && !prevGp1DpadUp) {
            BoxAutoPanels.TUNER_TARGET_POSITION_INDEX = Math.min(BoxAutoPanels.TUNER_TARGET_POSITION_INDEX + 1, BoxAutoPanels.POSITION_COUNT - 1);
        }
        if (gp1DD && !prevGp1DpadDown) {
            BoxAutoPanels.TUNER_TARGET_POSITION_INDEX = Math.max(BoxAutoPanels.TUNER_TARGET_POSITION_INDEX - 1, 0);
        }
        prevGp1DpadUp = gp1DU;
        prevGp1DpadDown = gp1DD;

        if (autoDriving) {
            handleAutoDrive();
        } else {
            handleManualDrive();
        }
    }

    private void handleAutoDrive() {
        int targetIdx = BoxAutoPanels.TUNER_TARGET_POSITION_INDEX;
        if (targetIdx < 0 || targetIdx >= BoxAutoPanels.POSITION_COUNT) return;

        Pose target = BoxAutoPanels.poseByIndex(targetIdx);

        boolean idxChanged = (targetIdx != lastTargetIndex);
        boolean poseChanged = (Math.abs(target.getX() - lastTargetPose.getX()) > 0.05
                || Math.abs(target.getY() - lastTargetPose.getY()) > 0.05
                || Math.abs(target.getHeading() - lastTargetPose.getHeading()) > 0.01);

        if (idxChanged || poseChanged) {
            lastTargetIndex = targetIdx;
            lastTargetPose = target;

            Pose current = follower.getPose();
            Path linePath = new Path(new BezierLine(current, target));
            linePath.setLinearHeadingInterpolation(current.getHeading(), target.getHeading());
            follower.followPath(linePath, true);
        }

        if (!follower.isBusy()) {
            autoDriving = false;
        }
    }

    private void handleManualDrive() {
        double forward = -gamepad1.left_stick_y;
        double strafe  =  gamepad1.left_stick_x;
        double turn    =  gamepad1.right_stick_x;

        if (Math.abs(forward) < DEADZONE) forward = 0;
        if (Math.abs(strafe)  < DEADZONE) strafe  = 0;
        if (Math.abs(turn)    < DEADZONE) turn    = 0;

        follower.setTeleOpDrive(forward, strafe, turn, true);
    }

    private void saveCurrentPoseToConfig(int index, Pose p) {
        double x = p.getX();
        double y = p.getY();
        double hDeg = Math.toDegrees(p.getHeading());
        
        switch (index) {
            case 0: BoxAutoPanels.START_X = x; BoxAutoPanels.START_Y = y; BoxAutoPanels.START_H_DEG = hDeg; break;
            case 1: BoxAutoPanels.SHELF1_X = x; BoxAutoPanels.SHELF1_Y = y; BoxAutoPanels.SHELF1_H_DEG = hDeg; break;
            case 2: BoxAutoPanels.SHELF2_X = x; BoxAutoPanels.SHELF2_Y = y; BoxAutoPanels.SHELF2_H_DEG = hDeg; break;
            case 3: BoxAutoPanels.SHELF3_X = x; BoxAutoPanels.SHELF3_Y = y; BoxAutoPanels.SHELF3_H_DEG = hDeg; break;
            case 4: BoxAutoPanels.DROP1_X = x; BoxAutoPanels.DROP1_Y = y; BoxAutoPanels.DROP1_H_DEG = hDeg; break;
            case 5: BoxAutoPanels.DROP2_X = x; BoxAutoPanels.DROP2_Y = y; BoxAutoPanels.DROP2_H_DEG = hDeg; break;
            case 6: BoxAutoPanels.DROP3_X = x; BoxAutoPanels.DROP3_Y = y; BoxAutoPanels.DROP3_H_DEG = hDeg; break;
            case 7: BoxAutoPanels.DROP4_X = x; BoxAutoPanels.DROP4_Y = y; BoxAutoPanels.DROP4_H_DEG = hDeg; break;
            case 8: BoxAutoPanels.EXTRA_PICK_X = x; BoxAutoPanels.EXTRA_PICK_Y = y; BoxAutoPanels.EXTRA_PICK_H_DEG = hDeg; break;
            case 9: BoxAutoPanels.EXTRA_DROP_X = x; BoxAutoPanels.EXTRA_DROP_Y = y; BoxAutoPanels.EXTRA_DROP_H_DEG = hDeg; break;
        }
    }

    // ═══════════════════════════════════════════════════════════
    //  LOGIC GAMEPAD 2: CƠ CẤU & TUNING ĐỒNG BỘ THEO TUNER_MODE
    // ═══════════════════════════════════════════════════════════
    private void handleMechanismTuning() {
        boolean gp2LB = gamepad2.left_bumper;
        boolean gp2RB = gamepad2.right_bumper;
        
        if (gp2LB && !prevGp2LB) {
            int newMode = BoxAutoPanels.TUNER_MODE - 1;
            if (newMode < 0) newMode = 11;
            BoxAutoPanels.TUNER_MODE = newMode;
            applyModeBehavior(newMode);
            lastActionFeedback = "Đổi Mode -> " + getModeName(newMode);
        }
        if (gp2RB && !prevGp2RB) {
            int newMode = BoxAutoPanels.TUNER_MODE + 1;
            if (newMode > 11) newMode = 0;
            BoxAutoPanels.TUNER_MODE = newMode;
            applyModeBehavior(newMode);
            lastActionFeedback = "Đổi Mode -> " + getModeName(newMode);
        }
        prevGp2LB = gp2LB;
        prevGp2RB = gp2RB;

        boolean gp2X = gamepad2.x;
        if (gp2X && !prevGp2X) {
            BoxAutoPanels.TUNER_MODE = 0;
            applyPickSetVariables(BoxAutoPanels.pickHome());
            applyDropSetVariables(BoxAutoPanels.dropClosed());
            lastActionFeedback = "KHẨN CẤP: Đưa target về HOME và Mode 0!";
        }
        prevGp2X = gp2X;

        boolean gp2Y = gamepad2.y;
        if (gp2Y && !prevGp2Y) {
            saveTuningValuesToConstants();
            lastActionFeedback = "ĐÃ LƯU góc tune vào cấu hình!";
        }
        prevGp2Y = gp2Y;

        handleDpadNavigation();

        double jogVal = -gamepad2.left_stick_y;
        if (Math.abs(jogVal) > 0.05) {
            executeJogging(jogVal * JOG_STEP);
        }

        handleSpecialButtons();
    }

    private void handleDpadNavigation() {
        boolean gp2DU = gamepad2.dpad_up;
        boolean gp2DD = gamepad2.dpad_down;
        boolean gp2DL = gamepad2.dpad_left;
        boolean gp2DR = gamepad2.dpad_right;

        int mode = BoxAutoPanels.TUNER_MODE;

        if (mode == 0) {
            if (gp2DU && !prevGp2DpadUp) selectedServoIndex = (selectedServoIndex - 1 + 5) % 5;
            if (gp2DD && !prevGp2DpadDown) selectedServoIndex = (selectedServoIndex + 1) % 5;
        } else if (mode == 9) {
            if (gp2DU && !prevGp2DpadUp) selectedServoIndex = (selectedServoIndex - 1 + 3) % 3;
            if (gp2DD && !prevGp2DpadDown) selectedServoIndex = (selectedServoIndex + 1) % 3;
        }

        // Với 8 bước chu trình gắp: chọn Cột (col L/R) và Hàng (row U/D)
        if (mode >= 1 && mode <= 8) {
            if (gp2DL && !prevGp2DpadLeft) {
                BoxAutoPanels.TUNER_SELECTED_COLUMN = 1;
                applyModeBehavior(mode);
                lastActionFeedback = "Chọn CỘT TRÁI (Col 1)";
            }
            if (gp2DR && !prevGp2DpadRight) {
                BoxAutoPanels.TUNER_SELECTED_COLUMN = 2;
                applyModeBehavior(mode);
                lastActionFeedback = "Chọn CỘT PHẢI (Col 2)";
            }
            if (gp2DU && !prevGp2DpadUp) {
                BoxAutoPanels.TUNER_SELECTED_ROW = 2;
                applyModeBehavior(mode);
                lastActionFeedback = "Chọn HÀNG TRÊN (Row 2)";
            }
            if (gp2DD && !prevGp2DpadDown) {
                BoxAutoPanels.TUNER_SELECTED_ROW = 1;
                applyModeBehavior(mode);
                lastActionFeedback = "Chọn HÀNG DƯỚI (Row 1)";
            }
        }

        // Với bước cất ngăn (Step 6-7) và mở xả (Mode 11): Dpad L/R đổi ngăn robot (compartment 1-4)
        if (mode == 6 || mode == 7 || mode == 11) {
            if (gp2DL && !prevGp2DpadLeft) {
                BoxAutoPanels.TUNER_SELECTED_COMPARTMENT = Math.max(BoxAutoPanels.TUNER_SELECTED_COMPARTMENT - 1, 1);
                applyModeBehavior(mode);
                lastActionFeedback = "Chọn Khay Chứa " + BoxAutoPanels.TUNER_SELECTED_COMPARTMENT;
            }
            if (gp2DR && !prevGp2DpadRight) {
                BoxAutoPanels.TUNER_SELECTED_COMPARTMENT = Math.min(BoxAutoPanels.TUNER_SELECTED_COMPARTMENT + 1, 4);
                applyModeBehavior(mode);
                lastActionFeedback = "Chọn Khay Chứa " + BoxAutoPanels.TUNER_SELECTED_COMPARTMENT;
            }
        }

        prevGp2DpadUp = gp2DU;
        prevGp2DpadDown = gp2DD;
        prevGp2DpadLeft = gp2DL;
        prevGp2DpadRight = gp2DR;
    }

    private void handleSpecialButtons() {
        boolean gp2A = gamepad2.a;
        boolean gp2B = gamepad2.b;
        int mode = BoxAutoPanels.TUNER_MODE;

        if (mode == 0) {
            if (gp2A && !prevGp2A) targetS3 = BoxAutoPanels.S3_CLOSED;
            if (gp2B && !prevGp2B) targetS3 = BoxAutoPanels.S3_OPEN;
        }

        // Bấm giữ [A] để mở thử nắp cửa xả ở các mode chu trình gắp thả
        if (mode >= 1 && mode <= 8 || mode == 11) {
            int comp = BoxAutoPanels.TUNER_SELECTED_COMPARTMENT;
            if (gp2A) {
                applyDropSetVariables(BoxAutoPanels.dropOpen(comp));
            } else {
                applyDropSetVariables(BoxAutoPanels.dropClosed());
            }
        }

        prevGp2A = gp2A;
        prevGp2B = gp2B;
    }

    private void applyModeBehavior(int mode) {
        int col = BoxAutoPanels.TUNER_SELECTED_COLUMN;
        int row = BoxAutoPanels.TUNER_SELECTED_ROW;
        int comp = BoxAutoPanels.TUNER_SELECTED_COMPARTMENT;

        if (mode == 0) return;
        
        switch (mode) {
            case 1: // Step 1: Ready (S4 grab, s5 swing L/R, S1 hạ hàng gắp, s2 co)
                applyPickSetVariables(BoxAutoPanels.pickReady(col, row));
                break;
            case 2: // Step 2: Extend (CHỈ S2 vươn)
                applyPickSetVariables(BoxAutoPanels.pickDown(col, row));
                break;
            case 3: // Step 3: Grab (kẹp s3)
                applyPickSetVariables(BoxAutoPanels.pickGrab(col, row));
                break;
            case 4: // Step 4: Lift Up (nhấc s1 lên offset)
                applyPickSetVariables(BoxAutoPanels.pickRetract(col, row));
                break;
            case 5: // Step 5: Retract telescop S2 co về
                applyPickSetVariables(BoxAutoPanels.pickRetractArm(col, row));
                break;
            case 6: // Step 6: Ready Store (Lật sau, S1 cao, s5 sang ngăn)
                applyPickSetVariables(BoxAutoPanels.pickReadyStore(comp));
                break;
            case 7: // Step 7: Store (S1 hạ thả khay robot, s2 vươn nhẹ, s3 mở)
                applyPickSetVariables(BoxAutoPanels.storeCompartment(comp));
                break;
            case 8: // Step 8: Reset Home an toàn
                applyPickSetVariables(BoxAutoPanels.pickHome());
                break;
            case 10:
                applyDropSetVariables(BoxAutoPanels.dropClosed());
                break;
            case 11:
                applyDropSetVariables(BoxAutoPanels.dropOpen(comp));
                break;
        }
    }

    private void executeJogging(double amount) {
        int mode = BoxAutoPanels.TUNER_MODE;

        if (mode == 0) {
            switch (selectedServoIndex) {
                case 0: targetS1 = clamp(targetS1 + amount); break;
                case 1: targetS2 = BoxAutoPanels.clampS2(targetS2 + amount); break;
                case 2: targetS3 = BoxAutoPanels.clampS3(targetS3 + amount); break;
                case 3: targetS4 = clamp(targetS4 + amount); break;
                case 4: targetS5 = clamp(targetS5 + amount); break;
            }
        } else if (mode == 9) {
            switch (selectedServoIndex) {
                case 0: targetD1 = clamp(targetD1 + amount); break;
                case 1: targetD2 = clamp(targetD2 + amount); break;
                case 2: targetD3 = clamp(targetD3 + amount); break;
            }
        } else if (mode >= 1 && mode <= 8) {
            switch (mode) {
                case 1: // Step 1: Ready - LS Y: Jog S5 Swing  |  Giữ LT jog S1 Lift gắp
                    if (gamepad2.left_trigger > 0.5) {
                        targetS1 = clamp(targetS1 + amount);
                    } else {
                        targetS5 = clamp(targetS5 + amount);
                    }
                    break;
                case 2: // Step 2: Extend - CHỈ jog S2 vươn
                    targetS2 = BoxAutoPanels.clampS2(targetS2 + amount);
                    break;
                case 3: // Step 3: Grab - Jog S3 gripper kẹp
                    targetS3 = BoxAutoPanels.clampS3(targetS3 + amount);
                    break;
                case 4: // Step 4: Lift Up - Jog S1 (nhấc offset)
                    targetS1 = clamp(targetS1 + amount);
                    break;
                case 5: // Step 5: Retract telescop co về - Jog S2
                    targetS2 = BoxAutoPanels.clampS2(targetS2 + amount);
                    break;
                case 6: // Step 6: Ready Store (LT jog S1, RT jog S4, không giữ jog S5 swing ngăn)
                    if (gamepad2.left_trigger > 0.5) {
                        targetS1 = clamp(targetS1 + amount);
                    } else if (gamepad2.right_trigger > 0.5) {
                        targetS4 = clamp(targetS4 + amount);
                    } else {
                        targetS5 = clamp(targetS5 + amount);
                    }
                    break;
                case 7: // Step 7: Store thả khay - LS Y jog S1  |  Giữ LT jog S2 vươn xả
                    if (gamepad2.left_trigger > 0.5) {
                        targetS2 = BoxAutoPanels.clampS2(targetS2 + amount);
                    } else {
                        targetS1 = clamp(targetS1 + amount);
                    }
                    break;
                case 8: // Step 8: Reset Home - LS Y jog các góc HOME
                    switch (selectedServoIndex) {
                        case 0: targetS1 = clamp(targetS1 + amount); break;
                        case 1: targetS2 = BoxAutoPanels.clampS2(targetS2 + amount); break;
                        case 2: targetS3 = BoxAutoPanels.clampS3(targetS3 + amount); break;
                        case 3: targetS4 = clamp(targetS4 + amount); break;
                        case 4: targetS5 = clamp(targetS5 + amount); break;
                    }
                    break;
            }
        }
    }

    private void saveTuningValuesToConstants() {
        int mode = BoxAutoPanels.TUNER_MODE;
        int col = BoxAutoPanels.TUNER_SELECTED_COLUMN;
        int row = BoxAutoPanels.TUNER_SELECTED_ROW;
        int comp = BoxAutoPanels.TUNER_SELECTED_COMPARTMENT;

        if (mode == 0) {
            switch (selectedServoIndex) {
                case 0: BoxAutoPanels.S1_HOME = targetS1; break;
                case 1: BoxAutoPanels.S2_HOME = targetS2; break;
                case 2: BoxAutoPanels.S3_CLOSED = targetS3; break;
                case 3: BoxAutoPanels.S4_GRAB = targetS4; break;
                case 4: BoxAutoPanels.S5_HOME = targetS5; break;
            }
        } else if (mode == 9) {
            switch (selectedServoIndex) {
                case 0: BoxAutoPanels.D1_CLOSED = targetD1; break;
                case 1: BoxAutoPanels.D2_CLOSED = targetD2; break;
                case 2: BoxAutoPanels.D3_CLOSED = targetD3; break;
            }
        } else if (mode >= 1 && mode <= 8) {
            switch (mode) {
                case 1: // S5 Swing cột và S1 Lift gắp
                    if (col == 1) BoxAutoPanels.S5_LEFT = targetS5;
                    else BoxAutoPanels.S5_RIGHT = targetS5;
                    if (row == 1) BoxAutoPanels.S1_ROW1 = targetS1;
                    else BoxAutoPanels.S1_ROW2 = targetS1;
                    break;
                case 2: // S2 Extend duỗi ra
                    BoxAutoPanels.S2_EXTEND = targetS2;
                    break;
                case 3: // S3 Gripper góc kẹp
                    BoxAutoPanels.S3_CLOSED = targetS3;
                    break;
                case 4: // Khoảng nâng offset
                    double originalS1 = (row == 1) ? BoxAutoPanels.S1_ROW1 : BoxAutoPanels.S1_ROW2;
                    BoxAutoPanels.S1_LIFT_UP_OFFSET = originalS1 - targetS1;
                    break;
                case 5: // S2 Home (teles co về)
                    BoxAutoPanels.S2_HOME = targetS2;
                    break;
                case 6: // S1 high store, S4 store lật, S5 ngăn chứa
                    BoxAutoPanels.S1_HIGH_STORE = targetS1;
                    BoxAutoPanels.S4_STORE = targetS4;
                    switch (comp) {
                        case 1: BoxAutoPanels.S5_STORE1 = targetS5; break;
                        case 2: BoxAutoPanels.S5_STORE2 = targetS5; break;
                        case 3: BoxAutoPanels.S5_STORE3 = targetS5; break;
                        case 4: BoxAutoPanels.S5_STORE4 = targetS5; break;
                    }
                    break;
                case 7: // S1 store hạ xả, S2 store vươn xả
                    BoxAutoPanels.S1_STORE = targetS1;
                    BoxAutoPanels.S2_STORE = targetS2;
                    break;
                case 8: // Các góc HOME
                    BoxAutoPanels.S1_HOME = targetS1;
                    BoxAutoPanels.S2_HOME = targetS2;
                    BoxAutoPanels.S3_HOME = targetS3;
                    BoxAutoPanels.S4_HOME = targetS4;
                    BoxAutoPanels.S5_HOME = targetS5;
                    break;
            }
        }
    }

    private void applyPickSetVariables(BoxAutoPanels.PickServoSet p) {
        if (p == null) return;
        targetS1 = p.s1;
        targetS2 = p.s2;
        targetS3 = p.s3;
        targetS4 = p.s4;
        targetS5 = p.s5;
    }

    private void applyDropSetVariables(BoxAutoPanels.DropServoSet d) {
        if (d == null) return;
        targetD1 = d.d1;
        targetD2 = d.d2;
        targetD3 = d.d3;
    }

    private void pcaSetServo(int channel, double position) {
        if (pcaRaw == null) return;
        double clamped = clamp(position);
        int pulseUs = 500 + (int)(clamped * 2000);
        
        try {
            java.lang.reflect.Method m = pcaRaw.getClass().getMethod("setServoPulseUs", int.class, int.class);
            m.invoke(pcaRaw, channel, pulseUs);
        } catch (Exception e) {}
    }

    private double clamp(double v) {
        if (v < 0.0) return 0.0;
        if (v > 1.0) return 1.0;
        return v;
    }

    private String getModeName(int mode) {
        switch (mode) {
            case 0: return "🛠️ TUNE LẺ PICK SERVO";
            case 1: return "1️⃣ READY (S1 hạ gắp, S4 trước, S5 sang cột)";
            case 2: return "2️⃣ EXTEND (CHỈ S2 vươn duỗi gắp hộp)";
            case 3: return "3️⃣ GRAB (Gripper S3 kẹp chặt)";
            case 4: return "4️⃣ LIFT UP (S1 nhấc nhẹ tránh vướng kệ)";
            case 5: return "5️⃣ RETRACT (Co telescope S2 về HOME)";
            case 6: return "6️⃣ READY STORE (Lật ngửa S4, S5 sang ngăn, S1 cực cao)";
            case 7: return "7️⃣ STORE/THẢ (S1 hạ, S2 vươn khay robot, nhả S3)";
            case 8: return "8️⃣ RESET HOME (Thu gọn cơ cấu gắp an toàn)";
            case 9: return "🛠️ TUNE LẺ DROP SERVO (Cửa xả gầm)";
            case 10: return "🔒 DROP CLOSED (Đóng các nắp xả)";
            case 11: return "🔓 DROP OPEN (Mở nắp khay được chọn)";
            default: return "Chưa rõ";
        }
    }

    private void showTelemetry() {
        telemetry.addLine("=== 🕹️ HỆ THỐNG CONFIG & TUNING THỰC TẾ ===");
        telemetry.addData("[TUNER_MODE]", BoxAutoPanels.TUNER_MODE + " -> " + getModeName(BoxAutoPanels.TUNER_MODE));
        telemetry.addData("Tốc độ", "GIẢM 50% (Độ khỏe giữ 100%)");
        telemetry.addData("Trạng thái hành động", lastActionFeedback);
        telemetry.addLine();

        // HIỂN THỊ PHẦN CỨNG DEBUG ĐỂ KIỂM TRA SERVO BỊ NULL
        telemetry.addLine("--- TRẠNG THÁI KẾT NỐI PHẦN CỨNG (Debug) ---");
        telemetry.addLine(String.format(Locale.US, "s1: %s | s2: %s | s3: %s | s4: %s | s5: %s",
                (s1 == null ? "⚠️ NULL" : "OK"),
                (s2 == null ? "⚠️ NULL" : "OK"),
                (s3 == null ? "⚠️ NULL" : "OK"),
                (s4 == null ? "⚠️ NULL" : "OK"),
                (s5 == null ? "⚠️ NULL" : "OK")
        ));
        telemetry.addLine(String.format(Locale.US, "drop1: %s | pca9685: %s",
                (drop1 == null ? "⚠️ NULL" : "OK"),
                (pcaRaw == null ? "⚠️ NULL" : "OK")
        ));
        telemetry.addLine();

        int mode = BoxAutoPanels.TUNER_MODE;
        if (mode == 0) {
            telemetry.addLine("👉 CHẾ ĐỘ A: TUNE LẺ PICK SERVO (Dpad U/D chọn):");
            String[] names = {"[S1] Lift cánh tay", "[S2] Telescope", "[S3] Gripper kẹp khay", "[S4] Wrist rotate cổ tay", "[S5] Swing cánh tay"};
            for (int i = 0; i < 5; i++) {
                String prefix = (i == selectedServoIndex) ? "  👉 " : "     ";
                double cur = 0, tar = 0;
                switch (i) {
                    case 0: cur = currentS1; tar = targetS1; break;
                    case 1: cur = currentS2; tar = targetS2; break;
                    case 2: cur = currentS3; tar = targetS3; break;
                    case 3: cur = currentS4; tar = targetS4; break;
                    case 4: cur = currentS5; tar = targetS5; break;
                }
                telemetry.addLine(prefix + names[i] + ": " + String.format(Locale.US, "%.3f (Đang hoạt động: %.3f)", tar, cur));
            }
        } else if (mode >= 1 && mode <= 8) {
            telemetry.addLine("👉 CHẾ ĐỘ B: TUNE THEO BƯỚC CHU TRÌNH THỰC TẾ (Chạy từ từ):");
            telemetry.addData("Cột đang chọn (Dpad L/R)", "Cột " + BoxAutoPanels.TUNER_SELECTED_COLUMN + " (" + (BoxAutoPanels.TUNER_SELECTED_COLUMN == 1 ? "TRÁI" : "PHẢI") + ")");
            telemetry.addData("Hàng đang chọn (Dpad U/D)", "Hàng " + BoxAutoPanels.TUNER_SELECTED_ROW + " (" + (BoxAutoPanels.TUNER_SELECTED_ROW == 1 ? "DƯỚI" : "TRÊN") + ")");
            telemetry.addData("Khay chứa (Dpad L/R ở Step 6-8)", "Compartment " + BoxAutoPanels.TUNER_SELECTED_COMPARTMENT);
            telemetry.addLine();
            telemetry.addLine("GÓC HIỆN TẠI (Mục tiêu -> Thực tế):");
            telemetry.addLine("  S1 (Lift) : " + String.format(Locale.US, "%.3f -> %.3f", targetS1, currentS1));
            telemetry.addLine("  S2 (Teles): " + String.format(Locale.US, "%.3f -> %.3f", targetS2, currentS2));
            telemetry.addLine("  S3 (Grip) : " + String.format(Locale.US, "%.3f -> %.3f", targetS3, currentS3));
            telemetry.addLine("  S4 (Wrist): " + String.format(Locale.US, "%.3f -> %.3f", targetS4, currentS4));
            telemetry.addLine("  S5 (Swing): " + String.format(Locale.US, "%.3f -> %.3f", targetS5, currentS5));
            telemetry.addLine();
            telemetry.addLine("SỬ DỤNG LEFT STICK Y ĐỂ CĂN CHỈNH:");
            switch (mode) {
                case 1: telemetry.addLine("  -> Đang jog S5 Swing. Cột góc: " + String.format(Locale.US, "%.3f", targetS5) + "\n     (Giữ LT jog S1 để hạ độ gắp chạm: " + String.format(Locale.US, "%.3f", targetS1) + ")"); break;
                case 2: telemetry.addLine("  -> Đang jog CHỈ S2 duỗi ra gắp. Góc: " + String.format(Locale.US, "%.3f", targetS2)); break;
                case 3: telemetry.addLine("  -> Đang jog S3 Gripper kẹp. Góc: " + String.format(Locale.US, "%.3f", targetS3)); break;
                case 4: telemetry.addLine("  -> Đang jog S1 nhấc lên (offset). Góc: " + String.format(Locale.US, "%.3f", targetS1)); break;
                case 5: telemetry.addLine("  -> Đang jog S2 co về (S2_HOME). Góc: " + String.format(Locale.US, "%.3f", targetS2)); break;
                case 6: telemetry.addLine("  -> Đang jog S5 ngăn robot. Góc: " + String.format(Locale.US, "%.3f", targetS5) + "\n     (Giữ LT jog S1 lộn sau: " + String.format(Locale.US, "%.3f", targetS1) + " | Giữ RT jog S4 ngửa sau: " + String.format(Locale.US, "%.3f", targetS4) + ")"); break;
                case 7: telemetry.addLine("  -> Đang jog S1 hạ xả khay. Góc: " + String.format(Locale.US, "%.3f", targetS1) + " (Giữ LT jog S2 vươn xả: " + String.format(Locale.US, "%.3f", targetS2) + ")"); break;
                case 8: telemetry.addLine("  -> Đang coi các góc HOME. Bấm [Y] để lưu làm S_HOME."); break;
            }
            telemetry.addLine();
            telemetry.addLine("Bấm giữ [A] để mở thử nắp cửa xả của Compartment hiện tại");
            telemetry.addLine("Bấm [Y] để lưu góc đang căn vào Panels.");
        } else if (mode == 9) {
            telemetry.addLine("👉 CHẾ ĐỘ C: TUNE LẺ DROP SERVO (NẮP XẢ) (Dpad U/D chọn):");
            String[] names = {"[D1] Cửa xả Hub", "[D2] Cửa xả PCA Ch1", "[D3] Cửa xả PCA Ch2"};
            for (int i = 0; i < 3; i++) {
                String prefix = (i == selectedServoIndex) ? "  👉 " : "     ";
                double cur = 0, tar = 0;
                switch (i) {
                    case 0: cur = currentD1; tar = targetD1; break;
                    case 1: cur = currentD2; tar = targetD2; break;
                    case 2: cur = currentD3; tar = targetD3; break;
                }
                telemetry.addLine(prefix + names[i] + ": " + String.format(Locale.US, "%.3f (Đang hoạt động: %.3f)", tar, cur));
            }
        }

        telemetry.addLine();
        telemetry.addLine("[LB] để LÙI chế độ  |  [RB] để TIẾN chế độ");
        telemetry.addLine("[X] khẩn cấp chuyển vể HOME / reset target");
        telemetry.addLine("[Y] lưu giá trị đang tune vào file cấu hình");

        telemetry.addLine();
        telemetry.addLine("=== 🚗 THỦ NGHIỆM DI CHUYỂN (GP1) ===");
        telemetry.addData("Tọa độ hiện tại", String.format(Locale.US, "X=%.2f Y=%.2f H=%.1f°", 
                follower.getPose().getX(), follower.getPose().getY(), Math.toDegrees(follower.getPose().getHeading())));
        telemetry.addData("Trục target index (Dpad U/D)", "P" + BoxAutoPanels.TUNER_TARGET_POSITION_INDEX + " -> " + 
                BoxAutoPanels.poseName(BoxAutoPanels.TUNER_TARGET_POSITION_INDEX));
        telemetry.addData("Lái tự động [A]", autoDriving ? "ĐANG CHẠY TỚI TARGET" : "TẮT (Lái tay)");
        telemetry.addData("Lưu tọa độ [Y]", savedPoseStr);
        telemetry.update();
    }
}
