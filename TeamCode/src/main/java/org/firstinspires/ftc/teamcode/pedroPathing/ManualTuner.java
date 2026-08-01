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
 * 🦾 GAMEPAD 2: CĂN CHỈNH BẰNG TAY & CHU TRÌNH
 *   - Nút [LB] / [RB] : Giảm / Tăng chế độ TUNER_MODE trực tiếp (0 đến 11).
 *   - Trục Left Stick Y: Đẩy lên / kéo xuống để JOG tăng giảm góc servo mịn (0.005).
 *   - Nút [Y]         : LƯU góc đang căn chỉnh trực tiếp vào file cấu hình BoxAutoPanels.
 *   - Nút [X]         : KHẨN CẤP đưa toàn bộ cơ cấu gắp về HOME và Mode 0.
 *
 * 📝 CÁC CHẾ ĐỘ TUNER_MODE (0 - 11):
 *   - MODE 0: TUNE LẺ CÁNH TAY (S1-S5). Dpad U/D chọn servo, Stick Y chỉnh góc. A/B kẹp/mở S3.
 *   - MODE 1: STEP 1 - READY. Chỉnh S5 Swing sang cột (cột 1/2 chọn bằng Dpad L/R).
 *   - MODE 2: STEP 2 - DOWN. Chỉnh S1 Lift hạ gắp (hàng 1/2 chọn bằng Dpad U/D).
 *   - MODE 3: STEP 3 - EXTEND. Chỉnh S2 Telescope vươn ra gắp.
 *   - MODE 4: STEP 4 - GRAB. Chỉnh S3 Gripper kẹp khít hộp.
 *   - MODE 5: STEP 5 - LIFT UP. Chỉnh khoảng nhích lift S1 tránh vướng kệ.
 *   - MODE 6: STEP 6 - RETRACT. Chỉnh S2 co tay gắp về.
 *   - MODE 7: STEP 7 - HIGH STORE. Chỉnh S1 nâng cực cao, S4 ngửa ra sau (lộn cánh tay).
 *   - MODE 8: STEP 8 - DROP STORE. Chỉnh S5 swing ngăn robot (Ngăn 1-4 chọn bằng Dpad L/R), nhả S3.
 *   - MODE 9: TUNE LẺ DROP SERVO CỬA XẢ (D1-D3). Dpad U/D chọn, Stick Y chỉnh góc.
 *   - MODE 10: ĐÓNG NẮP GẦM (Tất cả drop đóng).
 *   - MODE 11: MỞ NẮP GẦM. Mở nắp tương ứng khay chọn. Stick Y chỉnh góc mở nắp xả.
 * =========================================================================
 */
@TeleOp(name = "Pedro Manual & Cycle Tuner", group = "Pedro Pathing")
public class ManualTuner extends LinearOpMode {

    private Follower follower;
    private Servo s1, s2, s3, s4, s5;
    private Servo drop1;
    private com.qualcomm.robotcore.hardware.HardwareDevice pcaRaw; // PCA9685 driver class if loaded

    // State Variables for Auto Drive
    private boolean autoDriving = false;
    private int lastTargetIndex = -1;
    private Pose lastTargetPose = new Pose(0, 0, 0);

    // Jogging variables
    private int selectedServoIndex = 0; // 0=S1, 1=S2, 2=S3, 3=S4, 4=S5 for pick, or 0=D1, 1=D2, 2=D3 for drop
    
    // Cache angles to avoid I2C / servo bottlenecking
    private double currentS1 = 0.5;
    private double currentS2 = 0.75;
    private double currentS3 = 0.0;
    private double currentS4 = 0.375;
    private double currentS5 = 0.0;
    private double currentD1 = 0.0;
    private double currentD2 = 0.0;
    private double currentD3 = 0.0;

    private double lastWrittenS1 = -1;
    private double lastWrittenS2 = -1;
    private double lastWrittenS3 = -1;
    private double lastWrittenS4 = -1;
    private double lastWrittenS5 = -1;
    private double lastWrittenD1 = -1;
    private double lastWrittenD2 = -1;
    private double lastWrittenD3 = -1;

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

    @Override
    public void runOpMode() throws InterruptedException {
        // --- Khởi tạo Follower và Cơ cấu gắp ---
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(BoxAutoPanels.poseStart());

        try {
            s1 = hardwareMap.get(Servo.class, BoxAutoPanels.NAME_S1);
            s2 = hardwareMap.get(Servo.class, BoxAutoPanels.NAME_S2);
            s3 = hardwareMap.get(Servo.class, BoxAutoPanels.NAME_S3);
            s4 = hardwareMap.get(Servo.class, BoxAutoPanels.NAME_S4);
            s5 = hardwareMap.get(Servo.class, BoxAutoPanels.NAME_S5);
        } catch (Exception e) {
            telemetry.addData("WARNING", "Không tìm thấy đủ Servo cánh tay (s1-s5)");
        }

        try {
            drop1 = hardwareMap.get(Servo.class, BoxAutoPanels.NAME_DROP1);
        } catch (Exception e) {
            telemetry.addData("WARNING", "Không tìm thấy drop1");
        }

        try {
            // Thử khởi tạo PCA9685 nếu có trong hardwareMap
            pcaRaw = hardwareMap.get(com.qualcomm.robotcore.hardware.HardwareDevice.class, BoxAutoPanels.NAME_PCA9685);
        } catch (Exception e) {
            telemetry.addData("WARNING", "Không tìm thấy PCA9685");
        }

        // Lấy góc hiện tại từ hardware
        readCurrentAnglesFromHardware();

        telemetry.addData("Trạng thái", "Đã khởi tạo. Sẵn sàng!");
        telemetry.update();

        waitForStart();
        follower.startTeleopDrive();

        // Gửi trạng thái ban đầu của servo xuống phần cứng
        writeAnglesToHardware(true);

        while (opModeIsActive()) {
            // Refresh các giá trị cấu hình trực tiếp từ Panels
            BoxAutoPanels.refresh();

            follower.update();

            // --- GAMEPAD 1: DI CHUYỂN & CHỌN VỊ TRÍ SÂN ---
            handleDriving();

            // --- GAMEPAD 2: CONFIG / MỌI TÍNH NĂNG TUNE ---
            handleMechanismTuning();

            // --- ĐỒNG BỘ GÓC SERVO RA PHẦN CỨNG (TRÁNH LẶP GHI I2C) ---
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

    private void writeAnglesToHardware(boolean force) {
        // Chỉ setPosition khi góc thực sự thay đổi > 0.0005 hoặc force
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
        // --- LB/RB: CHUYỂN ĐỔI TUNER_MODE TRỰC TIẾP TRÊN GAMEPAD ---
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

        // --- Nút X: HOME khẩn cấp đưa về Mode 0 + Home servo ---
        boolean gp2X = gamepad2.x;
        if (gp2X && !prevGp2X) {
            BoxAutoPanels.TUNER_MODE = 0;
            applyPickSetVariables(BoxAutoPanels.pickHome());
            applyDropSetVariables(BoxAutoPanels.dropClosed());
            lastActionFeedback = "KHẨN CẤP: Đưa cơ cấu về HOME và Mode 0!";
        }
        prevGp2X = gp2X;

        // --- Nút Y: Ghi nhớ / Lưu vị trí đang tune vào Panels class hằng số ---
        boolean gp2Y = gamepad2.y;
        if (gp2Y && !prevGp2Y) {
            saveTuningValuesToConstants();
            lastActionFeedback = "ĐÃ LƯU góc tune vào cấu hình!";
        }
        prevGp2Y = gp2Y;

        // --- Dpad Left/Right và Up/Down để đổi tham số/lọc tùy theo Mode ---
        handleDpadNavigation();

        // --- DI CHUYỂN JOG BẰNG LEFT STICK Y ---
        double jogVal = -gamepad2.left_stick_y;
        if (Math.abs(jogVal) > 0.05) {
            executeJogging(jogVal * JOG_STEP);
        }

        // --- QUY ĐỊNH HÀNH VI ĐẶC BIỆT CỦA CÁC NÚT KHÁC THEO MODE ---
        handleSpecialButtons();
    }

    private void handleDpadNavigation() {
        boolean gp2DU = gamepad2.dpad_up;
        boolean gp2DD = gamepad2.dpad_down;
        boolean gp2DL = gamepad2.dpad_left;
        boolean gp2DR = gamepad2.dpad_right;

        int mode = BoxAutoPanels.TUNER_MODE;

        // Mode 0 (Tune lẻ Pick) & Mode 9 (Tune lẻ Drop): Chọn servo cần tune
        if (mode == 0) {
            if (gp2DU && !prevGp2DpadUp) selectedServoIndex = (selectedServoIndex - 1 + 5) % 5;
            if (gp2DD && !prevGp2DpadDown) selectedServoIndex = (selectedServoIndex + 1) % 5;
        } else if (mode == 9) {
            if (gp2DU && !prevGp2DpadUp) selectedServoIndex = (selectedServoIndex - 1 + 3) % 3;
            if (gp2DD && !prevGp2DpadDown) selectedServoIndex = (selectedServoIndex + 1) % 3;
        }

        // Mode 1-8 (các bước chu trình):
        // Dpad Left/Right: Đổi cột (col: 1=Left, 2=Right)
        if (mode >= 1 && mode <= 8) {
            if (gp2DL && !prevGp2DpadLeft) {
                BoxAutoPanels.TUNER_SELECTED_COLUMN = 1;
                applyModeBehavior(mode); // Cập nhật lại góc ngay lập tức
                lastActionFeedback = "Chọn CỘT TRÁI";
            }
            if (gp2DR && !prevGp2DpadRight) {
                BoxAutoPanels.TUNER_SELECTED_COLUMN = 2;
                applyModeBehavior(mode);
                lastActionFeedback = "Chọn CỘT PHẢI";
            }
            // Dpad Up/Down: Đổi hàng (row: 1=ROW1, 2=ROW2)
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

        // Đổi Compartment (1-4) bằng Dpad Left/Right ở các Mode liên quan (Mode 8 & 11)
        if (mode == 8 || mode == 11) {
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

        // Mode 0: A/B kẹp mở nhanh gripper
        if (mode == 0) {
            if (gp2A && !prevGp2A) currentS3 = BoxAutoPanels.S3_CLOSED;
            if (gp2B && !prevGp2B) currentS3 = BoxAutoPanels.S3_OPEN;
        }

        // Mode 1-8 hoặc Mode 11: A để kích hoạt xả thử khay được chọn
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

        if (mode == 0) {
            // Giữ nguyên góc hiện tại để căn chỉnh lẻ
            return;
        }
        if (mode >= 1 && mode <= 8) {
            // Áp dụng góc thiết lập của step
            switch (mode) {
                case 1: applyPickSetVariables(BoxAutoPanels.pickReady(col, row)); break;
                case 2: applyPickSetVariables(BoxAutoPanels.pickDown(col, row)); break;
                case 3: // Step 3: Extend S2
                    applyPickSetVariables(new BoxAutoPanels.PickServoSet(
                        (row == 1) ? BoxAutoPanels.S1_ROW1 : BoxAutoPanels.S1_ROW2,
                        BoxAutoPanels.S2_EXTEND,
                        BoxAutoPanels.S3_OPEN,
                        BoxAutoPanels.S4_GRAB,
                        (col == 1) ? BoxAutoPanels.S5_LEFT : BoxAutoPanels.S5_RIGHT
                    ));
                    break;
                case 4: applyPickSetVariables(BoxAutoPanels.pickGrab(col, row)); break;
                case 5: applyPickSetVariables(BoxAutoPanels.pickRetract(col, row)); break;
                case 6: // Step 6: Retract S2
                    applyPickSetVariables(new BoxAutoPanels.PickServoSet(
                        ((row == 1) ? BoxAutoPanels.S1_ROW1 : BoxAutoPanels.S1_ROW2) - BoxAutoPanels.S1_LIFT_UP_OFFSET,
                        BoxAutoPanels.S2_HOME,
                        BoxAutoPanels.S3_CLOSED,
                        BoxAutoPanels.S4_GRAB,
                        (col == 1) ? BoxAutoPanels.S5_LEFT : BoxAutoPanels.S5_RIGHT
                    ));
                    break;
                case 7: applyPickSetVariables(BoxAutoPanels.pickHighStore()); break;
                case 8: applyPickSetVariables(BoxAutoPanels.storeCompartment(comp)); break;
            }
        }
        if (mode == 9) {
            // Giữ góc drop lẻ để căn chỉnh
            return;
        }
        if (mode == 10) {
            applyDropSetVariables(BoxAutoPanels.dropClosed());
        }
        if (mode == 11) {
            applyDropSetVariables(BoxAutoPanels.dropOpen(comp));
        }
    }

    private void executeJogging(double amount) {
        int mode = BoxAutoPanels.TUNER_MODE;

        if (mode == 0) {
            // Tune lẻ pick
            switch (selectedServoIndex) {
                case 0: currentS1 = clamp(currentS1 + amount); break;
                case 1: currentS2 = BoxAutoPanels.clampS2(currentS2 + amount); break;
                case 2: currentS3 = BoxAutoPanels.clampS3(currentS3 + amount); break;
                case 3: currentS4 = clamp(currentS4 + amount); break;
                case 4: currentS5 = clamp(currentS5 + amount); break;
            }
        } else if (mode == 9) {
            // Tune lẻ drop
            switch (selectedServoIndex) {
                case 0: currentD1 = clamp(currentD1 + amount); break;
                case 1: currentD2 = clamp(currentD2 + amount); break;
                case 2: currentD3 = clamp(currentD3 + amount); break;
            }
        } else if (mode >= 1 && mode <= 8) {
            // Jog gán trực tiếp vào các biến góc đang hoạt động trong step để thử nghiệm trực tiếp
            switch (mode) {
                case 1: // Step 1: Ready - Jog S5 (Swing)
                    currentS5 = clamp(currentS5 + amount);
                    break;
                case 2: // Step 2: Down - Jog S1 (Lift)
                    currentS1 = clamp(currentS1 + amount);
                    break;
                case 3: // Step 3: Extend - Jog S2 (Teles)
                    currentS2 = BoxAutoPanels.clampS2(currentS2 + amount);
                    break;
                case 4: // Step 4: Grab - Jog S3 (Gripper kẹp)
                    currentS3 = BoxAutoPanels.clampS3(currentS3 + amount);
                    break;
                case 5: // Step 5: S1 nhấc lên nhích nâng
                    currentS1 = clamp(currentS1 + amount);
                    break;
                case 6: // Step 6: Retract - Jog S2 co
                    currentS2 = BoxAutoPanels.clampS2(currentS2 + amount);
                    break;
                case 7: // Step 7: Ready lộn - Jog S4 (Wrist) hoặc S1 (Lift)
                    if (gamepad2.left_trigger > 0.5) {
                        currentS1 = clamp(currentS1 + amount);
                    } else {
                        currentS4 = clamp(currentS4 + amount);
                    }
                    break;
                case 8: // Step 8: Store - Jog S5 angle để căn ngăn chứa
                    currentS5 = clamp(currentS5 + amount);
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
            // Cập nhật giá trị đơn lẻ
            switch (selectedServoIndex) {
                case 0: BoxAutoPanels.S1_HOME = currentS1; break;
                case 1: BoxAutoPanels.S2_HOME = currentS2; break;
                case 2: BoxAutoPanels.S3_CLOSED = currentS3; break;
                case 3: BoxAutoPanels.S4_GRAB = currentS4; break;
                case 4: BoxAutoPanels.S5_HOME = currentS5; break;
            }
        } else if (mode == 9) {
            // Cập nhật hằng số drop đơn lẻ
            switch (selectedServoIndex) {
                case 0: BoxAutoPanels.D1_CLOSED = currentD1; break;
                case 1: BoxAutoPanels.D2_CLOSED = currentD2; break;
                case 2: BoxAutoPanels.D3_CLOSED = currentD3; break;
            }
        } else if (mode >= 1 && mode <= 8) {
            // Ghi nhận góc JOG thực tế tại bước đó đè vào hằng số cấu hình tương ứng
            switch (mode) {
                case 1: // Lưu góc S5 sang cột tương ứng
                    if (col == 1) BoxAutoPanels.S5_LEFT = currentS5;
                    else BoxAutoPanels.S5_RIGHT = currentS5;
                    break;
                case 2: // Lưu góc S1 lift hạ xuống hàng tương ứng
                    if (row == 1) BoxAutoPanels.S1_ROW1 = currentS1;
                    else BoxAutoPanels.S1_ROW2 = currentS1;
                    break;
                case 3: // Lưu S2_EXTEND
                    BoxAutoPanels.S2_EXTEND = currentS2;
                    break;
                case 4: // Lưu S3_CLOSED
                    BoxAutoPanels.S3_CLOSED = currentS3;
                    break;
                case 5: // Chênh lệch lift offset
                    double originalS1 = (row == 1) ? BoxAutoPanels.S1_ROW1 : BoxAutoPanels.S1_ROW2;
                    BoxAutoPanels.S1_LIFT_UP_OFFSET = originalS1 - currentS1;
                    break;
                case 6: // Lưu S2_HOME
                    BoxAutoPanels.S2_HOME = currentS2;
                    break;
                case 7: // Lưu S1_HIGH_STORE hoặc S4_STORE
                    BoxAutoPanels.S1_HIGH_STORE = currentS1;
                    BoxAutoPanels.S4_STORE = currentS4;
                    break;
                case 8: // Lưu S5_STORE1-4 tương ứng compartment
                    switch (comp) {
                        case 1: BoxAutoPanels.S5_STORE1 = currentS5; break;
                        case 2: BoxAutoPanels.S5_STORE2 = currentS5; break;
                        case 3: BoxAutoPanels.S5_STORE3 = currentS5; break;
                        case 4: BoxAutoPanels.S5_STORE4 = currentS5; break;
                    }
                    break;
            }
        }
    }

    private void handlePanelsTuner() {
        if (!BoxAutoPanels.TUNER_APPLY) return;
        BoxAutoPanels.TUNER_APPLY = false;

        double pos = BoxAutoPanels.TUNER_SERVO_POS;
        int tunerMode = BoxAutoPanels.TUNER_MODE;

        if (tunerMode == 0) {
            int sv = BoxAutoPanels.TUNER_SELECTED_SERVO;
            switch (sv) {
                case 1: currentS1 = clamp(pos); break;
                case 2: currentS2 = BoxAutoPanels.clampS2(pos); break;
                case 3: currentS3 = BoxAutoPanels.clampS3(pos); break;
                case 4: currentS4 = clamp(pos); break;
                case 5: currentS5 = clamp(pos); break;
            }
        } else if (tunerMode == 9) {
            int dsv = BoxAutoPanels.TUNER_SELECTED_DROP_SERVO;
            switch (dsv) {
                case 1: currentD1 = clamp(pos); break;
                case 2: currentD2 = clamp(pos); break;
                case 3: currentD3 = clamp(pos); break;
            }
        } else {
            applyModeBehavior(tunerMode);
        }
        lastActionFeedback = "Áp dụng giá trị POS=" + pos + " từ Panels";
    }

    private void applyPickSetVariables(BoxAutoPanels.PickServoSet p) {
        if (p == null) return;
        currentS1 = p.s1;
        currentS2 = p.s2;
        currentS3 = p.s3;
        currentS4 = p.s4;
        currentS5 = p.s5;
    }

    private void applyDropSetVariables(BoxAutoPanels.DropServoSet d) {
        if (d == null) return;
        currentD1 = d.d1;
        currentD2 = d.d2;
        currentD3 = d.d3;
    }

    private void pcaSetServo(int channel, double position) {
        if (pcaRaw == null) return;
        double clamped = clamp(position);
        int pulseUs = 500 + (int)(clamped * 2000);
        
        try {
            java.lang.reflect.Method m = pcaRaw.getClass().getMethod("setServoPulseUs", int.class, int.class);
            m.invoke(pcaRaw, channel, pulseUs);
        } catch (Exception e) {
            // Ignored
        }
    }

    private double clamp(double v) {
        if (v < 0.0) return 0.0;
        if (v > 1.0) return 1.0;
        return v;
    }

    private String getModeName(int mode) {
        switch (mode) {
            case 0: return "🛠️ TUNE LẺ PICK SERVO";
            case 1: return "1️⃣ READY (Xoay S4 trước, S5 sang cột)";
            case 2: return "2️⃣ DOWN (Hạ lift S1 gắp)";
            case 3: return "3️⃣ EXTEND (Vươn telescope S2)";
            case 4: return "4️⃣ GRAB (Kẹp gripper S3)";
            case 5: return "5️⃣ LIFT OFFSET (Nâng nhẹ tránh va kệ)";
            case 6: return "6️⃣ RETRACT (Co telescope S2 về HOME)";
            case 7: return "7️⃣ HIGH STORE (S1 lên cao, S4 ngửa ra sau)";
            case 8: return "8️⃣ STORE COMPARTMENT (S5 xoay khay chứa, nhả)";
            case 9: return "🛠️ TUNE LẺ DROP SERVO (Cửa xả)";
            case 10: return "🔒 DROP CLOSED (Đóng các nắp xả)";
            case 11: return "🔓 DROP OPEN (Mở nắp khay được chọn)";
            default: return "Chưa rõ";
        }
    }

    private void showTelemetry() {
        telemetry.addLine("=== 🕹️ HỆ THỐNG CONFIG & TUNING THỰC TẾ ===");
        telemetry.addData("[TUNER_MODE]", BoxAutoPanels.TUNER_MODE + " -> " + getModeName(BoxAutoPanels.TUNER_MODE));
        telemetry.addData("Trạng thái hành động", lastActionFeedback);
        telemetry.addLine();

        int mode = BoxAutoPanels.TUNER_MODE;
        if (mode == 0) {
            telemetry.addLine("👉 CHẾ ĐỘ A: TUNE LẺ PICK SERVO (Dpad U/D chọn):");
            String[] names = {"[S1] Lift cánh tay", "[S2] Telescope", "[S3] Gripper kẹp khay", "[S4] Wrist rotate cổ tay", "[S5] Swing cánh tay"};
            for (int i = 0; i < 5; i++) {
                String prefix = (i == selectedServoIndex) ? "  👉 " : "     ";
                double cur = 0;
                switch (i) {
                    case 0: cur = currentS1; break;
                    case 1: cur = currentS2; break;
                    case 2: cur = currentS3; break;
                    case 3: cur = currentS4; break;
                    case 4: cur = currentS5; break;
                }
                telemetry.addLine(prefix + names[i] + ": " + String.format(Locale.US, "%.3f", cur));
            }
            telemetry.addLine("Lắc stick [Left Stick Y] để tăng giảm góc servo chọn");
            telemetry.addLine("Bấm [A] / [B] để kẹp/mở nhanh S3");
        } else if (mode >= 1 && mode <= 8) {
            telemetry.addLine("👉 CHẾ ĐỘ B: TUNE THEO BƯỚC CHU TRÌNH gắp thả:");
            telemetry.addData("Cột đang chọn (Dpad L/R)", "Cột " + BoxAutoPanels.TUNER_SELECTED_COLUMN + " (" + (BoxAutoPanels.TUNER_SELECTED_COLUMN == 1 ? "TRÁI" : "PHẢI") + ")");
            telemetry.addData("Hàng đang chọn (Dpad U/D)", "Hàng " + BoxAutoPanels.TUNER_SELECTED_ROW + " (" + (BoxAutoPanels.TUNER_SELECTED_ROW == 1 ? "DƯỚI" : "TRÊN") + ")");
            telemetry.addData("Khay chứa (Dpad L/R ở Step 8)", "Compartment " + BoxAutoPanels.TUNER_SELECTED_COMPARTMENT);
            telemetry.addLine();
            telemetry.addLine("SỬ DỤNG LEFT STICK Y ĐỂ CĂN CHỈNH:");
            switch (mode) {
                case 1: telemetry.addLine("  -> Đang jog S5 Swing. Cột góc: " + String.format(Locale.US, "%.3f", currentS5)); break;
                case 2: telemetry.addLine("  -> Đang jog S1 Hạ. Hàng góc: " + String.format(Locale.US, "%.3f", currentS1)); break;
                case 3: telemetry.addLine("  -> Đang jog S2 Vươn. Telescope góc: " + String.format(Locale.US, "%.3f", currentS2)); break;
                case 4: telemetry.addLine("  -> Đang jog S3 Grip. Góc kẹp: " + String.format(Locale.US, "%.3f", currentS3)); break;
                case 5: telemetry.addLine("  -> Đang jog S1 Nhích. Góc nhích lift: " + String.format(Locale.US, "%.3f", currentS1)); break;
                case 6: telemetry.addLine("  -> Đang jog S2 Rút. Telescope góc: " + String.format(Locale.US, "%.3f", currentS2)); break;
                case 7: telemetry.addLine("  -> Đang jog S4 lật ngược: " + String.format(Locale.US, "%.3f", currentS4) + " (Giữ LT để jog S1 cao: " + String.format(Locale.US, "%.3f", currentS1) + ")"); break;
                case 8: telemetry.addLine("  -> Đang jog S5 Swing ngăn chứa. Góc: " + String.format(Locale.US, "%.3f", currentS5)); break;
            }
            telemetry.addLine();
            telemetry.addLine("Bấm giữ [A] để mở thử nắp cửa xả của Compartment hiện tại");
            telemetry.addLine("Bấm [Y] để lưu góc đang căn vào Panels.");
        } else if (mode == 9) {
            telemetry.addLine("👉 CHẾ ĐỘ C: TUNE LẺ DROP SERVO (NẮP XẢ) (Dpad U/D chọn):");
            String[] names = {"[D1] Cửa xả Hub", "[D2] Cửa xả PCA Ch1", "[D3] Cửa xả PCA Ch2"};
            for (int i = 0; i < 3; i++) {
                String prefix = (i == selectedServoIndex) ? "  👉 " : "     ";
                double cur = 0;
                switch (i) {
                    case 0: cur = currentD1; break;
                    case 1: cur = currentD2; break;
                    case 2: cur = currentD3; break;
                }
                telemetry.addLine(prefix + names[i] + ": " + String.format(Locale.US, "%.3f", cur));
            }
            telemetry.addLine("Lắc stick [Left Stick Y] để tăng giảm góc servo chọn");
        } else {
            telemetry.addLine("👉 CHẾ ĐỘ CO/XẢ DROP TỰ ĐỘNG:");
            telemetry.addData("Khay chứa (Dpad L/R)", "Compartment " + BoxAutoPanels.TUNER_SELECTED_COMPARTMENT);
            telemetry.addLine("D1: " + String.format(Locale.US, "%.3f", currentD1) + " | D2: " + String.format(Locale.US, "%.3f", currentD2) + " | D3: " + String.format(Locale.US, "%.3f", currentD3));
        }

        telemetry.addLine();
        telemetry.addLine("[LB] để LÙI chế độ  |  [RB] để TIẾN chế độ");
        telemetry.addLine("[X] khẩn cấp chuyển về HOME / reset");
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
