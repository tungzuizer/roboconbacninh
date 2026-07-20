package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Tuner Servo SIÊU DỄ — TỰ ĐỘNG NHẬN DIỆN SLIDER.
 * 
 * CÁCH DÙNG:
 * - QUÊN HẾT TUNER_MODE VÀ TUNER_SELECTED_SLOT ĐI!
 * - Bạn chỉ cần kéo slider (ví dụ BL_S1), robot sẽ TỰ ĐỘNG chuyển sang tư thế BL và xoay theo.
 * - Kéo HOME_S2? Robot tự về thế Home.
 */
@TeleOp(name = "Servo Panels Tuner (Siêu Dễ)", group = "Tuning")
public class ServoPanelsTuner extends LinearOpMode {

    private Servo s1, s2, s3, s4, s5;
    private Servo drop1;
    private PCA9685 pca9685;
    private boolean dropReady = false;
    private boolean pcaReady = false;

    // Lưu trữ giá trị cũ để phát hiện bạn vừa kéo slider nào
    private BoxAutoPanels.PickServoSet lastHome, lastTL, lastTR, lastBL, lastBR;
    private BoxAutoPanels.PickServoSet lastDep1, lastDep2, lastDep3, lastDep4;
    private BoxAutoPanels.DropServoSet lastDropHome, lastDropRel;
    
    private String currentPoseName = "ĐANG TÌM KIẾM...";

    @Override
    public void runOpMode() {
        s1 = hardwareMap.get(Servo.class, BoxAutoPanels.NAME_S1);
        s2 = hardwareMap.get(Servo.class, BoxAutoPanels.NAME_S2);
        s3 = hardwareMap.get(Servo.class, BoxAutoPanels.NAME_S3);
        s4 = hardwareMap.get(Servo.class, BoxAutoPanels.NAME_S4);
        s5 = hardwareMap.get(Servo.class, BoxAutoPanels.NAME_S5);

        initDropIfEnabled();
        
        // Cập nhật giá trị ban đầu
        BoxAutoPanels.refresh();
        updateLastValues();
        applyPickSet(BoxAutoPanels.pickHome());
        currentPoseName = "HOME (Khởi động)";

        telemetry.addLine("=== CHẾ ĐỘ TUNE SIÊU DỄ ===");
        telemetry.addLine("CHỈ CẦN KÉO SLIDER LÀ ROBOT TỰ ĐỔI TƯ THẾ!");
        telemetry.update();

        // Chạy ngay trong lúc INIT
        while (opModeInInit()) {
            smartPollAndApply();
            if (isStopRequested()) return;
        }

        waitForStart();

        while (opModeIsActive()) {
            smartPollAndApply();
        }
    }

    private void smartPollAndApply() {
        BoxAutoPanels.refresh();
        initDropIfEnabled();

        // 1. Lấy giá trị hiện tại trên Panels
        BoxAutoPanels.PickServoSet curHome = BoxAutoPanels.pickHome();
        BoxAutoPanels.PickServoSet curTL = BoxAutoPanels.pickSlot(0);
        BoxAutoPanels.PickServoSet curTR = BoxAutoPanels.pickSlot(1);
        BoxAutoPanels.PickServoSet curBL = BoxAutoPanels.pickSlot(2);
        BoxAutoPanels.PickServoSet curBR = BoxAutoPanels.pickSlot(3);
        BoxAutoPanels.PickServoSet curDep1 = BoxAutoPanels.depositZone(1);
        BoxAutoPanels.PickServoSet curDep2 = BoxAutoPanels.depositZone(2);
        BoxAutoPanels.PickServoSet curDep3 = BoxAutoPanels.depositZone(3);
        BoxAutoPanels.PickServoSet curDep4 = BoxAutoPanels.depositZone(4);
        
        BoxAutoPanels.DropServoSet curDropHome = BoxAutoPanels.dropHome();
        BoxAutoPanels.DropServoSet curDropRel = BoxAutoPanels.dropReleaseOnField();

        // 2. So sánh xem bộ nào vừa bị thay đổi (bạn vừa kéo slider nào)
        if (isPickChanged(curBL, lastBL)) {
            applyPickSet(curBL);
            currentPoseName = "Ô DƯỚI-TRÁI (BL)";
        } 
        else if (isPickChanged(curHome, lastHome)) {
            applyPickSet(curHome);
            currentPoseName = "VỊ TRÍ HOME";
        }
        else if (isPickChanged(curTL, lastTL)) {
            applyPickSet(curTL);
            currentPoseName = "Ô TRÊN-TRÁI (TL)";
        }
        else if (isPickChanged(curTR, lastTR)) {
            applyPickSet(curTR);
            currentPoseName = "Ô TRÊN-PHẢI (TR)";
        }
        else if (isPickChanged(curBR, lastBR)) {
            applyPickSet(curBR);
            currentPoseName = "Ô DƯỚI-PHẢI (BR)";
        }
        else if (isPickChanged(curDep1, lastDep1)) {
            applyPickSet(curDep1);
            currentPoseName = "BỎ VÀO ROBOT - NGĂN 1 (box01)";
        }
        else if (isPickChanged(curDep2, lastDep2)) {
            applyPickSet(curDep2);
            currentPoseName = "BỎ VÀO ROBOT - NGĂN 2 (box02)";
        }
        else if (isPickChanged(curDep3, lastDep3)) {
            applyPickSet(curDep3);
            currentPoseName = "BỎ VÀO ROBOT - NGĂN 3 (box03)";
        }
        else if (isPickChanged(curDep4, lastDep4)) {
            applyPickSet(curDep4);
            currentPoseName = "BỎ VÀO ROBOT - NGĂN 4 (box04)";
        }
        else if (isDropChanged(curDropHome, lastDropHome)) {
            applyDropSet(curDropHome);
            currentPoseName = "THẢ - HOME";
        }
        else if (isDropChanged(curDropRel, lastDropRel)) {
            applyDropSet(curDropRel);
            currentPoseName = "THẢ - RA SÂN";
        }

        // 3. Cập nhật lại giá trị cũ
        updateLastValues();

        // Hiển thị telemetry đẹp mắt
        telemetry.addData("Tư thế hiện tại", currentPoseName);
        telemetry.addLine("--------------------------------");
        telemetry.addLine("MẸO: Bạn KHÔNG CẦN chỉnh TUNER_MODE.");
        telemetry.addLine("Cứ kéo thẳng slider (VD: BL_S1 hoặc DEP3_S2),");
        telemetry.addLine("robot sẽ tự hiểu và xoay theo!");
        telemetry.update();
    }
    
    // Lưu lại giá trị của vòng lặp hiện tại để so sánh với vòng lặp sau
    private void updateLastValues() {
        lastHome = BoxAutoPanels.pickHome();
        lastTL = BoxAutoPanels.pickSlot(0);
        lastTR = BoxAutoPanels.pickSlot(1);
        lastBL = BoxAutoPanels.pickSlot(2);
        lastBR = BoxAutoPanels.pickSlot(3);
        lastDep1 = BoxAutoPanels.depositZone(1);
        lastDep2 = BoxAutoPanels.depositZone(2);
        lastDep3 = BoxAutoPanels.depositZone(3);
        lastDep4 = BoxAutoPanels.depositZone(4);
        lastDropHome = BoxAutoPanels.dropHome();
        lastDropRel = BoxAutoPanels.dropReleaseOnField();
    }

    // Hàm kiểm tra xem có servo nào trong set gắp bị thay đổi không
    private boolean isPickChanged(BoxAutoPanels.PickServoSet cur, BoxAutoPanels.PickServoSet last) {
        if (last == null) return false;
        double epsilon = 0.001; // Bỏ qua sai số nhỏ
        return Math.abs(cur.s1 - last.s1) > epsilon ||
               Math.abs(cur.s2 - last.s2) > epsilon ||
               Math.abs(cur.s3 - last.s3) > epsilon ||
               Math.abs(cur.s4 - last.s4) > epsilon ||
               Math.abs(cur.s5 - last.s5) > epsilon;
    }
    
    // Hàm kiểm tra xem có servo nào trong set thả bị thay đổi không
    private boolean isDropChanged(BoxAutoPanels.DropServoSet cur, BoxAutoPanels.DropServoSet last) {
        if (last == null || !dropReady || !pcaReady) return false;
        double epsilon = 0.001;
        return Math.abs(cur.d1 - last.d1) > epsilon ||
               Math.abs(cur.d2 - last.d2) > epsilon ||
               Math.abs(cur.d3 - last.d3) > epsilon;
    }

    private void initDropIfEnabled() {
        if (!BoxAutoPanels.isDropEnabled()) {
            dropReady = false;
            pcaReady = false;
            return;
        }
        if (!dropReady) {
            try {
                drop1 = hardwareMap.get(Servo.class, BoxAutoPanels.NAME_DROP1);
                dropReady = true;
            } catch (Exception e) {
                dropReady = false;
            }
        }
        if (!pcaReady) {
            try {
                pca9685 = hardwareMap.get(PCA9685.class, BoxAutoPanels.NAME_PCA9685);
                pcaReady = true;
            } catch (Exception e) {
                pcaReady = false;
            }
        }
    }

    private void applyPickSet(BoxAutoPanels.PickServoSet set) {
        s1.setPosition(set.s1);
        s2.setPosition(set.s2);
        s3.setPosition(set.s3);
        s4.setPosition(set.s4);
        s5.setPosition(set.s5);
    }

    private void applyDropSet(BoxAutoPanels.DropServoSet set) {
        if (dropReady && drop1 != null) {
            drop1.setPosition(set.d1);
        }
        if (pcaReady && pca9685 != null) {
            pca9685.setServoAngle(BoxAutoPanels.DROP2_PCA_CHANNEL, set.d2 * 180.0);
            pca9685.setServoAngle(BoxAutoPanels.DROP3_PCA_CHANNEL, set.d3 * 180.0);
        }
    }
}