package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

@TeleOp(name = "Neural Read Only", group = "Test")
public class testauto extends LinearOpMode {

    private Limelight3A limelight;

    static final double TX_THRESHOLD = 8.0;
    static final double TY_THRESHOLD = 7.0;
    static final int    SAMPLE_COUNT = 5;   // số lần chụp
    static final int    SAMPLE_DELAY = 80;  // ms giữa mỗi lần chụp

    enum Slot { TOP_LEFT, TOP_RIGHT, BOT_LEFT, BOT_RIGHT, UNKNOWN }

    String tl = "--", tr = "--", bl = "--", br = "--";
    String[] lastOrder = new String[0];
    String[] lastViTri = new String[0];
    boolean hasResult = false;
    boolean lastA = false;

    @Override
    public void runOpMode() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(8);
        limelight.start();

        telemetry.addLine("Sẵn sàng! Bấm [A] để quét " + SAMPLE_COUNT + " lần.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            boolean currentA = gamepad1.a;

            if (currentA && !lastA) {

                // ── Voting maps cho từng slot ──
                Map<String, Integer> voteTL = new HashMap<>();
                Map<String, Integer> voteTR = new HashMap<>();
                Map<String, Integer> voteBL = new HashMap<>();
                Map<String, Integer> voteBR = new HashMap<>();

                // ── Chụp SAMPLE_COUNT lần ──
                for (int s = 0; s < SAMPLE_COUNT; s++) {
                    telemetry.addLine("Đang quét... " + (s + 1) + "/" + SAMPLE_COUNT);
                    telemetry.update();

                    sleep(SAMPLE_DELAY);

                    LLResult result = limelight.getLatestResult();
                    if (result == null || !result.isValid()) continue;

                    List<LLResultTypes.DetectorResult> detections = result.getDetectorResults();
                    if (detections == null || detections.isEmpty()) continue;

                    for (LLResultTypes.DetectorResult det : detections) {
                        Slot slot  = txtyToSlot(det.getTargetXDegrees(), det.getTargetYDegrees());
                        String lbl = det.getClassName();

                        switch (slot) {
                            case TOP_LEFT:  addVote(voteTL, lbl); break;
                            case TOP_RIGHT: addVote(voteTR, lbl); break;
                            case BOT_LEFT:  addVote(voteBL, lbl); break;
                            case BOT_RIGHT: addVote(voteBR, lbl); break;
                        }
                    }
                }

                // ── Lấy kết quả đa số mỗi slot ──
                tl = majority(voteTL);
                tr = majority(voteTR);
                bl = majority(voteBL);
                br = majority(voteBR);

                // ── Tính thứ tự lấy ──
                String[] priority  = {"box01", "box02", "box03", "box04"};
                String[] slotNames = {tl, tr, bl, br};
                String[] viTriAll  = {"Trên-Trái", "Trên-Phải", "Dưới-Trái", "Dưới-Phải"};

                lastOrder = new String[4];
                lastViTri = new String[4];
                int order = 0;
                for (String p : priority) {
                    for (int i = 0; i < 4; i++) {
                        if (slotNames[i].equals(p)) {
                            lastOrder[order] = p;
                            lastViTri[order] = viTriAll[i];
                            order++;
                        }
                    }
                }

                hasResult = true;
            }

            lastA = currentA;

            // ── Hiển thị ──
            telemetry.addLine("Bấm [A] để quét lại (" + SAMPLE_COUNT + " lần)");
            telemetry.addLine("─────────────────────────────");

            if (hasResult) {
                telemetry.addLine("KỆ 1:");
                telemetry.addLine("  Tầng trên: [" + tl + "] [" + tr + "]");
                telemetry.addLine("  Tầng dưới: [" + bl + "] [" + br + "]");
                telemetry.addLine("");
                telemetry.addLine("THỨ TỰ LẤY:");
                for (int i = 0; i < lastOrder.length; i++) {
                    if (lastOrder[i] != null) {
                        telemetry.addData((i + 1) + ". " + lastOrder[i], "→ " + lastViTri[i]);
                    }
                }
            } else {
                telemetry.addLine("(Chưa có kết quả)");
            }

            telemetry.update();
        }

        limelight.stop();
    }

    // Thêm 1 vote cho label
    private void addVote(Map<String, Integer> map, String label) {
        map.put(label, map.getOrDefault(label, 0) + 1);
    }

    // Trả về label có số vote cao nhất, hoặc "--" nếu không có
    private String majority(Map<String, Integer> map) {
        if (map.isEmpty()) return "--";
        String best = "--";
        int max = 0;
        for (Map.Entry<String, Integer> e : map.entrySet()) {
            if (e.getValue() > max) {
                max   = e.getValue();
                best  = e.getKey();
            }
        }
        return best;
    }

    private Slot txtyToSlot(double tx, double ty) {
        if (tx < -TX_THRESHOLD && ty < -TY_THRESHOLD) return Slot.TOP_LEFT;
        if (tx >  TX_THRESHOLD && ty < -TY_THRESHOLD) return Slot.TOP_RIGHT;
        if (tx < -TX_THRESHOLD && ty >  TY_THRESHOLD) return Slot.BOT_LEFT;
        if (tx >  TX_THRESHOLD && ty >  TY_THRESHOLD) return Slot.BOT_RIGHT;
        return Slot.UNKNOWN;
    }
}           