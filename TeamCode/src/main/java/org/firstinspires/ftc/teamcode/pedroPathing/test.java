package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import static com.pedropathing.ivy.Scheduler.*;
import static com.pedropathing.ivy.pedro.PedroCommands.*;
import static com.pedropathing.ivy.groups.Groups.*;
import static com.pedropathing.ivy.commands.Commands.*;

/**
 * ═══════════════════════════════════════════════════════════════
 *  AUTONOMOUS: Quét Limelight → Xác định 4 hộp → Gắp theo thứ tự
 *  Sử dụng Pedro Pathing Ivy Scheduler
 * ═══════════════════════════════════════════════════════════════
 *
 *  LUỒNG CHẠY:
 *  1. Đi tới điểm quan sát → giữ vị trí
 *  2. Bật Limelight → quét N mẫu → phân loại 4 ô (2 tầng)
 *  3. Tắt Limelight
 *  4. Lặp qua thứ tự ưu tiên (box01→box04): gắp hộp tương ứng
 *  5. Di chuyển tiếp / về đích
 */
@Autonomous(name = "Limelight Auto - Ivy", group = "Examples")
public class test extends LinearOpMode {

    // ╔══════════════════════════════════════════════════════════╗
    // ║  ZONE 1: TẤT CẢ THÔNG SỐ CẦN TUNE Ở ĐÂY            ║
    // ╚══════════════════════════════════════════════════════════╝

    // ── Tên thiết bị trong Robot Config ──
    private static final String NAME_LIMELIGHT = "limelight";
    private static final int    LL_PIPELINE    = 8;
    private static final String NAME_S1 = "s1";
    private static final String NAME_S2 = "s2";
    private static final String NAME_S3 = "s3";
    private static final String NAME_S4 = "s4";
    private static final String NAME_S5 = "s5";

    // ── Limelight Scan ──
    private static final double TX_THRESHOLD     = 8.0;
    private static final double TY_THRESHOLD     = 7.0;
    private static final int    SAMPLE_COUNT     = 13;     // Tăng mẫu để ổn định hơn
    private static final int    SAMPLE_DELAY     = 80;    // ms giữa mỗi lần chụp
    private static final long   LL_STARTUP_DELAY = 700;   // ms đợi camera ổn định

    // Lọc nhiễu detector. Confidence của Limelight detector là 0-100.
    private static final double MIN_CONFIDENCE   = 35.0;
    private static final double MIN_TARGET_AREA  = 0.05;  // bỏ detection quá nhỏ / xa / nhiễu
    private static final int    MIN_SLOT_VOTES   = 3;     // mỗi ô cần ít nhất N vote mới chấp nhận
    private static final int    MIN_WIN_MARGIN   = 1;     // best phải hơn hạng 2 ít nhất N vote

    // ── Độ chính xác trước khi quét/gắp ──
    // Robot chỉ bật Limelight và chạy servo khi đã vào vùng sai số này đủ lâu.
    private static final double PICKUP_XY_TOLERANCE_IN      = 0.45;  // inch
    private static final double PICKUP_HEADING_TOLERANCE_DEG = 1.5;   // độ
    private static final long   PICKUP_STABLE_TIME_MS        = 350;   // phải ổn định liên tục
    private static final long   PICKUP_TIMEOUT_MS            = 2500;  // chống kẹt nếu không đạt sai số

    // ── Tên 4 hộp (phải khớp với className trong Limelight) ──
    private static final String BOX_1 = "box01";
    private static final String BOX_2 = "box02";
    private static final String BOX_3 = "box03";
    private static final String BOX_4 = "box04";

    // ── Delay chờ servo di chuyển (ms) ──
    private static final int SERVO_DELAY_FAST = 200;
    private static final int SERVO_DELAY_MED  = 400;
    private static final int SERVO_DELAY_SLOW = 600;

    // ═══════════════════════════════════════════════════════════
    //  SERVO HOME (vị trí an toàn khi khởi động & sau khi gắp)
    // ═══════════════════════════════════════════════════════════
    private static final double S1_HOME = 0.5;
    private static final double S2_HOME = 0.5;
    private static final double S3_HOME = 0.5;
    private static final double S4_HOME = 0.5;
    private static final double S5_HOME = 0.5;

    // ═══════════════════════════════════════════════════════════
    //  SERVO POSITIONS CHO 4 Ô
    //  Mỗi ô = 5 giá trị servo riêng
    //  Thứ tự: s1(hạ/nâng), s2(kẹp), s3(cổ tay), s4(kẹp/phụ), s5(nâng/phụ)
    //  Tùy chỉnh con số này theo cơ khí thực tế
    // ═══════════════════════════════════════════════════════════

    // ── Ô TRÊN - TRÁI (TOP_LEFT) ── Tầng 2, bên trái
    private static final double TL_S1 = 0.1;
    private static final double TL_S2 = 0.2;
    private static final double TL_S3 = 0.3;
    private static final double TL_S4 = 0.4;
    private static final double TL_S5 = 0.5;

    // ── Ô TRÊN - PHẢI (TOP_RIGHT) ── Tầng 2, bên phải
    private static final double TR_S1 = 0.2;
    private static final double TR_S2 = 0.3;
    private static final double TR_S3 = 0.4;
    private static final double TR_S4 = 0.5;
    private static final double TR_S5 = 0.6;

    // ── Ô DƯỚI - TRÁI (BOT_LEFT) ── Tầng 1, bên trái
    private static final double BL_S1 = 0.7;
    private static final double BL_S2 = 0.8;
    private static final double BL_S3 = 0.9;
    private static final double BL_S4 = 1.0;
    private static final double BL_S5 = 0.1;

    // ── Ô DƯỚI - PHẢI (BOT_RIGHT) ── Tầng 1, bên phải
    private static final double BR_S1 = 0.5;
    private static final double BR_S2 = 0.6;
    private static final double BR_S3 = 0.7;
    private static final double BR_S4 = 0.8;
    private static final double BR_S5 = 0.9;

    // ╔══════════════════════════════════════════════════════════╗
    // ║  ZONE 2: TỌA ĐỘ DI CHUYỂN (Pedro Pathing)             ║
    // ╚══════════════════════════════════════════════════════════╝
    // Hệ tọa độ Pedro: x, y trong [0, 144] inch, (0,0) = góc dưới-trái sân
    private final Pose pose1 = new Pose(0,  0,  Math.toRadians(0));
    private final Pose pose2 = new Pose(48, 48, Math.toRadians(90));
    private final Pose pose3 = new Pose(48, 96, Math.toRadians(180));
    private final Pose pose4 = new Pose(0,  96, Math.toRadians(180));
    private final Pose pose5 = new Pose(0,  0,  Math.toRadians(0));

    // ╔══════════════════════════════════════════════════════════╗
    // ║  ZONE 3: CODE CHÍNH (không cần tune)                    ║
    // ╚══════════════════════════════════════════════════════════╝

    private Follower follower;
    private Limelight3A limelight;
    private Servo s1, s2, s3, s4, s5;

    private enum Slot { TOP_LEFT, TOP_RIGHT, BOT_LEFT, BOT_RIGHT, UNKNOWN }

    // Kết quả quét
    private String boxTL = "--", boxTR = "--", boxBL = "--", boxBR = "--";
    private String[] lastOrder = new String[4];
    private String[] lastViTri = new String[4];
    private Slot[]   lastSlots = new Slot[4];

    // Trạng thái quét
    private int scanCount = 0;
    private int validDetectionCount = 0;
    private int rejectedDetectionCount = 0;
    private long lastScanTime = 0;
    private long scanStartTime = 0;
    private long stabilizeStartTime = 0;
    private long firstStableTime = 0;
    private Map<String, Integer> voteTL, voteTR, voteBL, voteBR;

    // PathChains
    private PathChain path1to2, path2to3, path3to4, path4to5;

    // ═══════════════════════════════════════════════════════════
    //  XÂY DỰNG ĐƯỜNG CHẠY
    // ═══════════════════════════════════════════════════════════
    public void buildPaths() {
        path1to2 = follower.pathBuilder()
                .addPath(new BezierLine(pose1, pose2))
                .setLinearHeadingInterpolation(pose1.getHeading(), pose2.getHeading())
                .build();
        path2to3 = follower.pathBuilder()
                .addPath(new BezierLine(pose2, pose3))
                .setLinearHeadingInterpolation(pose2.getHeading(), pose3.getHeading())
                .build();
        path3to4 = follower.pathBuilder()
                .addPath(new BezierLine(pose3, pose4))
                .setLinearHeadingInterpolation(pose3.getHeading(), pose4.getHeading())
                .build();
        path4to5 = follower.pathBuilder()
                .addPath(new BezierLine(pose4, pose5))
                .setLinearHeadingInterpolation(pose4.getHeading(), pose5.getHeading())
                .build();
    }

    // ═══════════════════════════════════════════════════════════
    //  HỆ THỐNG VOTE (BẦU CHỌN ĐA SỐ)
    // ═══════════════════════════════════════════════════════════
    private void addVote(Map<String, Integer> map, String label) {
        if (label == null || label.equals("Unknown") || label.isEmpty()) return;
        Integer current = map.get(label);
        map.put(label, (current == null ? 0 : current) + 1);
    }

    private boolean isKnownBox(String label) {
        return BOX_1.equals(label) || BOX_2.equals(label) || BOX_3.equals(label) || BOX_4.equals(label);
    }

    private boolean isGoodDetection(LLResultTypes.DetectorResult det) {
        return det != null
                && isKnownBox(det.getClassName())
                && det.getConfidence() >= MIN_CONFIDENCE
                && det.getTargetArea() >= MIN_TARGET_AREA
                && txtyToSlot(det.getTargetXDegrees(), det.getTargetYDegrees()) != Slot.UNKNOWN;
    }

    private String majority(Map<String, Integer> map) {
        if (map.isEmpty()) return "--";
        String best = "--";
        int bestVotes = 0;
        int secondVotes = 0;

        String[] priority = {BOX_1, BOX_2, BOX_3, BOX_4};
        for (String label : priority) {
            Integer votesObj = map.get(label);
            int votes = votesObj == null ? 0 : votesObj;
            if (votes > bestVotes) {
                secondVotes = bestVotes;
                bestVotes = votes;
                best = label;
            } else if (votes > secondVotes) {
                secondVotes = votes;
            }
        }

        if (bestVotes < MIN_SLOT_VOTES) return "--";
        if (bestVotes - secondVotes < MIN_WIN_MARGIN) return "--";
        return best;
    }

    // ═══════════════════════════════════════════════════════════
    //  COMMAND: CHỜ ROBOT ỔN ĐỊNH TẠI ĐIỂM GẮP/QUÉT
    //  Đây là bước bù sai số nhỏ sau khi Pedro Pathing chạy xong.
    // ═══════════════════════════════════════════════════════════
    private Command waitUntilPoseStable(Pose targetPose) {
        return Command.build()
                .setStart(() -> {
                    stabilizeStartTime = System.currentTimeMillis();
                    firstStableTime = 0;
                    telemetry.addLine("⏳ Đang ổn định robot tại điểm gắp...");
                    telemetry.update();
                })
                .setExecute(() -> {
                    double xyError = getXyError(targetPose);
                    double headingErrorDeg = getHeadingErrorDeg(targetPose);
                    boolean stableNow = xyError <= PICKUP_XY_TOLERANCE_IN
                            && headingErrorDeg <= PICKUP_HEADING_TOLERANCE_DEG;

                    if (stableNow) {
                        if (firstStableTime == 0) {
                            firstStableTime = System.currentTimeMillis();
                        }
                    } else {
                        firstStableTime = 0;
                    }

                    telemetry.addData("Pickup xy error", "%.2f in", xyError);
                    telemetry.addData("Pickup heading error", "%.2f°", headingErrorDeg);
                    telemetry.addData("Stable", stableNow);
                    telemetry.update();
                })
                .setDone(() -> {
                    long now = System.currentTimeMillis();
                    boolean stableLongEnough = firstStableTime != 0
                            && now - firstStableTime >= PICKUP_STABLE_TIME_MS;
                    boolean timedOut = now - stabilizeStartTime >= PICKUP_TIMEOUT_MS;
                    return stableLongEnough || timedOut;
                })
                .setEnd(condition -> telemetry.addLine("✅ Kết thúc bước ổn định robot"));
    }

    private double getXyError(Pose targetPose) {
        double dx = follower.getPose().getX() - targetPose.getX();
        double dy = follower.getPose().getY() - targetPose.getY();
        return Math.sqrt(dx * dx + dy * dy);
    }

    private double getHeadingErrorDeg(Pose targetPose) {
        double error = follower.getPose().getHeading() - targetPose.getHeading();
        while (error > Math.PI) error -= 2.0 * Math.PI;
        while (error < -Math.PI) error += 2.0 * Math.PI;
        return Math.abs(Math.toDegrees(error));
    }

    // ═══════════════════════════════════════════════════════════
    //  PHÂN LOẠI Ô DỰA TRÊN TX, TY
    // ═══════════════════════════════════════════════════════════
    private Slot txtyToSlot(double tx, double ty) {
        if (tx < -TX_THRESHOLD && ty < -TY_THRESHOLD) return Slot.TOP_LEFT;
        if (tx >  TX_THRESHOLD && ty < -TY_THRESHOLD) return Slot.TOP_RIGHT;
        if (tx < -TX_THRESHOLD && ty >  TY_THRESHOLD) return Slot.BOT_LEFT;
        if (tx >  TX_THRESHOLD && ty >  TY_THRESHOLD) return Slot.BOT_RIGHT;
        return Slot.UNKNOWN;
    }

    // ═══════════════════════════════════════════════════════════
    //  COMMAND: QUÉT LIMELIGHT
    //  Bật camera → chụp N mẫu → bầu chọn → tắt camera
    // ═══════════════════════════════════════════════════════════
    public Command scanCommand() {
        return Command.build()
                .setStart(() -> {
                    limelight.start();
                    scanCount = 0;
                    lastScanTime = 0;
                    scanStartTime = System.currentTimeMillis();
                    validDetectionCount = 0;
                    rejectedDetectionCount = 0;
                    voteTL = new HashMap<>();
                    voteTR = new HashMap<>();
                    voteBL = new HashMap<>();
                    voteBR = new HashMap<>();
                    telemetry.addLine("▶ Bật camera, chờ khởi động...");
                    telemetry.update();
                })
                .setExecute(() -> {
                    long now = System.currentTimeMillis();
                    if (now - scanStartTime < LL_STARTUP_DELAY) return;
                    if (now - lastScanTime < SAMPLE_DELAY) return;
                    if (scanCount >= SAMPLE_COUNT) return;

                    LLResult result = limelight.getLatestResult();
                    if (result != null && result.isValid()) {
                        List<LLResultTypes.DetectorResult> detections = result.getDetectorResults();
                        if (detections != null) {
                            for (LLResultTypes.DetectorResult det : detections) {
                                if (!isGoodDetection(det)) {
                                    rejectedDetectionCount++;
                                    continue;
                                }

                                Slot slot  = txtyToSlot(det.getTargetXDegrees(), det.getTargetYDegrees());
                                String lbl = det.getClassName();
                                validDetectionCount++;
                                switch (slot) {
                                    case TOP_LEFT:  addVote(voteTL, lbl); break;
                                    case TOP_RIGHT: addVote(voteTR, lbl); break;
                                    case BOT_LEFT:  addVote(voteBL, lbl); break;
                                    case BOT_RIGHT: addVote(voteBR, lbl); break;
                                }
                            }
                        }
                    }
                    scanCount++;
                    lastScanTime = now;
                    telemetry.addLine("🔍 Quét " + scanCount + "/" + SAMPLE_COUNT
                            + " | OK=" + validDetectionCount
                            + " Reject=" + rejectedDetectionCount);
                    telemetry.update();
                })
                .setDone(() -> scanCount >= SAMPLE_COUNT)
                .setEnd(condition -> {
                    boxTL = majority(voteTL);
                    boxTR = majority(voteTR);
                    boxBL = majority(voteBL);
                    boxBR = majority(voteBR);

                    // Phân loại theo thứ tự ưu tiên
                    String[] priority = {BOX_1, BOX_2, BOX_3, BOX_4};
                    String[] labels   = {boxTL, boxTR, boxBL, boxBR};
                    Slot[]   slots    = {Slot.TOP_LEFT, Slot.TOP_RIGHT, Slot.BOT_LEFT, Slot.BOT_RIGHT};
                    String[] names    = {"Trên-Trái", "Trên-Phải", "Dưới-Trái", "Dưới-Phải"};

                    lastOrder  = new String[4];
                    lastViTri  = new String[4];
                    lastSlots  = new Slot[4];

                    int order = 0;
                    for (String p : priority) {
                        for (int i = 0; i < 4; i++) {
                            if (labels[i] != null && labels[i].equals(p)) {
                                lastOrder[order] = p;
                                lastViTri[order] = names[i];
                                lastSlots[order] = slots[i];
                                order++;
                            }
                        }
                    }
                    limelight.stop();
                    telemetry.addLine("✅ Quét xong! Tắt camera.");
                    telemetry.addData("Valid detections", validDetectionCount);
                    telemetry.addData("Rejected detections", rejectedDetectionCount);
                    telemetry.addData("Votes TL", voteTL.toString());
                    telemetry.addData("Votes TR", voteTR.toString());
                    telemetry.addData("Votes BL", voteBL.toString());
                    telemetry.addData("Votes BR", voteBR.toString());
                    telemetry.update();
                });
    }

    // ═══════════════════════════════════════════════════════════
    //  COMMAND: RESET TẤT CẢ SERVO VỀ HOME
    // ═══════════════════════════════════════════════════════════
    private Command resetServos() {
        return sequential(
            instant(() -> telemetry.addLine("🔄 Reset servo home")),
            instant(() -> s1.setPosition(S1_HOME)),
            instant(() -> s2.setPosition(S2_HOME)),
            instant(() -> s3.setPosition(S3_HOME)),
            instant(() -> s4.setPosition(S4_HOME)),
            instant(() -> s5.setPosition(S5_HOME)),
            waitMs(SERVO_DELAY_MED)
        );
    }

    // ═══════════════════════════════════════════════════════════
    //  COMMAND: GẮP HỘP TẠI 1 Ô CỤ THỂ
    //  Mỗi ô có chuỗi servo riêng + reset home cuối
    // ═══════════════════════════════════════════════════════════
    private Command pickBoxAction(Slot slot) {
        switch (slot) {
            case TOP_LEFT:
                return sequential(
                    instant(() -> telemetry.addLine("📦 Gắp: TRÊN-TRÁI")),
                    instant(() -> s1.setPosition(TL_S1)), waitMs(SERVO_DELAY_MED),
                    instant(() -> s2.setPosition(TL_S2)), waitMs(SERVO_DELAY_SLOW),
                    instant(() -> s3.setPosition(TL_S3)), waitMs(SERVO_DELAY_MED),
                    instant(() -> s4.setPosition(TL_S4)), waitMs(SERVO_DELAY_SLOW),
                    instant(() -> s5.setPosition(TL_S5)), waitMs(SERVO_DELAY_FAST),
                    resetServos()
                );
            case TOP_RIGHT:
                return sequential(
                    instant(() -> telemetry.addLine("📦 Gắp: TRÊN-PHẢI")),
                    instant(() -> s1.setPosition(TR_S1)), waitMs(SERVO_DELAY_MED),
                    instant(() -> s2.setPosition(TR_S2)), waitMs(SERVO_DELAY_SLOW),
                    instant(() -> s3.setPosition(TR_S3)), waitMs(SERVO_DELAY_MED),
                    instant(() -> s4.setPosition(TR_S4)), waitMs(SERVO_DELAY_SLOW),
                    instant(() -> s5.setPosition(TR_S5)), waitMs(SERVO_DELAY_FAST),
                    resetServos()
                );
            case BOT_LEFT:
                return sequential(
                    instant(() -> telemetry.addLine("📦 Gắp: DƯỚI-TRÁI")),
                    instant(() -> s1.setPosition(BL_S1)), waitMs(SERVO_DELAY_MED),
                    instant(() -> s2.setPosition(BL_S2)), waitMs(SERVO_DELAY_SLOW),
                    instant(() -> s3.setPosition(BL_S3)), waitMs(SERVO_DELAY_MED),
                    instant(() -> s4.setPosition(BL_S4)), waitMs(SERVO_DELAY_SLOW),
                    instant(() -> s5.setPosition(BL_S5)), waitMs(SERVO_DELAY_FAST),
                    resetServos()
                );
            case BOT_RIGHT:
                return sequential(
                    instant(() -> telemetry.addLine("📦 Gắp: DƯỚI-PHẢI")),
                    instant(() -> s1.setPosition(BR_S1)), waitMs(SERVO_DELAY_MED),
                    instant(() -> s2.setPosition(BR_S2)), waitMs(SERVO_DELAY_SLOW),
                    instant(() -> s3.setPosition(BR_S3)), waitMs(SERVO_DELAY_MED),
                    instant(() -> s4.setPosition(BR_S4)), waitMs(SERVO_DELAY_SLOW),
                    instant(() -> s5.setPosition(BR_S5)), waitMs(SERVO_DELAY_FAST),
                    resetServos()
                );
            default:
                return instant(() -> telemetry.addLine("⚠ Vị trí không xác định"));
        }
    }

    // ═══════════════════════════════════════════════════════════
    //  COMMAND: CHUỖI GẮP ĐỘNG (dựa trên kết quả quét)
    // ═══════════════════════════════════════════════════════════
    public Command dynamicPickSequence() {
        return lazy(() -> {
            List<Command> picks = new ArrayList<>();
            for (int i = 0; i < 4; i++) {
                if (lastOrder[i] != null && !lastOrder[i].equals("--") && lastSlots[i] != null) {
                    picks.add(pickBoxAction(lastSlots[i]));
                }
            }
            if (picks.isEmpty()) {
                return instant(() -> telemetry.addLine("❌ Không tìm thấy hộp!"));
            }
            return sequential(picks.toArray(new Command[0]));
        });
    }

    // ═══════════════════════════════════════════════════════════
    //  LUỒNG AUTONOMOUS CHÍNH
    // ═══════════════════════════════════════════════════════════
    public Command autoRoutine() {
        return sequential(
            // 1. Đi tới điểm quan sát → giữ vị trí
            follow(follower, path1to2, true),
            // 2. Chờ robot thật sự ổn định tại pose2 trước khi bật Limelight
            waitUntilPoseStable(pose2),
            // 3. Quét Limelight (tự bật/tắt camera)
            scanCommand(),
            // 4. Chờ ổn định thêm lần nữa trước khi chạy servo gắp
            waitUntilPoseStable(pose2),
            // 5. Gắp các hộp theo thứ tự ưu tiên
            dynamicPickSequence(),
            // 6. Di chuyển tiếp (không giữ ở giữa đường)
            follow(follower, path2to3, false),
            follow(follower, path3to4, false),
            // 5. Về đích → giữ vị trí
            follow(follower, path4to5, true)
        );
    }

    // ═══════════════════════════════════════════════════════════
    //  MAIN
    // ═══════════════════════════════════════════════════════════
    @Override
    public void runOpMode() {
        Scheduler.reset();
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(pose1);

        limelight = hardwareMap.get(Limelight3A.class, NAME_LIMELIGHT);
        limelight.pipelineSwitch(LL_PIPELINE);

        s1 = hardwareMap.get(Servo.class, NAME_S1);
        s2 = hardwareMap.get(Servo.class, NAME_S2);
        s3 = hardwareMap.get(Servo.class, NAME_S3);
        s4 = hardwareMap.get(Servo.class, NAME_S4);
        s5 = hardwareMap.get(Servo.class, NAME_S5);

        // Đặt servo về home khi Init
        s1.setPosition(S1_HOME);
        s2.setPosition(S2_HOME);
        s3.setPosition(S3_HOME);
        s4.setPosition(S4_HOME);
        s5.setPosition(S5_HOME);

        telemetry.addLine("═════════════════════════");
        telemetry.addLine(" Limelight Auto - Ivy");
        telemetry.addLine("═════════════════════════");
        telemetry.addData("Threshold TX", TX_THRESHOLD + "°");
        telemetry.addData("Threshold TY", TY_THRESHOLD + "°");
        telemetry.addData("Số mẫu quét", SAMPLE_COUNT);
        telemetry.addData("Min confidence", MIN_CONFIDENCE);
        telemetry.addData("Min area", MIN_TARGET_AREA);
        telemetry.addData("Min slot votes", MIN_SLOT_VOTES);
        telemetry.update();

        waitForStart();
        schedule(autoRoutine());

        while (opModeIsActive()) {
            follower.update();
            Scheduler.execute();

            telemetry.addData("x", "%.1f", follower.getPose().getX());
            telemetry.addData("y", "%.1f", follower.getPose().getY());
            telemetry.addData("heading", "%.1f°", Math.toDegrees(follower.getPose().getHeading()));

            if (scanCount >= SAMPLE_COUNT) {
                telemetry.addLine("─────────────────────────");
                telemetry.addData("TL", boxTL);
                telemetry.addData("TR", boxTR);
                telemetry.addData("BL", boxBL);
                telemetry.addData("BR", boxBR);
                telemetry.addData("Valid/Rejected", validDetectionCount + "/" + rejectedDetectionCount);
                telemetry.addData("Votes TL", voteTL == null ? "--" : voteTL.toString());
                telemetry.addData("Votes TR", voteTR == null ? "--" : voteTR.toString());
                telemetry.addData("Votes BL", voteBL == null ? "--" : voteBL.toString());
                telemetry.addData("Votes BR", voteBR == null ? "--" : voteBR.toString());
                telemetry.addLine("Thứ tự gắp:");
                for (int i = 0; i < 4; i++) {
                    if (lastOrder[i] != null && !lastOrder[i].equals("--")) {
                        telemetry.addData("  " + (i + 1) + ". " + lastOrder[i], "→ " + lastViTri[i]);
                    }
                }
            }
            telemetry.update();
        }

        limelight.stop();
    }
}
