package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import android.hardware.usb.UsbManager;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import org.firstinspires.ftc.robotcontroller.standalone.StandaloneAutoRuntime;

import static com.pedropathing.ivy.Scheduler.*;
import static com.pedropathing.ivy.pedro.PedroCommands.*;
import static com.pedropathing.ivy.groups.Groups.*;
import static com.pedropathing.ivy.commands.Commands.*;

/**
 * AUTONOMOUS: 3 kệ × 4 hộp = 12 hộp.
 *
 * Đã chuyển từ Limelight3A sang YOLO chạy trên Raspberry Pi, gửi kết quả
 * detection về Control Hub qua USB Serial (xem {@link PiVisionSerial}).
 *
 * TẤT CẢ THÔNG SỐ TUNE trên FTControl Panels → class {@link BoxAutoPanels}.
 * Xem trước servo: TeleOp "Servo Panels Tuner".
 */
@Autonomous(name = "12 Box Auto - Ivy (Pi YOLO)", group = "Examples")
public class BLUEROBOT extends LinearOpMode {

    private static final String BOX_1 = "box01";
    private static final String BOX_2 = "box02";
    private static final String BOX_3 = "box03";
    private static final String BOX_4 = "box04";
    private static final    String[] BOX_TYPES = {BOX_1, BOX_2, BOX_3, BOX_4};

    private static final int SHELF_COUNT = 3;
    private static final int BOXES_PER_SHELF = 4;
    private static final int TOTAL_BOXES = SHELF_COUNT * BOXES_PER_SHELF;

    /** Nếu không nhận được frame mới từ Pi trong khoảng này thì coi như mất kết nối. */
    private static final long PI_FRAME_STALE_MS = 500;

    private enum Slot { TOP_LEFT, TOP_RIGHT, BOT_LEFT, BOT_RIGHT, UNKNOWN }

    private Follower follower;
    private PiVisionSerial piVision;
    private Servo s1, s2, s3, s4, s5;
    private com.qualcomm.robotcore.hardware.DigitalChannel lineS1, lineS2, lineS3, lineS4, lineS5;
    private Servo drop1;
    private PCA9685 pca9685;

    private Pose poseStart;
    private Pose poseShelf1;
    private Pose poseShelf2;
    private Pose poseShelf3;

    private String boxTL = "--", boxTR = "--", boxBL = "--", boxBR = "--";
    private String[] pickOrder = new String[BOXES_PER_SHELF];
    private String[] pickViTri = new String[BOXES_PER_SHELF];
    private Slot[] pickSlots = new Slot[BOXES_PER_SHELF];

    private int scanCount = 0;
    private int validDetectionCount = 0;
    private int rejectedDetectionCount = 0;
    private int boxesCollected = 0;
    private int boxesDelivered = 0;
    /** 0=box01, 1=box02, 2=box03, 3=box04 */
    private final int[] zoneCounters = new int[4];
    // --- TỌA ĐỘ CHUẨN SÂN 4000mm x 2000mm (Đơn vị: Inch) ---
    // Mỗi khi chạm vạch kẻ ngang/dọc tương ứng, robot sẽ reset về các số này.
    public static Pose POSE_START = new Pose(10, 20, 0);
    
    // Giả định các kệ hàng nằm dọc theo trục X = 120 inch
    public static Pose POSE_SHELF_1 = new Pose(120, 20, 0); 
    public static Pose POSE_SHELF_2 = new Pose(120, 40, 0);
    public static Pose POSE_SHELF_3 = new Pose(120, 60, 0);
    
    public static Pose POSE_DEPOSIT_ZONE = new Pose(40, 40, 0); // Khu vực ném mẫu vật

    private int currentShelf = 0;
    private long lastScanTime = 0;
    private long scanStartTime = 0;
    private long stabilizeStartTime = 0;
    private long firstStableTime = 0;
    private Map<String, Integer> voteTL, voteTR, voteBL, voteBR;

    private PathChain pathStartToShelf1;
    private PathChain pathShelf1ToShelf2;
    private PathChain pathShelf2ToShelf3;

    private void refreshPanels() {
        BoxAutoPanels.refresh();
    }

    private void loadPosesFromPanels() {
        poseStart = BoxAutoPanels.poseStart();
        poseShelf1 = BoxAutoPanels.poseShelf1();
        poseShelf2 = BoxAutoPanels.poseShelf2();
        poseShelf3 = BoxAutoPanels.poseShelf3();
    }

    private void buildPaths() {
        pathStartToShelf1 = follower.pathBuilder()
                .addPath(new BezierLine(poseStart, poseShelf1))
                .setLinearHeadingInterpolation(poseStart.getHeading(), poseShelf1.getHeading())
                .build();
        pathShelf1ToShelf2 = follower.pathBuilder()
                .addPath(new BezierLine(poseShelf1, poseShelf2))
                .setLinearHeadingInterpolation(poseShelf1.getHeading(), poseShelf2.getHeading())
                .build();
        pathShelf2ToShelf3 = follower.pathBuilder()
                .addPath(new BezierLine(poseShelf2, poseShelf3))
                .setLinearHeadingInterpolation(poseShelf2.getHeading(), poseShelf3.getHeading())
                .build();

    }

    private int slotIndex(Slot slot) {
        switch (slot) {
            case TOP_LEFT:  return 0;
            case TOP_RIGHT: return 1;
            case BOT_LEFT:  return 2;
            case BOT_RIGHT: return 3;
            default:        return -1;
        }
    }

    private int boxTypeToZoneIndex(String boxType) {
        switch (boxType) {
            case BOX_1: return 0;
            case BOX_2: return 1;
            case BOX_3: return 2;
            case BOX_4: return 3;
            default:    return -1;
        }
    }

    private String zoneIndexToBoxType(int zoneIdx) {
        if (zoneIdx >= 0 && zoneIdx < BOX_TYPES.length) return BOX_TYPES[zoneIdx];
        return "--";
    }

    private void applyPickSet(BoxAutoPanels.PickServoSet set) {
        s1.setPosition(set.s1);
        s2.setPosition(set.s2);
        s3.setPosition(set.s3);
        s4.setPosition(set.s4);
        s5.setPosition(set.s5);
    }

    private void applyDropSet(BoxAutoPanels.DropServoSet set) {
        if (drop1 != null) {
            drop1.setPosition(set.d1);
        }
        if (pca9685 == null) return;
        pca9685.setServoAngle(BoxAutoPanels.DROP2_PCA_CHANNEL, set.d2 * 180.0);
        pca9685.setServoAngle(BoxAutoPanels.DROP3_PCA_CHANNEL, set.d3 * 180.0);
    }

    private Command movePickServosSequential(BoxAutoPanels.PickServoSet set) {
        return sequential(
                instant(() -> s1.setPosition(set.s1)), waitMs(BoxAutoPanels.PICK_SERVO_STEP_DELAY),
                instant(() -> s2.setPosition(set.s2)), waitMs(BoxAutoPanels.PICK_SERVO_STEP_DELAY),
                instant(() -> s3.setPosition(set.s3)), waitMs(BoxAutoPanels.PICK_SERVO_STEP_DELAY),
                instant(() -> s4.setPosition(set.s4)), waitMs(BoxAutoPanels.PICK_SERVO_STEP_DELAY),
                instant(() -> s5.setPosition(set.s5)), waitMs(BoxAutoPanels.PICK_SERVO_STEP_DELAY)
        );
    }

    private Command moveDropServosSequential(BoxAutoPanels.DropServoSet set) {
        return sequential(
                instant(() -> {
                    if (drop1 != null) drop1.setPosition(set.d1);
                }), waitMs(BoxAutoPanels.DROP_SERVO_STEP_DELAY),
                instant(() -> {
                    if (pca9685 != null) {
                        pca9685.setServoAngle(BoxAutoPanels.DROP2_PCA_CHANNEL, set.d2 * 180.0);
                    }
                }), waitMs(BoxAutoPanels.DROP_SERVO_STEP_DELAY),
                instant(() -> {
                    if (pca9685 != null) {
                        pca9685.setServoAngle(BoxAutoPanels.DROP3_PCA_CHANNEL, set.d3 * 180.0);
                    }
                }), waitMs(BoxAutoPanels.DROP_SERVO_STEP_DELAY)
        );
    }

    private Command goPickHome() {
        return lazy(() -> {
            refreshPanels();
            return sequential(
                    instant(() -> applyPickSet(BoxAutoPanels.pickHome())),
                    waitMs(BoxAutoPanels.PICK_SERVO_STEP_DELAY)
            );
        });
    }

    private Command goDropHome() {
        return lazy(() -> {
            refreshPanels();
            return sequential(
                    instant(() -> applyDropSet(BoxAutoPanels.dropHome())),
                    waitMs(BoxAutoPanels.DROP_SERVO_STEP_DELAY)
            );
        });
    }

    private Command pickFromShelf(Slot slot, String boxType, String viTri) {
        int idx = slotIndex(slot);
        if (idx < 0) {
            return instant(() -> telemetry.addLine("⚠ Ô kệ không hợp lệ"));
        }
        return lazy(() -> {
            refreshPanels();
            BoxAutoPanels.PickServoSet pick = BoxAutoPanels.pickSlot(idx);
            return sequential(
                    instant(() -> telemetry.addLine(" Gắp " + boxType + " @ " + viTri)),
                    movePickServosSequential(pick)
            );
        });
    }

    private Command depositInRobotZone(String boxType) {
        return lazy(() -> {
            refreshPanels();
            int zoneIdx = boxTypeToZoneIndex(boxType);
            if (zoneIdx < 0) {
                return instant(() -> telemetry.addLine(" Không biết ngăn cho " + boxType));
            }

            BoxAutoPanels.PickServoSet depositSet = BoxAutoPanels.depositZone(zoneIdx + 1);
            return sequential(
                    instant(() -> telemetry.addLine(" Bỏ " + boxType + " vào ngăn " + (zoneIdx + 1))),
                    movePickServosSequential(depositSet),
                    instant(() -> {
                        zoneCounters[zoneIdx]++;
                        boxesCollected++;
                        telemetry.addData("Đã thu", boxesCollected + "/" + TOTAL_BOXES);
                        telemetry.addData("Ngăn " + (zoneIdx + 1), zoneCounters[zoneIdx] + " hộp");
                        telemetry.update();
                    }),
                    goPickHome()
            );
        });
    }

    private Command releaseOneFromZone(int zoneIdx) {
        return lazy(() -> {
            refreshPanels();
            String boxType = zoneIndexToBoxType(zoneIdx);
            return sequential(
                    instant(() -> telemetry.addLine(" Thả " + boxType + " từ ngăn " + (zoneIdx + 1))),
                    moveDropServosSequential(BoxAutoPanels.dropReleaseOnField()),
                    goDropHome(),
                    instant(() -> {
                        if (zoneCounters[zoneIdx] > 0) zoneCounters[zoneIdx]--;
                        boxesDelivered++;
                        telemetry.addData("Đã thả", boxesDelivered + "/" + TOTAL_BOXES);
                        telemetry.addData("Còn ngăn " + (zoneIdx + 1), zoneCounters[zoneIdx] + " hộp");
                        telemetry.update();
                    })
            );
        });
    }

    private void addVote(Map<String, Integer> map, String label) {
        if (label == null || label.equals("Unknown") || label.isEmpty()) return;
        Integer current = map.get(label);
        map.put(label, (current == null ? 0 : current) + 1);
    }

    private boolean isKnownBox(String label) {
        return BOX_1.equals(label) || BOX_2.equals(label) || BOX_3.equals(label) || BOX_4.equals(label);
    }

    /**
     * Phân vùng dựa trên tx/ty normalized (-1..1) do Pi gửi về, thay cho tx/ty độ của Limelight.
     * Ngưỡng TX_THRESHOLD/TY_THRESHOLD trong BoxAutoPanels cần tune lại cho phù hợp thang normalized
     * (ví dụ 0.15 - 0.3 thay vì vài độ).
     */
    private Slot txtyToSlot(double tx, double ty) {
        if (tx < -BoxAutoPanels.TX_THRESHOLD && ty < -BoxAutoPanels.TY_THRESHOLD) return Slot.TOP_LEFT;
        if (tx >  BoxAutoPanels.TX_THRESHOLD && ty < -BoxAutoPanels.TY_THRESHOLD) return Slot.TOP_RIGHT;
        if (tx < -BoxAutoPanels.TX_THRESHOLD && ty >  BoxAutoPanels.TY_THRESHOLD) return Slot.BOT_LEFT;
        if (tx >  BoxAutoPanels.TX_THRESHOLD && ty >  BoxAutoPanels.TY_THRESHOLD) return Slot.BOT_RIGHT;
        return Slot.UNKNOWN;
    }

    private boolean isGoodDetection(PiVisionSerial.PiDetection det) {
        return det != null
                && isKnownBox(det.getClassName())
                && det.getConfidence() >= BoxAutoPanels.MIN_CONFIDENCE
                && det.getTargetArea() >= BoxAutoPanels.MIN_TARGET_AREA
                && txtyToSlot(det.getTargetXDegrees(), det.getTargetYDegrees()) != Slot.UNKNOWN;
    }

    private String majority(Map<String, Integer> map) {
        if (map.isEmpty()) return "--";
        String best = "--";
        int bestVotes = 0;
        int secondVotes = 0;

        for (String label : BOX_TYPES) {
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

        if (bestVotes < BoxAutoPanels.MIN_SLOT_VOTES) return "--";
        if (bestVotes - secondVotes < BoxAutoPanels.MIN_WIN_MARGIN) return "--";
        return best;
    }

    private Command driveToPose(Pose target, boolean holdEnd) {
        return lazy(() -> {
            Pose current = follower.getPose();
            PathChain path = follower.pathBuilder()
                    .addPath(new BezierLine(current, target))
                    .setLinearHeadingInterpolation(current.getHeading(), target.getHeading())
                    .build();
            return follow(follower, path, holdEnd);
        });
    }

    private Command waitUntilPoseStable(Pose targetPose) {
        return Command.build()
                .setStart(() -> {
                    stabilizeStartTime = System.currentTimeMillis();
                    firstStableTime = 0;
                })
                .setExecute(() -> {
                    refreshPanels();
                    double dx = follower.getPose().getX() - targetPose.getX();
                    double dy = follower.getPose().getY() - targetPose.getY();
                    double xyError = Math.sqrt(dx * dx + dy * dy);

                    double hError = follower.getPose().getHeading() - targetPose.getHeading();
                    while (hError > Math.PI) hError -= 2 * Math.PI;
                    while (hError < -Math.PI) hError += 2 * Math.PI;
                    double hErrorDeg = Math.abs(Math.toDegrees(hError));

                    boolean stableNow = xyError <= BoxAutoPanels.POSE_XY_TOLERANCE_IN
                            && hErrorDeg <= BoxAutoPanels.POSE_HEADING_TOLERANCE_DEG;

                    if (stableNow) {
                        if (firstStableTime == 0) firstStableTime = System.currentTimeMillis();
                    } else {
                        firstStableTime = 0;
                    }

                    // Telemetry
                    long now = System.currentTimeMillis();
                    long stableDurationMs = (firstStableTime != 0) ? (now - firstStableTime) : 0;
                    long timeoutRemainingMs = Math.max(0,
                            (long) BoxAutoPanels.POSE_TIMEOUT_MS - (now - stabilizeStartTime));

                    telemetry.addLine("[waitStable]")
                             .addData("Target", "(%.1f, %.1f) H=%.1f",
                                      targetPose.getX(), targetPose.getY(),
                                      Math.toDegrees(targetPose.getHeading()));
                    telemetry.addLine("[waitStable]")
                             .addData("errXY", "%.2f in", xyError)
                             .addData("errH",  "%.1f deg", hErrorDeg);
                    telemetry.addLine("[waitStable]")
                             .addData("stable", "%d / %d ms",
                                      stableDurationMs,
                                      (long) BoxAutoPanels.POSE_STABLE_TIME_MS)
                             .addData("OK", stableNow ? "YES" : "NO");
                    telemetry.addLine("[waitStable]")
                             .addData("timeout in", "%d ms", timeoutRemainingMs);
                })
                .setDone(() -> {
                    long now = System.currentTimeMillis();
                    return (firstStableTime != 0 && now - firstStableTime >= BoxAutoPanels.POSE_STABLE_TIME_MS)
                            || (now - stabilizeStartTime >= BoxAutoPanels.POSE_TIMEOUT_MS);
                });
    }

    public Command scanShelfCommand() {
        return Command.build()
                .setStart(() -> {
                    refreshPanels();
                    piVision.start();
                    piVision.sendCommand("SCAN");
                    scanCount = 0;
                    lastScanTime = 0;
                    scanStartTime = System.currentTimeMillis();
                    validDetectionCount = 0;
                    rejectedDetectionCount = 0;
                    voteTL = new HashMap<>();
                    voteTR = new HashMap<>();
                    voteBL = new HashMap<>();
                    voteBR = new HashMap<>();
                    telemetry.addLine("Quét kệ " + currentShelf + " (Pi YOLO)...");
                    telemetry.update();
                })
                .setExecute(() -> {
                    long now = System.currentTimeMillis();
                    if (now - scanStartTime < BoxAutoPanels.LL_STARTUP_DELAY) return;
                    if (now - lastScanTime < BoxAutoPanels.SAMPLE_DELAY) return;
                    if (scanCount >= BoxAutoPanels.SAMPLE_COUNT) return;

                    // Chỉ lấy mẫu nếu frame từ Pi còn mới (tránh dùng data cũ nếu Pi lag/rớt kết nối)
                    if (piVision.getLatestFrameAgeMs() <= PI_FRAME_STALE_MS) {
                        List<PiVisionSerial.PiDetection> detections = piVision.getLatestDetections();
                        for (PiVisionSerial.PiDetection det : detections) {
                            if (!isGoodDetection(det)) {
                                rejectedDetectionCount++;
                                continue;
                            }
                            Slot slot = txtyToSlot(det.getTargetXDegrees(), det.getTargetYDegrees());
                            validDetectionCount++;
                            switch (slot) {
                                case TOP_LEFT:  addVote(voteTL, det.getClassName()); break;
                                case TOP_RIGHT: addVote(voteTR, det.getClassName()); break;
                                case BOT_LEFT:  addVote(voteBL, det.getClassName()); break;
                                case BOT_RIGHT: addVote(voteBR, det.getClassName()); break;
                            }
                        }
                    }
                    scanCount++;
                    lastScanTime = now;
                })
                .setDone(() -> scanCount >= BoxAutoPanels.SAMPLE_COUNT)
                .setEnd(condition -> {
                    boxTL = majority(voteTL);
                    boxTR = majority(voteTR);
                    boxBL = majority(voteBL);
                    boxBR = majority(voteBR);

                    String[] labels = {boxTL, boxTR, boxBL, boxBR};
                    Slot[] slots = {Slot.TOP_LEFT, Slot.TOP_RIGHT, Slot.BOT_LEFT, Slot.BOT_RIGHT};
                    String[] names = {"Trên-Trái", "Trên-Phải", "Dưới-Trái", "Dưới-Phải"};

                    pickOrder = new String[BOXES_PER_SHELF];
                    pickViTri = new String[BOXES_PER_SHELF];
                    pickSlots = new Slot[BOXES_PER_SHELF];

                    int order = 0;
                    for (String boxType : BOX_TYPES) {
                        for (int i = 0; i < BOXES_PER_SHELF; i++) {
                            if (labels[i] != null && labels[i].equals(boxType)) {
                                pickOrder[order] = boxType;
                                pickViTri[order] = names[i];
                                pickSlots[order] = slots[i];
                                order++;
                            }
                        }
                    }
                    piVision.sendCommand("STOP");
                    piVision.stop();
                    telemetry.addLine(" Kệ " + currentShelf + ": TL=" + boxTL + " TR=" + boxTR
                            + " BL=" + boxBL + " BR=" + boxBR);
                    telemetry.update();
                });
    }

    private Command pickAndDeliverAllOnShelf(Pose shelfPose) {
        return lazy(() -> {
            refreshPanels();
            List<Command> steps = new ArrayList<>();

            for (int i = 0; i < BOXES_PER_SHELF; i++) {
                if (pickOrder[i] == null || pickOrder[i].equals("--") || pickSlots[i] == null) continue;

                final String boxType = pickOrder[i];
                final String viTri = pickViTri[i];
                final Slot slot = pickSlots[i];

                steps.add(waitUntilPoseStable(shelfPose));
                steps.add(pickFromShelf(slot, boxType, viTri));
                steps.add(depositInRobotZone(boxType));
            }
            if (steps.isEmpty()) {
                return instant(() -> telemetry.addLine(" Kệ " + currentShelf + ": không nhận diện hộp"));
            }
            return sequential(steps.toArray(new Command[0]));
        });
    }

    private Command dropAllZones() {
        return lazy(() -> {
            if (!BoxAutoPanels.isDropEnabled()) {
                return instant(() -> telemetry.addLine("ℹ Cơ cấu thả TẮT: chỉ thu hộp, không đi thả"));
            }

            List<Command> steps = new ArrayList<>();
            int[] order = (StandaloneAutoRuntime.getSelectedSide() == StandaloneAutoRuntime.Side.RED)
                    ? new int[]{3, 2, 1, 0}
                    : new int[]{0, 1, 2, 3};

            for (int zone : order) {
                final int zoneIdx = zone;
                final String boxType = zoneIndexToBoxType(zoneIdx);
                final int count = zoneCounters[zoneIdx];
                if (count <= 0) continue;

                final Pose fieldDrop = BoxAutoPanels.poseDropForBox(boxType);
                steps.add(instant(() -> {
                    telemetry.addLine(" Đi thả " + boxType + " từ ngăn " + (zoneIdx + 1));
                    telemetry.addData("Số hộp", count);
                    telemetry.update();
                }));
                steps.add(driveToPose(fieldDrop, true));
                steps.add(waitUntilPoseStable(fieldDrop));
                for (int i = 0; i < count; i++) {
                    steps.add(releaseOneFromZone(zoneIdx));
                }
            }

            if (steps.isEmpty()) {
                return instant(() -> telemetry.addLine(" Không có hộp nào để thả"));
            }
            return sequential(steps.toArray(new Command[0]));
        });
    }

    private Command pickSequenceFixed(Pose shelfPose, boolean isBlueAlliance) {
        return lazy(() -> {
            List<Command> steps = new ArrayList<>();
            steps.add(waitUntilPoseStable(shelfPose));

            for (int i = 1; i <= 4; i++) {
                final int targetSlotId = i;
                steps.add(lazy(() -> {
                    return sequential(
                        instant(() -> {
                            piVision.start();
                            piVision.sendCommand("SCAN");
                        }),
                        waitMs(500),
                        instant(() -> {
                            piVision.sendCommand("STOP");
                            piVision.stop();
                        }),
                        lazy(() -> {
                            int color = piVision.getSlotColor(targetSlotId);
                            int allianceColor = isBlueAlliance ? 3 : 2;

                            if (color == 1 || color == allianceColor) {
                                String boxType = (targetSlotId == 1 || targetSlotId == 4) ? "YELLOW" :
                                                 (targetSlotId == 2 ? "RED" : "BLUE");
                                String viTri = "Slot " + targetSlotId;

                                Slot targetSlot;
                                if (targetSlotId == 1)      targetSlot = Slot.TOP_LEFT;
                                else if (targetSlotId == 2) targetSlot = Slot.TOP_RIGHT;
                                else if (targetSlotId == 3) targetSlot = Slot.BOT_LEFT;
                                else                        targetSlot = Slot.BOT_RIGHT;

                                return sequential(
                                    pickFromShelf(targetSlot, boxType, viTri),
                                    depositInRobotZone(boxType)
                                );
                            } else {
                                return instant(() -> {
                                    telemetry.addLine("Slot " + targetSlotId + " không thỏa mãn màu sắc (Màu: " + color + "). Bỏ qua.");
                                    telemetry.update();
                                });
                            }
                        })
                    );
                }));
            }

            return sequential(steps.toArray(new Command[0]));
        });
    }

    private Command snapToLine(double timeoutMs) {
        return Command.build()
                .setStart(() -> {
                    stabilizeStartTime = System.currentTimeMillis();
                })
                .setExecute(() -> {
                    boolean b2 = !lineS2.getState();
                    boolean b3 = !lineS3.getState();
                    boolean b4 = !lineS4.getState();

                    Pose current = follower.getPose();
                    // Dùng bước nhích cực nhỏ (0.005) để PID của Pedro không bị sốc
                    double step = 0.005; 

                    if (b3) {
                        telemetry.addLine("🎯 Đã khớp tâm vạch!");
                    } else if (b2) {
                        // Nhích Pose cực nhẹ để Pedro tự điều chỉnh motor êm ái
                        follower.setPose(new Pose(current.getX(), current.getY() - step, current.getHeading()));
                        telemetry.addLine("⬅️ Đang nhích sang trái (Damped)...");
                    } else if (b4) {
                        follower.setPose(new Pose(current.getX(), current.getY() + step, current.getHeading()));
                        telemetry.addLine("➡️ Đang nhích sang phải (Damped)...");
                    }
                    telemetry.update();
                })
                .setDone(() -> {
                    return (!lineS3.getState()) || (System.currentTimeMillis() - stabilizeStartTime > timeoutMs);
                });
    }

    private Command alignAtIntersection(double timeoutMs) {
        return Command.build()
                .setStart(() -> {
                    piVision.clearAlignment();
                    piVision.sendCommand("ALIGN_START");
                    stabilizeStartTime = System.currentTimeMillis();
                })
                .setExecute(() -> {
                    com.pedropathing.geometry.Pose alignPose = piVision.getLatestAlignmentPose();
                    if (alignPose != null && piVision.getLatestAlignmentAgeMs() < 500) {
                        follower.setPose(alignPose);
                        telemetry.addLine("✅ Tái định vị thành công!");
                    } else {
                        telemetry.addLine("⌛ Đang chờ tín hiệu giao lộ từ Pi...");
                    }
                    telemetry.update();
                })
                .setDone(() -> piVision.getLatestAlignmentPose() != null || 
                           (System.currentTimeMillis() - stabilizeStartTime > timeoutMs));
    }

    /**
     * Di chuyển và liên tục sửa sai số Y bằng vạch kẻ sàn.
     * Thuật toán lọc tâm: Chỉ reset tọa độ khi robot nằm chính giữa vạch (mắt S3 tâm).
     */
    private Command followLineToPose(PathChain path, Pose targetPose, double absoluteY) {
        return Command.build()
                .setStart(() -> {
                    follower.followPath(path, true);
                })
                .setExecute(() -> {
                    boolean b2 = !lineS2.getState();
                    boolean b3 = !lineS3.getState();
                    boolean b4 = !lineS4.getState();

                    Pose current = follower.getPose();
                    
                    // Nếu robot bị lệch khỏi vạch (b3 không thấy), ta sửa Pose một cách từ từ
                    // Thay vì setPose nhảy vọt, ta chỉ dịch chuyển Pose hiện tại từng chút một
                    // để Pedro tự động điều chỉnh motor êm ái hơn.
                    if (b2 && !b4) {
                        // Lệch sang trái -> Dịch Y ảo để Pedro kéo robot sang phải
                        follower.setPose(new Pose(current.getX(), current.getY() - 0.02, current.getHeading()));
                    } else if (b4 && !b2) {
                        // Lệch sang phải -> Dịch Y ảo để Pedro kéo robot sang trái
                        follower.setPose(new Pose(current.getX(), current.getY() + 0.02, current.getHeading()));
                    } else if (b3) {
                        // Khi đã ở tâm, neo chặt tọa độ Y để triệt tiêu drift
                        follower.setPose(new Pose(current.getX(), absoluteY, current.getHeading()));
                    }

                    telemetry.addData("Line Correction", b3 ? "CENTERED" : (b2 ? "LEFT" : (b4 ? "RIGHT" : "SEARCHING")));
                })
                .setDone(() -> !follower.isBusy());
    }

    private Command shelfRoutine(Pose shelfPose, int shelfNumber) {
        // Đường chạy tiêu chuẩn của Pedro Pathing (An toàn tuyệt đối)
        PathChain pathToShelf = follower.pathBuilder()
                .addPath(new com.pedropathing.geometry.BezierLine(follower.getPose(), shelfPose))
                .setConstantHeadingInterpolation(shelfPose.getHeading())
                .build();

        return sequential(
                instant(() -> {
                    currentShelf = shelfNumber;
                    scanCount = 0;
                }),
                // 1. Chạy tới kệ bằng thuật toán nguyên bản của Pedro (Không can thiệp)
                follow(follower, pathToShelf, true),
                
                // 2. Chờ robot dừng hẳn và ổn định vị trí
                waitUntilPoseStable(shelfPose),
                
                // 3. CHỈ HIỆU CHỈNH KHI ĐÃ DỪNG (Hít vạch nhẹ nhàng)
                // Nếu sau 1.5s không khớp vạch, robot tự động bỏ qua để gắp tiếp
                snapToLine(1500),
                
                // 4. Ép tọa độ chuẩn để xóa drift trước khi gắp
                instant(() -> follower.setPose(shelfPose)),
                
                // 5. Tiến hành gắp mẫu vật
                pickSequenceFixed(shelfPose, StandaloneAutoRuntime.getSelectedSide() != StandaloneAutoRuntime.Side.RED)
        );
    }

    public Command autoRoutine() {
        return sequential(
                // Khởi tạo vị trí xuất phát
                instant(() -> follower.setPose(POSE_START)),
                
                // Thực hiện quy trình tại 3 kệ hàng
                shelfRoutine(POSE_SHELF_1, 1),
                shelfRoutine(POSE_SHELF_2, 2),
                shelfRoutine(POSE_SHELF_3, 3),
                
                // Quay về khu vực ném/xuất phát
                driveToPose(POSE_START, true)
        );
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();
        refreshPanels();
        loadPosesFromPanels();

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(poseStart);

        UsbManager usbManager = (UsbManager) hardwareMap.appContext.getSystemService(android.content.Context.USB_SERVICE);
        piVision = new PiVisionSerial(usbManager);
        
        // Khởi tạo BFD-1000 (Digital I/O)
        lineS1 = hardwareMap.get(com.qualcomm.robotcore.hardware.DigitalChannel.class, "lineS1");
        lineS2 = hardwareMap.get(com.qualcomm.robotcore.hardware.DigitalChannel.class, "lineS2");
        lineS3 = hardwareMap.get(com.qualcomm.robotcore.hardware.DigitalChannel.class, "lineS3");
        lineS4 = hardwareMap.get(com.qualcomm.robotcore.hardware.DigitalChannel.class, "lineS4");
        lineS5 = hardwareMap.get(com.qualcomm.robotcore.hardware.DigitalChannel.class, "lineS5");
        
        lineS1.setMode(com.qualcomm.robotcore.hardware.DigitalChannel.Mode.INPUT);
        lineS2.setMode(com.qualcomm.robotcore.hardware.DigitalChannel.Mode.INPUT);
        lineS3.setMode(com.qualcomm.robotcore.hardware.DigitalChannel.Mode.INPUT);
        lineS4.setMode(com.qualcomm.robotcore.hardware.DigitalChannel.Mode.INPUT);
        lineS5.setMode(com.qualcomm.robotcore.hardware.DigitalChannel.Mode.INPUT);

        boolean piConnected = piVision.connect();

        s1 = hardwareMap.get(Servo.class, BoxAutoPanels.NAME_S1);
        s2 = hardwareMap.get(Servo.class, BoxAutoPanels.NAME_S2);
        s3 = hardwareMap.get(Servo.class, BoxAutoPanels.NAME_S3);
        s4 = hardwareMap.get(Servo.class, BoxAutoPanels.NAME_S4);
        s5 = hardwareMap.get(Servo.class, BoxAutoPanels.NAME_S5);

        if (BoxAutoPanels.isDropEnabled()) {
            drop1 = hardwareMap.get(Servo.class, BoxAutoPanels.NAME_DROP1);
            pca9685 = hardwareMap.get(PCA9685.class, BoxAutoPanels.NAME_PCA9685);
            applyDropSet(BoxAutoPanels.dropHome());
        }

        applyPickSet(BoxAutoPanels.pickHome());

        telemetry.addLine("═══════════════════════════════");
        telemetry.addLine(" 12 Box Auto — Pi YOLO qua USB Serial");
        telemetry.addLine(" Class: BoxAutoPanels");
        telemetry.addLine("═══════════════════════════════");
        telemetry.addData("Pi kết nối", piConnected ? "OK" : "LỖI - kiểm tra cáp USB");
        telemetry.addData("Chiến thuật", "Thu 12 hộp trước → thả theo 4 ngăn");
        telemetry.addData("Cơ cấu thả", BoxAutoPanels.isDropEnabled() ? "BẬT" : "TẮT");
        telemetry.addData("Tổng hộp", TOTAL_BOXES);
        telemetry.update();

        waitForStart();

        refreshPanels();
        loadPosesFromPanels();

        schedule(autoRoutine());

        while (opModeIsActive()) {
            follower.update();
            Scheduler.execute();

            telemetry.addData("x", "%.1f", follower.getPose().getX());
            telemetry.addData("y", "%.1f", follower.getPose().getY());
            telemetry.addData("heading", "%.1f°", Math.toDegrees(follower.getPose().getHeading()));
            telemetry.addData("Kệ", currentShelf);
            telemetry.addData("Đã thu", boxesCollected + "/" + TOTAL_BOXES);
            telemetry.addData("Đã thả", boxesDelivered + "/" + TOTAL_BOXES);
            telemetry.addData("Ngăn", "1=" + zoneCounters[0] + " 2=" + zoneCounters[1]
                    + " 3=" + zoneCounters[2] + " 4=" + zoneCounters[3]);
            telemetry.addData("Pi frame age", piVision.getLatestFrameAgeMs() + " ms");

            if (scanCount >= BoxAutoPanels.SAMPLE_COUNT && currentShelf > 0) {
                telemetry.addLine("─────────────────────────");
                telemetry.addData("Scan", "TL=" + boxTL + " TR=" + boxTR + " BL=" + boxBL + " BR=" + boxBR);
                for (int i = 0; i < BOXES_PER_SHELF; i++) {
                    if (pickOrder[i] != null && !pickOrder[i].equals("--")) {
                        telemetry.addData((i + 1) + ". " + pickOrder[i],
                                pickViTri[i] + " → " + pickOrder[i]);
                    }
                }
            }
            telemetry.update();
        }

        piVision.sendCommand("STOP");
        piVision.close();
    }
}