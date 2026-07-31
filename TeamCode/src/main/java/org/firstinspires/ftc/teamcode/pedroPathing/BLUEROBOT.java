package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;
import java.util.List;

import static com.pedropathing.ivy.Scheduler.*;
import static com.pedropathing.ivy.pedro.PedroCommands.*;
import static com.pedropathing.ivy.groups.Groups.*;
import static com.pedropathing.ivy.commands.Commands.*;

/**
 * AUTONOMOUS: 12 Box Auto with Pi YOLO + Pedro Pathing.
 *
 * Workflow:
 *   1. P0 (Start) → P1 (Shelf 1)
 *   2. SCAN YOLO once at Shelf 1 to get all 4 boxes position.
 *   3. Pick 4 boxes one-by-one from Shelf 1, store in compartments 1-4 by type.
 *   4. P1 → P2 (Shelf 2)
 *   5. Pick 4 boxes from Shelf 2 (no scan, same layout).
 *   6. P2 → P3 (Shelf 3)
 *   7. Pick 4 boxes from Shelf 3 (no scan, same layout).
 *   8. Robot now holds 12 boxes (4 compartments × 3 boxes each).
 *   9. Go to P4 - P7 (Drop 1-4) in order, drop 3 boxes of matching type at each zone.
 *  10. Go to P8 (Extra Pick), pick 1 bonus box.
 *  11. Go to P9 (Extra Drop), drop the bonus box.
 *  12. Drive back to P0 (Start) → Done.
 */
@Autonomous(name = "12 Box Auto - YOLO", group = "Autonomous")
public class BLUEROBOT extends LinearOpMode {

    // --- Hard Constants ---
    private static final int SHELF_COUNT     = 3;
    private static final int BOXES_PER_SHELF = 4;
    private static final int TOTAL_BOXES     = SHELF_COUNT * BOXES_PER_SHELF;
    private static final long PI_SCAN_TIMEOUT_MS = 6000;

    // --- Hardware ---
    private Follower follower;
    private PiVisionSerial piVision;
    private Servo s1, s2, s3, s4, s5; // Pick
    private Servo drop1;              // Drop 1
    private PCA9685 pca;              // Drop 2 & 3

    // --- Runtime States ---
    private int currentShelf   = 0;
    private int boxesCollected = 0;
    private int boxesDelivered = 0;

    // Compartment slot counters (compartments 1 to 4)
    private final int[] compartmentCounters = new int[4];

    // YOLO scan results (1-based index matching slotId 1..4)
    // palletCol[id] / palletRow[id] are col/row coordinates of box identifier 'id'
    // col: 1..2, row: 1..2
    private final int[] palletCol = new int[5];
    private final int[] palletRow = new int[5];

    // Stabilize tracking
    private long stabilizeStartTime = 0;
    private long firstStableTime    = 0;

    @Override
    public void runOpMode() {
        Scheduler.reset();
        BoxAutoPanels.refresh();

        // Follower & Pose
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(BoxAutoPanels.poseStart());

        // Pi USB Serial CP2102
        piVision = new PiVisionSerial();
        boolean piConnected = piVision.connect(hardwareMap.appContext);
        if (piConnected) {
            piVision.start();
        }

        // Initialize Servos
        s1 = hardwareMap.get(Servo.class, BoxAutoPanels.NAME_S1);
        s2 = hardwareMap.get(Servo.class, BoxAutoPanels.NAME_S2);
        s3 = hardwareMap.get(Servo.class, BoxAutoPanels.NAME_S3);
        s4 = hardwareMap.get(Servo.class, BoxAutoPanels.NAME_S4);
        s5 = hardwareMap.get(Servo.class, BoxAutoPanels.NAME_S5);

        drop1 = hardwareMap.get(Servo.class, BoxAutoPanels.NAME_DROP1);
        pca   = hardwareMap.get(PCA9685.class, BoxAutoPanels.NAME_PCA9685);

        // Set initial positions
        s1.setPosition(BoxAutoPanels.S1_HOME);
        s2.setPosition(BoxAutoPanels.S2_HOME);
        s3.setPosition(BoxAutoPanels.S3_HOME);
        s4.setPosition(BoxAutoPanels.S4_HOME);
        s5.setPosition(BoxAutoPanels.S5_HOME);

        drop1.setPosition(BoxAutoPanels.D1_CLOSED);
        pcaSetServo(BoxAutoPanels.DROP2_PCA_CHANNEL, BoxAutoPanels.D2_CLOSED);
        pcaSetServo(BoxAutoPanels.DROP3_PCA_CHANNEL, BoxAutoPanels.D3_CLOSED);

        telemetry.addLine("=== 12 BOX AUTO READY ===");
        telemetry.addData("Pi connected", piConnected ? "OK" : "FAILED");
        telemetry.addData("Drop mechanism", BoxAutoPanels.ENABLE_DROP_MECHANISM ? "ENABLED" : "DISABLED");
        telemetry.update();

        waitForStart();

        BoxAutoPanels.refresh();
        schedule(autoRoutine());

        while (opModeIsActive()) {
            follower.update();
            Scheduler.execute();

            Pose pose = follower.getPose();
            telemetry.addData("X", "%.1f in", pose.getX());
            telemetry.addData("Y", "%.1f in", pose.getY());
            telemetry.addData("H", "%.1f deg", Math.toDegrees(pose.getHeading()));
            telemetry.addData("Shelf", currentShelf);
            telemetry.addData("Collected", boxesCollected + "/" + TOTAL_BOXES);
            telemetry.addData("Delivered", boxesDelivered + "/" + TOTAL_BOXES);
            telemetry.addData("Slots", "C1=" + compartmentCounters[0] +
                                     " C2=" + compartmentCounters[1] +
                                     " C3=" + compartmentCounters[2] +
                                     " C4=" + compartmentCounters[3]);
            telemetry.update();
        }

        piVision.sendCommand("STOP");
        piVision.stop();
        piVision.close();
    }

    // ═══════════════════════════════════════════════════════════
    //  AUTO SEQUENCE
    // ═══════════════════════════════════════════════════════════

    public Command autoRoutine() {
        return sequential(
                // 1. P0 → P1 (Shelf 1)
                driveToPose(BoxAutoPanels.poseShelf1()),
                waitUntilStable(BoxAutoPanels.poseShelf1()),

                // 2. Scan YOLO
                scanShelfCommand(),

                // 3. Shelf 1 Pick Routine
                shelfPickRoutine(1, BoxAutoPanels.poseShelf1()),

                // 4. P1 → P2 (Shelf 2)
                driveToPose(BoxAutoPanels.poseShelf2()),
                shelfPickRoutine(2, BoxAutoPanels.poseShelf2()),

                // 5. P2 → P3 (Shelf 3)
                driveToPose(BoxAutoPanels.poseShelf3()),
                shelfPickRoutine(3, BoxAutoPanels.poseShelf3()),

                // 6. Deliver sorted boxes (Drop zones P4 to P7)
                deliverSortedBoxes(),

                // 7. Extra Bonus pickup (P8) and drop (P9)
                extraBonusRound(),

                // 8. Return home
                driveToPose(BoxAutoPanels.poseStart()),
                waitUntilStable(BoxAutoPanels.poseStart())
        );
    }

    // ═══════════════════════════════════════════════════════════
    //  SHELF PICK ROUTINE
    // ═══════════════════════════════════════════════════════════

    private Command shelfPickRoutine(int shelfNum, Pose shelfPose) {
        return lazy(() -> {
            List<Command> steps = new ArrayList<>();
            steps.add(instant(() -> currentShelf = shelfNum));
            steps.add(waitUntilStable(shelfPose));
            steps.add(instant(() -> follower.setPose(shelfPose))); // Absolute snap

            // Walk through all 4 slots in order (palletId 1 to 4)
            for (int palletId = 1; palletId <= BOXES_PER_SHELF; palletId++) {
                final int pid = palletId;
                steps.add(lazy(() -> {
                    int col = palletCol[pid]; // 1=Left, 2=Right
                    int row = palletRow[pid]; // 1=Bottom, 2=Top
                    int compIdx = pid - 1; // 4 compartments (0..3)

                    double colSwingAngle = (col == 1) ? BoxAutoPanels.S5_LEFT : BoxAutoPanels.S5_RIGHT;
                    double liftRowHeight = (row == 1) ? BoxAutoPanels.S1_ROW1 : BoxAutoPanels.S1_ROW2;
                    double extendRowLength = BoxAutoPanels.S2_EXTEND;

                    return sequential(
                            instant(() -> telemetry.log().add("Gap slotId " + pid + " (C" + col + "R" + row + ")")),

                            // 1. Servo 4 quay ra trước để chuẩn bị gắp
                            instant(() -> s4.setPosition(BoxAutoPanels.S4_GRAB)),
                            waitMs(BoxAutoPanels.PICK_SERVO_STEP_DELAY),

                            // 2. Servo 5 di chuyển sang vị trí kệ trái hoặc phải
                            instant(() -> s5.setPosition(colSwingAngle)),
                            waitMs(BoxAutoPanels.PICK_SERVO_STEP_DELAY),

                            // 3. Servo 1 hạ xuống nâng hạ ở tầng 1 hoặc tầng 2
                            instant(() -> s1.setPosition(liftRowHeight)),
                            waitMs(BoxAutoPanels.PICK_SERVO_STEP_DELAY),

                            // 4. Servo 2 vươn ra tiếp cận hộp
                            instant(() -> s2.setPosition(BoxAutoPanels.clampS2(extendRowLength))),
                            waitMs(BoxAutoPanels.PICK_SERVO_STEP_DELAY),

                            // 5. Servo 3 kẹp chặt hộp
                            instant(() -> s3.setPosition(BoxAutoPanels.S3_CLOSED)),
                            waitMs(BoxAutoPanels.GRIPPER_SETTLE_MS),

                            // 6. Servo 1 nâng lên 1 chút (tránh chạm khay kệ)
                            instant(() -> s1.setPosition(liftRowHeight + BoxAutoPanels.S1_LIFT_UP_OFFSET)),
                            waitMs(BoxAutoPanels.PICK_SERVO_STEP_DELAY),

                            // 7. Servo 2 kéo cánh tay vươn về
                            instant(() -> s2.setPosition(BoxAutoPanels.S2_HOME)),
                            waitMs(BoxAutoPanels.PICK_SERVO_STEP_DELAY),

                            // 8. Servo 1 nâng lên vị trí cao để tránh va vướng cửa robot
                            instant(() -> s1.setPosition(BoxAutoPanels.S1_HIGH_STORE)),
                            waitMs(BoxAutoPanels.PICK_SERVO_STEP_DELAY),

                            // 9. Servo 4 quay ra đằng sau để cất khay thả
                            instant(() -> s4.setPosition(BoxAutoPanels.S4_STORE)),
                            waitMs(BoxAutoPanels.PICK_SERVO_STEP_DELAY),

                            // 10. Servo 5 quẹo đúng khoang chứa (1..4)
                            instant(() -> s5.setPosition(getS5StoreAngle(compIdx + 1))),
                            waitMs(BoxAutoPanels.PICK_SERVO_STEP_DELAY),

                            // 11. Servo 1 và Servo 2 căn chỉnh đặt vị trí cất hộp trong robot
                            instant(() -> {
                                s1.setPosition(BoxAutoPanels.S1_STORE);
                                s2.setPosition(BoxAutoPanels.clampS2(BoxAutoPanels.S2_STORE));
                            }),
                            waitMs(BoxAutoPanels.PICK_SERVO_STEP_DELAY),

                            // 12. Servo 3 nhả kẹp xả vô ngăn chứa
                            instant(() -> s3.setPosition(BoxAutoPanels.S3_OPEN)),
                            waitMs(BoxAutoPanels.GRIPPER_SETTLE_MS),

                            // 13. Thu dọn cánh tay gắp về an toàn chuẩn bị lượt sau
                            instant(() -> {
                                s2.setPosition(BoxAutoPanels.S2_HOME);
                                s1.setPosition(BoxAutoPanels.S1_HOME);
                                s5.setPosition(BoxAutoPanels.S5_HOME);
                            }),
                            waitMs(BoxAutoPanels.PICK_SERVO_STEP_DELAY),
                            instant(() -> {
                                s4.setPosition(BoxAutoPanels.S4_GRAB);
                                compartmentCounters[compIdx]++;
                                boxesCollected++;
                            }),
                            waitMs(BoxAutoPanels.PICK_SERVO_STEP_DELAY)
                    );
                }));
            }

            return sequential(steps.toArray(new Command[0]));
        });
    }

    private double getS5StoreAngle(int compartment) {
        switch (compartment) {
            case 1: return BoxAutoPanels.S5_STORE1;
            case 2: return BoxAutoPanels.S5_STORE2;
            case 3: return BoxAutoPanels.S5_STORE3;
            case 4: return BoxAutoPanels.S5_STORE4;
            default: return BoxAutoPanels.S5_HOME;
        }
    }

    // ═══════════════════════════════════════════════════════════
    //  DELIVERY ROUTINE
    // ═══════════════════════════════════════════════════════════

    private Command deliverSortedBoxes() {
        return lazy(() -> {
            if (!BoxAutoPanels.ENABLE_DROP_MECHANISM) {
                return instant(() -> telemetry.log().add("Drop mechanism disabled, skipping delivery."));
            }

            List<Command> steps = new ArrayList<>();

            // Compartment 1 (Type 1) → P4 (DROP1)
            steps.add(deliverCompartment(1, BoxAutoPanels.poseDrop1()));

            // Compartment 2 (Type 2) → P5 (DROP2)
            steps.add(deliverCompartment(2, BoxAutoPanels.poseDrop2()));

            // Compartment 3 (Type 3) → P6 (DROP3)
            steps.add(deliverCompartment(3, BoxAutoPanels.poseDrop3()));

            // Compartment 4 (Type 4) → P7 (DROP4)
            steps.add(deliverCompartment(4, BoxAutoPanels.poseDrop4()));

            return sequential(steps.toArray(new Command[0]));
        });
    }

    private Command deliverCompartment(int compartment, Pose dropPose) {
        return lazy(() -> {
            int boxCount = compartmentCounters[compartment - 1];
            if (boxCount <= 0) return instant(() -> {}); // Empty

            List<Command> steps = new ArrayList<>();
            steps.add(driveToPose(dropPose));
            steps.add(waitUntilStable(dropPose));

            // Release all boxes in this compartment (2-step drop door open/close)
            for (int i = 0; i < boxCount; i++) {
                steps.add(moveDropServosSeq(BoxAutoPanels.dropOpen(compartment)));
                steps.add(instant(() -> {
                    compartmentCounters[compartment - 1]--;
                    boxesDelivered++;
                }));
                steps.add(moveDropServosSeq(BoxAutoPanels.dropClosed()));
            }

            return sequential(steps.toArray(new Command[0]));
        });
    }

    // ═══════════════════════════════════════════════════════════
    //  EXTRA BONUS ROUND (P8 & P9)
    // ═══════════════════════════════════════════════════════════

    private Command extraBonusRound() {
        return sequential(
                // Go to extra pickup P8
                driveToPose(BoxAutoPanels.poseExtraPick()),
                waitUntilStable(BoxAutoPanels.poseExtraPick()),

                // Pick action
                movePickServosSeq(BoxAutoPanels.pickExtra()),
                waitMs(BoxAutoPanels.GRIPPER_SETTLE_MS),
                movePickServosSeq(BoxAutoPanels.pickHome()),

                // Go to extra drop P9
                driveToPose(BoxAutoPanels.poseExtraDrop()),
                waitUntilStable(BoxAutoPanels.poseExtraDrop()),

                // Drop action via pick arm
                movePickServosSeq(BoxAutoPanels.dropExtra()),
                waitMs(BoxAutoPanels.GRIPPER_SETTLE_MS),
                movePickServosSeq(BoxAutoPanels.pickHome())
        );
    }

    // ═══════════════════════════════════════════════════════════
    //  YOLO SCAN COMMAND
    // ═══════════════════════════════════════════════════════════

    private Command scanShelfCommand() {
        return Command.build()
                .setStart(() -> {
                    // Reset grid maps
                    for (int i = 1; i <= 4; i++) {
                        palletCol[i] = 0;
                        palletRow[i] = 0;
                    }
                    piVision.clearResult();
                    piVision.sendCommand("SCAN");
                    stabilizeStartTime = System.currentTimeMillis();
                })
                .setExecute(() -> {
                    PiVisionSerial.ScanResult r = piVision.getLatestResult();
                    if (r == null) {
                        long elapsed = System.currentTimeMillis() - stabilizeStartTime;
                        telemetry.addData("Scan", "Waiting for Pi YOLO... %d ms", elapsed);
                    }
                })
                .setDone(() -> piVision.getLatestResult() != null ||
                        (System.currentTimeMillis() - stabilizeStartTime > PI_SCAN_TIMEOUT_MS))
                .setEnd(condition -> {
                    // Fallback default mapping in case scan fails or timeout
                    // P1: C1R1, P2: C2R1, P3: C1R2, P4: C2R2
                    palletCol[1] = 1; palletRow[1] = 1;
                    palletCol[2] = 2; palletRow[2] = 1;
                    palletCol[3] = 1; palletRow[3] = 2;
                    palletCol[4] = 2; palletRow[4] = 2;

                    PiVisionSerial.ScanResult r = piVision.getLatestResult();
                    if (r != null) {
                        for (PiVisionSerial.PiDetection det : r.detections) {
                            int id = det.getSlotId();
                            if (id >= 1 && id <= 4) {
                                palletCol[id] = det.getCol();
                                palletRow[id] = det.getRow();
                            }
                        }
                        if (r.success) {
                            telemetry.log().add("Pi YOLO scan: success!");
                        } else {
                            telemetry.log().add("Pi YOLO error: " + r.errorReason + " (Fallback applied)");
                        }
                    } else {
                        telemetry.log().add("Pi YOLO scan timeout! (Fallback applied)");
                    }
                });
    }

    // ═══════════════════════════════════════════════════════════
    //  SERVO ACTION COMMAND GENERATORS
    // ═══════════════════════════════════════════════════════════

    private Command movePickServosSeq(BoxAutoPanels.PickServoSet set) {
        return sequential(
                instant(() -> s1.setPosition(set.s1)), waitMs(BoxAutoPanels.PICK_SERVO_STEP_DELAY),
                instant(() -> s2.setPosition(set.s2)), waitMs(BoxAutoPanels.PICK_SERVO_STEP_DELAY),
                instant(() -> s3.setPosition(set.s3)), waitMs(BoxAutoPanels.PICK_SERVO_STEP_DELAY),
                instant(() -> s4.setPosition(set.s4)), waitMs(BoxAutoPanels.PICK_SERVO_STEP_DELAY),
                instant(() -> s5.setPosition(set.s5)), waitMs(BoxAutoPanels.PICK_SERVO_STEP_DELAY)
        );
    }

    private Command moveDropServosSeq(BoxAutoPanels.DropServoSet set) {
        return sequential(
                instant(() -> drop1.setPosition(set.d1)),
                waitMs(BoxAutoPanels.DROP_SERVO_STEP_DELAY),
                instant(() -> pcaSetServo(BoxAutoPanels.DROP2_PCA_CHANNEL, set.d2)),
                waitMs(BoxAutoPanels.DROP_SERVO_STEP_DELAY),
                instant(() -> pcaSetServo(BoxAutoPanels.DROP3_PCA_CHANNEL, set.d3)),
                waitMs(BoxAutoPanels.DROP_SERVO_STEP_DELAY)
        );
    }

    private void pcaSetServo(int channel, double position) {
        double pulseUs = 500 + Math.max(0.0, Math.min(1.0, position)) * 2000;
        pca.setServoPulseUs(channel, pulseUs);
    }

    // ═══════════════════════════════════════════════════════════
    //  NAVIGATION HELPERS
    // ═══════════════════════════════════════════════════════════

    private Command driveToPose(Pose target) {
        return lazy(() -> {
            Pose current = follower.getPose();
            com.pedropathing.paths.PathChain path = follower.pathBuilder()
                    .addPath(new BezierLine(current, target))
                    .setLinearHeadingInterpolation(current.getHeading(), target.getHeading())
                    .build();
            return follow(follower, path, true);
        });
    }

    private Command waitUntilStable(Pose targetPose) {
        return Command.build()
                .setStart(() -> {
                    stabilizeStartTime = System.currentTimeMillis();
                    firstStableTime    = 0;
                })
                .setExecute(() -> {
                    BoxAutoPanels.refresh();
                    Pose cur = follower.getPose();
                    double dx = cur.getX() - targetPose.getX();
                    double dy = cur.getY() - targetPose.getY();
                    double xyError = Math.sqrt(dx * dx + dy * dy);

                    double hError = cur.getHeading() - targetPose.getHeading();
                    while (hError >  Math.PI) hError -= 2 * Math.PI;
                    while (hError < -Math.PI) hError += 2 * Math.PI;
                    double hErrorDeg = Math.abs(Math.toDegrees(hError));

                    boolean stable = xyError   <= BoxAutoPanels.POSE_XY_TOLERANCE_IN
                                  && hErrorDeg <= BoxAutoPanels.POSE_HEADING_TOLERANCE_DEG;

                    if (stable) {
                        if (firstStableTime == 0) {
                            firstStableTime = System.currentTimeMillis();
                        }
                    } else {
                        firstStableTime = 0;
                    }
                })
                .setDone(() -> {
                    long now = System.currentTimeMillis();
                    boolean hasSettled = (firstStableTime != 0 &&
                            now - firstStableTime >= BoxAutoPanels.POSE_STABLE_TIME_MS);
                    boolean isTimeout = (now - stabilizeStartTime >= BoxAutoPanels.POSE_TIMEOUT_MS);
                    return hasSettled || isTimeout;
                });
    }

    private int colRowToSlotIndex(int col, int row) {
        // Col: 1..2, Row: 1..2
        // SlotIndex mapping: TL=0, TR=1, BL=2, BR=3
        if (row == 2 && col == 1) return 0; // Top-Left
        if (row == 2 && col == 2) return 1; // Top-Right
        if (row == 1 && col == 1) return 2; // Bottom-Left
        if (row == 1 && col == 2) return 3; // Bottom-Right
        return 0; // Fallback
    }
}
