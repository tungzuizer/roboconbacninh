package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.PwmControl;

import java.util.Locale;

/**
 * BLUEROBOT Autonomous Routine - Fixed Center Gripper without Servo 5
 *  - Turn 1 (Lượt 1): Gắp 1 lần 2 hộp ở Tầng 2 (Row 2), cất vào Trụ 1 & Trụ 2 trong robot, đi thả nhà máy.
 *  - Turn 2 (Lượt 2): Gắp 1 lần 2 hộp ở Tầng 1 (Row 1), cất vào Trụ 1 & Trụ 2 trong robot, đi thả nhà máy.
 */
@Autonomous(name = "Blue Robot Auto (Dual-Pick No S5)", group = "Autonomous")
public class BLUEROBOT extends LinearOpMode {

    private Follower follower;
    private PiVisionSerial piVision;

    // Pick servos (Cố định trục xoay ở giữa robot)
    private Servo s1, s2, s3, s4, s5;
    // Drop servos
    private Servo drop1;
    private PCA9685 pca;

    // Công tắc phần cứng (Hardware Switches)
    private DigitalChannel allianceSwitch;      // Port 0: Công tắc gạt chọn Xanh/Đỏ (DigitalChannel)
    private TouchSensor allianceTouchSensor;   // Port 0: Công tắc TouchSensor fallback
    private DigitalChannel startButton;         // Port 3: Công tắc Digital Channel
    private TouchSensor startTouchSensor;      // Port 3: Công tắc TouchSensor fallback

    // Tracking state
    private long stabilizeStartTime = 0;
    private long firstStableTime = 0;
    private static final long PI_SCAN_TIMEOUT_MS = 4000;
    private boolean isRed = false;

    // S4 debug/command state for slow wrist motion in autonomous.
    private double s4SlowTarget = BoxAutoPanels.S4_HOME;
    private double s4SlowCurrent = BoxAutoPanels.S4_HOME;
    private boolean s4SlowActive = false;

    // Loại hộp gắp được trong mũi lượt đi thả
    private int post1BoxType = 1;
    private int post2BoxType = 2;
    private int turn2Post1BoxType = 3;
    private int turn2Post2BoxType = 4;
    private String lastScanSummary = "Chưa scan";
    private int scanAttempt = 0;
    private boolean skipCurrentShelf = false;

    // --- Dynamic Disconnect Prevention State ---
    private Object originalEventLoop = null;
    private Object eventLoopManagerInstance = null;
    private java.lang.reflect.Field loopField = null;

    @Override
    public void runOpMode() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(BoxAutoPanels.getTunablePose(0));

        // Init vision
        piVision = new PiVisionSerial();
        try {
            piVision.init(hardwareMap);
        } catch (Exception e) {
            telemetry.log().add("PiVisionSerial Init warning: " + e.getMessage());
        }

        // Init hardware switches (Port 0: Alliance, Port 3: Start)
        String[] allianceNames = {"touch0", "allianceSwitch", "port0", "digital0", "switch0"};
        for (String name : allianceNames) {
            try {
                allianceSwitch = hardwareMap.get(DigitalChannel.class, name);
                if (allianceSwitch != null) {
                    allianceSwitch.setMode(DigitalChannel.Mode.INPUT);
                    break;
                }
            } catch (Exception ignored) {}
        }
        if (allianceSwitch == null) {
            for (String name : allianceNames) {
                try {
                    allianceTouchSensor = hardwareMap.get(TouchSensor.class, name);
                    if (allianceTouchSensor != null) break;
                } catch (Exception ignored) {}
            }
        }

        String[] startNames = {"touch3", "startButton", "port3", "digital3", "switch3", "touchSensor3"};
        for (String name : startNames) {
            try {
                startButton = hardwareMap.get(DigitalChannel.class, name);
                if (startButton != null) {
                    startButton.setMode(DigitalChannel.Mode.INPUT);
                    break;
                }
            } catch (Exception ignored) {}
        }

        if (startButton == null) {
            for (String name : startNames) {
                try {
                    startTouchSensor = hardwareMap.get(TouchSensor.class, name);
                    if (startTouchSensor != null) break;
                } catch (Exception ignored) {}
            }
        }

        // Init servos
        s1 = hardwareMap.get(Servo.class, BoxAutoPanels.NAME_S1);
        s2 = hardwareMap.get(Servo.class, BoxAutoPanels.NAME_S2);
        s3 = hardwareMap.get(Servo.class, BoxAutoPanels.NAME_S3);
        s4 = hardwareMap.get(Servo.class, BoxAutoPanels.NAME_S4);
        s5 = hardwareMap.get(Servo.class, BoxAutoPanels.NAME_S5);

        if (s4 instanceof ServoImplEx) {
            ((ServoImplEx) s4).setPwmRange(new PwmControl.PwmRange(500, 2500));
        }

        // Mở rộng dải PWM của Servo S2, S3 và S5 lên tối đa 500us - 2500us để ngàm S3 và S5 hoạt động đồng tốc và cùng lực kẹp
        if (s2 instanceof ServoImplEx) {
            ((ServoImplEx) s2).setPwmRange(new PwmControl.PwmRange(500, 2500));
        }
        if (s3 instanceof ServoImplEx) {
            ((ServoImplEx) s3).setPwmRange(new PwmControl.PwmRange(500, 2500));
        }
        if (s5 instanceof ServoImplEx) {
            ((ServoImplEx) s5).setPwmRange(new PwmControl.PwmRange(500, 2500));
        }

        try { drop1 = hardwareMap.get(Servo.class, BoxAutoPanels.NAME_DROP1); } catch (Exception ignored) {}
        try { pca = hardwareMap.get(PCA9685.class, BoxAutoPanels.NAME_PCA9685); } catch (Exception ignored) {}

        // --- HẠ TỐC ĐỘ ROBOT 60% NHƯNG GIỮ KHỎE LỰC YẾU TỐ PID ---
        follower.setMaxPower(0.4); // Robot chạy 40%, kìm tốc độ lại để bám dính siêu khỏe.

        // Chặn tự động tắt máy khi ngắt Driver Station ngay lập tức từ khi bước vào trạng thái Init
        disableDisconnectAutoStop();

        // ── VÒNG LẶP CHỜ BẤM CÔNG TẮC PORT 3 ĐỂ CHẠY (KHÔNG CẦN DRIVER STATION) ──
        boolean switchTriggered = false;

        while (!isStopRequested()) {
            boolean isTriggered = false;
            if (startButton != null) {
                try {
                    isTriggered = !startButton.getState();
                } catch (Exception ignored) {}
            } else if (startTouchSensor != null) {
                try {
                    isTriggered = startTouchSensor.isPressed();
                } catch (Exception ignored) {}
            }

            isRed = false;
            if (allianceSwitch != null) {
                try {
                    isRed = allianceSwitch.getState();
                } catch (Exception ignored) {}
            } else if (allianceTouchSensor != null) {
                try {
                    isRed = allianceTouchSensor.isPressed();
                } catch (Exception ignored) {}
            }

            telemetry.addLine("=== BLUEROBOT - CHỜ CÔNG TẮC ===");
            telemetry.addData("Alliance (Port 0)", (allianceSwitch != null || allianceTouchSensor != null) ? (isRed ? "🔴 RED" : "🔵 BLUE") : "⚠️ CHƯA TÌM THẤY");
            telemetry.addData("Start (Port 3)", (startButton != null || startTouchSensor != null) ? (isTriggered ? "👉 ĐÃ BẤM!" : "Chờ bấm...") : "⚠️ CHƯA TÌM THẤY");
            telemetry.addLine("-> Bấm công tắc Port 3 để CHẠY.");
            telemetry.update();

            if (isTriggered) {
                switchTriggered = true;
                telemetry.addLine(">>> BẤM CÔNG TẮC! CHẠY TỰ ĐỘNG! <<<");
                telemetry.update();
                break;
            }


            sleep(30);
        }

        if (isStopRequested() || !switchTriggered) return;

        // Bắt đầu gồng lực và đưa các Servo về vị trí HOME chuẩn bị sau khi đã gạt nút
        s4SlowCurrent = s4.getPosition();
        s4SlowTarget = s4SlowCurrent;
        movePickServosInstant(BoxAutoPanels.allHome());
        s4SlowCurrent = BoxAutoPanels.allHome().s4;
        s4SlowTarget = s4SlowCurrent;
        if (drop1 != null) drop1.setPosition(BoxAutoPanels.D1_CLOSED);
        s5.setPosition(BoxAutoPanels.S5_CLOSED);
        if (pca != null) {
            pcaSetServo(BoxAutoPanels.DROP2_PCA_CHANNEL, BoxAutoPanels.D2_CLOSED);
            pcaSetServo(BoxAutoPanels.DROP3_PCA_CHANNEL, BoxAutoPanels.D3_CLOSED);
        }

        follower.startTeleopDrive();

        // Khởi động trận đấu: Chờ 1 giây an toàn, mở cơ cấu thả s5, tới viewpoint nhìn YOLO
        Command autoRoutine = sequential(
                waitMs(1000), // Chờ 1 giây an toàn sau khi bấm nút
                // ── ĐẦU TRẬN: MỞ S5, CHẠY TỚI ĐIỂM QUAN SÁT, NHẬN DIỆN 4 HỘP ──
                instant(() -> {
                    s5.setPosition(BoxAutoPanels.S5_OPEN);
                }),
                driveAndStabilize5Times(BoxAutoPanels.getTunablePose(8)), // Viewpoint kệ 1
                executeShelfIfScanOk(1, BoxAutoPanels.getTunablePose(1), 1, 2),

                // ── KỆ 2 (P2): Lái tới Viewpoint P13 của kệ 2 ──
                driveAndStabilize5Times(BoxAutoPanels.getTunablePose(13)),
                executeShelfIfScanOk(2, BoxAutoPanels.getTunablePose(2), 3, 4),

                // ── KỆ 3 (P3): Lái tới Viewpoint P14 của kệ 3 và quét ──
                driveAndStabilize5Times(BoxAutoPanels.getTunablePose(14)),
                executeShelfIfScanOk(3, BoxAutoPanels.getTunablePose(3), 5, 6),

                // Trở về vị trí xuất phát P0
                driveAndStabilize5Times(BoxAutoPanels.getTunablePose(0))
        );

        autoRoutine.start();
        while (!isStopRequested() && !autoRoutine.isDone()) {
            follower.update();
            autoRoutine.execute();

            Pose p = follower.getPose();
            telemetry.addData("Pose", String.format(Locale.US, "X=%.1f Y=%.1f H=%.1f°",
                    p.getX(), p.getY(), Math.toDegrees(p.getHeading())));
            telemetry.addData("S4 slow", String.format(Locale.US, "%s target %.3f current %.3f hw %.3f",
                    s4SlowActive ? "ON" : "OFF", s4SlowTarget, s4SlowCurrent, s4.getPosition()));
            addBoxScanTelemetry();
            telemetry.update();
            sleep(10);
        }

        // Khôi phục cơ chế ngắt nguyên bản khi kết thúc
        restoreDisconnectAutoStop();
    }

    // ═══════════════════════════════════════════════════════════
    //  DYNAMIC DS DISCONNECT PREVENTION (REFLECTION PROXY HACK)
    // ═══════════════════════════════════════════════════════════

    private void disableDisconnectAutoStop() {
        try {
            // 1. Lấy internalOpModeServices từ class cha (OpMode)
            java.lang.reflect.Field servicesField = com.qualcomm.robotcore.eventloop.opmode.OpMode.class.getDeclaredField("internalOpModeServices");
            servicesField.setAccessible(true);
            Object opModeManagerImpl = servicesField.get(this);
            if (opModeManagerImpl == null) {
                telemetry.log().add("Không thấy internalOpModeServices");
                return;
            }

            // 2. Lấy eventLoopManager từ OpModeManagerImpl
            java.lang.reflect.Field managerField = opModeManagerImpl.getClass().getDeclaredField("eventLoopManager");
            managerField.setAccessible(true);
            eventLoopManagerInstance = managerField.get(opModeManagerImpl);
            if (eventLoopManagerInstance == null) {
                telemetry.log().add("Không thấy eventLoopManager");
                return;
            }

            // 3. Lấy originalEventLoop từ eventLoopManager
            loopField = eventLoopManagerInstance.getClass().getDeclaredField("eventLoop");
            loopField.setAccessible(true);
            originalEventLoop = loopField.get(eventLoopManagerInstance);
            if (originalEventLoop == null) {
                telemetry.log().add("Không thấy eventLoop trong manager");
                return;
            }

            // 4. Tạo Dynamic Proxy cho EventLoop interface
            Class<?> eventLoopInterface = Class.forName("com.qualcomm.robotcore.eventloop.EventLoop");
            Object proxyEventLoop = java.lang.reflect.Proxy.newProxyInstance(
                    eventLoopInterface.getClassLoader(),
                    new Class[]{eventLoopInterface},
                    new java.lang.reflect.InvocationHandler() {
                        @Override
                        public Object invoke(Object proxy, java.lang.reflect.Method method, Object[] args) throws Throwable {
                            // Nếu getOpModeManager được gọi từ peer disconnect -> Trả về null để chặn dừng OpMode
                            if (method.getName().equals("getOpModeManager")) {
                                for (StackTraceElement element : Thread.currentThread().getStackTrace()) {
                                    if (element.getMethodName().contains("onPeerDisconnected")) {
                                        telemetry.log().add(">>> CHẶN BỊ DỪNG (Ngắt kết nối Driver Station) <<<");
                                        return null;
                                    }
                                }
                            }
                            // Forward lại cho event loop thực tế
                            try {
                                return method.invoke(originalEventLoop, args);
                            } catch (java.lang.reflect.InvocationTargetException ite) {
                                throw ite.getCause();
                            }
                        }
                    }
            );

            // 5. Thay thế EventLoop bằng Proxy
            loopField.set(eventLoopManagerInstance, proxyEventLoop);
            telemetry.log().add("✅ Đã tắt chế độ Tự dừng khi ngắt Driver Station!");
        } catch (Exception e) {
            telemetry.log().add("Không thể tắt tự dừng: " + e.getMessage());
        }
    }

    private void restoreDisconnectAutoStop() {
        try {
            if (eventLoopManagerInstance != null && loopField != null && originalEventLoop != null) {
                loopField.set(eventLoopManagerInstance, originalEventLoop);
                telemetry.log().add("Khôi phục cơ chế ngắt Driver Station mặc định.");
            }
        } catch (Exception ignored) {}
    }

    // ═══════════════════════════════════════════════════════════
    //  TURN CYCLE LOGIC (Turn 1 & Turn 2)
    // ═══════════════════════════════════════════════════════════

    private Command executeShelfIfScanOk(int shelfNum, Pose shelfPose, int topTurnNum, int bottomTurnNum) {
        return sequential(
                scanInitial4BoxesCommand(),
                conditional(
                        () -> !skipCurrentShelf,
                        sequential(
                                executeTurnCycle(topTurnNum, shelfPose, 2),
                                executeTurnCycle(bottomTurnNum, shelfPose, 1)
                        ),
                        instant(() -> telemetry.log().add("Skip shelf " + shelfNum + " after 3 invalid scans"))
                )
        );
    }

    private Command executeTurnCycle(int turnNum, Pose shelfPose, int targetRow) {
        return sequential(
                // 1. Lái tới Kệ (Shelf)
                driveAndStabilize5Times(shelfPose),

                // 2. Gắp LẦN LƯỢT CẢ 2 HỘP CÙNG 1 LẦN TỚI KỆ (S3 gắp Hộp 1, S5 gắp Hộp 2)
                pickDualBoxesSequence(targetRow),

                // 3. Di chuyển thả Hộp 1 tại Nhà máy tương ứng (xả ngàm S3)
                deliverPostBoxCommand(1, turnNum),

                // 4. Di chuyển thả Hộp 2 tại Nhà máy tương ứng (xả ngàm S5)
                deliverPostBoxCommand(2, turnNum)
        );
    }


    // ── Nomalized Pick Sequence: Hold in front, don't drop inside belly ──
    private Command pickDualBoxesSequence(int row) {
        return sequential(
                // BƯỚC 1: Quay ngàm S4 ra vị trí phía trước (S4_GRAB)
                movePickServosSeq(BoxAutoPanels.moveS4Grab()),
                waitMs(180),

                // BƯỚC 2: Hạ S1 xuống cao độ Pallet/Kệ TRƯỚC (S2 VẪN THU GỌN Ở HOME để không va đâm vách)
                movePickServosSeq(BoxAutoPanels.setElevatorLevel(row)),
                waitMs(200),

                // BƯỚC 3: S2 VƯƠN TAY RA TIẾP CẬN HỘP (S3/S5 VẪN MỞ HOÀN TOÀN TRƯỚC KHI KẸP)
                movePickServosSeq(BoxAutoPanels.extendS2(row)),
                waitMs(250),

                // BƯỚC 4: ĐÓNG KẸP CẢ 2 NGÀM S3 (trái) VÀ S5 (phải) ĐỂ GIỮ CHẮC 2 HỘP
                movePickServosSeq(BoxAutoPanels.clampBothS3S5(row)),
                waitMs(200),

                // BƯỚC 5: S2 THU TAY VỀ HOME HOÀN TOÀN (Kéo 2 hộp ra khỏi vách kệ, S1 VẪN GIỮ NGUYÊN CAO ĐỘ KỆ)
                movePickServosSeq(BoxAutoPanels.retractS2(row)),
                waitMs(250),

                // BƯỚC 6: SAU KHI S2 ĐÃ RÚT VỀ HOME MỚI NÂNG S1 LÊN CAO NHẤT (S1_HIGHEST)
                movePickServosSeq(BoxAutoPanels.s1Highest()),
                waitMs(250)
        );
    }

    // Quy ước ID Nhà Máy Thả Hộp:
    // 1: foxconn           -> P4_DROP1 (Pose 4) / DODGE1 (Pose 9)
    // 2: amkor             -> P5_DROP2 (Pose 5) / DODGE2 (Pose 10)
    // 3: hana_micron_vina  -> P6_DROP3 (Pose 6) / DODGE3 (Pose 11)
    // 4: samsung           -> P7_DROP4 (Pose 7) / DODGE4 (Pose 12)
    private int getFactoryIdFromName(String name) {
        if (name == null) return 1;
        String n = name.trim().toLowerCase();

        int blueId = 1;
        if (n.contains("foxconn")) blueId = 1;               // P4_DROP1
        else if (n.contains("amkor")) blueId = 2;            // P5_DROP2
        else if (n.contains("hana")) blueId = 3;             // P6_DROP3 (hana_micron_vina)
        else if (n.contains("samsung")) blueId = 4;          // P7_DROP4

        // Nếu là Red Alliance, thứ tự nhà máy bị ngược lại hoàn toàn so với Blue
        if (isRed) {
            switch (blueId) {
                case 1: return 4; // Foxconn -> P7
                case 2: return 3; // Amkor -> P6
                case 3: return 2; // Hana -> P5
                case 4: return 1; // Samsung -> P4
                default: return 4;
            }
        }
        return blueId;
    }

    private Pose getFactoryDropPose(int boxType) {
        switch (boxType) {
            case 1: return BoxAutoPanels.getTunablePose(4); // P4_DROP1 (foxconn)
            case 2: return BoxAutoPanels.getTunablePose(5); // P5_DROP2 (amkor)
            case 3: return BoxAutoPanels.getTunablePose(6); // P6_DROP3 (hana_micron_vina)
            case 4: return BoxAutoPanels.getTunablePose(7); // P7_DROP4 (samsung)
            default: return BoxAutoPanels.getTunablePose(4);
        }
    }

    private Pose getFactoryDodgePose(int boxType) {
        switch (boxType) {
            case 1: return BoxAutoPanels.getTunablePose(9);  // DODGE1
            case 2: return BoxAutoPanels.getTunablePose(10); // DODGE2
            case 3: return BoxAutoPanels.getTunablePose(11); // DODGE3
            case 4: return BoxAutoPanels.getTunablePose(12); // DODGE4
            default: return BoxAutoPanels.getTunablePose(9);
        }
    }

    // ── Quét YOLO 4 hộp từ đầu trận ──
    private Command scanInitial4BoxesCommand() {
        return CustomCommand.create()
                .setStart(() -> {
                    // Khi camera scan, nâng Servo 1 lên cao nhất để không che camera/va hộp.
                    BoxAutoPanels.PickServoSet highSet = BoxAutoPanels.s1Highest();
                    s1.setPosition(highSet.s1);
                    scanAttempt = 1;
                    skipCurrentShelf = false;
                    startScanAttempt();
                })
                .setExecute(() -> {
                    long elapsed = System.currentTimeMillis() - stabilizeStartTime;
                    telemetry.addData("Scan", "YOLO attempt %d/3... %d ms", scanAttempt, elapsed);
                    telemetry.addData("Scan Valid", piVision.hasAll4ShelfLabels() ? "ĐỦ 4 HỘP" : "CHƯA ĐỦ 4 HỘP");
                    telemetry.addData("R1C1 (Tầng 2 Trái)", piVision.r1c1);
                    telemetry.addData("R1C2 (Tầng 2 Phải)", piVision.r1c2);
                    telemetry.addData("R2C1 (Tầng 1 Trái)", piVision.r2c1);
                    telemetry.addData("R2C2 (Tầng 1 Phải)", piVision.r2c2);
                    addBoxScanTelemetry();

                    boolean attemptFinishedInvalid = piVision.isScanDone() && !piVision.hasAll4ShelfLabels();
                    boolean attemptTimedOut = elapsed > PI_SCAN_TIMEOUT_MS;
                    if ((attemptFinishedInvalid || attemptTimedOut) && scanAttempt < 3) {
                        telemetry.log().add("YOLO scan retry " + scanAttempt + "/3 failed");
                        scanAttempt++;
                        startScanAttempt();
                    }
                })
                .setDone(() -> piVision.hasAll4ShelfLabels()
                        || ((piVision.isScanDone() || System.currentTimeMillis() - stabilizeStartTime > PI_SCAN_TIMEOUT_MS)
                        && scanAttempt >= 3))
                .setEnd(condition -> {
                    if (!piVision.hasAll4ShelfLabels()) {
                        skipCurrentShelf = true;
                        lastScanSummary = "SKIP KỆ: scan 3 lần không đủ 4 hộp";
                        telemetry.log().add(lastScanSummary);
                        return;
                    }

                    // R1C1 (Tầng 2 Trái -> S3): Hộp 1 Lượt 1
                    // R1C2 (Tầng 2 Phải -> S5): Hộp 2 Lượt 1
                    post1BoxType = getFactoryIdFromName(piVision.r1c1);
                    post2BoxType = getFactoryIdFromName(piVision.r1c2);

                    // R2C1 (Tầng 1 Trái -> S3): Hộp 1 Lượt 2
                    // R2C2 (Tầng 1 Phải -> S5): Hộp 2 Lượt 2
                    turn2Post1BoxType = getFactoryIdFromName(piVision.r2c1);
                    turn2Post2BoxType = getFactoryIdFromName(piVision.r2c2);

                    lastScanSummary = String.format(Locale.US,
                            "T1 S3=%s->F%d, S5=%s->F%d | T2 S3=%s->F%d, S5=%s->F%d",
                            piVision.r1c1, post1BoxType,
                            piVision.r1c2, post2BoxType,
                            piVision.r2c1, turn2Post1BoxType,
                            piVision.r2c2, turn2Post2BoxType);
                    telemetry.log().add("YOLO Scan Complete. " + lastScanSummary);
                });
    }

    private void startScanAttempt() {
        piVision.clearResult();
        piVision.sendCommand("SCAN");
        stabilizeStartTime = System.currentTimeMillis();
    }

    // ── Nomalized Drop Sequence: Thả bằng ngàm kẹp tại nhà máy ──
    private Command deliverPostBoxCommand(int postIndex, int turnNum) {
        return lazy(() -> {
            int boxType;
            if (turnNum == 1) {
                boxType = (postIndex == 1) ? post1BoxType : post2BoxType;
            } else {
                boxType = (postIndex == 1) ? turn2Post1BoxType : turn2Post2BoxType;
            }

            Pose factoryDropPose = getFactoryDropPose(boxType);
            Pose factoryDodgePose = getFactoryDodgePose(boxType);
            telemetry.log().add(String.format(Locale.US,
                    "Drop decision: turn=%d post=%d boxType=%d drop=(%.1f, %.1f, %.1f°) dodge=(%.1f, %.1f, %.1f°)",
                    turnNum, postIndex, boxType,
                    factoryDropPose.getX(), factoryDropPose.getY(), Math.toDegrees(factoryDropPose.getHeading()),
                    factoryDodgePose.getX(), factoryDodgePose.getY(), Math.toDegrees(factoryDodgePose.getHeading())));

            // Xác định xem hộp này (postIndex) đang được ngậm ở ngàm S3 (hộp đầu) hay S5 (hộp thứ hai)
            // Theo thuật toán lấy, turnNum=1 & postIndex=1 tương ứng phase 0 -> S3
            // postIndex=2 tương ứng phase 1 -> S5
            BoxAutoPanels.PickServoSet dropMacro = (postIndex == 1) ? BoxAutoPanels.openS3Drop() : BoxAutoPanels.openS5Drop();
            BoxAutoPanels.PickServoSet liftAfterDrop = (postIndex == 1) ? BoxAutoPanels.s1LiftUpAfterDrop1() : BoxAutoPanels.s1LiftUpAfterDrop();
            BoxAutoPanels.PickServoSet retractHomeAfterDrop = (postIndex == 1) ? BoxAutoPanels.s2RetractHomeAfterDrop1() : BoxAutoPanels.s2RetractHomeAfterDrop();

            return sequential(
                    // Lái tới đúng nhà máy của loại hộp - Định vị 5 lần
                    driveAndStabilize5Times(factoryDropPose),

                    // Bước 1: Lật S4 ra sau trước khi S2 còn đang thu gọn
                    movePickServosSeq(BoxAutoPanels.s4FlipBackOnlyHigh()),

                    // Bước 2: Sau khi S4 lật xong mới vươn S2 ra sau
                    movePickServosSeq(BoxAutoPanels.s2ExtendHighForDrop()),

                    // Bước 3: Hạ S1 xuống vị trí thả trực tiếp bằng ngàm S3/S5
                    movePickServosSeq((postIndex == 1) ? BoxAutoPanels.prepareS3LowDrop() : BoxAutoPanels.prepareS5LowDrop()),

                    // Bước 4: MỞ NGÀM S3/S5 tương ứng để thả hộp
                    movePickServosSeq(dropMacro),

                    // Lách sang điểm Dodge Point lùi lại để tránh kẹt - Định vị 5 lần
                    driveAndStabilize5Times(factoryDodgePose),

                    // Bước 5: NÂNG S1 LÊN CAO NHẤT (S1_HIGHEST) để thoát cọc thả an toàn
                    movePickServosSeq(liftAfterDrop),

                    // Bước 6: Sau đó mới rút S2 về HOME để không bị kẹt
                    movePickServosSeq(retractHomeAfterDrop),

                    // Bước 7: Nếu đã thả xong cả 2 hộp (postIndex == 2), xoay S4 về lại phía trước chuẩn bị cho Turn kế tiếp
                    (postIndex == 2) ? movePickServosSeq(BoxAutoPanels.moveS4Grab()) : instant(() -> {})
            );
        });
    }


    private void addBoxScanTelemetry() {
        telemetry.addLine("─── YOLO BOX POSITIONS ───");
        telemetry.addData("Last Scan", lastScanSummary);
        telemetry.addData("Scan Attempt", "%d/3", scanAttempt);
        telemetry.addData("Scan Valid", piVision.hasAll4ShelfLabels() ? "ĐỦ 4 HỘP" : "CHƯA ĐỦ 4 HỘP");
        telemetry.addData("R1C1 Tầng 2 Trái", "%s -> Factory %d", piVision.r1c1, post1BoxType);
        telemetry.addData("R1C2 Tầng 2 Phải", "%s -> Factory %d", piVision.r1c2, post2BoxType);
        telemetry.addData("R2C1 Tầng 1 Trái", "%s -> Factory %d", piVision.r2c1, turn2Post1BoxType);
        telemetry.addData("R2C2 Tầng 1 Phải", "%s -> Factory %d", piVision.r2c2, turn2Post2BoxType);
    }

    private void triggerDropDoor(int postIndex, boolean open) {
        if (!BoxAutoPanels.ENABLE_DROP_MECHANISM) return;
        // Trụ 1 tương ứng Drop 1
        if (postIndex == 1) {
            if (drop1 != null) {
                drop1.setPosition(open ? BoxAutoPanels.D1_OPEN1 : BoxAutoPanels.D1_CLOSED);
            }
        } 
        // Trụ 2 tương ứng Drop 2 trên kênh PCA 0
        else if (postIndex == 2) {
            if (pca != null) {
                pcaSetServo(BoxAutoPanels.DROP2_PCA_CHANNEL, open ? BoxAutoPanels.D2_OPEN1 : BoxAutoPanels.D2_CLOSED);
            }
        }
    }

    // ═══════════════════════════════════════════════════════════
    //  HELPERS
    // ═══════════════════════════════════════════════════════════

    private Command movePickServosSeq(BoxAutoPanels.PickServoSet set) {
        return sequential(
                instant(() -> {
                    s1.setPosition(set.s1);
                    s2.setPosition(set.s2);
                    s3.setPosition(set.s3);
                    // S5 set trực tiếp, dứt khoát, không chờ S4
                    s5.setPosition(set.s5);
                    // S4 cũng set trực tiếp luôn, không quay chậm từng bước nữa
                    s4.setPosition(set.s4);
                    s4SlowCurrent = set.s4;
                    s4SlowTarget = set.s4;
                }),
                waitMs(BoxAutoPanels.PICK_SERVO_STEP_DELAY)
        );
    }

    private Command moveS4SlowSeq(double targetS4) {
        final double clampedTarget = BoxAutoPanels.clamp(targetS4);
        return new Command() {
            private boolean started = false;
            private boolean wroteFinal = false;

            @Override public void start() {
                s4SlowTarget = clampedTarget;
                s4SlowCurrent = s4.getPosition();
                s4SlowActive = true;
                telemetry.log().add(String.format(Locale.US,
                        "S4 slow start: current=%.3f target=%.3f step=%.4f name=%s",
                        s4SlowCurrent, s4SlowTarget, BoxAutoPanels.S4_SPEED_STEP, BoxAutoPanels.NAME_S4));
            }

            @Override public void execute() {
                started = true;
                double delta = s4SlowTarget - s4SlowCurrent;
                if (Math.abs(delta) <= BoxAutoPanels.S4_SPEED_STEP) {
                    s4SlowCurrent = s4SlowTarget;
                    wroteFinal = true;
                } else {
                    s4SlowCurrent += Math.signum(delta) * BoxAutoPanels.S4_SPEED_STEP;
                }
                s4.setPosition(s4SlowCurrent);
            }

            @Override public boolean isDone() {
                return started && wroteFinal;
            }

            @Override public void end() {
                s4.setPosition(s4SlowTarget);
                s4SlowCurrent = s4SlowTarget;
                s4SlowActive = false;
                telemetry.log().add(String.format(Locale.US,
                        "S4 slow done: target=%.3f hw=%.3f", s4SlowTarget, s4.getPosition()));
            }
        };
    }

    private void movePickServosInstant(BoxAutoPanels.PickServoSet set) {
        s1.setPosition(set.s1);
        s2.setPosition(set.s2);
        s3.setPosition(set.s3);
        s4.setPosition(set.s4);
        s4SlowCurrent = set.s4;
        s4SlowTarget = set.s4;
    }

    private void pcaSetServo(int channel, double position) {
        if (pca == null) return;
        double pulseUs = 500 + Math.max(0.0, Math.min(1.0, position)) * 2000;
        pca.setServoPulseUs(channel, pulseUs);
    }

    // ═══════════════════════════════════════════════════════════
    //  DRIVE AND STABILIZE 5 TIMES TO ELIMINATE DRIFT
    // ═══════════════════════════════════════════════════════════

    private Command driveAndStabilize5Times(Pose target) {
        return sequential(
                alignStepCommand(target, 1),
                alignStepCommand(target, 2),
                alignStepCommand(target, 3),
                alignStepCommand(target, 4),
                alignStepCommand(target, 5)
        );
    }

    private Command alignStepCommand(Pose target, int step) {
        return sequential(
                instant(() -> telemetry.addData("Align", "Step %d/5", step)),
                driveToPose(target),
                waitUntilStable(target)
        );
    }

    private Command driveToPose(Pose target) {
        return lazy(() -> {
            Pose current = follower.getPose();
            Path linePath = new Path(new BezierLine(current, target));
            linePath.setLinearHeadingInterpolation(current.getHeading(), target.getHeading());
            return follow(follower, linePath, true);
        });
    }

    private Command waitUntilStable(Pose targetPose) {
        return CustomCommand.create()
                .setStart(() -> {
                    stabilizeStartTime = System.currentTimeMillis();
                    firstStableTime = 0;
                })
                .setExecute(() -> {
                    BoxAutoPanels.refresh();
                    Pose cur = follower.getPose();
                    double dx = cur.getX() - targetPose.getX();
                    double dy = cur.getY() - targetPose.getY();
                    double xyError = Math.sqrt(dx * dx + dy * dy);

                    double hError = cur.getHeading() - targetPose.getHeading();
                    while (hError > Math.PI) hError -= 2 * Math.PI;
                    while (hError < -Math.PI) hError += 2 * Math.PI;
                    double hErrorDeg = Math.abs(Math.toDegrees(hError));

                    boolean stable = xyError <= BoxAutoPanels.POSE_XY_TOLERANCE_IN
                            && hErrorDeg <= BoxAutoPanels.POSE_HEADING_TOLERANCE_DEG;

                    if (stable) {
                        if (firstStableTime == 0) firstStableTime = System.currentTimeMillis();
                    } else {
                        firstStableTime = 0;
                    }
                })
                .setDone(() -> {
                    long now = System.currentTimeMillis();
                    boolean hasSettled = (firstStableTime != 0 && now - firstStableTime >= BoxAutoPanels.POSE_STABLE_TIME_MS);
                    boolean isTimeout = (now - stabilizeStartTime >= BoxAutoPanels.POSE_TIMEOUT_MS);
                    return hasSettled || isTimeout;
                });
    }

    // ═══════════════════════════════════════════════════════════
    //  SIMPLE COMMAND FRAMEWORK
    // ═══════════════════════════════════════════════════════════

    private abstract static class Command {
        public void start() {}
        public void execute() {}
        public boolean isDone() { return true; }
        public void end() {}
    }

    private static class CustomCommand extends Command {
        private Runnable startAction, executeAction;
        private java.util.function.BooleanSupplier doneCondition;
        private java.util.function.Consumer<Boolean> endAction;

        public static CustomCommand create() { return new CustomCommand(); }

        public CustomCommand setStart(Runnable r) { startAction = r; return this; }
        public CustomCommand setExecute(Runnable r) { executeAction = r; return this; }
        public CustomCommand setDone(java.util.function.BooleanSupplier b) { doneCondition = b; return this; }
        public CustomCommand setEnd(java.util.function.Consumer<Boolean> c) { endAction = c; return this; }

        @Override public void start() { if (startAction != null) startAction.run(); }
        @Override public void execute() { if (executeAction != null) executeAction.run(); }
        @Override public boolean isDone() { return doneCondition != null ? doneCondition.getAsBoolean() : true; }
        @Override public void end() { if (endAction != null) endAction.accept(true); }
    }

    private Command instant(Runnable action) {
        return new Command() {
            @Override public void start() { action.run(); }
        };
    }

    private Command lazy(java.util.function.Supplier<Command> supplier) {
        return new Command() {
            private Command inner;
            @Override public void start() { inner = supplier.get(); if (inner != null) inner.start(); }
            @Override public void execute() { if (inner != null) inner.execute(); }
            @Override public boolean isDone() { return inner == null || inner.isDone(); }
            @Override public void end() { if (inner != null) inner.end(); }
        };
    }

    private Command conditional(java.util.function.BooleanSupplier condition, Command ifTrue, Command ifFalse) {
        return lazy(() -> condition.getAsBoolean() ? ifTrue : ifFalse);
    }

    private Command waitMs(long ms) {
        return new Command() {
            private long startTime;
            @Override public void start() { startTime = System.currentTimeMillis(); }
            @Override public boolean isDone() { return System.currentTimeMillis() - startTime >= ms; }
        };
    }

    private Command sequential(Command... commands) {
        return new Command() {
            private int index = 0;
            private boolean currentStarted = false;

            @Override public void start() { index = 0; currentStarted = false; }
            @Override public void execute() {
                if (index >= commands.length) return;
                Command curr = commands[index];
                if (!currentStarted) { curr.start(); currentStarted = true; }
                curr.execute();
                if (curr.isDone()) {
                    curr.end();
                    index++;
                    currentStarted = false;
                }
            }
            @Override public boolean isDone() { return index >= commands.length; }
        };
    }

    private Command follow(Follower f, com.pedropathing.paths.PathChain path, boolean holdEnd) {
        return new Command() {
            @Override public void start() { f.followPath(path, holdEnd); }
            @Override public boolean isDone() { return !f.isBusy(); }
        };
    }

    private Command follow(Follower f, com.pedropathing.paths.Path path, boolean holdEnd) {
        return new Command() {
            @Override public void start() { f.followPath(path, holdEnd); }
            @Override public boolean isDone() { return !f.isBusy(); }
        };
    }
}
