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
 * MANUAL TUNER: Lái tay bằng Gamepad + tune giá trị bằng FTControl Panels.
 *
 * GAMEPAD 1 — DI CHUYỂN:
 *   Left stick  = Strafe (X) + Forward (Y)
 *   Right stick = Turn
 *   A           = Auto-drive tới vị trí Panels (TUNER_TARGET_POSITION_INDEX)
 *   B           = Hủy auto-drive, quay về lái tay
 *   Y           = Lưu vị trí hiện tại → telemetry
 *   Dpad U/D    = Chọn vị trí target (tăng/giảm index)
 *
 * GAMEPAD 2 — CƠ CẤU GẮP:
 *   Right trigger = S1 nâng lên (incremental)
 *   Left trigger  = S1 hạ xuống (incremental)
 *   Right bumper  = S5 swing phải (incremental)
 *   Left bumper   = S5 swing trái (incremental)
 *   A             = S3 grip (đóng)
 *   B             = S3 release (mở)
 *   X             = HOME tất cả servo
 *   Y             = Áp dụng Panels servo (TUNER_APPLY)
 *   Dpad Up       = Preset: pickReady
 *   Dpad Down     = Preset: pickDown
 *   Dpad Right    = Preset: pickGrab → pickRetract
 *   Dpad Left     = Preset: storeCompartment
 *
 * PANELS (FTControl):
 *   - TUNER_TARGET_POSITION_INDEX: chọn P0-P9
 *   - TUNER_DRIVE_TO_SELECTED: bật/tắt auto-drive
 *   - TUNER_SELECTED_COLUMN / TUNER_SELECTED_ROW / TUNER_SELECTED_COMPARTMENT: cho presets
 *   - TUNER_APPLY + TUNER_MODE + TUNER_SERVO_POS: tune servo trực tiếp
 */
@TeleOp(name = "Manual Tuner", group = "Tuner")
public class ManualTuner extends LinearOpMode {

    // === Hardware ===
    private Follower follower;
    private Servo s1, s2, s3, s4, s5;
    private Servo drop1;
    private PCA9685 pca;

    // === Drive State ===
    private boolean autoDriving = false;
    private int lastTargetIndex = -1;
    private Pose lastTargetPose = new Pose(0, 0, 0);

    // === Servo Tracking ===
    private double posS1 = 0.5;   // S1 lift position (0-1)
    private double posS5 = 0.5;   // S5 swing position (0-1)

    // === Increment Settings ===
    private static final double S1_STEP = 0.01;
    private static final double S5_STEP = 0.01;
    private static final double DEADZONE = 0.05;

    // === Debounce ===
    private boolean prevGp1A = false, prevGp1B = false, prevGp1Y = false;
    private boolean prevGp1DpadUp = false, prevGp1DpadDown = false;
    private boolean prevGp2A = false, prevGp2B = false, prevGp2X = false, prevGp2Y = false;
    private boolean prevGp2DpadUp = false, prevGp2DpadDown = false;
    private boolean prevGp2DpadLeft = false, prevGp2DpadRight = false;

    // === Saved Pose ===
    private String savedPoseStr = "(chưa lưu)";

    @Override
    public void runOpMode() throws InterruptedException {

        // ── Init Hardware ──────────────────────────────────
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(BoxAutoPanels.poseStart());

        s1 = hardwareMap.get(Servo.class, BoxAutoPanels.NAME_S1);
        s2 = hardwareMap.get(Servo.class, BoxAutoPanels.NAME_S2);
        s3 = hardwareMap.get(Servo.class, BoxAutoPanels.NAME_S3);
        s4 = hardwareMap.get(Servo.class, BoxAutoPanels.NAME_S4);
        s5 = hardwareMap.get(Servo.class, BoxAutoPanels.NAME_S5);

        drop1 = hardwareMap.get(Servo.class, BoxAutoPanels.NAME_DROP1);
        pca   = hardwareMap.get(PCA9685.class, BoxAutoPanels.NAME_PCA9685);

        // ── Home tất cả servo ──────────────────────────────
        applyPickSet(BoxAutoPanels.pickHome());
        applyDropSet(BoxAutoPanels.dropClosed());

        // Đọc vị trí home thực tế
        posS1 = s1.getPosition();
        posS5 = s5.getPosition();

        // ── Ready ──────────────────────────────────────────
        telemetry.addLine("═══ MANUAL TUNER READY ═══");
        telemetry.addLine("GP1: Lái tay | GP2: Cơ cấu gắp");
        telemetry.update();

        follower.startTeleopDrive();
        waitForStart();

        // ═══════════════════════════════════════════════════
        //                   MAIN LOOP
        // ═══════════════════════════════════════════════════
        while (opModeIsActive()) {

            // ── 1. Gamepad 1: Di chuyển ────────────────────
            handleDriving();

            // ── 2. Gamepad 2: Cơ cấu gắp ──────────────────
            handleMechanism();

            // ── 3. Panels: Servo tuner trực tiếp ───────────
            handlePanelsTuner();

            // ── 4. Update & Telemetry ──────────────────────
            follower.update();
            showTelemetry();

            sleep(20);
        }
    }

    // ═══════════════════════════════════════════════════════════
    //  GAMEPAD 1: DI CHUYỂN
    // ═══════════════════════════════════════════════════════════

    private void handleDriving() {

        // --- Nút A: Bật auto-drive tới Panels target ---
        boolean gp1A = gamepad1.a;
        if (gp1A && !prevGp1A) {
            autoDriving = true;
            lastTargetIndex = -1; // force re-path
        }
        prevGp1A = gp1A;

        // --- Nút B: Hủy auto-drive ---
        boolean gp1B = gamepad1.b;
        if (gp1B && !prevGp1B) {
            autoDriving = false;
            follower.breakFollowing();
            lastTargetIndex = -1;
        }
        prevGp1B = gp1B;

        // --- Nút Y: Lưu pose hiện tại ---
        boolean gp1Y = gamepad1.y;
        if (gp1Y && !prevGp1Y) {
            Pose p = follower.getPose();
            savedPoseStr = String.format(Locale.US,
                    "X=%.2f  Y=%.2f  H=%.1f°",
                    p.getX(), p.getY(), Math.toDegrees(p.getHeading()));
        }
        prevGp1Y = gp1Y;

        // --- Dpad UP/DOWN: Chọn target index ---
        boolean gp1DU = gamepad1.dpad_up;
        boolean gp1DD = gamepad1.dpad_down;
        if (gp1DU && !prevGp1DpadUp) {
            BoxAutoPanels.TUNER_TARGET_POSITION_INDEX =
                    Math.min(BoxAutoPanels.TUNER_TARGET_POSITION_INDEX + 1,
                            BoxAutoPanels.POSITION_COUNT - 1);
        }
        if (gp1DD && !prevGp1DpadDown) {
            BoxAutoPanels.TUNER_TARGET_POSITION_INDEX =
                    Math.max(BoxAutoPanels.TUNER_TARGET_POSITION_INDEX - 1, 0);
        }
        prevGp1DpadUp = gp1DU;
        prevGp1DpadDown = gp1DD;

        // --- Auto-drive hoặc lái tay ---
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

        // Nếu đến nơi → tự tắt auto-drive
        if (!follower.isBusy()) {
            autoDriving = false;
        }
    }

    private void handleManualDrive() {
        double forward = -gamepad1.left_stick_y;
        double strafe  =  gamepad1.left_stick_x;
        double turn    =  gamepad1.right_stick_x;

        // Áp deadzone
        if (Math.abs(forward) < DEADZONE) forward = 0;
        if (Math.abs(strafe)  < DEADZONE) strafe  = 0;
        if (Math.abs(turn)    < DEADZONE) turn    = 0;

        follower.setTeleOpDrive(forward, strafe, turn, true);
    }

    // ═══════════════════════════════════════════════════════════
    //  GAMEPAD 2: CƠ CẤU GẮP
    // ═══════════════════════════════════════════════════════════

    private void handleMechanism() {

        // --- Right/Left Trigger: S1 nâng/hạ ---
        if (gamepad2.right_trigger > 0.1) {
            posS1 = Math.min(1.0, posS1 + S1_STEP * gamepad2.right_trigger);
            s1.setPosition(posS1);
        }
        if (gamepad2.left_trigger > 0.1) {
            posS1 = Math.max(0.0, posS1 - S1_STEP * gamepad2.left_trigger);
            s1.setPosition(posS1);
        }

        // --- Right/Left Bumper: S5 swing ---
        if (gamepad2.right_bumper) {
            posS5 = Math.min(1.0, posS5 + S5_STEP);
            s5.setPosition(posS5);
        }
        if (gamepad2.left_bumper) {
            posS5 = Math.max(0.0, posS5 - S5_STEP);
            s5.setPosition(posS5);
        }

        // --- A: S3 grip (đóng) ---
        boolean gp2A = gamepad2.a;
        if (gp2A && !prevGp2A) {
            s3.setPosition(BoxAutoPanels.S3_CLOSED);
        }
        prevGp2A = gp2A;

        // --- B: S3 release (mở) ---
        boolean gp2B = gamepad2.b;
        if (gp2B && !prevGp2B) {
            s3.setPosition(BoxAutoPanels.S3_OPEN);
        }
        prevGp2B = gp2B;

        // --- X: HOME tất cả ---
        boolean gp2X = gamepad2.x;
        if (gp2X && !prevGp2X) {
            applyPickSet(BoxAutoPanels.pickHome());
            applyDropSet(BoxAutoPanels.dropClosed());
            posS1 = s1.getPosition();
            posS5 = s5.getPosition();
        }
        prevGp2X = gp2X;

        // --- Y: Áp dụng Panels tuner (giống ServoPanelsTuner) ---
        boolean gp2Y = gamepad2.y;
        if (gp2Y && !prevGp2Y) {
            BoxAutoPanels.TUNER_APPLY = true;
        }
        prevGp2Y = gp2Y;

        // --- Dpad presets ---
        int col = BoxAutoPanels.TUNER_SELECTED_COLUMN;
        int compartment = BoxAutoPanels.TUNER_SELECTED_COMPARTMENT;

        boolean gp2DU = gamepad2.dpad_up;
        if (gp2DU && !prevGp2DpadUp) {
            applyPickSet(BoxAutoPanels.pickReady(col));
        }
        prevGp2DpadUp = gp2DU;

        boolean gp2DD = gamepad2.dpad_down;
        if (gp2DD && !prevGp2DpadDown) {
            applyPickSet(BoxAutoPanels.pickDown(col));
        }
        prevGp2DpadDown = gp2DD;

        boolean gp2DR = gamepad2.dpad_right;
        if (gp2DR && !prevGp2DpadRight) {
            applyPickSet(BoxAutoPanels.pickGrab(col));
        }
        prevGp2DpadRight = gp2DR;

        boolean gp2DL = gamepad2.dpad_left;
        if (gp2DL && !prevGp2DpadLeft) {
            applyPickSet(BoxAutoPanels.storeCompartment(compartment));
        }
        prevGp2DpadLeft = gp2DL;
    }

    // ═══════════════════════════════════════════════════════════
    //  PANELS: SERVO TUNER TRỰC TIẾP
    // ═══════════════════════════════════════════════════════════

    private void handlePanelsTuner() {
        if (!BoxAutoPanels.TUNER_APPLY) return;
        BoxAutoPanels.TUNER_APPLY = false;

        int mode = BoxAutoPanels.TUNER_MODE;
        double pos = BoxAutoPanels.TUNER_SERVO_POS;
        int col = BoxAutoPanels.TUNER_SELECTED_COLUMN;
        int compartment = BoxAutoPanels.TUNER_SELECTED_COMPARTMENT;

        switch (mode) {
            case 0: // Individual pick servo
                applyIndividualPickServo(pos);
                break;
            case 1: // Individual drop servo
                applyIndividualDropServo(pos);
                break;
            case 2: // pickHome
                applyPickSet(BoxAutoPanels.pickHome());
                break;
            case 3: // pickReady
                applyPickSet(BoxAutoPanels.pickReady(col));
                break;
            case 4: // pickDown
                applyPickSet(BoxAutoPanels.pickDown(col));
                break;
            case 5: // pickGrab
                applyPickSet(BoxAutoPanels.pickGrab(col));
                break;
            case 6: // pickRetract
                applyPickSet(BoxAutoPanels.pickRetract(col));
                break;
            case 7: // storeCompartment
                applyPickSet(BoxAutoPanels.storeCompartment(compartment));
                break;
            case 8: // dropOpen
                applyDropSet(BoxAutoPanels.dropOpen(compartment));
                break;
            case 9: // dropClosed
                applyDropSet(BoxAutoPanels.dropClosed());
                break;
        }

        // Cập nhật tracking
        posS1 = s1.getPosition();
        posS5 = s5.getPosition();
    }

    private void applyIndividualPickServo(double pos) {
        int sel = BoxAutoPanels.TUNER_SELECTED_SERVO;
        switch (sel) {
            case 1: s1.setPosition(pos); posS1 = pos; break;
            case 2: s2.setPosition(pos); break;
            case 3: s3.setPosition(BoxAutoPanels.clampS3(pos)); break;
            case 4: s4.setPosition(pos); break;
            case 5: s5.setPosition(pos); posS5 = pos; break;
        }
    }

    private void applyIndividualDropServo(double pos) {
        int sel = BoxAutoPanels.TUNER_SELECTED_DROP_SERVO;
        switch (sel) {
            case 1: drop1.setPosition(pos); break;
            case 2: pcaSetServo(BoxAutoPanels.DROP2_PCA_CHANNEL, pos); break;
            case 3: pcaSetServo(BoxAutoPanels.DROP3_PCA_CHANNEL, pos); break;
        }
    }

    // ═══════════════════════════════════════════════════════════
    //  HELPERS
    // ═══════════════════════════════════════════════════════════

    private void applyPickSet(BoxAutoPanels.PickServoSet set) {
        s1.setPosition(set.s1);
        s2.setPosition(set.s2);
        s3.setPosition(BoxAutoPanels.clampS3(set.s3));
        s4.setPosition(set.s4);
        s5.setPosition(set.s5);
    }

    private void applyDropSet(BoxAutoPanels.DropServoSet set) {
        drop1.setPosition(set.d1);
        pcaSetServo(BoxAutoPanels.DROP2_PCA_CHANNEL, set.d2);
        pcaSetServo(BoxAutoPanels.DROP3_PCA_CHANNEL, set.d3);
    }

    private void pcaSetServo(int channel, double position) {
        double pulseUs = 500 + clamp(position) * 2000;
        pca.setServoPulseUs(channel, pulseUs);
    }

    private double clamp(double v) {
        return Math.max(0.0, Math.min(1.0, v));
    }

    // ═══════════════════════════════════════════════════════════
    //  TELEMETRY
    // ═══════════════════════════════════════════════════════════

    private void showTelemetry() {
        Pose p = follower.getPose();
        int idx = BoxAutoPanels.TUNER_TARGET_POSITION_INDEX;
        String targetName = (idx >= 0 && idx < BoxAutoPanels.POSITION_COUNT)
                ? BoxAutoPanels.poseName(idx)
                : "N/A";

        telemetry.addLine("═══ MANUAL TUNER ═══");

        // Drive
        telemetry.addLine(String.format(Locale.US,
                "Pose: X=%.2f  Y=%.2f  H=%.1f°",
                p.getX(), p.getY(), Math.toDegrees(p.getHeading())));

        telemetry.addData("Mode",
                autoDriving ? "🚗 AUTO → [" + idx + "] " + targetName
                            : "🎮 LÁI TAY");

        if (autoDriving) {
            telemetry.addData("Busy", follower.isBusy());
        }

        telemetry.addData("Saved Pose", savedPoseStr);

        // Mechanism
        telemetry.addLine("─── CƠ CẤU GẮP ───");
        telemetry.addData("S1 (lift)", String.format(Locale.US, "%.3f", posS1));
        telemetry.addData("S5 (swing)", String.format(Locale.US, "%.3f", posS5));
        telemetry.addData("S3 (grip)", String.format(Locale.US, "%.3f", s3.getPosition()));

        // Panels tuner info
        telemetry.addLine("─── PANELS ───");
        telemetry.addData("Target [" + idx + "]", targetName);
        telemetry.addData("Column", BoxAutoPanels.TUNER_SELECTED_COLUMN);
        telemetry.addData("Row", BoxAutoPanels.TUNER_SELECTED_ROW);
        telemetry.addData("Compartment", BoxAutoPanels.TUNER_SELECTED_COMPARTMENT);

        telemetry.update();
    }
}
