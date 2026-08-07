package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.PwmControl;

import java.util.Locale;

/**
 * MANUAL TUNER: Tune tay gamepad 1 & 2 trực tiếp.
 *  - Gamepad 1: Lái robot tay / Tự động lái tới điểm P(0..9)
 *  - Gamepad 2: Tune góc Servo gắp (S1..S4) và Servo thả (D1..D3)
 */
@TeleOp(name = "Manual Tuner", group = "Tuner")
public class ManualTuner extends LinearOpMode {

    private Follower follower;

    // Servos
    private Servo s1, s2, s3, s4, s5;
    private Servo drop1;
    private PCA9685 pca;

    // Jogging tracking
    private int selectedServoIndex = 0; // 0=S1, 1=S2, 2=S3, 3=S4 for pick
    private int selectedDropServoIndex = 0; // 0=D1, 1=D2, 2=D3

    private double targetS1 = BoxAutoPanels.S1_HOME;
    private double targetS2 = BoxAutoPanels.S2_HOME;
    private double targetS3 = BoxAutoPanels.S3_OPEN;
    private double targetS4 = BoxAutoPanels.S4_GRAB;

    private double targetD1 = BoxAutoPanels.D1_CLOSED;
    private double targetD2 = BoxAutoPanels.D2_CLOSED;
    private double targetD3 = BoxAutoPanels.D3_CLOSED;
    private double targetS5 = BoxAutoPanels.S5_CLOSED;

    private double currentS1 = BoxAutoPanels.S1_HOME;
    private double currentS2 = BoxAutoPanels.S2_HOME;
    private double currentS3 = BoxAutoPanels.S3_OPEN;
    private double currentS4 = BoxAutoPanels.S4_GRAB;

    private double currentD1 = BoxAutoPanels.D1_CLOSED;
    private double currentD2 = BoxAutoPanels.D2_CLOSED;
    private double currentD3 = BoxAutoPanels.D3_CLOSED;
    private double currentS5 = BoxAutoPanels.S5_CLOSED;

    private double lastWrittenS1 = -1, lastWrittenS2 = -1, lastWrittenS3 = -1, lastWrittenS4 = -1;
    private double lastWrittenD1 = -1, lastWrittenS5 = -1;

    // Gamepad debouncers
    private boolean lastLb1 = false, lastRb1 = false;
    private boolean lastX1 = false;
    private boolean lastLb2 = false, lastRb2 = false;
    private boolean lastDpadUp2 = false, lastDpadDown2 = false;
    private boolean lastA2 = false, lastB2 = false;

    private int selectedPoseIndex = 0;
    private boolean isDrivingToTarget = false;
    private int mode = 1;
    private int targetRow = 2; // Default row 2
    private int targetPost = 1; // Default post 1

    @Override
    public void runOpMode() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(BoxAutoPanels.getTunablePose(0));

        try { s1 = hardwareMap.get(Servo.class, BoxAutoPanels.NAME_S1); } catch (Exception ignored) {}
        try { s2 = hardwareMap.get(Servo.class, BoxAutoPanels.NAME_S2); } catch (Exception ignored) {}
        try { s3 = hardwareMap.get(Servo.class, BoxAutoPanels.NAME_S3); } catch (Exception ignored) {}
        try {
            s4 = hardwareMap.get(Servo.class, BoxAutoPanels.NAME_S4);
            if (s4 instanceof ServoImplEx) {
                ((ServoImplEx) s4).setPwmRange(new PwmControl.PwmRange(500, 2500));
            }
        } catch (Exception ignored) {}
        try { s5 = hardwareMap.get(Servo.class, BoxAutoPanels.NAME_S5); } catch (Exception ignored) {}

        if (s2 instanceof ServoImplEx) {
            ((ServoImplEx) s2).setPwmRange(new PwmControl.PwmRange(500, 2500));
        }

        try { drop1 = hardwareMap.get(Servo.class, BoxAutoPanels.NAME_DROP1); } catch (Exception ignored) {}
        try { pca = hardwareMap.get(PCA9685.class, BoxAutoPanels.NAME_PCA9685); } catch (Exception ignored) {}

        readCurrentServoPositions();

        telemetry.addLine("Manual Tuner ready");
        telemetry.update();
        waitForStart();

        follower.startTeleopDrive();

        while (opModeIsActive()) {
            BoxAutoPanels.refresh();

            handleGamepad1Drive();
            handleGamepad2Tuner();

            interpolateServos();
            writeServosToHardware(false);

            follower.update();
            showTelemetry();
            sleep(20);
        }
    }

    private void readCurrentServoPositions() {
        if (s1 != null) currentS1 = targetS1 = s1.getPosition();
        if (s2 != null) currentS2 = targetS2 = s2.getPosition();
        if (s3 != null) currentS3 = targetS3 = s3.getPosition();
        if (s4 != null) currentS4 = targetS4 = s4.getPosition();
        if (drop1 != null) currentD1 = targetD1 = drop1.getPosition();
        if (s5 != null) currentS5 = targetS5 = s5.getPosition();
    }

    private void handleGamepad1Drive() {
        boolean lb = gamepad1.left_bumper;
        boolean rb = gamepad1.right_bumper;
        boolean x  = gamepad1.x;

        if (lb && !lastLb1) {
            selectedPoseIndex = (selectedPoseIndex - 1 + 10) % 10;
        }
        if (rb && !lastRb1) {
            selectedPoseIndex = (selectedPoseIndex + 1) % 10;
        }
        lastLb1 = lb;
        lastRb1 = rb;

        if (x && !lastX1) {
            isDrivingToTarget = !isDrivingToTarget;
            if (isDrivingToTarget) {
                Pose target = BoxAutoPanels.getTunablePose(selectedPoseIndex);
                Pose current = follower.getPose();
                Path linePath = new Path(new BezierLine(current, target));
                linePath.setLinearHeadingInterpolation(current.getHeading(), target.getHeading());
                follower.followPath(linePath, true);
            } else {
                follower.breakFollowing();
            }
        }
        lastX1 = x;

        double drive  = -gamepad1.left_stick_y * 0.35;
        double strafe =  gamepad1.left_stick_x * 0.35;
        double turn   =  gamepad1.right_stick_x * 0.35;

        if (Math.abs(drive) > 0.05 || Math.abs(strafe) > 0.05 || Math.abs(turn) > 0.05) {
            if (isDrivingToTarget) {
                isDrivingToTarget = false;
                follower.breakFollowing();
            }
            follower.setTeleOpDrive(drive, strafe, turn, true);
        }
    }

    private void handleGamepad2Tuner() {
        boolean lb = gamepad2.left_bumper;
        boolean rb = gamepad2.right_bumper;
        boolean dpUp = gamepad2.dpad_up;
        boolean dpDown = gamepad2.dpad_down;
        boolean a = gamepad2.a;
        boolean b = gamepad2.b;

        if (lb && !lastLb2) {
            mode = (mode - 1 + 17) % 17;
            applyModePreset();
        }
        if (rb && !lastRb2) {
            mode = (mode + 1) % 17;
            applyModePreset();
        }
        lastLb2 = lb;
        lastRb2 = rb;

        if (mode == 0) {
            if (dpUp && !lastDpadUp2) selectedServoIndex = (selectedServoIndex - 1 + 5) % 5;
            if (dpDown && !lastDpadDown2) selectedServoIndex = (selectedServoIndex + 1) % 5;
            if (a && !lastA2) targetS3 = BoxAutoPanels.S3_CLOSED;
            if (b && !lastB2) targetS3 = BoxAutoPanels.S3_OPEN;
        } else if (mode == 16) {
            if (dpUp && !lastDpadUp2) selectedDropServoIndex = (selectedDropServoIndex - 1 + 4) % 4;
            if (dpDown && !lastDpadDown2) selectedDropServoIndex = (selectedDropServoIndex + 1) % 4;
        } else {
            if (dpUp && !lastDpadUp2) {
                targetRow = (targetRow == 1) ? 2 : 1;
                applyModePreset();
            }
            if (dpDown && !lastDpadDown2) {
                targetPost = (targetPost == 1) ? 2 : 1;
                applyModePreset();
            }
        }
        lastDpadUp2 = dpUp;
        lastDpadDown2 = dpDown;
        lastA2 = a;
        lastB2 = b;

        double stickY = -gamepad2.left_stick_y;
        if (Math.abs(stickY) > 0.1) {
            double step = stickY * 0.005;
            jogActiveServo(step);
        }
    }

    private void jogActiveServo(double amount) {
        if (mode == 0) {
            switch (selectedServoIndex) {
                case 0: targetS1 = clamp(targetS1 + amount); currentS1 = targetS1; break;
                case 1: targetS2 = BoxAutoPanels.clampS2(targetS2 + amount); currentS2 = targetS2; break;
                case 2: targetS3 = BoxAutoPanels.clampS3(targetS3 + amount); currentS3 = targetS3; break;
                case 3: targetS4 = clamp(targetS4 + amount); currentS4 = targetS4; break;
                case 4: targetS5 = clamp(targetS5 + amount); currentS5 = targetS5; break;
            }
        } else if (mode == 16) {
            switch (selectedDropServoIndex) {
                case 0: targetD1 = clamp(targetD1 + amount); currentD1 = targetD1; break;
                case 1: targetD2 = clamp(targetD2 + amount); currentD2 = targetD2; break;
                case 2: targetD3 = clamp(targetD3 + amount); currentD3 = targetD3; break;
                case 3: targetS5 = clamp(targetS5 + amount); currentS5 = targetS5; break;
            }
        } else {
            // Cho phép nhích góc Servo đang chọn ở BẤT KỲ Mode nào (Mode 1..13)
            switch (selectedServoIndex) {
                case 0: targetS1 = clamp(targetS1 + amount); currentS1 = targetS1; break;
                case 1: targetS2 = BoxAutoPanels.clampS2(targetS2 + amount); currentS2 = targetS2; break;
                case 2: targetS3 = BoxAutoPanels.clampS3(targetS3 + amount); currentS3 = targetS3; break;
                case 3: targetS4 = clamp(targetS4 + amount); currentS4 = targetS4; break;
                case 4: targetS5 = clamp(targetS5 + amount); currentS5 = targetS5; break;
            }
        }
        syncToBoxAutoPanels();
    }

    private void syncToBoxAutoPanels() {
        switch (mode) {
            case 1: BoxAutoPanels.S4_GRAB = targetS4; break;
            case 2:
                if (targetRow == 1) BoxAutoPanels.S1_ROW1 = targetS1;
                else BoxAutoPanels.S1_ROW2 = targetS1;
                BoxAutoPanels.S2_EXTEND = targetS2;
                break;
            case 3:
                BoxAutoPanels.S3_CLOSED = targetS3;
                BoxAutoPanels.S5_CLOSED = targetS5;
                break;
            case 4: BoxAutoPanels.S2_HOME = targetS2; break;
            case 5:
                double baseLift = (targetRow == 1) ? BoxAutoPanels.S1_ROW1 : BoxAutoPanels.S1_ROW2;
                BoxAutoPanels.S1_LIFT_UP_OFFSET = baseLift - targetS1;
                break;
            case 6: BoxAutoPanels.S1_HIGHEST = targetS1; break;
            case 7: BoxAutoPanels.S4_STORE = targetS4; break;
            case 8: BoxAutoPanels.S2_STORE_EXTEND = targetS2; break;
            case 9: BoxAutoPanels.S1_DROP_LOW = targetS1; break;
            case 10: BoxAutoPanels.S3_OPEN = targetS3; break;
            case 11: BoxAutoPanels.S5_OPEN = targetS5; break;
            case 12: BoxAutoPanels.S1_HOME = targetS1; break;
        }
    }

    private void applyModePreset() {
        switch (mode) {
            case 1: applySet(BoxAutoPanels.moveS4Grab()); break;
            case 2: applySet(BoxAutoPanels.setElevatorLevel(targetRow)); break; // Step 2: S1 hạ tầng pallet (S2 VẪN HOME)
            case 3: applySet(BoxAutoPanels.extendS2(targetRow)); break;        // Step 3: S2 vươn ra tiếp cận hộp
            case 4: applySet(BoxAutoPanels.clampBothS3S5(targetRow)); break;   // Step 4: Kẹp cả 2 ngàm S3 & S5
            case 5: applySet(BoxAutoPanels.retractS2(targetRow)); break;      // Step 5: S2 thu tay về HOME ngay lập tức
            case 6: applySet(BoxAutoPanels.liftUpS1(targetRow)); break;        // Step 6: S1 nhích nâng lên
            case 7: applySet(BoxAutoPanels.s1Highest()); break;                // Step 7: S1 nâng HIGHEST
            case 8: applySet(BoxAutoPanels.s4FlipBackOnlyHigh()); break;      // Step 8: S4 lật ra sau
            case 9: applySet(BoxAutoPanels.s2ExtendHighForDrop()); break;     // Step 9: S2 vươn ra sau khi S4 lật xong
            case 10:
                if (targetPost == 1) applySet(BoxAutoPanels.prepareS3LowDrop());
                else applySet(BoxAutoPanels.prepareS5LowDrop());
                break;
            case 11:
                if (targetPost == 1) applySet(BoxAutoPanels.openS3Drop());
                else applySet(BoxAutoPanels.openS5Drop());
                break;
            case 12: applySet(BoxAutoPanels.allHome()); break;
        }
    }

    private void applySet(BoxAutoPanels.PickServoSet p) {
        targetS1 = p.s1;
        targetS2 = p.s2;
        targetS3 = p.s3;
        targetS4 = p.s4;
        targetS5 = p.s5;
    }

    private void interpolateServos() {
        double alpha = 0.2;
        currentS1 += (targetS1 - currentS1) * alpha;
        currentS2 = targetS2; // Tốc độ & lực tối đa hardware cho Servo 2 (Bỏ trễ phần mềm)
        currentS3 = targetS3;
        double s4Delta = targetS4 - currentS4;
        if (Math.abs(s4Delta) <= BoxAutoPanels.S4_SPEED_STEP) {
            currentS4 = targetS4;
        } else {
            currentS4 += Math.signum(s4Delta) * BoxAutoPanels.S4_SPEED_STEP;
        }
        currentD1 += (targetD1 - currentD1) * alpha;
        currentD2 += (targetD2 - currentD2) * alpha;
        currentD3 += (targetD3 - currentD3) * alpha;
        currentS5 += (targetS5 - currentS5) * alpha;
    }

    private void writeServosToHardware(boolean force) {
        if (s1 != null && (force || Math.abs(currentS1 - lastWrittenS1) > 0.0005)) {
            s1.setPosition(currentS1); lastWrittenS1 = currentS1;
        }
        if (s2 != null && (force || Math.abs(currentS2 - lastWrittenS2) > 0.0005)) {
            s2.setPosition(currentS2); lastWrittenS2 = currentS2;
        }
        if (s3 != null && (force || Math.abs(currentS3 - lastWrittenS3) > 0.0005)) {
            s3.setPosition(currentS3); lastWrittenS3 = currentS3;
        }
        if (s4 != null && (force || Math.abs(currentS4 - lastWrittenS4) > 0.0005)) {
            s4.setPosition(currentS4); lastWrittenS4 = currentS4;
        }
        if (s5 != null && (force || Math.abs(currentS5 - lastWrittenS5) > 0.0005)) {
            s5.setPosition(currentS5); lastWrittenS5 = currentS5;
        }
        if (drop1 != null && (force || Math.abs(currentD1 - lastWrittenD1) > 0.0005)) {
            drop1.setPosition(currentD1); lastWrittenD1 = currentD1;
        }
        if (pca != null) {
            pca.setServoPulseUs(BoxAutoPanels.DROP2_PCA_CHANNEL, 500 + currentD2 * 2000);
            pca.setServoPulseUs(BoxAutoPanels.DROP3_PCA_CHANNEL, 500 + currentD3 * 2000);
        }
    }

    private void showTelemetry() {
        telemetry.addLine("═══ MANUAL TUNER (DUAL GRIPPER S3 & S5) ═══");
        telemetry.addData("MODE", modeName(mode));
        telemetry.addData("Target P(0..9)", BoxAutoPanels.getTunablePoseName(selectedPoseIndex));
        telemetry.addData("Autodrive to P", isDrivingToTarget ? "ACTIVE" : "OFF");
        telemetry.addLine("");

        Pose robot = follower.getPose();
        telemetry.addData("Robot Pose", String.format(Locale.US, "X=%.1f Y=%.1f H=%.1f°",
                robot.getX(), robot.getY(), Math.toDegrees(robot.getHeading())));
        telemetry.addLine("");

        telemetry.addData("S1 (Lift)", String.format(Locale.US, "%.3f -> %.3f", targetS1, currentS1));
        telemetry.addData("S2 (Extend)", String.format(Locale.US, "%.3f -> %.3f", targetS2, currentS2));
        telemetry.addData("S3 (Gripper L)", String.format(Locale.US, "%.3f -> %.3f", targetS3, currentS3));
        telemetry.addData("S4 (Wrist)", String.format(Locale.US, "%.3f -> %.3f", targetS4, currentS4));
        telemetry.addData("S5 (Gripper R)", String.format(Locale.US, "%.3f -> %.3f", targetS5, currentS5));
        telemetry.addData("D1", String.format(Locale.US, "%.3f -> %.3f", targetD1, currentD1));
        telemetry.update();
    }

    private String modeName(int m) {
        switch (m) {
            case 0: return "0: TUNE LẺ PICK SERVO (S1-S5)";
            case 1: return "1: S4 Quay Trước";
            case 2: return "2: S1 Hạ Tầng & S2 Vươn Tiếp Cận Trực Tiếp";
            case 3: return "3: Kẹp Đôi S3 & S5";
            case 4: return "4: S2 Thu Tay Về HOME Ngay Lập Tức";
            case 5: return "5: S1 Nhích Nâng Lên";
            case 6: return "6: S1 Nâng Cao HIGHEST";
            case 7: return "7: S4 Lật Ra Sau (S2 vẫn HOME)";
            case 8: return "8: S2 Vươn Sau Khi S4 Xong";
            case 9: return "9: S1 Hạ Vị Trí Thả Trực Tiếp";
            case 10: return "10: Nhả Hộp Trái (S3 Open)";
            case 11: return "11: Nhả Hộp Phải (S5 Open)";
            case 12: return "12: ALL HOME";
            case 13: return "13: TUNE LẺ DROP (D1-D3)";
            default: return "?";
        }
    }

    private static double clamp(double v) {
        return Math.max(0.0, Math.min(1.0, v));
    }
}
