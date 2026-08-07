package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import java.util.Locale;

/**
 * TUNER SERVO PANELS: Tune góc servo theo đúng chu trình gắp gạt của robot:
 *   S1: Lift  |  S2: Extension  |  S3: Gripper  |  S4: Wrist
 */
@TeleOp(name = "Servo Panels Tuner", group = "Tuner")
public class ServoPanelsTuner extends LinearOpMode {

    private Follower follower;
    private Servo s1, s2, s3, s4, s5;
    private Servo drop1;
    private PCA9685 pca;

    // Quản lý tracking lái robot
    // Quản lý tracking lái robot
    private int lastTargetIndex = -1;
    private Pose lastTargetPose = new Pose(0, 0, 0);

    // Servo S4 control
    private double targetS4 = 0.5;
    private double currentS4 = 0.5;
    private double lastWrittenS4 = 0.5;

    @Override
    public void runOpMode() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(BoxAutoPanels.getTunablePose(0));

        s1 = hardwareMap.get(Servo.class, BoxAutoPanels.NAME_S1);
        s2 = hardwareMap.get(Servo.class, BoxAutoPanels.NAME_S2);
        if (s2 instanceof ServoImplEx) {
            ((ServoImplEx) s2).setPwmRange(new PwmControl.PwmRange(500, 2500));
        }
        s3 = hardwareMap.get(Servo.class, BoxAutoPanels.NAME_S3);
        s4 = hardwareMap.get(Servo.class, BoxAutoPanels.NAME_S4);
        if (s4 instanceof com.qualcomm.robotcore.hardware.ServoImplEx) {
            ((com.qualcomm.robotcore.hardware.ServoImplEx) s4).setPwmRange(new com.qualcomm.robotcore.hardware.PwmControl.PwmRange(500, 2500));
        }
        try { s5 = hardwareMap.get(Servo.class, BoxAutoPanels.NAME_S5); } catch (Exception ignored) {}

        drop1 = hardwareMap.get(Servo.class, BoxAutoPanels.NAME_DROP1);
        pca   = hardwareMap.get(PCA9685.class, BoxAutoPanels.NAME_PCA9685);

        // Home tất cả servo
        applyPickSet(BoxAutoPanels.allHome());
        applyDropSet(BoxAutoPanels.D1_CLOSED, BoxAutoPanels.D2_CLOSED, BoxAutoPanels.D3_CLOSED);
        if (s5 != null) s5.setPosition(BoxAutoPanels.S5_CLOSED);

        telemetry.addLine("Servo Panels Tuner ready");
        telemetry.addLine("Press PLAY to start");
        telemetry.update();
        waitForStart();

        follower.startTeleopDrive();

        while (opModeIsActive()) {
            BoxAutoPanels.refresh();

            // Xử lý tự động lái tới vị trí Panels gắp/thả
            handlePanelsAutodrive();
            handleManualDrive();

            // Tự động áp dụng góc Servo liên tục (luôn ghi góc từ Panels xuống Servo mà không phụ thuộc vào TUNER_APPLY)
            applyTunerMode();
            
            // Cập nhật góc quay chậm cho Servo S4
            updateS4Slow();

            follower.update();
            showTelemetry();
            sleep(20);
        }
    }

    private void handlePanelsAutodrive() {
        if (BoxAutoPanels.TUNER_DRIVE_TO_SELECTED) {
            int targetIdx = BoxAutoPanels.TUNER_TARGET_POSITION_INDEX;
            if (targetIdx >= 0 && targetIdx < BoxAutoPanels.POSITION_COUNT) {
                Pose target = BoxAutoPanels.getTunablePose(targetIdx);

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
            }
        } else {
            if (lastTargetIndex != -1) {
                lastTargetIndex = -1;
                follower.breakFollowing();
            }
        }
    }

    private void handleManualDrive() {
        double drive  = -gamepad1.left_stick_y * 0.35;
        double strafe =  gamepad1.left_stick_x * 0.35;
        double turn   =  gamepad1.right_stick_x * 0.35;
        if (Math.abs(drive) > 0.05 || Math.abs(strafe) > 0.05 || Math.abs(turn) > 0.05) {
            if (BoxAutoPanels.TUNER_DRIVE_TO_SELECTED) {
                BoxAutoPanels.TUNER_DRIVE_TO_SELECTED = false;
                BoxAutoPanels.refresh();
            }
            follower.setTeleOpDrive(drive, strafe, turn, true);
        }
    }

    private void applyTunerMode() {
        switch (BoxAutoPanels.TUNER_MODE) {
            case 0: // Pick: Individual Servo
                applyIndividualPickServo();
                break;
            case 1: // Step 1: Ready to Grab (S4 Grab)
                applyPickSet(BoxAutoPanels.moveS4Grab());
                break;
            case 2: // Step 2: S1 hạ tầng pallet (S2 VẪN HOME ĐỂ TRÁNH VA CHẠM)
                applyPickSet(BoxAutoPanels.setElevatorLevel(BoxAutoPanels.TUNER_SELECTED_ROW));
                break;
            case 3: // Step 3: S2 Vươn tay ra tiếp cận hộp SAU KHI S1 ĐÃ HẠ XONG
                applyPickSet(BoxAutoPanels.extendS2(BoxAutoPanels.TUNER_SELECTED_ROW));
                break;
            case 4: // Step 4: Kẹp cả 2 ngàm S3 & S5
                applyPickSet(BoxAutoPanels.clampBothS3S5(BoxAutoPanels.TUNER_SELECTED_ROW));
                break;
            case 5: // Step 5: S2 Thu tay về HOME ngay lập tức kéo hộp ra khỏi kệ
                applyPickSet(BoxAutoPanels.retractS2(BoxAutoPanels.TUNER_SELECTED_ROW));
                break;
            case 6: // Step 6: S1 Lift slightly (Nhích lên nâng nhẹ)
                applyPickSet(BoxAutoPanels.liftUpS1(BoxAutoPanels.TUNER_SELECTED_ROW));
                break;
            case 7: // Step 7: Push S1 Highest
                applyPickSet(BoxAutoPanels.s1Highest());
                break;
            case 8: // Step 8: S4 lật ra sau trước (S2 vẫn thu gọn HOME)
                applyPickSet(BoxAutoPanels.s4FlipBackOnlyHigh());
                break;
            case 9: // Step 9: S2 vươn ra sau khi S4 đã lật xong
                applyPickSet(BoxAutoPanels.s2ExtendHighForDrop());
                break;
            case 10: // Step 10: S1 hạ xuống vị trí thả trực tiếp
                if (BoxAutoPanels.TUNER_SELECTED_POST == 1) {
                    applyPickSet(BoxAutoPanels.prepareS3LowDrop());
                } else {
                    applyPickSet(BoxAutoPanels.prepareS5LowDrop());
                }
                break;
            case 11: // Step 11: Drop Left Box (S3 Open)
                applyPickSet(BoxAutoPanels.openS3Drop());
                break;
            case 12: // Step 12: Drop Right Box (S5 Open)
                applyPickSet(BoxAutoPanels.openS5Drop());
                break;
            case 13: // Step 13: S1 nâng lên S1_HIGHEST sau khi thả để rút ngàm khỏi cọc
                applyPickSet(BoxAutoPanels.s1LiftUpAfterDrop());
                break;
            case 14: // Step 14: S2 thu về HOME đằng sau ở độ cao S1_HIGHEST
                applyPickSet(BoxAutoPanels.s2RetractHomeAfterDrop());
                break;
            case 15: // Step 15: ALL HOME (Tất cả Servo về vị trí xuất phát an toàn)
                applyPickSet(BoxAutoPanels.allHome());
                break;
            case 16: // Drop Servos: Tuner
                applyIndividualDropServo();
                break;
        }
    }

    private void applyIndividualPickServo() {
        double pos = BoxAutoPanels.clampS4(BoxAutoPanels.TUNER_SERVO_POS);
        switch (BoxAutoPanels.TUNER_SELECTED_SERVO) {
            case 1: s1.setPosition(pos); break;
            case 2: s2.setPosition(BoxAutoPanels.clampS2(pos)); break;
            case 3: s3.setPosition(BoxAutoPanels.clampS3(pos)); break;
            case 4: s4.setPosition(pos); break;
            case 5: if (s5 != null) s5.setPosition(pos); break;
        }
    }

    private void applyIndividualDropServo() {
        double pos = clamp(BoxAutoPanels.TUNER_SERVO_POS);
        switch (BoxAutoPanels.TUNER_SELECTED_DROP_SERVO) {
            case 1: drop1.setPosition(pos); break;
            case 2: pcaSetServo(BoxAutoPanels.DROP2_PCA_CHANNEL, pos); break;
            case 3: pcaSetServo(BoxAutoPanels.DROP3_PCA_CHANNEL, pos); break;
            case 4: if (s5 != null) s5.setPosition(pos); break;
        }
    }

    private void applyPickSet(BoxAutoPanels.PickServoSet set) {
        s1.setPosition(set.s1);
        s2.setPosition(set.s2);
        s3.setPosition(set.s3);
        targetS4 = clamp(set.s4);
        if (s5 != null) s5.setPosition(set.s5);
    }

    private void updateS4Slow() {
        if (BoxAutoPanels.TUNER_MODE == 0 && BoxAutoPanels.TUNER_SELECTED_SERVO == 4) {
            // Đang test lẻ S4 ở mode 0 thì bỏ qua updateS4Slow() để tránh đè giá trị
            return;
        }
        double delta = targetS4 - currentS4;
        if (Math.abs(delta) <= BoxAutoPanels.S4_SPEED_STEP) {
            currentS4 = targetS4;
        } else {
            currentS4 += Math.signum(delta) * BoxAutoPanels.S4_SPEED_STEP;
        }
        writeS4IfNeeded(false);
    }

    private void writeS4IfNeeded(boolean force) {
        if (force || Math.abs(currentS4 - lastWrittenS4) > 0.0005) {
            s4.setPosition(currentS4);
            lastWrittenS4 = currentS4;
        }
    }

    private void applyDropSet(double d1, double d2, double d3) {
        drop1.setPosition(d1);
        pcaSetServo(BoxAutoPanels.DROP2_PCA_CHANNEL, d2);
        pcaSetServo(BoxAutoPanels.DROP3_PCA_CHANNEL, d3);
    }

    private void pcaSetServo(int channel, double position) {
        double pulseUs = 500 + clamp(position) * 2000;
        pca.setServoPulseUs(channel, pulseUs);
    }

    private void showTelemetry() {
        telemetry.addLine("═══ SERVO PANELS TUNER ═══");
        telemetry.addData("APPLY SERVOS", BoxAutoPanels.TUNER_APPLY ? "ON" : "OFF");
        telemetry.addData("DRIVE ACTIVE", BoxAutoPanels.TUNER_DRIVE_TO_SELECTED ? "ON -> " + BoxAutoPanels.poseName(BoxAutoPanels.TUNER_TARGET_POSITION_INDEX) : "OFF");
        telemetry.addData("MODE", modeName(BoxAutoPanels.TUNER_MODE));
        telemetry.addLine("");

        Pose robot = follower.getPose();
        telemetry.addData("Robot Pose", String.format(Locale.US, "X=%.1f Y=%.1f H=%.1f°",
                robot.getX(), robot.getY(), Math.toDegrees(robot.getHeading())));
        telemetry.addLine("");

        telemetry.addData("Selected Row", BoxAutoPanels.TUNER_SELECTED_ROW == 1 ? "BOTTOM (ROW1)" : "TOP (ROW2)");
        telemetry.addData("Selected Post", BoxAutoPanels.TUNER_SELECTED_POST);
        telemetry.addLine("");

        // Live servo positions
        telemetry.addData("S1 (Lift)", String.format(Locale.US, "%.3f", s1.getPosition()));
        telemetry.addData("S2 (Extend)", String.format(Locale.US, "%.3f", s2.getPosition()));
        telemetry.addData("S3 (Gripper)", String.format(Locale.US, "%.3f", s3.getPosition()));
        telemetry.addData("S4 (Wrist target->current->hw)", String.format(Locale.US, "%.3f -> %.3f -> %.3f", targetS4, currentS4, s4.getPosition()));
        telemetry.addData("S4 Speed Step", String.format(Locale.US, "%.4f", BoxAutoPanels.S4_SPEED_STEP));
        telemetry.addData("S4 Config Name", BoxAutoPanels.NAME_S4);
        if (s4PresetsTooClose()) telemetry.addLine("WARN: S4_HOME / S4_GRAB / S4_STORE gần bằng nhau nên servo có thể không thấy quay.");
        if (s5 != null) telemetry.addData("S5 (Drop)", String.format(Locale.US, "%.3f", s5.getPosition()));

        telemetry.addData("D1", String.format(Locale.US, "%.3f", drop1.getPosition()));
        telemetry.addData("D2(PCA)", "ch " + BoxAutoPanels.DROP2_PCA_CHANNEL);
        telemetry.addData("D3(PCA)", "ch " + BoxAutoPanels.DROP3_PCA_CHANNEL);
        telemetry.update();
    }

    private String modeName(int mode) {
        switch (mode) {
            case 0: return "0: Pick Individual Servo";
            case 1: return "1: Step 1 (S4 Grab)";
            case 2: return "2: Step 2 (S1 Lower to Shelf, S2 Home)";
            case 3: return "3: Step 3 (S2 Extend to Box)";
            case 4: return "4: Step 4 (Clamp S3 & S5)";
            case 5: return "5: Step 5 (S2 Retract HOME Immediately)";
            case 6: return "6: Step 6 (S1 Lift Slightly)";
            case 7: return "7: Step 7 (S1 Highest)";
            case 8: return "8: Step 8 (S4 Flip Back, S2 Home)";
            case 9: return "9: Step 9 (S2 Extend After S4)";
            case 10: return "10: Step 10 (S1 Direct Drop Low)";
            case 11: return "11: Step 11 (S3 Drop Left Box)";
            case 12: return "12: Step 12 (S5 Drop Right Box)";
            case 13: return "13: Step 13 (S1 Lift Up to S1_HIGHEST After Drop)";
            case 14: return "14: Step 14 (S2 Retract HOME After Drop)";
            case 15: return "15: Step 15 (ALL HOME)";
            case 16: return "16: Drop Individual Servo";
            default: return "?";
        }
    }

    private boolean s4PresetsTooClose() {
        return Math.abs(BoxAutoPanels.S4_HOME - BoxAutoPanels.S4_GRAB) < 0.02
                && Math.abs(BoxAutoPanels.S4_HOME - BoxAutoPanels.S4_STORE) < 0.02
                && Math.abs(BoxAutoPanels.S4_GRAB - BoxAutoPanels.S4_STORE) < 0.02;
    }

    private static double clamp(double v) {
        return Math.max(0.0, Math.min(1.0, v));
    }
}
