package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.Locale;

/**
 * TeleOp tuner cho 10 vị trí trên sân (P0–P9).
 *
 * Hỗ trợ lái qua nút bấm tay Gamepad hoặc qua sliders trên FTControl Panels:
 *   - Trên Panels: bật `TUNER_DRIVE_TO_SELECTED = true`, điền `TUNER_TARGET_POSITION_INDEX` 0–9.
 *     Thay đổi tọa độ X, Y, H của vị trí đang chọn trên Panels sẽ làm robot cập nhật và lái tới ngay lập tức.
 *   - Trên Gamepad:
 *     D-pad UP / DOWN   → chọn vị trí P0–P9
 *     A                  → lái đến vị trí đã chọn
 *     B                  → in tọa độ hiện tại (copy → BoxAutoPanels)
 *     Y                  → refresh Panels
 *     Left stick         → lái tay (chậm, để tinh chỉnh)
 *     Right stick X      → xoay robot
 */
@TeleOp(name = "Pedro 10-Position Tuner", group = "Tuner")
public class PedroDemoTuner extends LinearOpMode {

    private Follower follower;
    private int gamepadSelectedIndex = 0;

    // Theo dõi target đang chạy qua Panels
    private int lastTargetIndex = -1;
    private Pose lastTargetPose = new Pose(0, 0, 0);

    // debounce flags
    private boolean prevDpadUp   = false;
    private boolean prevDpadDown = false;
    private boolean prevA        = false;
    private boolean prevB        = false;
    private boolean prevY        = false;

    @Override
    public void runOpMode() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(BoxAutoPanels.poseStart());

        telemetry.addLine("Pedro 10-Position Tuner ready");
        telemetry.addLine("Press PLAY to start");
        telemetry.update();
        waitForStart();

        follower.startTeleopDrive();

        while (opModeIsActive()) {
            BoxAutoPanels.refresh();

            handlePanelsAutodrive();
            handleGamepadInput();

            follower.update();
            showTelemetry();
        }
    }

    private void handlePanelsAutodrive() {
        if (BoxAutoPanels.TUNER_DRIVE_TO_SELECTED) {
            int targetIdx = BoxAutoPanels.TUNER_TARGET_POSITION_INDEX;
            if (targetIdx >= 0 && targetIdx < BoxAutoPanels.POSITION_COUNT) {
                Pose target = BoxAutoPanels.poseByIndex(targetIdx);

                // Nếu đổi index hoặc tọa độ slider bị lệch so với target trước đó
                boolean idxChanged = (targetIdx != lastTargetIndex);
                boolean poseChanged = (Math.abs(target.getX() - lastTargetPose.getX()) > 0.05
                        || Math.abs(target.getY() - lastTargetPose.getY()) > 0.05
                        || Math.abs(target.getHeading() - lastTargetPose.getHeading()) > 0.01);

                if (idxChanged || poseChanged) {
                    lastTargetIndex = targetIdx;
                    lastTargetPose = target;

                    // Lái robot đến tọa độ mới
                    Pose current = follower.getPose();
                    Path linePath = new Path(new BezierLine(current, target));
                    linePath.setLinearHeadingInterpolation(current.getHeading(), target.getHeading());
                    follower.followPath(linePath, true);
                }
            }
        } else {
            // Panels tắt drive_to_selected → hủy target tracking
            if (lastTargetIndex != -1) {
                lastTargetIndex = -1;
                follower.breakFollowing();
            }
        }
    }

    private void handleGamepadInput() {
        // D-pad UP / DOWN chọn index trên tay cầm
        if (gamepad1.dpad_up && !prevDpadUp) {
            gamepadSelectedIndex = (gamepadSelectedIndex - 1 + BoxAutoPanels.POSITION_COUNT)
                    % BoxAutoPanels.POSITION_COUNT;
        }
        prevDpadUp = gamepad1.dpad_up;

        if (gamepad1.dpad_down && !prevDpadDown) {
            gamepadSelectedIndex = (gamepadSelectedIndex + 1) % BoxAutoPanels.POSITION_COUNT;
        }
        prevDpadDown = gamepad1.dpad_down;

        // A → lái tay robot tới vị trí đang chọn trên Gamepad
        if (gamepad1.a && !prevA) {
            // Gamepad ghi đè chế độ Panels
            BoxAutoPanels.TUNER_DRIVE_TO_SELECTED = false;
            BoxAutoPanels.refresh();

            Pose target = BoxAutoPanels.poseByIndex(gamepadSelectedIndex);
            Pose current = follower.getPose();
            Path linePath = new Path(new BezierLine(current, target));
            linePath.setLinearHeadingInterpolation(current.getHeading(), target.getHeading());
            follower.followPath(linePath, true);
        }
        prevA = gamepad1.a;

        // B → in tọa độ hiện tại ra log
        if (gamepad1.b && !prevB) {
            Pose cur = follower.getPose();
            double hDeg = Math.toDegrees(cur.getHeading());
            telemetry.log().add(String.format(Locale.US,
                    "POSE >> X=%.2f  Y=%.2f  H=%.1f°", cur.getX(), cur.getY(), hDeg));
        }
        prevB = gamepad1.b;

        // Y → Force refresh Panels
        if (gamepad1.y && !prevY) {
            BoxAutoPanels.refresh();
            telemetry.log().add("BoxAutoPanels refreshed ✓");
        }
        prevY = gamepad1.y;

        // Manual drive (stick) — chỉ hoạt động khi không followPath
        double drive  = -gamepad1.left_stick_y * 0.35;
        double strafe =  gamepad1.left_stick_x * 0.35;
        double turn   =  gamepad1.right_stick_x * 0.35;
        if (Math.abs(drive) > 0.05 || Math.abs(strafe) > 0.05 || Math.abs(turn) > 0.05) {
            // Người dùng tác động JoyStick → Tắt chế độ Panels tự động
            if (BoxAutoPanels.TUNER_DRIVE_TO_SELECTED) {
                BoxAutoPanels.TUNER_DRIVE_TO_SELECTED = false;
                BoxAutoPanels.refresh();
            }
            follower.setTeleOpDrive(drive, strafe, turn, true);
        }
    }

    private void showTelemetry() {
        telemetry.addLine("═══ 10-POSITION TUNER ═══");
        telemetry.addData("Panels Auto-drive", BoxAutoPanels.TUNER_DRIVE_TO_SELECTED ? "ON (P" + BoxAutoPanels.TUNER_TARGET_POSITION_INDEX + ")" : "OFF");
        telemetry.addLine("");

        for (int i = 0; i < BoxAutoPanels.POSITION_COUNT; i++) {
            String marker = (i == gamepadSelectedIndex) ? " >> " : "    ";
            Pose p = BoxAutoPanels.poseByIndex(i);
            telemetry.addData(marker + BoxAutoPanels.poseName(i),
                    String.format(Locale.US, "X=%.1f Y=%.1f H=%.0f",
                            p.getX(), p.getY(), Math.toDegrees(p.getHeading())));
        }

        telemetry.addLine("");
        Pose robot = follower.getPose();
        telemetry.addData("Robot Current X", String.format(Locale.US, "%.2f", robot.getX()));
        telemetry.addData("Robot Current Y", String.format(Locale.US, "%.2f", robot.getY()));
        telemetry.addData("Robot Current H",
                String.format(Locale.US, "%.1f deg", Math.toDegrees(robot.getHeading())));
        telemetry.addLine("");
        telemetry.addLine("[A]=Gamepad Drive  [B]=Print Pose  [Y]=Refresh");
        telemetry.update();
    }
}
