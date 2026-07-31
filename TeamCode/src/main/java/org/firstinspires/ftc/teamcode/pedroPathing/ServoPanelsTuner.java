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
 * TUNER SERVO PANELS: Tune góc servo theo đúng chu trình gắp gạt của robot:
 *   S1: Lift  |  S2: Extension  |  S3: Gripper  |  S4: Wrist  |  S5: Side selection
 */
@TeleOp(name = "Servo Panels Tuner", group = "Tuner")
public class ServoPanelsTuner extends LinearOpMode {

    private Follower follower;
    private Servo s1, s2, s3, s4, s5;
    private Servo drop1;
    private PCA9685 pca;

    // Quản lý tracking lái robot
    private int lastTargetIndex = -1;
    private Pose lastTargetPose = new Pose(0, 0, 0);

    @Override
    public void runOpMode() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(BoxAutoPanels.poseStart());

        s1 = hardwareMap.get(Servo.class, BoxAutoPanels.NAME_S1);
        s2 = hardwareMap.get(Servo.class, BoxAutoPanels.NAME_S2);
        s3 = hardwareMap.get(Servo.class, BoxAutoPanels.NAME_S3);
        s4 = hardwareMap.get(Servo.class, BoxAutoPanels.NAME_S4);
        s5 = hardwareMap.get(Servo.class, BoxAutoPanels.NAME_S5);

        drop1 = hardwareMap.get(Servo.class, BoxAutoPanels.NAME_DROP1);
        pca   = hardwareMap.get(PCA9685.class, BoxAutoPanels.NAME_PCA9685);

        // Home tất cả servo
        applyPickSet(BoxAutoPanels.pickHome());
        applyDropSet(BoxAutoPanels.dropClosed());

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

            // Áp dụng góc Servo
            if (BoxAutoPanels.TUNER_APPLY) {
                applyTunerMode();
            }

            follower.update();
            showTelemetry();
            sleep(20);
        }
    }

    private void handlePanelsAutodrive() {
        if (BoxAutoPanels.TUNER_DRIVE_TO_SELECTED) {
            int targetIdx = BoxAutoPanels.TUNER_TARGET_POSITION_INDEX;
            if (targetIdx >= 0 && targetIdx < BoxAutoPanels.POSITION_COUNT) {
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
        double colSwing = (BoxAutoPanels.TUNER_SELECTED_COLUMN == 1) ? BoxAutoPanels.S5_LEFT : BoxAutoPanels.S5_RIGHT;
        double liftHeight = (BoxAutoPanels.TUNER_SELECTED_ROW == 1) ? BoxAutoPanels.S1_ROW1 : BoxAutoPanels.S1_ROW2;
        double extendLength = BoxAutoPanels.S2_EXTEND;

        switch (BoxAutoPanels.TUNER_MODE) {
            case 0: // Pick: Individual Servo
                applyIndividualPickServo();
                break;
            case 1: // Step 1: Prep Wrist swing S4 / S5
                s4.setPosition(BoxAutoPanels.S4_GRAB);
                s5.setPosition(colSwing);
                break;
            case 2: // Step 2: Set Lift height S1
                s4.setPosition(BoxAutoPanels.S4_GRAB);
                s5.setPosition(colSwing);
                s1.setPosition(liftHeight);
                break;
            case 3: // Step 3: Extend S2
                s4.setPosition(BoxAutoPanels.S4_GRAB);
                s5.setPosition(colSwing);
                s1.setPosition(liftHeight);
                s2.setPosition(BoxAutoPanels.clampS2(extendLength));
                break;
            case 4: // Step 4: Grip S3 (0.1)
                s4.setPosition(BoxAutoPanels.S4_GRAB);
                s5.setPosition(colSwing);
                s1.setPosition(liftHeight);
                s2.setPosition(BoxAutoPanels.clampS2(extendLength));
                s3.setPosition(BoxAutoPanels.S3_CLOSED); // 0.1
                break;
            case 5: // Step 5: S1 Lift slightly
                s4.setPosition(BoxAutoPanels.S4_GRAB);
                s5.setPosition(colSwing);
                s1.setPosition(liftHeight + BoxAutoPanels.S1_LIFT_UP_OFFSET);
                s2.setPosition(BoxAutoPanels.clampS2(extendLength));
                s3.setPosition(BoxAutoPanels.S3_CLOSED);
                break;
            case 6: // Step 6: S2 Retract (HOME)
                s4.setPosition(BoxAutoPanels.S4_GRAB);
                s5.setPosition(colSwing);
                s1.setPosition(liftHeight + BoxAutoPanels.S1_LIFT_UP_OFFSET);
                s2.setPosition(BoxAutoPanels.S2_HOME); // 0.75
                s3.setPosition(BoxAutoPanels.S3_CLOSED);
                break;
            case 7: // Step 7: HIGH_STORE raise + S4 rotate store
                s1.setPosition(BoxAutoPanels.S1_HIGH_STORE);
                s2.setPosition(BoxAutoPanels.S2_HOME);
                s3.setPosition(BoxAutoPanels.S3_CLOSED);
                s4.setPosition(BoxAutoPanels.S4_STORE);
                break;
            case 8: // Step 8: S5 quẹo Compartment & S1/S2 store config (S3 open)
                s4.setPosition(BoxAutoPanels.S4_STORE);
                s5.setPosition(getS5StoreAngle(BoxAutoPanels.TUNER_SELECTED_COMPARTMENT));
                s1.setPosition(BoxAutoPanels.S1_STORE);
                s2.setPosition(BoxAutoPanels.clampS2(BoxAutoPanels.S2_STORE));
                s3.setPosition(BoxAutoPanels.S3_OPEN); // 0.0
                break;
            case 9: // Drop: Individual Drop Servo
                applyIndividualDropServo();
                break;
            case 10: // Drop: Closed
                applyDropSet(BoxAutoPanels.dropClosed());
                break;
            case 11: // Drop: Open Compartment
                applyDropSet(BoxAutoPanels.dropOpen(BoxAutoPanels.TUNER_SELECTED_COMPARTMENT));
                break;
        }
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

    private void applyIndividualPickServo() {
        double pos = clamp(BoxAutoPanels.TUNER_SERVO_POS);
        switch (BoxAutoPanels.TUNER_SELECTED_SERVO) {
            case 1: s1.setPosition(pos); break;
            case 2: s2.setPosition(BoxAutoPanels.clampS2(pos)); break;
            case 3: s3.setPosition(BoxAutoPanels.clampS3(pos)); break;
            case 4: s4.setPosition(pos); break;
            case 5: s5.setPosition(pos); break;
        }
    }

    private void applyIndividualDropServo() {
        double pos = clamp(BoxAutoPanels.TUNER_SERVO_POS);
        switch (BoxAutoPanels.TUNER_SELECTED_DROP_SERVO) {
            case 1: drop1.setPosition(pos); break;
            case 2: pcaSetServo(BoxAutoPanels.DROP2_PCA_CHANNEL, pos); break;
            case 3: pcaSetServo(BoxAutoPanels.DROP3_PCA_CHANNEL, pos); break;
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
        drop1.setPosition(set.d1);
        pcaSetServo(BoxAutoPanels.DROP2_PCA_CHANNEL, set.d2);
        pcaSetServo(BoxAutoPanels.DROP3_PCA_CHANNEL, set.d3);
    }

    private void pcaSetServo(int channel, double position) {
        double pulseUs = 500 + clamp(position) * 2000;
        pca.setServoPulseUs(channel, pulseUs);
    }

    private void showTelemetry() {
        telemetry.addLine("═══ SERVO PANELS TUNER ═══");
        telemetry.addData("APPLY SERVOS", BoxAutoPanels.TUNER_APPLY ? "ON" : "OFF");
        telemetry.addData("DRIVE ACTIVE", BoxAutoPanels.TUNER_DRIVE_TO_SELECTED ? "ON (P" + BoxAutoPanels.TUNER_TARGET_POSITION_INDEX + ")" : "OFF");
        telemetry.addData("MODE", modeName(BoxAutoPanels.TUNER_MODE));
        telemetry.addLine("");

        Pose robot = follower.getPose();
        telemetry.addData("Robot Pose", String.format(Locale.US, "X=%.1f Y=%.1f H=%.1f°",
                robot.getX(), robot.getY(), Math.toDegrees(robot.getHeading())));
        telemetry.addLine("");

        telemetry.addData("Selected Column", BoxAutoPanels.TUNER_SELECTED_COLUMN == 1 ? "LEFT" : "RIGHT");
        telemetry.addData("Selected Row", BoxAutoPanels.TUNER_SELECTED_ROW == 1 ? "BOTTOM (ROW1)" : "TOP (ROW2)");
        telemetry.addData("Selected Compartment", BoxAutoPanels.TUNER_SELECTED_COMPARTMENT);
        telemetry.addLine("");

        // Live servo positions
        telemetry.addData("S1 (Lift)", String.format(Locale.US, "%.3f", s1.getPosition()));
        telemetry.addData("S2 (Extend)", String.format(Locale.US, "%.3f", s2.getPosition()));
        telemetry.addData("S3 (Gripper)", String.format(Locale.US, "%.3f", s3.getPosition()));
        telemetry.addData("S4 (Wrist)", String.format(Locale.US, "%.3f", s4.getPosition()));
        telemetry.addData("S5 (Side)", String.format(Locale.US, "%.3f", s5.getPosition()));
        telemetry.addData("D1", String.format(Locale.US, "%.3f", drop1.getPosition()));
        telemetry.addData("D2(PCA)", "ch " + BoxAutoPanels.DROP2_PCA_CHANNEL);
        telemetry.addData("D3(PCA)", "ch " + BoxAutoPanels.DROP3_PCA_CHANNEL);
        telemetry.update();
    }

    private String modeName(int mode) {
        switch (mode) {
            case 0: return "0: Pick Individual Servo";
            case 1: return "1: Step 1 (Prep Wrist/Swing)";
            case 2: return "2: Step 2 (Lift height S1)";
            case 3: return "3: Step 3 (Extend S2)";
            case 4: return "4: Step 4 (Grip S3)";
            case 5: return "5: Step 5 (S1 Lift slightly)";
            case 6: return "6: Step 6 (S2 Retract)";
            case 7: return "7: Step 7 (HIGH_STORE wrist rotation)";
            case 8: return "8: Step 8 (Swing Compartment & Open)";
            case 9: return "9: Drop Individual Drop Servo";
            case 10: return "10: Drop Closed";
            case 11: return "11: Drop Open Compartment";
            default: return "?";
        }
    }

    private static double clamp(double v) {
        return Math.max(0.0, Math.min(1.0, v));
    }
}
