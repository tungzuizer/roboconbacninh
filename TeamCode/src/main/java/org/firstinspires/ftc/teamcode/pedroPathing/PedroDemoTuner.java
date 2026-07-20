package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.*;
import static com.pedropathing.ivy.pedro.PedroCommands.*;

/**
 * OpMode chạy thử (Demo) và Tinh chỉnh tọa độ (Tuner):
 * 1. Chạy tới 1 vị trí mục tiêu bằng Pedro Pathing.
 * 2. Dừng hẳn và tự hít vào vạch bằng BFD-1000.
 * 3. Hiển thị tọa độ Tuyệt đối để bạn điền vào code Auto chính.
 */
@Autonomous(name = "Pedro Demo & Position Tuner", group = "Test")
public class PedroDemoTuner extends LinearOpMode {
    private Follower follower;
    private DigitalChannel lineS1, lineS2, lineS3, lineS4, lineS5;

    // --- BẠN SỬA CÁC GIÁ TRỊ NÀY ĐỂ TUNE ---
    public static double TEST_X = 120.0; 
    public static double TEST_Y = 20.0;  
    public static double TEST_H = 0.0;   
    // ---------------------------------------

    private long stabilizeStartTime;

    @Override
    public void runOpMode() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0,0,0));

        lineS1 = hardwareMap.get(DigitalChannel.class, "lineS1");
        lineS2 = hardwareMap.get(DigitalChannel.class, "lineS2");
        lineS3 = hardwareMap.get(DigitalChannel.class, "lineS3");
        lineS4 = hardwareMap.get(DigitalChannel.class, "lineS4");
        lineS5 = hardwareMap.get(DigitalChannel.class, "lineS5");

        telemetry.addLine("Bấm START để chạy Demo thử nghiệm.");
        telemetry.update();

        waitForStart();

        Pose targetPose = new Pose(TEST_X, TEST_Y, Math.toRadians(TEST_H));

        PathChain demoPath = follower.pathBuilder()
                .addPath(new com.pedropathing.geometry.BezierLine(new Pose(0,0,0), targetPose))
                .setConstantHeadingInterpolation(targetPose.getHeading())
                .build();

        Scheduler.schedule(sequential(
            follow(follower, demoPath, true),
            
            Command.build().setDone(() -> {
                double dx = follower.getPose().getX() - targetPose.getX();
                double dy = follower.getPose().getY() - targetPose.getY();
                return Math.sqrt(dx*dx + dy*dy) < 1.5 || !follower.isBusy();
            }),
            
            waitMs(500),
            
            snapToLineDemo(1500),
            
            instant(() -> {
                Pose finalPose = follower.getPose();
                telemetry.addLine("=== KẾT QUẢ TUNE ===");
                telemetry.addData("X Tuyệt đối", "%.3f", finalPose.getX());
                telemetry.addData("Y Tuyệt đối", "%.3f", finalPose.getY());
                telemetry.addData("H Tuyệt đối", "%.1f", Math.toDegrees(finalPose.getHeading()));
                telemetry.update();
            }),
            
            waitMs(30000)
        ));

        while (opModeIsActive()) {
            follower.update();
            Scheduler.execute();
            telemetry.update();
        }
    }

    private Command snapToLineDemo(double timeoutMs) {
        return Command.build()
                .setStart(() -> stabilizeStartTime = System.currentTimeMillis())
                .setExecute(() -> {
                    boolean b2 = !lineS2.getState();
                    boolean b3 = !lineS3.getState();
                    boolean b4 = !lineS4.getState();
                    Pose curr = follower.getPose();
                    
                    if (b3) {
                        telemetry.addLine("🎯 ĐÃ KHỚP TÂM VẠCH!");
                    } else if (b2) {
                        follower.setPose(new Pose(curr.getX(), curr.getY() + 0.05, curr.getHeading()));
                        telemetry.addLine("⬅️ Đang nhích TRÁI...");
                    } else if (b4) {
                        follower.setPose(new Pose(curr.getX(), curr.getY() - 0.05, curr.getHeading()));
                        telemetry.addLine("➡️ Đang nhích PHẢI...");
                    }
                })
                .setDone(() -> !lineS3.getState() || (System.currentTimeMillis() - stabilizeStartTime > timeoutMs));
    }
}
