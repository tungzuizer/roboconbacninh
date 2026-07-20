package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.*;
import static com.pedropathing.ivy.pedro.PedroCommands.*;

@Autonomous(name = "Pedro Accuracy Verifier", group = "Test")
public class PedroAccuracyVerifier extends LinearOpMode {
    private Follower follower;
    private DigitalChannel lineS1, lineS2, lineS3, lineS4, lineS5;

    // Tọa độ test (điền số sau khi đo từ Tuner)
    public static Pose TEST_POSE_1 = new Pose(24.0, 24.0, 0); 
    public static Pose TEST_POSE_2 = new Pose(24.0, 72.0, 0); 

    private long stabilizeStartTime;

    @Override
    public void runOpMode() {
        follower = Constants.createFollower(hardwareMap);
        lineS1 = hardwareMap.get(DigitalChannel.class, "lineS1");
        lineS2 = hardwareMap.get(DigitalChannel.class, "lineS2");
        lineS3 = hardwareMap.get(DigitalChannel.class, "lineS3");
        lineS4 = hardwareMap.get(DigitalChannel.class, "lineS4");
        lineS5 = hardwareMap.get(DigitalChannel.class, "lineS5");

        waitForStart();

        Scheduler.schedule(sequential(
            followWithLine(TEST_POSE_1),
            waitMs(2000),
            followWithLine(TEST_POSE_2),
            instant(() -> telemetry.addLine("✅ HOÀN THÀNH KIỂM TRA!"))
        ));

        while (opModeIsActive()) {
            follower.update();
            Scheduler.execute();
            telemetry.update();
        }
    }

    private Command followWithLine(Pose target) {
        return sequential(
            follow(follower, follower.pathBuilder().addPath(new com.pedropathing.geometry.BezierLine(follower.getPose(), target)).setConstantHeadingInterpolation(target.getHeading()).build(), true),
            Command.build()
                .setStart(() -> stabilizeStartTime = System.currentTimeMillis())
                .setExecute(() -> {
                    boolean b2 = !lineS2.getState();
                    boolean b3 = !lineS3.getState();
                    boolean b4 = !lineS4.getState();
                    Pose curr = follower.getPose();
                    if (b2 && !b4) follower.setPose(new Pose(curr.getX(), curr.getY() - 0.02, curr.getHeading()));
                    else if (b4 && !b2) follower.setPose(new Pose(curr.getX(), curr.getY() + 0.02, curr.getHeading()));
                    else if (b3) {
                        follower.setPose(target); // Ép tọa độ tuyệt đối
                        telemetry.addLine("🎯 ĐÃ KHỚP TÂM VẠCH!");
                    }
                    telemetry.update();
                })
                .setDone(() -> !lineS3.getState() || (System.currentTimeMillis() - stabilizeStartTime > 3000))
        );
    }
}
