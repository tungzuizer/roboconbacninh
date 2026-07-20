package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@TeleOp(name = "Pedro Line Snap Tuner", group = "Tuner")
public class PedroLineTuner extends LinearOpMode {
    private Follower follower;
    private DigitalChannel lineS1, lineS2, lineS3, lineS4, lineS5;
    private DcMotorEx leftFront, leftRear, rightFront, rightRear;

    @Override
    public void runOpMode() {
        follower = Constants.createFollower(hardwareMap);
        
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");

        lineS1 = hardwareMap.get(DigitalChannel.class, "lineS1");
        lineS2 = hardwareMap.get(DigitalChannel.class, "lineS2");
        lineS3 = hardwareMap.get(DigitalChannel.class, "lineS3");
        lineS4 = hardwareMap.get(DigitalChannel.class, "lineS4");
        lineS5 = hardwareMap.get(DigitalChannel.class, "lineS5");

        waitForStart();

        while (opModeIsActive()) {
            follower.update();

            double drive = -gamepad1.left_stick_y;
            double strafe = -gamepad1.left_stick_x;
            double rotate = -gamepad1.right_stick_x;

            if (gamepad1.a) {
                boolean b2 = !lineS2.getState();
                boolean b3 = !lineS3.getState();
                boolean b4 = !lineS4.getState();

                if (b2 && !b4) {
                    strafe = 0.15;
                    telemetry.addLine("⬅️ ĐANG CHỈNH LỀ TRÁI...");
                } else if (b4 && !b2) {
                    strafe = -0.15;
                    telemetry.addLine("➡️ ĐANG CHỈNH LỀ PHẢI...");
                } else if (b3) {
                    strafe = 0;
                    telemetry.addLine("🎯 ĐÃ KHỚP TÂM VẠCH!");
                }
                drive *= 0.2; rotate *= 0.1;
            }

            double lf = drive + strafe + rotate;
            double lr = drive - strafe + rotate;
            double rf = drive - strafe - rotate;
            double rr = drive + strafe - rotate;

            leftFront.setPower(lf); leftRear.setPower(lr);
            rightFront.setPower(rf); rightRear.setPower(rr);

            Pose p = follower.getPose();
            telemetry.addData("TUYỆT ĐỐI", "X:%.3f Y:%.3f H:%.1f", p.getX(), p.getY(), Math.toDegrees(p.getHeading()));
            telemetry.update();
        }
    }
}
