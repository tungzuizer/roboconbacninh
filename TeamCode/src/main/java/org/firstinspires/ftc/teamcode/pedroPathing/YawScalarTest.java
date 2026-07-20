package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import java.util.Locale;

/**
 * YawScalarTest — Tuner tính yawScalar cho GoBILDA Pinpoint
 *
 * HƯỚNG DẪN SỬ DỤNG:
 * =====================
 * 1. Chạy OpMode này (TeleOp, không cần chạy robot).
 * 2. Đặt robot về 0° (mũi robot hướng về phía trước).
 * 3. Nhấn gamepad1.a để bắt đầu đếm.
 * 4. Xoay robot 10 vòng tròn đầy đủ THEO CHIỀU KIM ĐỒNG HỒ (360° x 10 = 3600°).
 *    - Nên xoay chậm và đều tay.
 *    - Dùng băng keo đánh dấu vị trí ban đầu trên sàn để đếm chính xác.
 * 5. Nhấn gamepad1.b để dừng đo.
 * 6. Đọc giá trị "Calculated yawScalar" trên Driver Station.
 * 7. Ghi giá trị đó vào Constants.java: .yawScalar(xxx)
 *    rồi bỏ comment dòng yawScalar trong localizerConstants.
 *
 * LƯU Ý: yawScalar = 3600 / (tổng góc Pinpoint đo được tính bằng độ)
 *         Nếu robot xoay CW 10 vòng mà Pinpoint báo ~3600° → yawScalar = 1.0 (đã chuẩn)
 *         Nếu Pinpoint báo 3550° → yawScalar = 3600/3550 ≈ 1.014
 */
@TeleOp(name = "Yaw Scalar Tuner", group = "Tuning")
public class YawScalarTest extends LinearOpMode {

    private GoBildaPinpointDriver pinpoint;

    // Trạng thái đo
    private boolean measuring = false;
    private double startHeadingDeg  = 0.0;  // reserved for future use
    private double lastHeadingDeg  = 0.0;
    private double accumulatedDeg  = 0.0;  // tổng góc tích lũy (UNnormalized)

    // Debounce nút
    private boolean aWasPressed = false;
    private boolean bWasPressed = false;

    // Thời gian đo
    private final ElapsedTime measureTimer = new ElapsedTime();
    private double elapsedSec = 0;

    // Kết quả tính toán
    private double calculatedYawScalar = Double.NaN;

    @Override
    public void runOpMode() {
        // ── Khởi tạo Pinpoint ──────────────────────────────────────────────────
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class,
                Constants.localizerConstants.hardwareMapName);

        // Reset vị trí & IMU về 0
        pinpoint.resetPosAndIMU();

        telemetry.addLine("=== Yaw Scalar Tuner ===");
        telemetry.addLine("Nhấn [A] để bắt đầu đo (sau khi đặt robot về 0°)");
        telemetry.addLine("Nhấn [B] để dừng và tính kết quả");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // Lấy heading ban đầu
        pinpoint.update();
        lastHeadingDeg = pinpoint.getHeading(AngleUnit.DEGREES);

        // ── Vòng lặp chính ─────────────────────────────────────────────────────
        while (opModeIsActive()) {
            pinpoint.update();

            double currentHeadingDeg = pinpoint.getHeading(AngleUnit.DEGREES);

            // ── Tích lũy góc (wrap-aware) ───────────────────────────────────────
            if (measuring) {
                double delta = currentHeadingDeg - lastHeadingDeg;
                // Xử lý wrap-around ±180° → ±360°
                if (delta > 180.0)  delta -= 360.0;
                if (delta < -180.0) delta += 360.0;
                accumulatedDeg += delta;
                elapsedSec = measureTimer.seconds();
            }
            lastHeadingDeg = currentHeadingDeg;

            // ── Xử lý nút [A] → Bắt đầu đo ────────────────────────────────────
            boolean aPressed = gamepad1.a;
            if (aPressed && !aWasPressed) {
                measuring        = true;
                accumulatedDeg   = 0.0;
                startHeadingDeg  = currentHeadingDeg;
                calculatedYawScalar = Double.NaN;
                measureTimer.reset();
                elapsedSec = 0;
            }
            aWasPressed = aPressed;

            // ── Xử lý nút [B] → Dừng đo ───────────────────────────────────────
            boolean bPressed = gamepad1.b;
            if (bPressed && !bWasPressed && measuring) {
                measuring = false;
                if (Math.abs(accumulatedDeg) > 10.0) {
                    // yawScalar = 3600 / (tổng độ đo được) — dự kiến robot xoay 10 vòng CW
                    calculatedYawScalar = 3600.0 / accumulatedDeg;
                }
            }
            bWasPressed = bPressed;

            // ── Telemetry ───────────────────────────────────────────────────────
            telemetry.addLine("=== YAW SCALAR TUNER ===");
            telemetry.addLine(measuring ? ">> ĐANG ĐO << (nhấn [B] khi xong 10 vòng)" :
                    "DỪNG. Nhấn [A] để đo lại.");

            telemetry.addLine("");
            telemetry.addData("Heading hiện tại (°)", "%.2f", currentHeadingDeg);
            telemetry.addData("Tổng góc tích lũy (°)", "%.2f", accumulatedDeg);
            telemetry.addData("Số vòng ước tính",
                    "%.2f / 10.0", accumulatedDeg / 360.0);
            telemetry.addData("Thời gian đo (s)", "%.1f", elapsedSec);

            telemetry.addLine("");
            telemetry.addData("yawScalar hiện tại (Pinpoint)", "%.4f",
                    (double) pinpoint.getYawScalar());

            telemetry.addLine("");
            if (!Double.isNaN(calculatedYawScalar)) {
                telemetry.addLine("┌── KẾT QUẢ ──────────────────────────┐");
                telemetry.addData("│ Calculated yawScalar", "%.6f", calculatedYawScalar);
                telemetry.addData("│ Measured accumulated (°)", "%.4f", accumulatedDeg);
                telemetry.addLine("└──────────────────────────────────────┘");
                telemetry.addLine("");
                telemetry.addLine("→ Copy giá trị vào Constants.java:");
                telemetry.addLine(
                        "   .yawScalar(" + String.format(Locale.US, "%.6f", calculatedYawScalar) + ")");
            } else {
                telemetry.addLine("[Kết quả] đang chờ đo...");
            }

            telemetry.addLine("");
            telemetry.addLine("[A] Bắt đầu  |  [B] Dừng & Tính");
            telemetry.update();
        }
    }
}
