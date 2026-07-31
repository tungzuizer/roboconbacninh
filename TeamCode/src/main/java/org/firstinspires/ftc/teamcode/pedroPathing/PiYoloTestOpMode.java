package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.List;

/**
 * Test YOLO qua UART (2x CP2102):
 *   [A] = Gửi SCAN → Pi chụp ảnh, nhận dạng → trả PICK kết quả
 *         Chờ tối đa 10 giây, sau đó tự gửi STOP
 *   [Y] = Xoá kết quả
 *
 * Telemetry hiện vị trí từng hộp (box01..04) trên kệ 2x2.
 */
@TeleOp(name = "YOLO Test", group = "Test")
public class PiYoloTestOpMode extends OpMode {

    private PiVisionSerial uart;
    private boolean connected = false;

    // Trạng thái SCAN
    private boolean scanning = false;
    private long scanStartTime = 0;
    private static final long SCAN_TIMEOUT_MS = 10000;  // 10 giây

    // Chống nhấn lặp
    private boolean prevA = false;
    private boolean prevY = false;

    // Lưu kết quả hiển thị
    private String resultDisplay = "(chua quet)";

    @Override
    public void init() {
        uart = new PiVisionSerial();
        connected = uart.connect(hardwareMap.appContext);

        if (connected) {
            uart.start();
            telemetry.addLine("CP2102 ket noi thanh cong!");
        } else {
            telemetry.addLine("Khong thay CP2102 - kiem tra cam USB");
        }
        telemetry.update();
    }

    @Override
    public void loop() {
        boolean a = gamepad1.a;
        boolean y = gamepad1.y;

        // [A] = SCAN
        if (a && !prevA && connected && !scanning) {
            uart.clearResult();
            uart.sendCommand("SCAN");
            scanning = true;
            scanStartTime = System.currentTimeMillis();
            resultDisplay = "Dang quet...";
        }

        // [Y] = Xoa ket qua
        if (y && !prevY) {
            uart.clearResult();
            scanning = false;
            resultDisplay = "(da xoa)";
        }

        prevA = a;
        prevY = y;

        // Kiem tra ket qua hoac timeout
        if (scanning) {
            PiVisionSerial.ScanResult result = uart.getLatestResult();
            long elapsed = System.currentTimeMillis() - scanStartTime;

            if (result != null) {
                scanning = false;
                uart.sendCommand("STOP");
                resultDisplay = formatResult(result);

            } else if (elapsed >= SCAN_TIMEOUT_MS) {
                scanning = false;
                uart.sendCommand("STOP");
                resultDisplay = "Timeout 10s - chua nhan duoc ket qua";
            }
        }

        // Hien thi
        telemetry.addData("Trang thai", connected ? "CONNECTED" : "NOT CONNECTED");
        telemetry.addData("Scanning", scanning ? "DANG QUET..." : "San sang");

        if (scanning) {
            long elapsed = System.currentTimeMillis() - scanStartTime;
            telemetry.addData("Thoi gian", String.format("%.1fs / 10.0s", elapsed / 1000.0));
        }

        telemetry.addLine("");
        telemetry.addLine("[A] = SCAN   [Y] = Xoa");
        telemetry.addLine("--- Ket qua ---");
        telemetry.addLine(resultDisplay);

        telemetry.update();
    }

    @Override
    public void stop() {
        if (uart != null) {
            uart.sendCommand("STOP");
            uart.close();
        }
    }

    private String formatResult(PiVisionSerial.ScanResult result) {
        if (!result.success) {
            StringBuilder sb = new StringBuilder();
            sb.append("LOI: ").append(result.errorReason);
            if (!result.uncertainPallets.isEmpty()) {
                sb.append("\n   Pallet chua chac: ").append(result.uncertainPallets);
            }
            return sb.toString();
        }

        List<PiVisionSerial.PiDetection> dets = result.detections;
        if (dets.isEmpty()) {
            return "Khong phat hien hop nao";
        }

        StringBuilder sb = new StringBuilder();
        sb.append("Tim thay ").append(dets.size()).append(" hop:\n");

        String[][] grid = new String[2][2];
        for (PiVisionSerial.PiDetection d : dets) {
            int r = d.getRow() - 1;
            int c = d.getCol() - 1;
            if (r >= 0 && r < 2 && c >= 0 && c < 2) {
                grid[r][c] = "P" + d.getSlotId();
            }
            sb.append(String.format("  box%02d -> Col %d, Row %d\n",
                    d.getSlotId(), d.getCol(), d.getRow()));
        }

        sb.append("\n  +-----+-----+\n");
        sb.append(String.format("  | %-3s | %-3s |  Row 1\n",
                grid[0][0] != null ? grid[0][0] : "---",
                grid[0][1] != null ? grid[0][1] : "---"));
        sb.append("  +-----+-----+\n");
        sb.append(String.format("  | %-3s | %-3s |  Row 2\n",
                grid[1][0] != null ? grid[1][0] : "---",
                grid[1][1] != null ? grid[1][1] : "---"));
        sb.append("  +-----+-----+\n");
        sb.append("   C1     C2");

        return sb.toString();
    }
}
