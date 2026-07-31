package org.firstinspires.ftc.teamcode.pedroPathing;

import android.content.Context;
import android.hardware.usb.UsbDevice;
import android.hardware.usb.UsbDeviceConnection;
import android.hardware.usb.UsbManager;
import android.util.Log;

import com.hoho.android.usbserial.driver.UsbSerialDriver;
import com.hoho.android.usbserial.driver.UsbSerialPort;
import com.hoho.android.usbserial.driver.UsbSerialProber;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.io.IOException;
import java.util.List;

/**
 * Test đơn giản: 2 con CP2102 nối TX↔RX giữa Control Hub và Pi.
 *
 * Control Hub cắm CP2102 #1 qua USB.
 * Pi cắm CP2102 #2 qua USB → Pi thấy /dev/ttyUSB0.
 *
 * Gamepad:
 *   [A] = Gửi "HELLO\n" đến Pi
 *   [B] = Gửi "SCAN\n" đến Pi
 *   [Y] = Xoá dòng nhận được
 */
@TeleOp(name = "UART Simple Test", group = "Test")
public class UartTestSimple extends OpMode {

    private static final String TAG      = "UartTest";
    private static final int    BAUD     = 115200;

    private UsbSerialPort port = null;
    private boolean       connected = false;

    // Bộ đệm đọc
    private final StringBuilder lineBuffer = new StringBuilder();
    private final StringBuilder log        = new StringBuilder();  // hiển thị lên telemetry

    // Thread đọc nền
    private Thread readThread;
    private volatile boolean running = false;

    // Chống nhấn lặp
    private boolean prevA = false, prevB = false, prevY = false;

    // ──────────────────────────────────────────────────────────────────────
    @Override
    public void init() {
        telemetry.addLine("Đang tìm CP2102...");
        telemetry.update();

        connected = openSerial(hardwareMap.appContext);

        if (connected) {
            startReader();
            telemetry.addLine("✅ CP2102 kết nối thành công!");
        } else {
            telemetry.addLine("❌ Không thấy CP2102 — kiểm tra cắm USB");
        }
        telemetry.update();
    }

    @Override
    public void loop() {
        boolean a = gamepad1.a;
        boolean b = gamepad1.b;
        boolean y = gamepad1.y;

        if (connected) {
            if (a && !prevA) send("HELLO");
            if (b && !prevB) send("SCAN");
        }
        if (y && !prevY) {
            synchronized (log) { log.setLength(0); }
        }

        prevA = a; prevB = b; prevY = y;

        // Hiển thị
        telemetry.addData("Trạng thái", connected ? "✅ CONNECTED" : "❌ NOT CONNECTED");
        telemetry.addLine("[A]=HELLO  [B]=SCAN  [Y]=Xoá");
        telemetry.addLine("─── Nhận từ Pi ───");
        synchronized (log) {
            telemetry.addLine(log.length() == 0 ? "(chưa nhận gì)" : log.toString());
        }
        telemetry.update();
    }

    @Override
    public void stop() {
        running = false;
        if (readThread != null) readThread.interrupt();
        if (port != null) {
            try { port.close(); } catch (IOException ignored) {}
        }
    }

    // ──────────────────────────────────────────────────────────────────────
    /** Mở CP2102 đầu tiên tìm thấy */
    private boolean openSerial(Context ctx) {
        UsbManager mgr = (UsbManager) ctx.getSystemService(Context.USB_SERVICE);
        if (mgr == null) return false;

        // Log tất cả USB device để debug
        for (UsbDevice d : mgr.getDeviceList().values()) {
            Log.i(TAG, String.format("USB: %s  VID=0x%04X PID=0x%04X",
                    d.getDeviceName(), d.getVendorId(), d.getProductId()));
        }

        List<UsbSerialDriver> drivers = UsbSerialProber.getDefaultProber().findAllDrivers(mgr);
        if (drivers.isEmpty()) {
            Log.e(TAG, "Không tìm thấy USB serial driver");
            return false;
        }

        UsbSerialDriver driver = drivers.get(0);      // lấy con đầu tiên
        UsbDeviceConnection conn = mgr.openDevice(driver.getDevice());
        if (conn == null) {
            Log.e(TAG, "Không mở được USB connection (cần USB permission?)");
            return false;
        }

        UsbSerialPort p = driver.getPorts().get(0);
        try {
            p.open(conn);
            p.setParameters(BAUD, 8, UsbSerialPort.STOPBITS_1, UsbSerialPort.PARITY_NONE);
            port = p;
            Log.i(TAG, "Opened " + driver.getDevice().getDeviceName() + " @ " + BAUD);
            return true;
        } catch (IOException e) {
            Log.e(TAG, "open error: " + e.getMessage());
            try { p.close(); } catch (IOException ignored) {}
            return false;
        }
    }

    /** Gửi 1 dòng lệnh */
    private void send(String cmd) {
        if (port == null) return;
        try {
            port.write((cmd + "\n").getBytes("UTF-8"), 200);
            Log.d(TAG, "Sent: " + cmd);
        } catch (IOException e) {
            Log.e(TAG, "send error: " + e.getMessage());
        }
    }

    /** Thread đọc nền */
    private void startReader() {
        running = true;
        readThread = new Thread(() -> {
            byte[] buf = new byte[256];
            while (running && !Thread.currentThread().isInterrupted()) {
                try {
                    int n = port.read(buf, 200);
                    if (n > 0) {
                        lineBuffer.append(new String(buf, 0, n, "UTF-8"));
                        int idx;
                        while ((idx = lineBuffer.indexOf("\n")) >= 0) {
                            String line = lineBuffer.substring(0, idx).trim();
                            lineBuffer.delete(0, idx + 1);
                            if (!line.isEmpty()) {
                                Log.d(TAG, "Recv: " + line);
                                synchronized (log) {
                                    // Giữ tối đa 5 dòng gần nhất
                                    String[] lines = log.toString().split("\n", -1);
                                    if (lines.length >= 5) {
                                        log.setLength(0);
                                        for (int i = 1; i < lines.length; i++) {
                                            log.append(lines[i]).append("\n");
                                        }
                                    }
                                    log.append(line).append("\n");
                                }
                            }
                        }
                    }
                } catch (IOException e) {
                    Log.e(TAG, "read error: " + e.getMessage());
                    break;
                }
            }
        }, "UartReader");
        readThread.setDaemon(true);
        readThread.start();
    }
}
