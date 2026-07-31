package org.firstinspires.ftc.teamcode.pedroPathing;

import android.content.Context;
import android.hardware.usb.UsbDevice;
import android.hardware.usb.UsbDeviceConnection;
import android.hardware.usb.UsbManager;
import android.util.Log;

import com.hoho.android.usbserial.driver.UsbSerialDriver;
import com.hoho.android.usbserial.driver.UsbSerialPort;
import com.hoho.android.usbserial.driver.UsbSerialProber;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicLong;
import java.util.concurrent.atomic.AtomicReference;

/**
 * PiVisionSerial — Giao tiếp UART với Raspberry Pi qua CP2102 (USB Serial).
 *
 * Pi → Hub:
 *   PICK,<slotId>,<col>,<row>\n   (lặp cho mỗi pallet)
 *   END\n
 *   hoặc ERROR,<reason>[,p1,...]\n + END\n
 *
 * Hub → Pi:
 *   SCAN\n   /   STOP\n
 */
public class PiVisionSerial {

    // ─── Data classes ──────────────────────────────────────────────────────
    public static class PiDetection {
        private final int slotId, col, row;
        public PiDetection(int slotId, int col, int row) {
            this.slotId = slotId; this.col = col; this.row = row;
        }
        public int getSlotId() { return slotId; }
        public int getCol()    { return col; }
        public int getRow()    { return row; }
        @Override public String toString() {
            return "P" + slotId + "(c=" + col + ",r=" + row + ")";
        }
    }

    public static class ScanResult {
        public final boolean success;
        public final List<PiDetection> detections;
        public final String errorReason;
        public final List<Integer> uncertainPallets;
        public ScanResult(List<PiDetection> detections, String errorReason, List<Integer> uncertainPallets) {
            this.detections       = Collections.unmodifiableList(detections);
            this.errorReason      = errorReason;
            this.uncertainPallets = Collections.unmodifiableList(uncertainPallets);
            this.success          = (errorReason == null || errorReason.isEmpty());
        }
    }

    // ─── Config ────────────────────────────────────────────────────────────
    private static final int    BAUD_RATE = 115200;
    private static final String TAG       = "PiVisionSerial";
    private static final int    READ_TIMEOUT_MS  = 200;
    private static final int    WRITE_TIMEOUT_MS = 200;

    // ─── State ─────────────────────────────────────────────────────────────
    private UsbSerialPort port;
    private Thread        readThread;
    private final AtomicBoolean running = new AtomicBoolean(false);

    private final List<PiDetection> pendingPicks     = Collections.synchronizedList(new ArrayList<>());
    private volatile String         pendingReason    = "";
    private final List<Integer>     pendingUncertain = new ArrayList<>();
    private final StringBuilder     lineBuffer       = new StringBuilder();

    private final AtomicReference<ScanResult> latestResult          = new AtomicReference<>(null);
    private final AtomicLong                  latestResultTimestamp = new AtomicLong(0);

    // ─── Connect (tự tìm CP2102) ───────────────────────────────────────────
    /**
     * @param context  OpMode.hardwareMap.appContext
     * @return true nếu tìm thấy và mở được CP2102
     */
    public boolean connect(Context context) {
        UsbManager usbManager = (UsbManager) context.getSystemService(Context.USB_SERVICE);
        if (usbManager == null) {
            Log.e(TAG, "UsbManager null");
            return false;
        }

        // Log tất cả USB device để debug
        for (UsbDevice dev : usbManager.getDeviceList().values()) {
            Log.i(TAG, String.format("USB device: %s  VID=0x%04X PID=0x%04X",
                    dev.getDeviceName(), dev.getVendorId(), dev.getProductId()));
        }

        List<UsbSerialDriver> drivers = UsbSerialProber.getDefaultProber().findAllDrivers(usbManager);
        if (drivers.isEmpty()) {
            Log.e(TAG, "Không tìm thấy USB serial driver nào (CP2102 chưa cắm?)");
            return false;
        }

        for (UsbSerialDriver driver : drivers) {
            UsbDevice dev = driver.getDevice();
            Log.i(TAG, String.format("Found serial: %s VID=0x%04X PID=0x%04X",
                    dev.getDeviceName(), dev.getVendorId(), dev.getProductId()));

            UsbDeviceConnection conn = usbManager.openDevice(dev);
            if (conn == null) {
                Log.w(TAG, "Không có quyền mở " + dev.getDeviceName() + " — cần USB permission");
                continue;
            }

            UsbSerialPort p = driver.getPorts().get(0);
            try {
                p.open(conn);
                p.setParameters(BAUD_RATE, 8, UsbSerialPort.STOPBITS_1, UsbSerialPort.PARITY_NONE);
                port = p;
                Log.i(TAG, "Connected: " + dev.getDeviceName() + " @ " + BAUD_RATE + " baud");
                return true;
            } catch (IOException e) {
                Log.e(TAG, "Không mở được port: " + e.getMessage());
                try { p.close(); } catch (IOException ignored) {}
            }
        }

        Log.e(TAG, "Không kết nối được bất kỳ USB serial nào");
        return false;
    }

    // ─── Start / Stop ──────────────────────────────────────────────────────
    public void start() {
        if (!running.compareAndSet(false, true)) return;
        readThread = new Thread(this::readLoop, "PiUart-Reader");
        readThread.setDaemon(true);
        readThread.start();
        Log.i(TAG, "Reader thread started");
    }

    public void stop() {
        running.set(false);
        if (readThread != null) readThread.interrupt();
    }

    public void close() {
        stop();
        if (port != null) {
            try { port.close(); } catch (IOException ignored) {}
        }
    }

    // ─── Gửi lệnh ──────────────────────────────────────────────────────────
    public void sendCommand(String cmd) {
        if (port == null) {
            Log.w(TAG, "sendCommand: chưa kết nối");
            return;
        }
        try {
            byte[] data = (cmd.trim() + "\n").getBytes("UTF-8");
            port.write(data, WRITE_TIMEOUT_MS);
            Log.d(TAG, "Sent: " + cmd.trim());
        } catch (IOException e) {
            Log.e(TAG, "sendCommand error: " + e.getMessage());
        }
    }

    // ─── Getters ───────────────────────────────────────────────────────────
    public boolean isConnected()           { return port != null; }
    public ScanResult getLatestResult()    { return latestResult.get(); }
    public long getLatestResultAgeMs() {
        long ts = latestResultTimestamp.get();
        return (ts == 0) ? Long.MAX_VALUE : System.currentTimeMillis() - ts;
    }
    public void clearResult() {
        latestResult.set(null);
        latestResultTimestamp.set(0);
    }
    public List<PiDetection> getLatestDetections() {
        ScanResult r = latestResult.get();
        return (r == null || !r.success) ? Collections.emptyList() : r.detections;
    }

    // ─── Read loop ─────────────────────────────────────────────────────────
    private void readLoop() {
        byte[] buf = new byte[256];
        while (running.get() && !Thread.currentThread().isInterrupted()) {
            try {
                int n = port.read(buf, READ_TIMEOUT_MS);
                if (n > 0) {
                    String chunk = new String(buf, 0, n, "UTF-8");
                    lineBuffer.append(chunk);
                    // Xử lý từng dòng hoàn chỉnh
                    int idx;
                    while ((idx = lineBuffer.indexOf("\n")) >= 0) {
                        String line = lineBuffer.substring(0, idx).trim();
                        lineBuffer.delete(0, idx + 1);
                        if (!line.isEmpty()) {
                            Log.d(TAG, "Received: " + line);
                            processLine(line);
                        }
                    }
                }
            } catch (IOException e) {
                if (running.get()) {
                    Log.e(TAG, "readLoop IO error: " + e.getMessage());
                    try { Thread.sleep(100); } catch (InterruptedException ie) { break; }
                }
            }
        }
        Log.i(TAG, "readLoop ended");
    }

    // ─── Xử lý từng dòng ────────────────────────────────────────────────────
    private void processLine(String line) {
        if (line.startsWith("PICK,")) {
            String[] p = line.split(",");
            if (p.length >= 4) {
                try {
                    int slotId = Integer.parseInt(p[1].trim());
                    int col    = Integer.parseInt(p[2].trim());
                    int row    = Integer.parseInt(p[3].trim());
                    PiDetection det = new PiDetection(slotId, col, row);
                    pendingPicks.add(det);
                    Log.i(TAG, "Parsed: " + det);
                } catch (NumberFormatException e) {
                    Log.w(TAG, "Bad PICK line: " + line);
                }
            }
        } else if (line.startsWith("ERROR,")) {
            String[] p = line.split(",");
            pendingReason = (p.length >= 2) ? p[1].trim() : "UNKNOWN";
            pendingUncertain.clear();
            for (int i = 2; i < p.length; i++) {
                try { pendingUncertain.add(Integer.parseInt(p[i].trim())); }
                catch (NumberFormatException ignored) {}
            }
            Log.w(TAG, "Error from Pi: " + pendingReason + " " + pendingUncertain);
        } else if (line.equals("END")) {
            ScanResult result = new ScanResult(
                    new ArrayList<>(pendingPicks),
                    pendingReason,
                    new ArrayList<>(pendingUncertain));
            latestResult.set(result);
            latestResultTimestamp.set(System.currentTimeMillis());
            pendingPicks.clear();
            pendingReason = "";
            pendingUncertain.clear();
            Log.i(TAG, "ScanResult: success=" + result.success + " " + result.detections);
        } else {
            Log.d(TAG, "Unknown: " + line);
        }
    }
}
