package org.firstinspires.ftc.teamcode.pedroPathing;

import android.util.Log;
import com.hoho.android.usbserial.driver.UsbSerialDriver;
import com.hoho.android.usbserial.driver.UsbSerialProber;
import com.hoho.android.usbserial.driver.UsbSerialPort;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicLong;
import java.util.concurrent.atomic.AtomicReference;

public class PiVisionSerial {

    private static final String TAG = "PiVisionSerial";
    private static final int BAUD_RATE = 115200;
    private static final int READ_TIMEOUT_MS = 100;
    private static final int WRITE_TIMEOUT_MS = 200;

    private UsbSerialPort port;
    private final AtomicBoolean running = new AtomicBoolean(false);
    private Thread readThread;
    private final StringBuilder lineBuffer = new StringBuilder();

    private final List<PiDetection> pendingPicks = new ArrayList<>();
    private String pendingReason = "";
    private final List<Integer> pendingUncertain = new ArrayList<>();

    private final AtomicReference<ScanResult> latestResult = new AtomicReference<>(null);
    private final AtomicLong latestResultTimestamp = new AtomicLong(0);

    public static class PiDetection {
        public final int slotId;
        public final int col;
        public final int row;
        public final int boxType;

        public PiDetection(int slotId, int col, int row, int boxType) {
            this.slotId = slotId;
            this.col = col;
            this.row = row;
            this.boxType = boxType;
        }

        public PiDetection(int slotId, int col, int row) {
            this(slotId, col, row, 1);
        }

        public int getSlotId() { return slotId; }
        public int getCol() { return col; }
        public int getRow() { return row; }
        public int getBoxType() { return boxType; }

        @Override
        public String toString() {
            return String.format("PiDetection{slot=%d, col=%d, row=%d, type=%d}", slotId, col, row, boxType);
        }
    }

    public static class ScanResult {
        public final boolean success;
        public final List<PiDetection> detections;
        public final String errorReason;
        public final List<Integer> uncertainSlots;
        public final List<Integer> uncertainPallets;

        public ScanResult(List<PiDetection> detections, String errorReason, List<Integer> uncertainSlots) {
            this.detections = Collections.unmodifiableList(new ArrayList<>(detections));
            this.errorReason = errorReason;
            this.uncertainSlots = Collections.unmodifiableList(new ArrayList<>(uncertainSlots));
            this.uncertainPallets = this.uncertainSlots;
            this.success = errorReason.isEmpty() && !this.detections.isEmpty();
        }
    }

    public boolean connect(android.content.Context context) {
        return isConnected();
    }

    public boolean init(HardwareMap hardwareMap) {
        android.hardware.usb.UsbManager manager =
                (android.hardware.usb.UsbManager) hardwareMap.appContext.getSystemService(android.content.Context.USB_SERVICE);
        if (manager == null) {
            Log.e(TAG, "UsbManager null");
            return false;
        }

        List<UsbSerialDriver> drivers = UsbSerialProber.getDefaultProber().findAllDrivers(manager);
        if (drivers.isEmpty()) {
            Log.w(TAG, "Không tìm thấy USB Serial Driver");
            return false;
        }

        for (UsbSerialDriver driver : drivers) {
            UsbSerialPort p = driver.getPorts().get(0);
            try {
                android.hardware.usb.UsbDeviceConnection conn = manager.openDevice(driver.getDevice());
                if (conn == null) continue;
                p.open(conn);
                p.setParameters(BAUD_RATE, 8, UsbSerialPort.STOPBITS_1, UsbSerialPort.PARITY_NONE);
                p.setDTR(true);
                p.setRTS(true);
                this.port = p;
                Log.i(TAG, "Đã mở USB serial port thành công!");
                start();
                return true;
            } catch (IOException e) {
                Log.e(TAG, "Không mở được port: " + e.getMessage());
                try { p.close(); } catch (IOException ignored) {}
            }
        }
        return false;
    }

    public void start() {
        if (!running.compareAndSet(false, true)) return;
        readThread = new Thread(this::readLoop, "PiUart-Reader");
        readThread.setDaemon(true);
        readThread.start();
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

    public void sendCommand(String cmd) {
        if (port == null) return;
        try {
            byte[] data = (cmd.trim() + "\n").getBytes("UTF-8");
            port.write(data, WRITE_TIMEOUT_MS);
        } catch (IOException e) {
            Log.e(TAG, "sendCommand error: " + e.getMessage());
        }
    }

    public boolean isConnected() { return port != null; }
    public ScanResult getLatestResult() { return latestResult.get(); }

    public String r1c1 = "";
    public String r1c2 = "";
    public String r2c1 = "";
    public String r2c2 = "";
    private boolean scanDone = false;

    public boolean isScanDone() {
        return scanDone || latestResult.get() != null;
    }

    public boolean hasAll4ShelfLabels() {
        return isValidFactoryName(r1c1)
                && isValidFactoryName(r1c2)
                && isValidFactoryName(r2c1)
                && isValidFactoryName(r2c2);
    }

    public boolean isValidFactoryName(String name) {
        if (name == null) return false;
        String n = name.trim().toLowerCase();
        return n.contains("foxconn")
                || n.contains("amkor")
                || n.contains("hana")
                || n.contains("samsung");
    }

    public void clearResult() {
        latestResult.set(null);
        latestResultTimestamp.set(0);
        r1c1 = "";
        r1c2 = "";
        r2c1 = "";
        r2c2 = "";
        scanDone = false;
    }

    private void readLoop() {
        byte[] buf = new byte[256];
        while (running.get() && !Thread.currentThread().isInterrupted()) {
            try {
                int n = port.read(buf, READ_TIMEOUT_MS);
                if (n > 0) {
                    String chunk = new String(buf, 0, n, "UTF-8");
                    lineBuffer.append(chunk);
                    int idx;
                    while ((idx = lineBuffer.indexOf("\n")) >= 0) {
                        String line = lineBuffer.substring(0, idx).trim();
                        lineBuffer.delete(0, idx + 1);
                        if (!line.isEmpty()) processLine(line);
                    }
                }
            } catch (IOException e) {
                if (running.get()) {
                    try { Thread.sleep(100); } catch (InterruptedException ie) { break; }
                }
            }
        }
    }

    private void processLine(String line) {
        String trimmed = line.trim();
        if (trimmed.startsWith("R1C1:")) {
            r1c1 = trimmed.substring(5).trim();
        } else if (trimmed.startsWith("R1C2:")) {
            r1c2 = trimmed.substring(5).trim();
        } else if (trimmed.startsWith("R2C1:")) {
            r2c1 = trimmed.substring(5).trim();
        } else if (trimmed.startsWith("R2C2:")) {
            r2c2 = trimmed.substring(5).trim();
        } else if (trimmed.equalsIgnoreCase("END")) {
            scanDone = true;
            ScanResult result = new ScanResult(new ArrayList<>(pendingPicks), pendingReason, new ArrayList<>(pendingUncertain));
            latestResult.set(result);
            latestResultTimestamp.set(System.currentTimeMillis());
            pendingPicks.clear();
            pendingReason = "";
            pendingUncertain.clear();
        } else if (trimmed.startsWith("PICK,")) {
            String[] p = trimmed.split(",");
            if (p.length >= 4) {
                try {
                    int slotId = Integer.parseInt(p[1].trim());
                    int col    = Integer.parseInt(p[2].trim());
                    int row    = Integer.parseInt(p[3].trim());
                    int type   = (p.length >= 5) ? Integer.parseInt(p[4].trim()) : 1;
                    pendingPicks.add(new PiDetection(slotId, col, row, type));
                } catch (NumberFormatException ignored) {}
            }
        }
    }
}
