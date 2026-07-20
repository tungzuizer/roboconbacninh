package org.firstinspires.ftc.teamcode.pedroPathing;

import android.hardware.usb.UsbDevice;
import android.hardware.usb.UsbDeviceConnection;
import android.hardware.usb.UsbManager;

import com.hoho.android.usbserial.driver.UsbSerialDriver;
import com.hoho.android.usbserial.driver.UsbSerialPort;
import com.hoho.android.usbserial.driver.UsbSerialProber;
import com.qualcomm.robotcore.util.RobotLog;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicLong;

/**
 * Đọc tọa độ mẫu vật từ Raspberry Pi qua USB Serial (CP2102 hoặc tương đương).
 * Giao thức: PICK,<slotId>,<cột>,<hàng>\n ... END\n
 */
public class PiVisionSerial {

    public static class PiDetection {
        private final int slotId, color, col, row;

        public PiDetection(int slotId, int col, int row) {
            this.slotId = slotId;
            this.col = col;
            this.row = row;
            // Ánh xạ màu dựa trên slotId: 1=Yellow, 2=Red, 3=Blue
            if (slotId == 1) this.color = 1;
            else if (slotId == 2) this.color = 2;
            else if (slotId == 3) this.color = 3;
            else this.color = 1; 
        }

        public int getSlotId() { return slotId; }
        public int getColor() { return color; }
        public int getCol() { return col; }
        public int getRow() { return row; }

        public String getClassName() { return String.format(java.util.Locale.US, "box%02d", slotId); }
        public double getConfidence() { return 1.0; }
        public double getTargetArea() { return 1.0; }
        public double getTargetXDegrees() { return col; }
        public double getTargetYDegrees() { return row; }
    }

    private static final int BAUD_RATE = 115200;
    private static final int READ_TIMEOUT_MS = 50;

    private final UsbManager usbManager;
    private UsbSerialPort port;
    private Thread readThread;
    private final AtomicBoolean running = new AtomicBoolean(false);

    private final List<PiDetection> pendingFrame = Collections.synchronizedList(new ArrayList<>());
    private volatile List<PiDetection> latestFrame = new ArrayList<>();
    private final AtomicLong latestFrameTimestamp = new AtomicLong(0);

    private volatile com.pedropathing.geometry.Pose latestAlignmentPose = null;
    private final AtomicLong latestAlignmentTimestamp = new AtomicLong(0);

    private final StringBuilder lineBuffer = new StringBuilder();

    public PiVisionSerial(UsbManager usbManager) {
        this.usbManager = usbManager;
    }

    public boolean connect() {
        List<UsbSerialDriver> drivers = UsbSerialProber.getDefaultProber().findAllDrivers(usbManager);
        if (drivers.isEmpty()) return false;
        UsbSerialDriver driver = drivers.get(0);
        UsbDeviceConnection connection = usbManager.openDevice(driver.getDevice());
        if (connection == null) return false;
        port = driver.getPorts().get(0);
        try {
            port.open(connection);
            port.setParameters(BAUD_RATE, 8, UsbSerialPort.STOPBITS_1, UsbSerialPort.PARITY_NONE);
        } catch (IOException e) { return false; }
        return true;
    }

    public void start() {
        if (port == null || running.get()) return;
        running.set(true);
        readThread = new Thread(this::readLoop, "PiVisionSerial-Read");
        readThread.setDaemon(true);
        readThread.start();
    }

    public void stop() {
        running.set(false);
        if (readThread != null) {
            try { readThread.join(200); } catch (InterruptedException ignored) {}
            readThread = null;
        }
    }

    public void close() {
        stop();
        if (port != null) { try { port.close(); } catch (IOException ignored) {} port = null; }
    }

    public List<PiDetection> getLatestDetections() {
        return latestFrame;
    }

    public long getLatestFrameAgeMs() {
        long ts = latestFrameTimestamp.get();
        return ts == 0 ? Long.MAX_VALUE : System.currentTimeMillis() - ts;
    }

    public int getSlotColor(int slotId) {
        List<PiDetection> currentDetections = getLatestDetections();
        if (currentDetections == null) return -1;
        synchronized (currentDetections) {
            for (PiDetection det : currentDetections) {
                if (det.getSlotId() == slotId) return det.getColor();
            }
        }
        return -1;
    }

    public com.pedropathing.geometry.Pose getLatestAlignmentPose() {
        return latestAlignmentPose;
    }

    public long getLatestAlignmentAgeMs() {
        long ts = latestAlignmentTimestamp.get();
        return ts == 0 ? Long.MAX_VALUE : System.currentTimeMillis() - ts;
    }

    public void clearAlignment() {
        latestAlignmentPose = null;
        latestAlignmentTimestamp.set(0);
    }

    public void sendCommand(String command) {
        if (port == null) return;
        try { port.write((command + "\n").getBytes(), 100); } catch (IOException e) { RobotLog.ee("PiVisionSerial", "Lỗi gửi lệnh: " + e.getMessage()); }
    }

    private void readLoop() {
        byte[] buf = new byte[256];
        while (running.get()) {
            try {
                int len = port.read(buf, READ_TIMEOUT_MS);
                if (len > 0) {
                    for (int i = 0; i < len; i++) {
                        char c = (char) buf[i];
                        if (c == '\n') {
                            processLine(lineBuffer.toString().trim());
                            lineBuffer.setLength(0);
                        } else if (c != '\r') { lineBuffer.append(c); }
                    }
                }
            } catch (IOException e) { running.set(false); }
        }
    }

    private volatile double lineOffX = 0, lineOffY = 0, lineAngle = 0;
    private final AtomicLong lastLineTimestamp = new AtomicLong(0);

    public double getLineOffX() { return lineOffX; }
    public double getLineOffY() { return lineOffY; }
    public double getLineAngle() { return lineAngle; }
    public long getLineDataAgeMs() { return System.currentTimeMillis() - lastLineTimestamp.get(); }

    private void processLine(String line) {
        if (line.isEmpty()) return;
        if (line.equals("END")) {
            latestFrame = new ArrayList<>(pendingFrame);
            latestFrameTimestamp.set(System.currentTimeMillis());
            pendingFrame.clear();
            return;
        }
        String[] parts = line.split(",");
        try {
            if (parts.length == 4) {
                if ("PICK".equals(parts[0].trim())) {
                    pendingFrame.add(new PiDetection(
                        Integer.parseInt(parts[1].trim()), 
                        Integer.parseInt(parts[2].trim()), 
                        Integer.parseInt(parts[3].trim())
                    ));
                } else if ("ALIGN".equals(parts[0].trim())) {
                    latestAlignmentPose = new com.pedropathing.geometry.Pose(
                        Double.parseDouble(parts[1].trim()),
                        Double.parseDouble(parts[2].trim()),
                        Math.toRadians(Double.parseDouble(parts[3].trim()))
                    );
                    latestAlignmentTimestamp.set(System.currentTimeMillis());
                } else if ("LINE".equals(parts[0].trim())) {
                    lineOffX = Double.parseDouble(parts[1].trim());
                    lineOffY = Double.parseDouble(parts[2].trim());
                    lineAngle = Double.parseDouble(parts[3].trim());
                    lastLineTimestamp.set(System.currentTimeMillis());
                }
            }
        } catch (NumberFormatException e) {
            RobotLog.ee("PiVisionSerial", "Lỗi dữ liệu Serial (Nhiễu): " + line);
        }
    }
}
