package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;

/**
 * Driver cho module PCA9685 (16-channel PWM/Servo Driver) dùng với FTC SDK.
 *
 * Cách dùng cơ bản trong OpMode:
 *   PCA9685 pca = hardwareMap.get(PCA9685.class, "pca9685");
 *   pca.setServoAngle(0, 90);   // kênh 0,     góc 90 độ
 *
 * Lưu ý: PCA9685 hoạt động ở tần số PWM ~50Hz cho servo tiêu chuẩn.
 */
@I2cDeviceType
@DeviceProperties(name = "PCA9685 Servo Driver", xmlTag = "PCA9685", description = "16-Channel PWM/Servo Driver")
public class PCA9685 extends I2cDeviceSynchDevice<I2cDeviceSynchSimple> {

    // ==== Địa chỉ thanh ghi PCA9685 ====
    private static final int MODE1 = 0x00;
    private static final int MODE2 = 0x01;
    private static final int PRESCALE = 0xFE;
    private static final int LED0_ON_L = 0x06; // kênh 0 bắt đầu tại đây, mỗi kênh chiếm 4 byte

    private static final int DEFAULT_ADDRESS = 0x40; // địa chỉ I2C mặc định của PCA9685

    // Thông số xung PWM cho servo tiêu chuẩn (điều chỉnh nếu servo của bạn khác)
    private static final int SERVO_MIN_PULSE_US = 500;   // 0 độ  (~500us)
    private static final int SERVO_MAX_PULSE_US = 2500;  // 180 độ (~2500us)
    private static final int PWM_FREQ_HZ = 50;            // tần số chuẩn cho servo

    public PCA9685(I2cDeviceSynchSimple deviceClient, boolean deviceClientIsOwned) {
        super(deviceClient, deviceClientIsOwned);
        this.deviceClient.setI2cAddress(I2cAddr.create7bit(DEFAULT_ADDRESS));
        super.registerArmingStateCallback(false);
        // Lưu ý: I2cDeviceSynchSimple không có phương thức engage(), nên bỏ qua dòng này.
    }

    @Override
    protected boolean doInitialize() {
        // Reset MODE1
        writeReg(MODE1, 0x00);
        setPWMFreq(PWM_FREQ_HZ);
        return true;
    }

    @Override
    public HardwareDevice.Manufacturer getManufacturer() {
        return HardwareDevice.Manufacturer.Other;
    }

    @Override
    public String getDeviceName() {
        return "PCA9685 Servo Driver";
    }

    // ==== Hàm nội bộ ====

    private void writeReg(int reg, int value) {
        deviceClient.write8(reg, value);
    }

    private int readReg(int reg) {
        return deviceClient.read8(reg);
    }

    /**
     * Thiết lập tần số PWM (Hz). Servo tiêu chuẩn dùng 50Hz.
     */
    private void setPWMFreq(int freqHz) {
        double prescaleval = 25000000.0; // 25MHz oscillator nội bộ
        prescaleval /= 4096.0;           // 12-bit
        prescaleval /= freqHz;
        prescaleval -= 1.0;
        int prescale = (int) Math.floor(prescaleval + 0.5);

        int oldMode = readReg(MODE1);
        int newMode = (oldMode & 0x7F) | 0x10; // sleep mode để đổi prescale
        writeReg(MODE1, newMode);
        writeReg(PRESCALE, prescale);
        writeReg(MODE1, oldMode);
        try {
            Thread.sleep(5);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
        writeReg(MODE1, oldMode | 0x80); // bật lại, auto-increment
    }

    /**
     * Xuất xung PWM thô cho 1 kênh.
     * @param channel 0-15
     * @param on thời điểm bắt đầu bật (0-4095)
     * @param off thời điểm tắt (0-4095)
     */
    public void setPWM(int channel, int on, int off) {
        if (channel < 0 || channel > 15) {
            throw new IllegalArgumentException("Kênh phải trong khoảng 0-15");
        }
        int reg = LED0_ON_L + 4 * channel;
        writeReg(reg, on & 0xFF);
        writeReg(reg + 1, (on >> 8) & 0xFF);
        writeReg(reg + 2, off & 0xFF);
        writeReg(reg + 3, (off >> 8) & 0xFF);
    }

    /**
     * Điều khiển servo theo góc (0-180 độ) trên 1 kênh cụ thể.
     * @param channel kênh servo (0-15)
     * @param angleDegrees góc mong muốn, 0-180
     */
    public void setServoAngle(int channel, double angleDegrees) {
        angleDegrees = Math.max(0, Math.min(180, angleDegrees));
        double pulseUs = SERVO_MIN_PULSE_US +
                (angleDegrees / 180.0) * (SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US);
        setServoPulseUs(channel, pulseUs);
    }

    /**
     * Điều khiển servo bằng độ rộng xung trực tiếp (micro giây).
     * Hữu ích nếu servo của bạn cần calibrate riêng ngoài khoảng 500-2500us.
     */
    public void setServoPulseUs(int channel, double pulseUs) {
        double periodUs = 1000000.0 / PWM_FREQ_HZ; // ví dụ 20000us cho 50Hz
        int offValue = (int) Math.round((pulseUs / periodUs) * 4096.0);
        setPWM(channel, 0, offValue);
    }
}