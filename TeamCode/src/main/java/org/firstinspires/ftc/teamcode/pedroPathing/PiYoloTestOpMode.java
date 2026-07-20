package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import android.hardware.usb.UsbManager;
import java.util.List;

@TeleOp(name = "Pi YOLO Serial Test", group = "Test")
public class PiYoloTestOpMode extends LinearOpMode {
    private PiVisionSerial piVision;

    @Override
    public void runOpMode() {
        UsbManager usbManager = (UsbManager) hardwareMap.appContext.getSystemService(android.content.Context.USB_SERVICE);
        piVision = new PiVisionSerial(usbManager);
        
        if (!piVision.connect()) {
            telemetry.addLine("Không kết nối được USB Serial!");
            telemetry.update();
            waitForStart();
            return;
        }

        telemetry.addLine("Nhấn [A] để bắt đầu SCAN và [B] để STOP");
        telemetry.update();
        
        waitForStart();
        
        piVision.start(); // Bật thread đọc serial

        while (opModeIsActive()) {
            if (gamepad1.a) {
                piVision.sendCommand("SCAN");
            }
            if (gamepad1.b) {
                piVision.sendCommand("STOP");
            }

            List<PiVisionSerial.PiDetection> detections = piVision.getLatestDetections();
            
            telemetry.addData("Số mẫu tìm thấy", detections.size());
            for (PiVisionSerial.PiDetection det : detections) {
                telemetry.addData("Mẫu", "Slot=%d, Color=%d, Col=%d, Row=%d", 
                    det.getSlotId(), det.getColor(), det.getCol(), det.getRow());
            }
            telemetry.update();
        }
        piVision.stop();
        piVision.close();
    }
}
