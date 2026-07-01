package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Tune từng khớp servo bằng FTControl Panels.
 *
 * Cách dùng:
 * 1. Chạy TeleOp "Servo Panels Tuner".
 * 2. Mở Panels → chỉnh các biến static bên dưới.
 * 3. Bật APPLY = true để servo chạy theo giá trị trên Panels.
 * 4. Tune từng khớp bằng SELECTED_SERVO hoặc tune theo từng ô bằng SELECTED_SLOT.
 */
@Configurable
@TeleOp(name = "Servo Panels Tuner", group = "Tuning")
public class ServoPanelsTuner extends LinearOpMode {

    // ╔══════════════════════════════════════════════════════════╗
    // ║  ZONE 1: TUNE TRÊN PANELS                              ║
    // ╚══════════════════════════════════════════════════════════╝

    // Bật/tắt apply. Để false khi mới mở để tránh servo nhảy bất ngờ.
    public static boolean APPLY = false;

    // Chế độ tune:
    // 0 = tune từng servo riêng bằng SELECTED_SERVO + SERVO_POS
    // 1 = tune theo 1 ô: set cả 5 servo bằng TOP_LEFT/TOP_RIGHT/BOT_LEFT/BOT_RIGHT
    // 2 = reset tất cả servo về HOME
    public static int MODE = 0;

    // Dùng khi MODE = 0
    // 1=s1, 2=s2, 3=s3, 4=s4, 5=s5
    public static int SELECTED_SERVO = 1;
    public static double SERVO_POS = 0.50;

    // Dùng khi MODE = 1
    // 0=TOP_LEFT, 1=TOP_RIGHT, 2=BOT_LEFT, 3=BOT_RIGHT
    public static int SELECTED_SLOT = 0;

    // Tên servo trong Robot Config
    public static String NAME_S1 = "s1";
    public static String NAME_S2 = "s2";
    public static String NAME_S3 = "s3";
    public static String NAME_S4 = "s4";
    public static String NAME_S5 = "s5";

    // HOME an toàn
    public static double S1_HOME = 0.50;
    public static double S2_HOME = 0.50;
    public static double S3_HOME = 0.50;
    public static double S4_HOME = 0.50;
    public static double S5_HOME = 0.50;

    // Ô TRÊN - TRÁI
    public static double TL_S1 = 0.10;
    public static double TL_S2 = 0.20;
    public static double TL_S3 = 0.30;
    public static double TL_S4 = 0.40;
    public static double TL_S5 = 0.50;

    // Ô TRÊN - PHẢI
    public static double TR_S1 = 0.20;
    public static double TR_S2 = 0.30;
    public static double TR_S3 = 0.40;
    public static double TR_S4 = 0.50;
    public static double TR_S5 = 0.60;

    // Ô DƯỚI - TRÁI
    public static double BL_S1 = 0.70;
    public static double BL_S2 = 0.80;
    public static double BL_S3 = 0.90;
    public static double BL_S4 = 1.00;
    public static double BL_S5 = 0.10;

    // Ô DƯỚI - PHẢI
    public static double BR_S1 = 0.50;
    public static double BR_S2 = 0.60;
    public static double BR_S3 = 0.70;
    public static double BR_S4 = 0.80;
    public static double BR_S5 = 0.90;

    // ╔══════════════════════════════════════════════════════════╗
    // ║  CODE CHÍNH                                             ║
    // ╚══════════════════════════════════════════════════════════╝

    private Servo s1, s2, s3, s4, s5;

    @Override
    public void runOpMode() {
        PanelsConfigurables.INSTANCE.refreshClass(this);

        s1 = hardwareMap.get(Servo.class, NAME_S1);
        s2 = hardwareMap.get(Servo.class, NAME_S2);
        s3 = hardwareMap.get(Servo.class, NAME_S3);
        s4 = hardwareMap.get(Servo.class, NAME_S4);
        s5 = hardwareMap.get(Servo.class, NAME_S5);

        // Về home khi Init để an toàn
        setHome();

        telemetry.addLine("Servo Panels Tuner ready");
        telemetry.addLine("APPLY=false lúc đầu để tránh servo nhảy bất ngờ");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            PanelsConfigurables.INSTANCE.refreshClass(this);

            if (APPLY) {
                if (MODE == 0) {
                    applySingleServo();
                } else if (MODE == 1) {
                    applySlot();
                } else if (MODE == 2) {
                    setHome();
                }
            }

            telemetry.addData("APPLY", APPLY);
            telemetry.addData("MODE", modeName());
            telemetry.addData("SELECTED_SERVO", SELECTED_SERVO);
            telemetry.addData("SERVO_POS", "%.3f", clamp(SERVO_POS));
            telemetry.addData("SELECTED_SLOT", slotName(SELECTED_SLOT));
            telemetry.addLine("Copy giá trị đã tune sang test.java ZONE 1");
            telemetry.update();
        }
    }

    private void applySingleServo() {
        double p = clamp(SERVO_POS);
        switch (SELECTED_SERVO) {
            case 1: s1.setPosition(p); break;
            case 2: s2.setPosition(p); break;
            case 3: s3.setPosition(p); break;
            case 4: s4.setPosition(p); break;
            case 5: s5.setPosition(p); break;
            default: setHome(); break;
        }
    }

    private void applySlot() {
        switch (SELECTED_SLOT) {
            case 0:
                setAll(TL_S1, TL_S2, TL_S3, TL_S4, TL_S5);
                break;
            case 1:
                setAll(TR_S1, TR_S2, TR_S3, TR_S4, TR_S5);
                break;
            case 2:
                setAll(BL_S1, BL_S2, BL_S3, BL_S4, BL_S5);
                break;
            case 3:
                setAll(BR_S1, BR_S2, BR_S3, BR_S4, BR_S5);
                break;
            default:
                setHome();
                break;
        }
    }

    private void setHome() {
        setAll(S1_HOME, S2_HOME, S3_HOME, S4_HOME, S5_HOME);
    }

    private void setAll(double p1, double p2, double p3, double p4, double p5) {
        s1.setPosition(clamp(p1));
        s2.setPosition(clamp(p2));
        s3.setPosition(clamp(p3));
        s4.setPosition(clamp(p4));
        s5.setPosition(clamp(p5));
    }

    private double clamp(double v) {
        if (v < 0.0) return 0.0;
        if (v > 1.0) return 1.0;
        return v;
    }

    private String modeName() {
        switch (MODE) {
            case 0: return "0 - Single Servo";
            case 1: return "1 - Slot 5 Servos";
            case 2: return "2 - Home";
            default: return "Unknown";
        }
    }

    private String slotName(int slot) {
        switch (slot) {
            case 0: return "TOP_LEFT";
            case 1: return "TOP_RIGHT";
            case 2: return "BOT_LEFT";
            case 3: return "BOT_RIGHT";
            default: return "Unknown";
        }
    }
}
