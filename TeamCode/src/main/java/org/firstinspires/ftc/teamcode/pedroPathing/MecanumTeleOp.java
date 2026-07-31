package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import java.util.Locale;

/**
 * TELEOP: Mecanum Drive with Field-Centric mode & Pinpoint IMU heading hold.
 *
 * Controls:
 *   Left Stick  → Move (strafe & drive)
 *   Right Stick → Turn
 *   BACK Button → Reset heading (re-zero Pinpoint IMU)
 *   RIGHT Trigger → Slow mode (0.35 speed)
 *   LEFT Trigger  → Turbo mode (1.00 speed)
 *   Default Speed → 0.65 speed
 */
@TeleOp(name = "Mecanum Drive - Field Centric", group = "TeleOp")
public class MecanumTeleOp extends LinearOpMode {

    private DcMotor leftFront, leftBack, rightFront, rightBack;
    private GoBildaPinpointDriver pinpoint;

    private double offsetHeading = 0.0;

    @Override
    public void runOpMode() {
        // --- Motor configuration ---
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack   = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack  = hardwareMap.get(DcMotor.class, "rightBack");

        // Directions: left side reversed, right side forward
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);

        // Run with encoders / zero power behavior
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // --- Pinpoint Localizer ---
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.resetPosAndIMU();

        telemetry.addLine("Mecanum Drive Ready.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            pinpoint.update();

            // Press BACK to re-zero heading
            if (gamepad1.back) {
                pinpoint.resetPosAndIMU();
                offsetHeading = 0.0;
            }

            // Get current heading from Pinpoint (in radians)
            double heading = pinpoint.getHeading(org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS) - offsetHeading;

            // Drive control sticks
            double y  = -gamepad1.left_stick_y; // Forward
            double x  =  gamepad1.left_stick_x; // Strafe
            double rx =  gamepad1.right_stick_x; // Turn

            // Field-centric translation rotation
            double rotX = x * Math.cos(-heading) - y * Math.sin(-heading);
            double rotY = x * Math.sin(-heading) + y * Math.cos(-heading);

            // Speed multiplier: triggers
            double speed = 0.65;
            if (gamepad1.right_trigger > 0.1) {
                speed = 0.35; // Slow precision mode
            } else if (gamepad1.left_trigger > 0.1) {
                speed = 1.00; // Turbo mode
            }

            // Mecanum drive power calculation
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1.0);
            double lfPower = (rotY + rotX + rx) / denominator;
            double lbPower = (rotY - rotX + rx) / denominator;
            double rfPower = (rotY - rotX - rx) / denominator;
            double rbPower = (rotY + rotX - rx) / denominator;

            // Apply powers with multiplier
            leftFront.setPower(lfPower * speed);
            leftBack.setPower(lbPower * speed);
            rightFront.setPower(rfPower * speed);
            rightBack.setPower(rbPower * speed);

            // Telemetry
            telemetry.addLine("═══ FIELD-CENTRIC TELEOP ═══");
            telemetry.addData("Heading", String.format(Locale.US, "%.1f deg", Math.toDegrees(heading)));
            telemetry.addData("Speed factor", String.format(Locale.US, "%.2f", speed));
            telemetry.addData("Power LF/LB", String.format(Locale.US, "%.2f / %.2f", lfPower, lbPower));
            telemetry.addData("Power RF/RB", String.format(Locale.US, "%.2f / %.2f", rfPower, rbPower));
            telemetry.update();
        }
    }
}
