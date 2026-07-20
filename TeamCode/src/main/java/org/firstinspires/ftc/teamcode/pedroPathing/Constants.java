package org.firstinspires.ftc.teamcode.pedroPathing;


import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;


import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(5)
            .forwardZeroPowerAcceleration(-500.29166758285331)
            .lateralZeroPowerAcceleration(-500.3634241994715)


            // ===== Translational PIDF (vị trí x-y) =====
            // PIDF chính: dùng khi robot còn CÁCH3 XA target hơn translationalPIDFSwitch (inch)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.015, 0, 0.0001, 0.0025))
            // Ngưỡng (inch) để chuyển từ PIDF chính sang PIDF phụ. TUNE: thử 3-6 inch.
            .translationalPIDFSwitch(1)
            // PIDF phụ: dùng khi robot đã GẦN target (< switch), cần P cao hơn để bám chính xác.
            // TUNE: nếu robot dừng hụt/lệch vị trí cuối, tăng P. Nếu rung/dao động quanh target, giảm P.
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.015, 0, 0.001, 0.25))


            // ===== Heading PIDF (góc xoay) =====
            .headingPIDFCoefficients(new PIDFCoefficients(0.9, 0, 0.0001, 0.0025))
            // PIDF phụ cho heading khi góc lệch đã nhỏ, cần chính xác cao.
            // TUNE: nếu robot xoay hụt góc cuối, tăng P. Nếu rung lắc khi gần đúng góc, giảm P.
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(4, 0, 0.0058, 0.0058))


            // ===== Drive PIDF (lực kéo bánh xe theo path) =====
            // PIDF chính: dùng khi vận tốc còn cao / xa target
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.0011, 0, 0.001, 0.6, 0.008))
            // Ngưỡng vận tốc để chuyển sang PIDF phụ. TUNE theo xVelocity/yVelocity thực tế của bạn.
            .drivePIDFSwitch(6)
            // PIDF phụ: dùng khi vận tốc thấp / gần target, cần bám chính xác hơn.
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.01, 0, 0.000005, 0.6, 0.0001))


            // Bù lực ly tâm khi vào cua. TUNE: nếu robot bị văng/lệ    ch ra ngoài khi rẽ cong gấp, tăng số này.
            .centripetalScaling(0.00005)
            ;


    public static MecanumConstants driveConstants = new MecanumConstants()
            // Giữ nguyên maxPower = 0.5 theo yêu cầu của bạn (robot chạy ở 50% công suất)
            .maxPower(0.5)


            .rightFrontMotorName("rightFront")
            .rightRearMotorName("rightBack")
            .leftRearMotorName("leftBack")
            .leftFrontMotorName("leftFront")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)


            // !!! QUAN TRỌNG: 2 giá trị bên dưới PHẢI được đo lại (chạy lệnh Tuner velocity tune)
            // SAU KHI maxPower đã được set = 0.5. Nếu bạn tune chúng lúc maxPower = 1.0
            // thì follower sẽ nghĩ robot nhanh hơn thực tế => robot chạy max power suốt path,
            // không giảm tốc đúng, và bạn sẽ thấy nó "vẫn nhanh" dù maxPower đã giảm.
            .xVelocity(341.121864859513416)   // <-- cần đo lại với maxPower(0.5)
            .yVelocity(346.53757098340613)    // <-- cần đo lại với maxPower(0.5)
            ;




    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(5.90551181)
            .strafePodX(0)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .customEncoderResolution(4096 / (2 * Math.PI * 17.5)) // ticks/mm — luôn là mm bất kể distanceUnit ở trên
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .yawScalar(1.000322)

            ;   //yawScalar = 3600 / (giá trị Pinpoint đo được)



    ;



    public static PathConstraints pathConstraints = new PathConstraints(
            0.995,   // tValueConstraint
            0.1,     // velocityConstraint
            0.1,     // translationalConstraint
            0.009,   // headingConstraint
            50,      // timeoutConstraint
            1.0,     // brakingStrength
            10,      // BEZIER_CURVE_SEARCH_LIMIT (giữ nguyên 10)
            0.85     // brakingStart
    );




    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}
