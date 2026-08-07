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
                .forwardZeroPowerAcceleration(-72.93326609712362)
                .lateralZeroPowerAcceleration(-64.4756501207455)

                // ===== Translational PIDF (vị trí x-y) =====
                // PIDF chính: dùng khi robot còn CÁCH XA target hơn translationalPIDFSwitch (inch)
                // GIẢM P xuống (từ 0.04 -> 0.02) để tránh việc robot bốc đầu/tăng tốc quá mạnh khi điểm xa.
                .translationalPIDFCoefficients(new PIDFCoefficients(0.02, 0, 0.000001, 0.002))
                
                // Ngưỡng (inch) chuyển từ PIDF chính (mềm) sang PIDF phụ (cứng/khỏe). 
                // Tăng lên 4 inch để robot sớm "gồng" kềm hãm tốc độ trước khi vào điểm.
                .translationalPIDFSwitch(4)
                
                // PIDF phụ: Dùng khi GẦN target. Tăng P = 0.15 và I = 0.01 để có lực KHỎE kìm hãm vị trí
                .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.15, 0.01, 0.005, 0.05))

                // ===== Heading PIDF (góc xoay) =====
                .headingPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0.000001, 0.00001))
                // Bắt góc cứng sớm hơn ở 0.3 radian (tầm 17 độ)
                .headingPIDFSwitch(0.3)
                // Tăng nhẹ P và I để khóa chết đầu hướng robot
                .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(1.5, 0.01, 0.000001, 0.05))


                // ===== Drive PIDF (lực kéo bánh xe theo path) =====
                // PIDF chính: dùng khi vận tốc còn cao / xa target
               // .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.0011, 0, 0.001, 0.6, 0.008))
                // Ngưỡng vận tốc để chuyển sang PIDF phụ. TUNE theo xVelocity/yVelocity thực tế của bạn.
               // .drivePIDFSwitch(6)
                // PIDF phụ: dùng khi vận tốc thấp / gần tar
                //
                //
                //
                //
                //
                //
                //
                // get, cần bám chính xác hơn.
                // GIẢM FEEDFORWARD (kV) từ 1.0 xuống 0.6 và P xuống 0.002 để tránh PIDF vọt lên max power
                .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.002, 0, 0.000001, 0.6, 0.001))
              //  .drivePIDFSwitch(4)
            //    .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.0011, 0, 0.001, 0.6, 0.008))

                // Bù lực ly tâm khi vào cua. TUNE: nếu robot bị văng/lệ    ch ra ngoài khi rẽ cong gấp, tăng số này.
                .centripetalScaling(0.00005)
                ;


        public static MecanumConstants driveConstants = new MecanumConstants()
                // MaxPower của phần cứng: KHÔNG NÊN KHÓA cứng ở đây dưới 0.5 vì làm giảm feedforward PID.
                // 1.0 là công suất chuẩn để PID tính toán chính xác lực. 
                // Ta sẽ khóa tốc độ 40% ở hàm follower.setMaxPower(0.4) ở trong BLUEROBOT.java.
                .maxPower(1.0)


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
                .xVelocity(84.3096238984826)   // <-- cần đo lại với maxPower(0.5)
                .yVelocity(67.11373540923351)    // <-- cần đo lại với maxPower(0.5)
                ;




        public static PinpointConstants localizerConstants = new PinpointConstants()
                .forwardPodY(-6.165628268024115)
                .strafePodX(-0.0687786040807456095)
                .distanceUnit(DistanceUnit.MM)
                .hardwareMapName("pinpoint")
                .customEncoderResolution(4096 / (2 * Math.PI * 17.5)) // ticks/mm — luôn là mm bất kể distanceUnit ở trên
                .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
                .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
                .yawScalar(1.000322)

                ;   //yawScalar = 3600 / (giá trị Pinpoint đo được)



        ;



        public static PathConstraints pathConstraints = new PathConstraints(
                0.8,     // **GIẢM MAX SPEED_Multiplier / tValueConstraint**: Giới hạn tốc độ max trên đường thẳng
                0.2,     // **velocityConstraint**: có thể tăng nhẹ lên 0.2 để đỡ giật
                0.2,     // **translationalConstraint**: Tăng nhẹ để PID tránh bị giật
                0.009,   // headingConstraint
                50,      // timeoutConstraint
                1.0,     // brakingStrength
                10,      // BEZIER_CURVE_SEARCH_LIMIT (giữ nguyên 10)
                0.15     // **brakingStart**: bắt đầu phanh SỚM HƠN (từ 0.85 -> 0.15 để phanh dài ra)
        );




        public static Follower createFollower(HardwareMap hardwareMap) {
            return new FollowerBuilder(followerConstants, hardwareMap)
                    .pinpointLocalizer(localizerConstants)
                    .pathConstraints(pathConstraints)
                    .mecanumDrivetrain(driveConstants)
                    .build();
        }
    }
