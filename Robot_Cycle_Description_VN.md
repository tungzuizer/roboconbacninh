# MÔ TẢ CHI TIẾT TRÌNH TỰ VẬN HÀNH ROBOT (BLUE ALLIANCE)

Tài liệu này trình bày lại cụ thể chuỗi hành động theo từng bước thời gian thực (timeline sequence) của robot dựa sát vào source code `BLUEROBOT.java` và toạ độ tại `BoxAutoPanels.java`.

## CÁC QUY CƯỚC TỌA ĐỘ VÀ GÓC QUAY SERVO

- **P0 (Start):** X = 0.0, Y = 0.0, Góc = 0°
- **P8 (Viewpoint YOLO - Kệ 1):** EXTRA_PICK_X = 0.0, EXTRA_PICK_Y = 0.0
- **P13 (Viewpoint YOLO - Kệ 2):** VIEWPOINT2_X = 25.5, VIEWPOINT2_Y = 0.0
- **P14 (Viewpoint YOLO - Kệ 3):** VIEWPOINT3_X = 51.0, VIEWPOINT3_Y = 0.0
- **Kệ hàng (Shelfs):**
  - P1 (Kệ 1): X = 0.0, Y = 20.0
  - P2 (Kệ 2): X = 25.5, Y = 20.0
  - P3 (Kệ 3): X = 51.0, Y = 20.0
- **Rổ thả hàng Factory (Drop):**
  - P4 (Rổ 1 / Drop 1): X = 7.0, Y = -21.0
  - P5 (Rổ 2 / Drop 2): X = 16.0, Y = -21.0
  - P6 (Rổ 3 / Drop 3): X = 38.0, Y = -21.0
  - P7 (Rổ 4 / Drop 4): X = 48.0, Y = -21.0
- **Điểm thoát hiểm (Dodge):** Các điểm chéo cách rổ thả y=-15.0 để né gầm.

---

## I. GIAI ĐOẠN KHỞI ĐỘNG VÀ QUÉT YOLO (VISION SCAN KỆ 1)
Ngay khi bắt đầu (Start), Robot kích hoạt Sequential Auto Routine.

1. **Khởi hành từ vị trí P0 (0,0):** Mở cửa Servo 5 (S5_OPEN) để rơi rào chắn nếu có.
2. **Di chuyển tới P8 (Viewpoint Kệ 1):** Hệ thống PedroPathing điều hướng robot tới Pose 8 `(0.0, 0.0)` để camera Pi nhìn bao quát 4 hộp.
3. **Chờ ổn định (`waitUntilStable`):** Xe giữ vị trí tĩnh để camera không bị nhòe.
4. **Quét Camera (`scanInitial4BoxesCommand`):** Quét YOLO tầng 2 và tầng 1 của Kệ 1.

---

## II. GIAI ĐOẠN ĐI LẤY & THẢ HÀNG (EXECUTE TURN CYCLE)

### 1. KỆ 1 (P1 - X:0.0, Y:20.0)

#### LƯỢT 1: Gắp Tầng 2 tại Kệ 1
1. **Lái từ Viewpoint P8 đến P1 (0.0, 20.0, 0°):** Di chuyển thẳng tiến lên kệ.
2. **Setup ngàm kẹp (`moveS4Grab`):** Cổ tay S4 ngửa gập xuống mặt kệ, ngàm kẹp S3 và S5 há hốc. Thanh vươn S2 rút sát bụng.
3. **Thanh nâng (`setElevatorLevel`):** Thanh nâng Z (S1) tụt xuống cao độ Tầng 2 (Row 2, giá trị 0.175).
4. **Thanh vươn (`extendS2`):** Thanh vươn chữ X (S2) bung dài (0.8) lao tới tóm hộp.
5. **Ngoạm hộp (`clampBothS3S5`):** Khóa kẹp lại. S3 và S5 siết chặt 2 hộp cùng lúc.
6. **Nhấc khe hở (`liftUpS1`):** Nâng cao S1 lên thêm 0.2 nhằm nhấc hẫng hai hộp lên không trung.
7. **Rút tay về (`retractS2`):** Thu cùm vươn S2 về gần sát (0.4) sát yếm robot để di chuyển an toàn tuyệt đối.
8. **Nâng hết cỡ (`s1Highest`):** Phóng trục nâng S1 lên điểm cực đỉnh (0.0), chừa không gian trống bên dưới.

#### LƯỢT 1: Thả Hộp Thứ Nhất
Biến `post1BoxType` (YOLO) quyết định hộp nằm ở rổ P4, P5, P6 hay P7. Lấy rổ **P4** làm ví dụ:
1. **Điều hướng xe (`driveToPose`):** Xe lùi từ Kệ 1 về Rổ thả số 1 (7.0, -21.0, 0°).
2. **Lật cổ tay ra sau (`s4FlipBackOnlyHigh`):** Cổ tay S4 bẻ cong vút ra sau lưng (`0.75`). S2 vẫn giữ ở Home thu gọn.
3. **Đẩy ngàm ra sau lưng (`s2ExtendHighForDrop`):** Thanh S2 vươn xa ra sau (`0.9`), chĩa hộp vào đúng họng rơi mặt nhà máy.
4. **Hạ tay chúi xuống móc (`prepareS3LowDrop`):** Thanh nâng vút sập xuống đáy (`0.2`).
5. **Nhả hộp 1:** Bật nhả móng vuốt bên TRÁI `S3` -> Hộp rớt.
6. **Lùi né gầm (`factoryDodgePose` P9):** Bò chéo sang mép P9 để thoát ngàm khỏi miệng giếng rổ.
7. **Thu hồi tay (`s1LiftUp...`):** Phi lên và thu tay vươn về. Cổ tay S4 vẫn đang úp ở sau lưng chờ gắp thứ hai.

#### LƯỢT 1: Chạy ngay sang giỏ kế bên Thả Hộp Thứ Hai
1. **Lái ngang xe:** Tiến tới giỏ thứ hai là `P5` (16.0, -21.0).
2. S2 phi thẳng vào Rổ, S1 thụt sập luôn vào túi rổ hệt thả hộp 1.
3. **Nhả hộp 2:** Há móng vuốt bên PHẢI `S5` -> Hộp văng nốt.
4. **Lách gầm, dọn tay về HOME:** Bò ra điểm Dodge P10, thuọn gọn tay bọc vô bụng.

#### LƯỢT 2: Trở lại Kệ 1 Gắp Tầng 1
1. **Lái xe (`driveToPose`):** Bò ngược lại sát vị trí kệ hàng P1 (0.0, 20.0).
2. Cổ tay S4 lại vòng lên bọc xuống góc ngậm đằng trước. S1 tụt xuống chạm sát mặt sàn kệ Tầng 1 (Row 1).
3. Đóng kẹp gắp hai hộp -> hất lên -> lòi tay thu về y hệt Lượt 1.
4. Chở đi tới Rổ thả nhả từ ngàm một. Và lách Dodge. Xong Kệ 1.

---

### 2. QUÉT VÀ GẮP KỆ 2 (P2 - X:25.5, Y:20.0)
1. **Tới Viewpoint mới P13:** Thay vì P8, Robot tiến ngang về góc mới P13 (`25.5, 0.0`) để camera chiếu thẳng vào Kệ 2.
2. Dừng đợi ổn định, và gọi `scanInitial4BoxesCommand()` bắt nhận diện YOLO cho nhóm hộp ở Kệ 2.
3. **Gắp Tầng 2:** Chạy tiến thẳng lên ngang mặt Kệ 2 `P2(25.5, 20.0)`. Vươn tay S2, cắn hộp S3&S5. Rút lật tay, về ngã rổ thả P4/P5/P6/P7 (sau đó Dodge ra).
4. **Gắp Tầng 1:** Lùi thẳng lên lại P2 `(25.5, 20.0)`, hạ S1 sát đáy vớt ráo tầng 1. Đi xả rổ.

---

### 3. QUÉT VÀ GẮP KỆ 3 (P3 - X:51.0, Y:20.0)
1. **Tới Viewpoint mới P14:** Chạy lùi về góc hẹp bên lề phải P14 (`51.0, 0.0`) cho camera hướng thẳng Kệ 3. Chờ ổn định và gửi `SCAN_ALL_4`.
2. **Gắp Tầng 2:** Tiến lên tọa độ lõi Kệ 3 `P3(51.0, 20.0)`. Đâm ngàm kép tầng 2, chở quặt ra Rổ rớt.
3. **Gắp Tầng 1:** Đảo quành lên P3 chốt luôn Tầng 1 của Kệ 3. Lôi 2 box cuối cùng đi xả Rổ, thụt lách Dodge giọt nước cuối.

---

## IV. HUYỀN THOẠI TRỞ VỀ START POINT 
Sau khi đổ mẻ xả sập ngàm số 5 cuối cùng của Tầng 1 Kệ 3, Robot thu gọn lòng ngực tay kẹp (`allHome`). 
Lệnh lái `driveToPose(P0)` bẻ lái đưa con Bot lùi êm ru về vị trí bản lề Khởi Thủy **(0.0, 0.0)**. Hoàn tất mỹ mãn phần thi Autonomous Blue.