# Fixed-Order Picking Algorithm (1->2->3->4) with Color Filters

Cập nhật thuật toán gắp mẫu vật theo thứ tự khay cố định từ Slot 1 đến Slot 4. Robot chỉ tiến hành gắp nếu mẫu vật ở Slot đó là màu Yellow (1) hoặc màu liên minh (3 nếu Blue, 2 nếu Red).

## Proposed Changes

### Pedro Pathing Component

#### [PiVisionSerial.java](file:///C:/Users/tungh/Downloads/Quickstart-master/Quickstart-master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/PiVisionSerial.java)

1. Thêm phương thức kiểm tra màu của một Slot cụ thể:
   - `public int getSlotColor(int slotId)`
   - Duyệt qua danh sách `latestFrame`, tìm detection có `slotId` khớp và trả về màu sắc của nó (1, 2 hoặc 3). Nếu không tìm thấy, trả về `-1`.
2. Giữ nguyên class `PiDetection` và logic parse Serial mới.

#### [BLUEROBOT.java](file:///C:/Users/tungh/Downloads/Quickstart-master/Quickstart-master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/BLUEROBOT.java)

1. Thêm phương thức gắp theo thứ tự cố định `pickSequenceFixed(Pose shelfPose, boolean isBlueAlliance)`:
   - Loop từ `targetSlotId = 1` đến `4`.
   - Ở mỗi lượt:
     - Bật quét YOLO trên Pi (`piVision.start()`, gửi lệnh `"SCAN"`).
     - Đợi 500ms.
     - Tắt quét YOLO trên Pi (`piVision.stop()`, gửi `"STOP"`).
     - Kiểm tra màu của Slot hiện tại: `int color = piVision.getSlotColor(targetSlotId)`.
     - Lọc màu: Nếu `color == 1` (Vàng) hoặc `color == (isBlueAlliance ? 3 : 2)` (Màu liên minh):
       - Gọi `pickFromShelf` cho Slot hiện tại.
       - Gọi `depositInRobotZone` cho loại Box tương ứng để ném mẫu.
     - Nếu Slot không có hoặc không khớp màu, in log/telemetry và tự động chuyển sang xét Slot tiếp theo.
2. Cập nhật phương thức `shelfRoutine(Pose, PathChain, int)` để gọi `pickSequenceFixed(shelfPose, true)` (Blue alliance).

---

## Verification Plan

### Automated Tests
- Chạy task Gradle build để đảm bảo code compile thành công không có lỗi cú pháp hoặc lỗi kiểu:
  ```powershell
  ./gradlew :TeamCode:assembleDebug
  ```

### Manual Verification
- Deploy lên robot và test thực tế quá trình gắp động của robot tại khay chứa mẫu vật.
