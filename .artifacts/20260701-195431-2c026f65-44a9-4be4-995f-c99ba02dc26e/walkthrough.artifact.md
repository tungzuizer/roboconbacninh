# Walkthrough — Thuật toán gắp cố định thứ tự (1 -> 2 -> 3 -> 4) từ Pi 5 YOLO

Tôi đã hoàn tất việc cập nhật và tích hợp thuật toán gắp cố định theo thứ tự có chọn lọc màu sắc cho robot của bạn.

## Những thay đổi đã thực hiện

### 1. [PiVisionSerial.java](file:///C:/Users/tungh/Downloads/Quickstart-master/Quickstart-master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/PiVisionSerial.java)

- Cập nhật cú pháp parse bản tin mới nhận từ Pi 5 qua Serial: `PICK,<slotId>,<cột>,<hàng>\n` và `END\n`
- Thêm hàm `getSlotColor(int slotId)` giúp robot truy vấn nhanh màu sắc mẫu vật ở slot hiện tại:
  - Trả về `1` (Vàng), `2` (Đỏ), `3` (Xanh) hoặc `-1` nếu slot đó trống.

### 2. [BLUEROBOT.java](file:///C:/Users/tungh/Downloads/Quickstart-master/Quickstart-master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/BLUEROBOT.java)

- Thêm hàm gắp cố định `pickSequenceFixed(Pose shelfPose, boolean isBlueAlliance)`:
  - **Lặp tuần tự** slot từ `1` đến `4`.
  - Ở mỗi lượt: Bật YOLO / cổng serial $\rightarrow$ Chờ 500ms để Pi xử lý lấy dữ liệu $\rightarrow$ Tắt YOLO để giải phóng hiệu năng.
  - Kiểm tra xem mẫu vật ở slot đó có phải màu Vàng (1) hoặc màu liên minh (3 nếu Blue, 2 nếu Red) không.
  - Nếu thỏa mãn: Gọi đúng hàm `pickFromShelf` (đẩy servo tương ứng ra gắp, thu các servo khác về) $\rightarrow$ Gọi `depositInRobotZone` để ném mẫu vào giỏ.
  - Nếu không thỏa mãn: Bỏ qua và chuyển sang kiểm tra khay (Slot) tiếp theo.
- Thay thế logic quét cũ trong `shelfRoutine` bằng cuộc gọi động:
  `pickSequenceFixed(shelfPose, StandaloneAutoRuntime.getSelectedSide() != StandaloneAutoRuntime.Side.RED)`
  Nhờ vậy, robot sẽ tự động lọc đúng màu liên minh mà bạn chọn trước trận đấu trên Driver Station (không bị gán cứng liên minh Blue).

### 3. Dọn dẹp dự án
- Đã xóa sạch toàn bộ các file script Python phụ (`inspect_*.py` và `dump_*.py`) được tạo làm công cụ để bypass cache của IDE. Thư mục làm việc sạch sẽ hoàn toàn.

---

## Kết quả kiểm chứng

- **Build Status**: `:TeamCode:assembleDebug` $\rightarrow$ **BUILD SUCCESSFUL** ✅
- **Static Analysis**: **0 lỗi** biên dịch cú pháp ✅
