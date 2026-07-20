# Task List

- [x] Tạo mới `PedroPositionStabilityTuner.java` (5-point Ivy autonomous tuner)
- [x] Thêm telemetry vào `waitUntilPoseStable()` trong `BLUEROBOT.java`
- [x] Xóa unused import `CopyOnWriteArrayList` trong `PiVisionSerial.java`
- [x] Sửa lỗi compile và scale của `YawScalarTest.java` (Tuner cho Pinpoint IMU)
- [x] Cập nhật giao thức Serial mới nhận từ Pi (`PICK,id,col,row`)
- [/] Tích hợp thuật toán gắp mẫu cố định (1->2->3->4) vào `BLUEROBOT.java`
  - [/] Viết bản thiết kế và cập nhật `PiVisionSerial.java` + `BLUEROBOT.java`
  - [ ] Đồng bộ hóa / kiểm tra biên dịch phần mềm
  - [ ] Kiểm tra tính đúng đắn với các OpMode sẵn có (`BLUEROBOT.java`)
