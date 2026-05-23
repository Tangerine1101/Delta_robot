# Hướng dẫn Cấu hình Hệ thống Robot Delta

Tài liệu này giải thích chi tiết ý nghĩa vật lý, quy tắc hệ toạ độ và phạm vi an toàn của các thông số trong file cấu hình `modules/config.json`.

---

## 1. Hệ tọa độ Trục Z âm (Negative Z Axis)
Robot Delta sử dụng hệ tọa độ cực gốc với trục Z hướng xuống dưới.
* **Tọa độ Z gần `0.0`:** Robot thu cánh tay lên cao nhất (gần đế robot).
* **Tọa độ Z âm lớn (ví dụ `-310.0`):** Robot vươn cánh tay xuống thấp nhất (gần mặt bàn/băng chuyền).

**Quy tắc bất biến an toàn (Safety Invariant):**
$$\text{clearance\_height} > \text{pre\_pick\_height} > \text{pickup\_height}$$
*(Ví dụ: $-290.0 > -300.0 > -310.0$)*

Nếu vi phạm quy tắc này (ví dụ đặt điểm chuẩn bị gắp thấp hơn điểm gắp thực tế), bộ lập lịch sẽ báo lỗi ngay khi khởi động để bảo vệ robot tránh va chạm.

---

## 2. Chi tiết các thông số Cao độ (Heights)

| Khóa cấu hình | Đơn vị | Giá trị khuyên dùng | Ý nghĩa vật lý |
| :--- | :---: | :---: | :--- |
| `home_position` | mm | `[0.0, 0.0, -290.0]` | Vị trí nghỉ mặc định của robot khi không thực hiện nhiệm vụ. |
| `clearance_height` | mm | `-290.0` | **Chiều cao di chuyển ngang an toàn.** Độ cao Z mà tại đó robot thực hiện các hành trình di chuyển ngang XY giữa vị trí Home, vị trí gắp và thả để tránh va đụng chướng ngại vật. |
| `pre_pick_height` | mm | `-300.0` | **Chiều cao chờ gắp.** Nằm cao hơn mặt vật thể khoảng 10mm. Robot di chuyển nhanh tới độ cao này trước khi hạ từ từ xuống tiếp xúc vật để tránh va đập cơ khí mạnh. |
| `pickup_height` | mm | `-310.0` | **Chiều cao gắp thực tế.** Độ cao Z mà giác hút tiếp xúc vật lý sát mặt băng tải và kích hoạt lực hút. |
| `place_height` | mm | `-290.0` | **Chiều cao thả vật.** Độ cao Z tại vị trí phân loại nơi robot nhả vật ra. |

---

## 3. Các thông số Động học & Điều khiển quỹ đạo

| Khóa cấu hình | Đơn vị | Mặc định | Ý nghĩa vật lý |
| :--- | :---: | :---: | :--- |
| `corner_blend_xy` | mm | `35.0` | Bán kính vạt/bo góc tối đa trên mặt phẳng XY để giảm gia tốc khúc cua và làm mượt chuyển động cơ khí. |
| `nominal_xy_speed` | mm/s | `220.0` | Vận tốc định mức trên mặt XY dùng để ước lượng thời gian di chuyển giữa các phân đoạn. |
| `nominal_z_speed` | mm/s | `180.0` | Vận tốc định mức trên trục Z dùng để ước lượng thời gian nâng/hạ. |

---

## 4. Các thông số Độ trễ & Bù trừ thời gian (Delays)

Để robot gắp chính xác vật thể đang di chuyển trên băng tải, bộ lập lịch cần gửi lệnh gắp trước một khoảng thời gian bù trừ độ trễ:
$$\text{Thời điểm gửi lệnh} = \text{Thời điểm gắp dự tính} - (\text{robot\_movement\_delay\_s} + \text{ethernet\_delay\_s})$$

| Khóa cấu hình | Đơn vị | Mặc định | Ý nghĩa vật lý |
| :--- | :---: | :---: | :--- |
| `robot_movement_delay_s` | giây | `0.05` | Độ trễ phản hồi cơ học và thời gian bắt đầu chuyển động thực tế của cánh tay robot. |
| `ethernet_delay_s` | giây | `0.002` | Độ trễ truyền gói tin qua giao thức EtherNet/IP giữa PC và PLC Omron. |
| `intercept_lead_time_s` | giây | `0.14` | Biên thời gian an toàn robot cần đến điểm chuẩn bị gắp sớm hơn thời điểm gắp lý thuyết. |

---

## 5. Cấu hình Vùng làm việc (Workspace)

Các giới hạn cửa sổ tọa độ để bộ lập lịch lọc và quyết định có gắp vật thể hay không. Nếu vật thể nằm ngoài vùng này, bộ lập lịch sẽ bỏ qua để tránh robot vươn quá giới hạn cơ khí gây lỗi Servo.
* `pickup_window_x`: Khoảng giới hạn trục X `[X_min, X_max]`, ví dụ `[-120.0, 50.0]`.
* `pickup_window_y`: Khoảng giới hạn trục Y `[Y_min, Y_max]`, ví dụ `[-60.0, 60.0]`.
