# Hướng dẫn Cấu hình PLC Siemens S7-1200 giao tiếp với PC (Python - Snap7)

Tài liệu này hướng dẫn kỹ sư lập trình PLC Siemens cách cấu hình phần cứng, thiết lập Data Blocks (DBs) và định dạng dữ liệu trên phần mềm **TIA Portal** để tương thích với Gateway giao tiếp [EthernetCom.py](file:///home/tangerine/Share/Global%20Share/Documents/Delta_robot/modules/EthernetCom.py) sử dụng thư viện `python-snap7`.

---

## 1. Cấu hình phần cứng CPU (TIA Portal)

Để cho phép máy tính (PC) kết nối và trao đổi dữ liệu với PLC qua thư viện Snap7, bạn bắt buộc phải bật tính năng cho phép giao tiếp PUT/GET trên CPU Siemens S7-1200.

1. Mở dự án trên **TIA Portal**.
2. Chọn **Device configuration** -> Click đúp vào hình ảnh **CPU S7-1200**.
3. Tại tab thuộc tính bên dưới, chọn **Properties** -> **General** -> **Protection & Security** -> **Connection mechanisms**.
4. Tích chọn vào ô: **"Permit access with PUT/GET communication from remote partner"** (Cho phép truy cập với giao tiếp PUT/GET từ đối tác từ xa).
5. Compile và Download cấu hình phần cứng xuống PLC.

---

## 2. Cấu hình thuộc tính của Data Block (DB)

Mặc định, các dòng PLC S7-1200/S7-1500 sử dụng cơ chế truy cập tối ưu hóa bộ nhớ (**Optimized block access**). Cơ chế này ẩn địa chỉ Offset (byte) của các biến, khiến thư viện Snap7 không thể truy cập trực tiếp bằng địa chỉ byte được.

Bạn cần tắt tính năng này cho các DB giao tiếp:

1. Click chuột phải vào Data Block cần cấu hình (ví dụ `DB1`, `DB2`) -> Chọn **Properties**.
2. Chọn mục **Attributes**.
3. Bỏ tích chọn ô **"Optimized block access"**.
4. Nhấn **OK**.
5. Thực hiện **Compile** lại Data Block. Lúc này, cột **Offset** sẽ xuất hiện hiển thị địa chỉ byte của từng biến (`0.0`, `4.0`, `8.0`...).

---

## 3. Định nghĩa Cấu trúc Dữ liệu trong DB

Để quá trình đọc/ghi mảng byte thô (`raw bytes`) giữa Python và PLC không bị lệch hoặc sai định dạng, cấu trúc các biến khai báo trong DB của PLC phải khớp hoàn toàn về **thứ tự**, **kiểu dữ liệu** và **kích thước** với các Struct trong Python.

### 3.1. DB Ghi lệnh xuống PLC (PC -> PLC)
* **Số hiệu DB mặc định:** `1` (Tương ứng với `SIEMENS_DB_WRITE = 1` trong Python)
* **Offset bắt đầu ghi:** `0` (Tương ứng với `SIEMENS_DB_WRITE_OFFSET = 0`)
* **Cấu trúc khai báo trong PLC (TIA Portal):**

| Tên biến (PLC) | Kiểu dữ liệu (PLC) | Địa chỉ Offset | Mô tả | Mapped Python Field |
| :--- | :--- | :--- | :--- | :--- |
| **CommandID** | DINT (Double Integer) | `0.0` (4 bytes) | Mã lệnh điều khiển | `CommandID` (ctypes.c_int32) |
| **rotate** | REAL (Floating Point) | `4.0` (4 bytes) | Góc quay tuyệt đối (4th DOF) | `rotate` (ctypes.c_float) |
| **speed** | REAL (Floating Point) | `8.0` (4 bytes) | Tốc độ băng tải yêu cầu | `speed` (ctypes.c_float) |

> [!IMPORTANT]
> Tổng kích thước gói tin gửi xuống PLC là **12 bytes**. Trong PLC, bạn phải đảm bảo không khai báo thêm biến nào chèn vào giữa hoặc làm thay đổi kích thước của 3 biến này trong vùng offset từ `0.0` đến `11.0`.

### 3.2. DB Đọc trạng thái phản hồi (PLC -> PC)
* **Số hiệu DB mặc định:** `2` (Tương ứng với `SIEMENS_DB_READ = 2` trong Python)
* **Offset bắt đầu đọc:** `0` (Tương ứng với `SIEMENS_DB_READ_OFFSET = 0`)
* **Cấu trúc khai báo trong PLC (TIA Portal):**

| Tên biến (PLC) | Kiểu dữ liệu (PLC) | Địa chỉ Offset | Mô tả | Mapped Python Field |
| :--- | :--- | :--- | :--- | :--- |
| **rotate_current** | REAL (Floating Point) | `0.0` (4 bytes) | Góc quay hiện tại của giác hút | `rotate_current` (ctypes.c_float) |
| **speed_current** | REAL (Floating Point) | `4.0` (4 bytes) | Tốc độ thực tế hiện tại của băng tải | `speed_current` (ctypes.c_float) |
| **task_doing** | DINT (Double Integer) | `8.0` (4 bytes) | Lệnh đang thực hiện | `task_doing` (ctypes.c_int32) |
| **task_state** | DINT (Double Integer) | `12.0` (4 bytes) | Trạng thái lệnh (0: Chờ, 1: Chạy, 2: Xong...) | `task_state` (ctypes.c_int32) |

> [!IMPORTANT]
> Tổng kích thước gói tin đọc lên PC là **16 bytes** tương ứng từ offset `0.0` đến `15.0`.

---

## 4. Lưu ý quan trọng về thứ tự Byte (Endianness)

* **Sự khác biệt hệ thống:** 
  * Máy tính PC (kiến trúc x86/x64) lưu trữ dữ liệu dạng **Little-Endian** (byte thấp trước).
  * PLC Siemens S7-1200 lưu trữ dữ liệu dạng **Big-Endian** (byte cao trước).
* **Ảnh hưởng:** Khi truyền trực tiếp mảng byte thô qua Snap7 mà không xử lý, các giá trị số thực (`REAL`) hoặc số nguyên lớn (`DINT`) sẽ bị đảo ngược thứ tự các byte dẫn đến hiển thị sai số trị.
* **Cách khắc phục:**
  * **Cách A (Khuyên dùng phía PLC):** Trong chương trình PLC, sau khi nhận gói tin từ PC hoặc trước khi gửi dữ liệu lên PC, hãy sử dụng lệnh hoán đổi byte (**SWAP** hoặc **CAD/CAW**) đối với các biến DINT và REAL để đảm bảo giá trị đọc được là chính xác.
  * **Cách B (Khế ước phía Python):** Nếu phía PLC giữ nguyên Big-Endian thô, cấu trúc struct trên code Python cần được điều chỉnh từ `ctypes.Structure` sang `ctypes.BigEndianStructure` để tự động xử lý việc đảo byte trên máy tính PC.
