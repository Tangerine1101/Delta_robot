# Cân chỉnh độ phơi sáng camera thủ công (Manual Exposure)

## 1. Lý thuyết

### Tại sao auto-exposure làm giảm FPS?

Camera webcam mặc định chạy chế độ **auto-exposure**: tự động điều chỉnh thời gian phơi sáng
(shutter speed) để ảnh đủ sáng trong mọi điều kiện ánh sáng.

```
Ánh sáng tốt  → exposure ngắn (~1/100s) → camera đọc xong nhanh → FPS cao (~30)
Ánh sáng kém  → exposure dài (~1/15s)   → camera phải chờ sáng  → FPS giảm (~8-15)
```

Khi dùng conveyor belt hoặc pipeline thời gian thực, FPS thấp = miss detection,
latency cao, tracker bị drift. Cần **ghim cứng exposure** để FPS ổn định.

### Các thông số liên quan

| Thông số | Ý nghĩa |
|---|---|
| **Shutter speed** | Thời gian sensor nhận sáng cho mỗi frame. Càng ngắn → tối hơn, nhưng FPS cao hơn và ít motion blur hơn |
| **Gain / ISO** | Khuếch đại tín hiệu. Tăng gain khi thiếu sáng, nhưng tăng noise |
| **Auto-exposure** | Firmware camera tự chọn shutter speed + gain để đạt mức sáng mục tiêu |

---

## 2. Thư viện và API

### OpenCV (cv2)

Tất cả đều thông qua `cap.set(property_id, value)` và `cap.get(property_id)`.

| Property ID | Hằng số OpenCV | Mô tả |
|---|---|---|
| Auto-exposure mode | `cv2.CAP_PROP_AUTO_EXPOSURE` | Bật/tắt auto-exposure |
| Exposure value | `cv2.CAP_PROP_EXPOSURE` | Giá trị exposure khi manual |
| Gain | `cv2.CAP_PROP_GAIN` | Gain / ISO |
| Brightness | `cv2.CAP_PROP_BRIGHTNESS` | Độ sáng tổng thể |

### Backend camera theo OS

OpenCV gọi xuống backend khác nhau tùy OS — **giá trị của `CAP_PROP_AUTO_EXPOSURE` khác nhau**:

| OS | Backend | Auto-exposure ON | Auto-exposure OFF (manual) |
|---|---|---|---|
| **Windows** | DirectShow (DSHOW) | `3` hoặc `0.75` | `1` hoặc `0.25` |
| **Linux** | V4L2 | `3` | `1` |
| **macOS** | AVFoundation | `1` | `0` |

> **Lưu ý**: Đây là quirk lịch sử của OpenCV — giá trị không có nghĩa trực quan,
> chỉ cần nhớ: trên Windows dùng `1` để tắt auto.

### Giá trị `CAP_PROP_EXPOSURE` (exposure value)


> FPS thực tế bị giới hạn bởi `CAP_PROP_FPS` của camera (thường max 30fps).
> Dùng `-6` hoặc `-7` là đủ để ghim 30fps — tối hơn thì giảm xuống `-5`.

Trên **Linux V4L2**, đơn vị là microseconds (μs):

```python
cap.set(cv2.CAP_PROP_EXPOSURE, 10000)  # 10ms = 10000μs
```

---

## 3. Code mẫu

### Cách đơn giản nhất (Windows)

```python
import cv2

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH,  1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
cap.set(cv2.CAP_PROP_FPS, 30)

# Tắt auto-exposure
cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)    # 1 = manual trên Windows DirectShow

# Set exposure cố định
cap.set(cv2.CAP_PROP_EXPOSURE, -6)        # ~15ms, đủ cho 30fps

while True:
    ret, frame = cap.read()
    cv2.imshow("cam", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
```

### Đọc giá trị hiện tại để debug

```python
cap = cv2.VideoCapture(0)
print("AUTO_EXPOSURE :", cap.get(cv2.CAP_PROP_AUTO_EXPOSURE))
print("EXPOSURE      :", cap.get(cv2.CAP_PROP_EXPOSURE))
print("FPS           :", cap.get(cv2.CAP_PROP_FPS))
print("GAIN          :", cap.get(cv2.CAP_PROP_GAIN))
```

### Đọc từ config (pattern dùng trong project này)

```yaml
# config/system_config.yaml
camera:
  source: 0
  width: 1280
  height: 720
  fps: 30
  auto_exposure: false   # true = auto, false = manual
  exposure: -6           # giá trị khi manual
```

```python
def open_source(cfg: dict) -> cv2.VideoCapture:
    cam = cfg["camera"]
    src = cam["source"]
    cap = cv2.VideoCapture(src)

    if isinstance(src, int):
        cap.set(cv2.CAP_PROP_FRAME_WIDTH,  cam.get("width",  1280))
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, cam.get("height", 720))
        cap.set(cv2.CAP_PROP_FPS,          cam.get("fps",    30))

        if not cam.get("auto_exposure", True):
            cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)   # Windows DirectShow
            cap.set(cv2.CAP_PROP_EXPOSURE, float(cam.get("exposure", -6)))

    if not cap.isOpened():
        raise RuntimeError(f"Could not open camera source: {src!r}")
    return cap
```

### Cross-platform (Windows + Linux)

```python
import sys
import cv2

def set_manual_exposure(cap: cv2.VideoCapture, exposure_value: float) -> bool:
    """
    Tắt auto-exposure và set exposure cố định.

    Windows (DirectShow): exposure_value là log2(seconds), vd: -6 ≈ 15ms
    Linux (V4L2):         exposure_value là microseconds, vd: 15000 = 15ms

    Returns True nếu set thành công.
    """
    if sys.platform == "win32":
        ok1 = cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
        ok2 = cap.set(cv2.CAP_PROP_EXPOSURE, exposure_value)
    else:
        # Linux V4L2: manual mode = 1
        ok1 = cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
        ok2 = cap.set(cv2.CAP_PROP_EXPOSURE, exposure_value)

    # Đọc lại để verify (một số camera ignore set nếu không hỗ trợ)
    actual = cap.get(cv2.CAP_PROP_EXPOSURE)
    print(f"[cam] Exposure set={exposure_value}, actual={actual}")
    return ok1 and ok2
```

---

## 4. Thứ tự gọi quan trọng

**Phải set `AUTO_EXPOSURE` trước, rồi mới set `EXPOSURE`.**
Nếu set ngược lại, camera firmware sẽ override giá trị exposure bằng giá trị auto của nó.

```python
# ĐÚNG
cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)   # tắt auto trước
cap.set(cv2.CAP_PROP_EXPOSURE, -6)       # rồi mới set giá trị

# SAI — camera sẽ ignore exposure vì auto vẫn đang bật
cap.set(cv2.CAP_PROP_EXPOSURE, -6)
cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
```

---

## 5. Tìm giá trị exposure phù hợp

### Script thử nhanh (interactive)

```python
import cv2

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
cap.set(cv2.CAP_PROP_FPS, 30)
cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)

exposure = -6
cap.set(cv2.CAP_PROP_EXPOSURE, exposure)

print("W/S = tăng/giảm exposure | Q = thoát")
print(f"Exposure hiện tại: {exposure}")

while True:
    ret, frame = cap.read()
    cv2.putText(frame, f"Exposure: {exposure}", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    cv2.imshow("Tune Exposure", frame)

    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break
    elif key == ord('w'):
        exposure += 1
        cap.set(cv2.CAP_PROP_EXPOSURE, exposure)
        print(f"Exposure: {exposure}")
    elif key == ord('s'):
        exposure -= 1
        cap.set(cv2.CAP_PROP_EXPOSURE, exposure)
        print(f"Exposure: {exposure}")

cap.release()
cv2.destroyAllWindows()
```

### Heuristic chọn giá trị

```
Bắt đầu từ -6, rồi điều chỉnh:

Ảnh quá tối  → tăng lên: -5, -4, -3
Ảnh quá sáng → giảm xuống: -7, -8
Vật di chuyển bị blur → giảm xuống (shutter nhanh hơn): -7, -8
```

---

## 6. Các lỗi thường gặp

### Camera không nhận set

Một số camera (đặc biệt camera tích hợp laptop) không hỗ trợ manual exposure qua DirectShow.
Kiểm tra bằng cách đọc lại sau khi set:

```python
cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
cap.set(cv2.CAP_PROP_EXPOSURE, -6)

auto = cap.get(cv2.CAP_PROP_AUTO_EXPOSURE)
exp  = cap.get(cv2.CAP_PROP_EXPOSURE)
print(f"AUTO={auto}, EXP={exp}")
# Nếu AUTO vẫn = 3.0 và EXP ≠ -6 → camera không hỗ trợ
```

Giải pháp: dùng **OBS Virtual Camera**, **v4l2-ctl** (Linux), hoặc phần mềm camera
của manufacturer để set manual trước khi mở bằng OpenCV.

### FPS vẫn thấp dù đã set manual

Kiểm tra `CAP_PROP_FPS` sau khi mở:

```python
print(cap.get(cv2.CAP_PROP_FPS))  # nếu = 0 → camera không báo FPS
```

Đảm bảo set FPS **trước** khi set exposure:
```python
cap.set(cv2.CAP_PROP_FPS, 30)
cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
cap.set(cv2.CAP_PROP_EXPOSURE, -6)
```

### Giá trị `AUTO_EXPOSURE` trả về là 0.75 thay vì 3

Một số build OpenCV encode giá trị theo tỉ lệ `value/4.0`. Cả 2 đều có nghĩa như nhau:
- `3.0` = auto → tương đương `0.75`
- `1.0` = manual → tương đương `0.25`

Set `1` hoặc `0.25` đều được, tuỳ camera accept cái nào:

```python
# Thử cả hai nếu một cái không work
cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
# hoặc
cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
```

---

## 7. Tóm tắt nhanh

```python
import cv2

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
cap.set(cv2.CAP_PROP_FPS, 30)
cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)   # 1 = manual (Windows)
cap.set(cv2.CAP_PROP_EXPOSURE, -6)       # -6 ≈ 15ms ≈ 30fps

# Verify
print(cap.get(cv2.CAP_PROP_AUTO_EXPOSURE))  # mong đợi: 1.0
print(cap.get(cv2.CAP_PROP_EXPOSURE))       # mong đợi: -6.0
```

**Không cần cài thêm thư viện** — `cv2.CAP_PROP_AUTO_EXPOSURE` và `cv2.CAP_PROP_EXPOSURE`
là built-in của OpenCV, hoạt động với mọi camera USB chuẩn UVC trên Windows/Linux/macOS.
