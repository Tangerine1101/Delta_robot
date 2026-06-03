# Thiết Kế Hệ Thống Băng Tải: Tốc Độ Thích Ứng & Hướng Động Lực

## Mở Đầu

### Phạm Vi (Scope)
Tài liệu này quy định chi tiết thiết kế cho hai chức năng mở rộng của hệ thống Delta Robot:

1. **Adaptive Conveyor Speed (A)**: Điều chỉnh tự động tốc độ băng tải theo số lượng sản phẩm, nhằm tối ưu hóa thông lượng pick-and-place.
2. **Dynamic Conveyor Orientation (B)**: Hỗ trợ băng tải nằm ở bất kỳ góc θ nào trong mặt phẳng XY, và dự báo vị trí pickup bằng encoder feedback thực tế.

Cả hai tính năng kết hợp để tạo nên một hệ thống pick-and-place thích ứng cao, sẵn sàng triển khai trên setup vật lý.

### Mục Tiêu
- **Mục tiêu A**: Giảm thời gian chế độ chờ (idle), tăng số pick/phút bằng cách điều chỉnh tốc độ theo mật độ hàng.
- **Mục tiêu B**: Cho phép báng tải xoay tùy ý và dự báo chính xác vị trí pickup thông qua đáp ứng encoder thực tế, chứ không phải setpoint.
- **Mục tiêu Toàn Hệ Thống**: Đảm bảo safety (Z-hierarchy), latency < 100 ms, và khả năng lấy từ encoder feedback Siemens.

### Giả Định Thiết Lập
Hệ thống hoạt động trên cơ sở sau:

- **Hai vùng làm việc**:
  - **Thượng nguồn** (Upstream): Camera đọc vị trí sản phẩm + đếm lượng hàng đợi.
  - **Hạ nguồn** (Downstream): `pickup_window` (~170 mm × 130 mm), nơi robot lấy hàng.
  
- **Encoder Feedback**: Siemens S7-1200 đọc xung encoder từ motor và tính toán tốc độ thực **v_conveyor** (mm/s) trong frame của băng tải.

- **Khung Tính Toán**:
  - Băng tải nằm trong phần tư (-X, -Y) của robot, xoay với góc θ từ trục X âm.
  - Encoder trả về **scalar tốc độ v_belt** (mm/s) dọc trục dài của băng.
  - Vector vận tốc trong frame robot: **v_conveyor = (vx, vy) = v_belt × (−cos θ, −sin θ)**.

- **Liên Lạc**: PC ↔ Siemens qua Modbus TCP / Snap7, Omron qua EtherNet/IP (pylogix).

---

## Bằng Chứng Code Hiện Tại

### Hardcode Y-Only & Thiếu Hỗ Trợ Góc θ

#### 1. **scheduler.py:517** — Tính toán vị trí chỉ dùng vy
```python
current_y = detection.y + self.latest_speed.vy * dt
```
🔴 **Vấn đề**: Chỉ cộng vy vào Y; nếu băng xoay, vx không được dùng.

#### 2. **scheduler.py:646–648** — Kiểm tra Z-window chỉ xét vy
```python
if speed_sample.vy > 0.001 and detection.y < self.settings.pickup_window_y[0]:
    t_enter = detection.timestamp + (self.settings.pickup_window_y[0] - detection.y) / speed_sample.vy
```
🔴 **Vấn đề**: Giả định băng chỉ di chuyển theo Y; nếu xoay, cần chiếu vận tốc lên pickup window.

#### 3. **scheduler.py:654–655** — Dự báo vị trí cứng Y
```python
predicted_x = detection.x + speed_sample.vx * dt
predicted_y = detection.y + speed_sample.vy * dt
```
✅ Đúng rồi, dùng vector (vx, vy), nhưng default_speed và RealSpeedSource vẫn cứng Y-only.

#### 4. **scheduler.py:298** — RealSpeedSource trả về vx=0, vy=tốc độ
```python
return SpeedSample(vx=0.0, vy=speed, timestamp=now)
```
🔴 **Vấn đề**: Hardcode vx=0; encoder Siemens chỉ đọc scalar, không xoay frame.

#### 5. **config.json:41–43** — default_speed cứng [0, 80]
```json
"default_speed": [0.0, 0.0]
```
✅ Tuy nhiên, không có trường `conveyor.theta_deg` hay `conveyor.origin_xy` để xoay frame.

#### 6. **modules/EthernetCom.py:28–34** — SiemensSendPacket cứng scalar speed
```python
class SiemensSendPacket(ctypes.BigEndianStructure):
    _fields_ = [
        ("CommandID", ctypes.c_int32),
        ("rotate", ctypes.c_float),      # Suction cup 4th DOF
        ("speed", ctypes.c_float),       # ← Chỉ scalar (mm/s)
    ]
```
🔴 **Vấn đề**: Command ID 8 (`change_speed`) gửi scalar speed, không vận tốc vector.

#### 7. **scheduler.py:280–281** — SimulatedSpeedSource nhân vô hướng
```python
vx = self.settings.default_speed[0] * scale
vy = self.settings.default_speed[1] * scale
```
✅ Dùng vector, nhưng `scale` là hằng số từ số sản phẩm (chưa có adaptive logic).

### Tóm Lại Hardcode
| Vấn đề | File:Line | Ảnh Hưởng | Ưu Tiên Sửa |
|--------|-----------|---------|-----------|
| Y-only t_enter check | scheduler.py:646 | Dự báo sai nếu xoay | Cao |
| vx=0 cứng ở RealSpeedSource | scheduler.py:298 | Không dùng encoder hoàn toàn | Cao |
| Thiếu theta_deg, origin_xy config | config.json | Không thể xoay frame | Cao |
| Command speed là scalar | EthernetCom.py:33 | Siemens không nhận vector speed | Trung |
| Không có giá trị A cho adaptive | scheduler.py,config.json | Không tối ưu tốc độ | Trung |

---

## Phần A — Adaptive Conveyor Speed

### A.1. Mô Hình Toán Học Cơ Sở

Mối quan hệ giữa tốc độ băng (v_belt, mm/s) và lượng sản phẩm (N, số lượng) trong vùng thượng nguồn:

$$v_{\text{belt}} = A \times N + v_{\text{min}}$$

**Định nghĩa tham số**:
- **A** (hằng số tốc độ, mm/s/sản phẩm): Mục tiêu là giữ tốc độ dòng chảy sản phẩm qua vùng pickup window ổn định, sao cho robot có thời gian đủ để pick.
- **N** (số sản phẩm đợi, cập nhật từ camera): Đếm số hàng trong thượng nguồn.
- **v_min** (tốc độ tối thiểu, mm/s): Đảm bảo băng không dừng, ví dụ 20 mm/s.

**Ưu điểm**:
- ✅ Mô hình tuyến tính đơn giản, dễ tune và debug.
- ✅ Có bản ghi lịch sử sản phẩm từ camera để suy ra N.

**Nhược điểm**:
- ❌ Không phản ứng trực tiếp với độ dài queue trong pickup window.
- ❌ Độ trễ từ thượng nguồn → hạ nguồn (khoảng 1–2 s) làm A vô ích nếu N thay đổi nhanh.
- ❌ Nếu robot pick chậm, hàng vẫn phát từ thượng nguồn → tắc cứng.

### A.2. Ba Phương Án Điều Chỉnh Tốc Độ

#### **Phương Án 1: Dual-Zone (Khuyến Nghị ⭐)**

**Ý tưởng**: Siemens điều khiển **2 stepper riêng** (thượng nguồn + hạ nguồn), mỗi một có hằng số A riêng.

| Thành Phần | Công Thức | Ghi Chú |
|-----------|-----------|--------|
| **Upstream stepper** | $v_{\text{up}} = A_{\text{up}} \times N_{\text{queue\_up}}$ | Điều khiển lưu lượng từ camera |
| **Downstream stepper** | $v_{\text{down}} = A_{\text{down}} \times N_{\text{queue\_down}}$ | Điều khiển lưu lượng qua pickup window |
| **Sync constraint** | Nếu $N_{\text{up}} \gg N_{\text{down}}$, giảm $A_{\text{up}}$ | Tránh tắc upstream |
| **Fallback** | Nếu không đọc được queue, dùng $v = v_{\text{nominal}}$ | Safety valve |

**Pseudo-code**:
```python
# PC (scheduler.py)
def update_conveyor_speed(camera_detections):
    n_upstream = len([d for d in recent_detections if d.y < -150])  # Camera detection
    n_downstream = self.estimated_queue_downstream()  # Từ vị trí predicted_y
    
    v_up = A_UP * max(1, n_upstream) + V_MIN  # >= 20 mm/s
    v_down = A_DOWN * max(1, n_downstream) + V_MIN
    
    # Cân bằng áp lực
    if n_upstream > 5 and n_downstream < 2:
        v_up = min(v_up * 0.7, V_MAX)  # Giảm upstream
    
    # Gửi command lên Siemens (scalar cho mỗi stepper)
    send_siemens_speed_command(v_up, v_down)

# Siemens (TIA Portal, FB hoặc block)
IF N_Queue_Up > 0 THEN
    Speed_Up := A_UP * N_Queue_Up + MIN_SPEED
ELSE
    Speed_Up := MIN_SPEED
END_IF
```

**Ưu điểm**:
- ✅ Kiểm soát cân bằng lưu lượng giữa 2 vùng.
- ✅ Đơn giản implement (chỉ cần 2 hằng số A).
- ✅ Dễ tune bằng thực tế.

**Nhược điểm**:
- ❌ Phải có thông tin queue từ camera (latency).
- ❌ Giả định Siemens có 2 stepper (kiểm tra phần cứng).

**Trade-off**: **Độ Phức Tạp** vs **Hiệu Năng**
| Điểm | Dual-Zone | PI Control | MPC |
|-----|-----------|-----------|-----|
| Tuning | Đơn giản (2 param) | Trung bình (3 param) | Phức tạp (mô hình) |
| Thích Nghi | Tuy thích (lookup A) | Tốt (closed-loop) | Tốt nhất (predictive) |
| Latency | ~200 ms | ~100 ms | ~50 ms (batch) |
| **Khuyến Nghị** | ⭐⭐⭐ | ⭐⭐ | ⭐ (tương lai) |

---

#### **Phương Án 2: PI Controller (Dành Cho Hạ Nguồn)**

**Ý tưởng**: Dùng encoder feedback Siemens để đo lưu lượng sản phẩm qua pickup window, so sánh với setpoint, điều chỉnh speed bằng PI loop.

| Biến | Định Nghĩa | Đơn Vị |
|-----|----------|-------|
| $N_{\text{target}}$ | Số sản phẩm muốn qua pickup/phút | sản phẩm/phút |
| $N_{\text{actual}}$ | Số sản phẩm thực tế từ encoder | sản phẩm/phút |
| $e(t)$ | Sai số: $N_{\text{target}} - N_{\text{actual}}$ | sản phẩm/phút |
| $v_{\text{belt}}(t)$ | Tốc độ điều khiển | mm/s |

**Công thức PI**:
$$v_{\text{belt}}(t) = K_P \times e(t) + K_I \times \int_0^t e(\tau) d\tau$$

**Pseudo-code**:
```python
# Siemens (hoặc PC)
ERROR := TARGET_RATE - ACTUAL_RATE
INTEGRAL += ERROR * DT
V_BELT := KP * ERROR + KI * INTEGRAL
V_BELT := CLAMP(V_BELT, V_MIN, V_MAX)
```

**Ưu điểm**:
- ✅ Phản ứng trực tiếp với encoder feedback (không giả định queue).
- ✅ Tự điều chỉnh nếu tốc độ robot thay đổi.

**Nhược điểm**:
- ❌ Cần đo chính xác lưu lượng (khó từ encoder pulse).
- ❌ Overshoot / oscillation nếu tune sai KP, KI.
- ❌ Chỉ kiểm soát hạ nguồn; thượng nguồn vẫn cần cơ chế riêng.

---

#### **Phương Án 3: MPC (Model Predictive Control) — Tương Lai**

**Ý tưởng**: Mô phỏng khởi động toàn chuỗi (thượng → hạ, robot pick, dispense) và tối ưu tốc độ để tối đa hóa throughput trên cửa sổ thời gian T.

**Pseudo-code** (sketch):
```python
# PC (Python, ví dụ với scipy.optimize)
def optimize_speed_mpc(state, horizon_s=5.0):
    state = {
        'n_up': queue_upstream,
        'n_down': queue_downstream,
        'robot_position': current_pos,
        'throughput_so_far': completed_picks,
    }
    
    def cost_fn(v_belt):
        # Mô phỏng 5 giây tiếp theo với v_belt
        sim = simulate_5s(state, v_belt)
        return -sim.final_throughput  # Maximize throughput = minimize -throughput
    
    v_optimal = minimize(cost_fn, bounds=(V_MIN, V_MAX))
    return v_optimal
```

**Ưu điểm**:
- ✅ Tối ưu toàn cục trên horizon.
- ✅ Xử lý ràng buộc phức tạp (robot latency, pickup window, ...).

**Nhược điểm**:
- ❌ Mô hình hoá phức tạp, dễ bug.
- ❌ Tính toán CPU cao (~10–100 ms / iteration).
- ❌ Dễ unstable nếu mô hình không đúng thực tế.

**Kết Luận A**: **Chọn Phương Án 1 (Dual-Zone)** để triển khai ngay. PI Controller hoặc MPC để tương lai nâng cấp.

---

### A.3. Cách Suy Ra Hằng Số A

**Mục tiêu**: Tìm A sao cho thông lượng robot tối ưu mà không tắc upstream.

**Giả Định**:
- Robot pick time: $t_{\text{pick}} = 2.5$ s (từ goto + pick trajectory).
- Pickup window dài: $L_{\text{window}} = 130$ mm (theo Y).
- Thời gian sản phẩm qua window: $t_{\text{transit}} = L_{\text{window}} / v_{\text{belt}}$.
- Robot có thể pick cách nhau tối thiểu: $t_{\text{min\_gap}} = 0.5$ s (overlap, parallel motion).

**Công Thức**:
Muốn robot lấy mỗi sản phẩm mất tối đa $t_{\text{pick}}$ với gap $t_{\text{min\_gap}}$:

$$t_{\text{transit}} \geq t_{\text{pick}} - t_{\text{min\_gap}} = 2.5 - 0.5 = 2.0 \text{ s}$$

$$v_{\text{belt}} \leq L_{\text{window}} / t_{\text{transit}} = 130 / 2.0 = 65 \text{ mm/s}$$

Nếu muốn điều chỉnh theo số sản phẩm, định nghĩa **A** như sau:

**Giả sử**:
- Khi $N = 1$ sản phẩm đợi → tốc độ bình thường 40 mm/s.
- Khi $N = 5$ sản phẩm đợi → tốc độ cao 80 mm/s.

$$v_{\text{belt}}(N=1) = A \times 1 + v_{\text{min}} = 40 \Rightarrow A + v_{\text{min}} = 40$$
$$v_{\text{belt}}(N=5) = A \times 5 + v_{\text{min}} = 80 \Rightarrow 5A + v_{\text{min}} = 80$$

Giải hệ:
$$5A + v_{\text{min}} - A - v_{\text{min}} = 80 - 40$$
$$4A = 40 \Rightarrow A = 10 \text{ mm/s/sản phẩm}$$
$$v_{\text{min}} = 40 - 10 = 30 \text{ mm/s}$$

**Bảng Giá Trị Đề Xuất**:
| N (sản phẩm) | v (mm/s) | Ghi Chú |
|-------------|---------|--------|
| 0–1 | 30 | Tối thiểu |
| 2–3 | 50 | Bình thường |
| 4–5 | 70 | Cao |
| ≥6 | 80 | Tối đa, có rủi ro |

---

### A.4. Safety Constraints & Rate Limiting

**Ràng Buộc Tốc Độ**:
```python
# Python (scheduler.py hoặc new module adaptive_speed.py)

V_MIN = 20.0  # mm/s — Tối thiểu để không bị treo
V_MAX = 100.0  # mm/s — Tối đa, encoder Siemens + mechanics
A_UP = 10.0  # mm/s / sản phẩm — upstream
A_DOWN = 15.0  # mm/s / sản phẩm — downstream (hạ nguồn ưu tiên)

DV_DT_MAX = 5.0  # mm/s² — Rate limit, tránh jerk cơ học
HYSTERESIS = 2.0  # mm/s — Tránh oscillate khi N gần ngưỡng
LP_ALPHA = 0.2  # Low-pass filter: v_new = α×v_measured + (1-α)×v_last

def clamp_speed(v_requested, v_last, dt):
    """Clamp tốc độ theo rate limit và hysteresis."""
    # Rate limit
    v_max_change = DV_DT_MAX * dt
    v_clamped = clamp(v_requested, v_last - v_max_change, v_last + v_max_change)
    
    # Hysteresis (if change < hysteresis, keep old speed)
    if abs(v_clamped - v_last) < HYSTERESIS:
        v_clamped = v_last
    
    # Low-pass filter
    v_filtered = LP_ALPHA * v_clamped + (1 - LP_ALPHA) * v_last
    
    # Final clamp to physical limits
    return clamp(v_filtered, V_MIN, V_MAX)
```

---

### A.5. Config JSON — Mở Rộng Phần Mới

Thêm section `conveyor.adaptive_speed` vào `config.json`:

```json
{
  "conveyor": {
    "enabled": true,
    "adaptive_speed": {
      "enabled": true,
      "strategy": "dual_zone",
      "a_upstream_mm_per_product": 10.0,
      "a_downstream_mm_per_product": 15.0,
      "v_min_mm_s": 20.0,
      "v_max_mm_s": 100.0,
      "dv_dt_max_mm_s2": 5.0,
      "hysteresis_mm_s": 2.0,
      "low_pass_alpha": 0.2,
      "queue_detection_upstream_y_threshold": -150.0,
      "queue_detection_downstream_y_threshold": -65.0
    },
    "theta_deg": 0.0,
    "origin_xy": [0.0, 0.0]
  }
}
```

---

## Phần B — Dynamic Conveyor Orientation

### B.1. Biểu Diễn Toán Học

Băng tải xoay góc **θ** từ trục **−X** (hướng phải) theo chiều ngược chiều kim đồng hồ (quy ước toán học).

**Khung Tọa Độ**:
- **Robot Frame**: (X, Y, Z) — tâm tại lỗ quay robot, Z âm hướng xuống.
- **Belt Frame**: (u, v) — u dọc chiều dài băng, v ngang băng.

**Ma Trận Xoay** (từ Belt → Robot):
$$\begin{bmatrix} X \\ Y \end{bmatrix} = \begin{bmatrix} \cos\theta & -\sin\theta \\ \sin\theta & \cos\theta \end{bmatrix} \begin{bmatrix} u \\ v \end{bmatrix}$$

**Ngược Lại** (Robot → Belt):
$$\begin{bmatrix} u \\ v \end{bmatrix} = \begin{bmatrix} \cos\theta & \sin\theta \\ -\sin\theta & \cos\theta \end{bmatrix} \begin{bmatrix} X \\ Y \end{bmatrix}$$

### B.2. Phép Biến Đổi Vận Tốc & Vị Trí

**Encoder Feedback** (từ Siemens):
- Encoder đọc **xung** dọc cơ cấu bánh răng, tính tốc độ scalar: $v_{\text{belt}}$ (mm/s).
- Hướng động: $\vec{d}_{\text{belt}} = (-\cos\theta, -\sin\theta)$ (hướng âm của axis u).

**Vận Tốc Trong Robot Frame**:
$$\vec{v}_{\text{conveyor}} = v_{\text{belt}} \times \vec{d}_{\text{belt}} = v_{\text{belt}} \times (-\cos\theta, -\sin\theta)$$

$$v_x = -v_{\text{belt}} \cos\theta$$
$$v_y = -v_{\text{belt}} \sin\theta$$

**Dự Báo Vị Trí Pickup**:
Khi camera phát hiện sản phẩm tại $(x_{\text{detect}}, y_{\text{detect}})$ tại thời điểm $t_{\text{detect}}$, pickup được dự báo xảy ra tại:

$$\vec{P}_{\text{pick}}(t_{\text{pick}}) = \vec{P}_{\text{detect}} + \vec{v}_{\text{conveyor}} \times (t_{\text{pick}} - t_{\text{detect}})$$

$$\vec{P}_{\text{pick}} = (x_{\text{detect}} + v_x \cdot \Delta t, y_{\text{detect}} + v_y \cdot \Delta t, z_{\text{pickup}})$$

### B.3. Xử Lý Pickup Window Khi Xoay

**Vấn Đề**: pickup_window được định nghĩa trong robot frame. Khi băng xoay, cần kiểm tra xem sản phẩm dự báo có nằm trong window không.

**Option A: Giữ Pickup Window Trong Robot Frame (Khuyến Nghị ⭐)**

```
pickup_window_x = [-120, 50] (mm)
pickup_window_y = [-65, 65] (mm)
```

**Ưu điểm**:
- ✅ Config không thay đổi khi θ thay đổi.
- ✅ Sản phẩm đặt từ phía nào, robot vẫn lấy từ vùng cố định.
- ✅ Dễ kiểm tra bounds trong code (chỉ kiểm tra X, Y của pickup).

**Nhược điểm**:
- ❌ Nếu băng xoay 45°, vùng tương ứng trên băng không phải hình chữ nhật.

**Recommendation**: **Dùng Option A** cho tính đơn giản.

---

### B.4. Pseudo-code Hàm Dự Báo Mới

```python
# modules/scheduler.py — Thay thế hoặc mở rộng _predict_pick_position()

import math

def _predict_pick_position_with_rotation(
    detection: ObjectDetection,
    speed_sample: SpeedSample,
    now: float,
    settings: SchedulerSettings,
    conveyor_theta_deg: float,
) -> tuple[float, float, Position3D] | None:
    """
    Dự báo vị trí pickup với xoay của băng tải.
    
    Args:
        detection: Phát hiện từ camera.
        speed_sample: Mẫu tốc độ từ encoder (đã là vector vx, vy).
        now: Thời gian hiện tại.
        settings: Cấu hình scheduler.
        conveyor_theta_deg: Góc xoay băng (độ, ngược chiều kim).
    
    Returns:
        (t_pick, t_dispatch, position_3d) hoặc None nếu out-of-bounds.
    """
    
    theta_rad = math.radians(conveyor_theta_deg)
    
    # --- Bước 1: Nếu encoder trả về scalar (v_belt) ---
    # Chuyển đổi sang vector (vx, vy) nếu cần
    # (Hiện tại speed_sample đã là vector từ RealSpeedSource hoặc SimulatedSpeedSource)
    
    vx = speed_sample.vx
    vy = speed_sample.vy
    
    # --- Bước 2: Kiểm tra sản phẩm sắp vào window Y ---
    command_delay_s = settings.robot_movement_delay_s + settings.ethernet_delay_s
    guess_pick_time = now + max(settings.intercept_lead_time_s, command_delay_s)
    
    t_enter = detection.timestamp
    # Nếu động học chỉ dọc Y (vx ≈ 0), dùng vy để tính t_enter
    if speed_sample.vy > 0.001 and detection.y < settings.pickup_window_y[0]:
        t_enter = detection.timestamp + (settings.pickup_window_y[0] - detection.y) / speed_sample.vy
        guess_pick_time = max(guess_pick_time, t_enter)
    elif abs(speed_sample.vy) <= 0.001 and abs(speed_sample.vx) > 0.001:
        # Nếu vx lớn hơn, dùng vx (băng xoay ~90°)
        # ...logic tương tự cho trục X
        pass
    
    # --- Bước 3: Vòng lặp iterative để tìm t_pick ---
    predicted_x = detection.x
    predicted_y = detection.y
    for iteration in range(6):
        dt = max(0.0, guess_pick_time - detection.timestamp)
        predicted_x = detection.x + vx * dt
        predicted_y = detection.y + vy * dt
        pick_position = (predicted_x, predicted_y, settings.pickup_height)
        
        # Kiểm tra bounds trong robot frame
        if (
            predicted_y > settings.pickup_window_y[1]
            or predicted_x < settings.pickup_window_x[0]
            or predicted_x > settings.pickup_window_x[1]
        ):
            return None
        
        # Tính thời gian đi từ vị trí hiện tại đến pickup
        goto_points = _build_goto_geometry(
            self.current_position,
            pick_position,
            settings,
        )
        goto_times = _build_goto_timing(
            self.current_position,
            goto_points,
            settings,
        )
        
        new_guess = now + sum(goto_times) + command_delay_s
        new_guess = max(new_guess, t_enter)
        
        if abs(new_guess - guess_pick_time) < 0.01:
            guess_pick_time = new_guess
            break
        guess_pick_time = new_guess
    
    # --- Bước 4: Tính lần cuối & kiểm tra lần nữa ---
    dt = max(0.0, guess_pick_time - detection.timestamp)
    predicted_x = detection.x + vx * dt
    predicted_y = detection.y + vy * dt
    pick_position = (predicted_x, predicted_y, settings.pickup_height)
    
    if not _within_workspace(pick_position, settings):
        return None
    
    pick_dispatch_time = guess_pick_time - command_delay_s
    return guess_pick_time, pick_dispatch_time, pick_position
```

**Lưu Ý**:
- Nếu encoder Siemens **vẫn trả về scalar** (chỉ có `speed`, không có `vx, vy`), cần thêm bước chuyển đổi:
  ```python
  # Trong RealSpeedSource.sample() hoặc scheduler.py.update_speed()
  theta_rad = math.radians(scheduler_settings.conveyor_theta_deg)
  v_belt_scalar = status.get("speed_current", 0.0)
  vx = -v_belt_scalar * math.cos(theta_rad)
  vy = -v_belt_scalar * math.sin(theta_rad)
  return SpeedSample(vx=vx, vy=vy, timestamp=now)
  ```

---

### B.5. Config JSON — Thêm Hỗ Trợ Góc & Origin

```json
{
  "conveyor": {
    "enabled": true,
    "theta_deg": 0.0,
    "origin_xy": [0.0, 0.0],
    "adaptive_speed": {
      "enabled": true,
      "strategy": "dual_zone",
      "a_upstream_mm_per_product": 10.0,
      "a_downstream_mm_per_product": 15.0,
      "v_min_mm_s": 20.0,
      "v_max_mm_s": 100.0,
      "dv_dt_max_mm_s2": 5.0,
      "hysteresis_mm_s": 2.0,
      "low_pass_alpha": 0.2,
      "queue_detection_upstream_y_threshold": -150.0,
      "queue_detection_downstream_y_threshold": -65.0
    }
  }
}
```

**Giải Thích**:
- `theta_deg`: Góc xoay của băng (0 = song song với −X, dương = ngược chiều kim).
- `origin_xy`: Vị trí tâm của băng trong robot frame (dùng cho tương lai nếu cần biến đổi không gian).

---

## Phần C — Checklist Sửa Theo File

### C.1. `modules/config.json`

**Thay Đổi**: Thêm section `conveyor` với tất cả tham số.

**Chỗ Sửa**: Sau `scheduler` block, thêm:
```json
  "conveyor": {
    "enabled": true,
    "theta_deg": 0.0,
    "origin_xy": [0.0, 0.0],
    "adaptive_speed": {
      "enabled": true,
      "strategy": "dual_zone",
      "a_upstream_mm_per_product": 10.0,
      "a_downstream_mm_per_product": 15.0,
      "v_min_mm_s": 20.0,
      "v_max_mm_s": 100.0,
      "dv_dt_max_mm_s2": 5.0,
      "hysteresis_mm_s": 2.0,
      "low_pass_alpha": 0.2,
      "queue_detection_upstream_y_threshold": -150.0,
      "queue_detection_downstream_y_threshold": -65.0
    }
  }
```

---

### C.2. `modules/scheduler.py`

**Thay Đổi 1**: Mở rộng `SchedulerSettings` dataclass (dòng ~117–150)
```python
@dataclass(frozen=True)
class SchedulerSettings:
    # ... existing fields ...
    
    # === Mới: Conveyor Config ===
    conveyor_theta_deg: float  # Góc xoay
    conveyor_origin_xy: tuple[float, float]  # Origin (dự phòng)
    adaptive_speed_enabled: bool
    adaptive_speed_strategy: str  # "dual_zone", "pi", "disabled"
    a_upstream_mm_per_product: float
    a_downstream_mm_per_product: float
    v_min_mm_s: float
    v_max_mm_s: float
    dv_dt_max_mm_s2: float
    hysteresis_mm_s: float
    low_pass_alpha: float
    queue_detection_upstream_y_threshold: float
    queue_detection_downstream_y_threshold: float
```

**Thay Đổi 2**: Sửa `from_config()` classmethod (dòng ~177)
```python
@classmethod
def from_config(cls, config: Any) -> "SchedulerSettings":
    scheduler_raw = getattr(config, "scheduler", {}) or {}
    conveyor_raw = getattr(config, "conveyor", {}) or {}
    
    # ... existing code ...
    
    return cls(
        # ... existing assignments ...
        
        # === Mới ===
        conveyor_theta_deg=float(conveyor_raw.get("theta_deg", 0.0)),
        conveyor_origin_xy=_coerce_vector2d(conveyor_raw.get("origin_xy", [0.0, 0.0]), (0.0, 0.0)),
        adaptive_speed_enabled=bool(conveyor_raw.get("adaptive_speed", {}).get("enabled", True)),
        adaptive_speed_strategy=str(conveyor_raw.get("adaptive_speed", {}).get("strategy", "dual_zone")),
        a_upstream_mm_per_product=float(conveyor_raw.get("adaptive_speed", {}).get("a_upstream_mm_per_product", 10.0)),
        a_downstream_mm_per_product=float(conveyor_raw.get("adaptive_speed", {}).get("a_downstream_mm_per_product", 15.0)),
        v_min_mm_s=float(conveyor_raw.get("adaptive_speed", {}).get("v_min_mm_s", 20.0)),
        v_max_mm_s=float(conveyor_raw.get("adaptive_speed", {}).get("v_max_mm_s", 100.0)),
        dv_dt_max_mm_s2=float(conveyor_raw.get("adaptive_speed", {}).get("dv_dt_max_mm_s2", 5.0)),
        hysteresis_mm_s=float(conveyor_raw.get("adaptive_speed", {}).get("hysteresis_mm_s", 2.0)),
        low_pass_alpha=float(conveyor_raw.get("adaptive_speed", {}).get("low_pass_alpha", 0.2)),
        queue_detection_upstream_y_threshold=float(conveyor_raw.get("adaptive_speed", {}).get("queue_detection_upstream_y_threshold", -150.0)),
        queue_detection_downstream_y_threshold=float(conveyor_raw.get("adaptive_speed", {}).get("queue_detection_downstream_y_threshold", -65.0)),
    )
```

**Thay Đổi 3**: Sửa `_predict_pick_position()` (dòng ~636)

Thay thế:
```python
def _predict_pick_position(
    self,
    detection: ObjectDetection,
    speed_sample: SpeedSample,
    now: float,
) -> tuple[float, float, Position3D] | None:
```

Thành:
```python
def _predict_pick_position(
    self,
    detection: ObjectDetection,
    speed_sample: SpeedSample,
    now: float,
) -> tuple[float, float, Position3D] | None:
    """
    Dự báo vị trí pickup với hỗ trợ xoay băng tải.
    Encoder feedback đã là vector (vx, vy) trong robot frame.
    """
    # ... (toàn bộ logic như pseudo-code B.4 ở trên) ...
```

**Thay Đổi 4**: Thêm helper function (dòng cuối trước class định nghĩa):
```python
def _within_workspace(position: Position3D, settings: SchedulerSettings) -> bool:
    """Kiểm tra vị trí có nằm trong pickup window không."""
    return (
        settings.pickup_window_x[0] <= position[0] <= settings.pickup_window_x[1]
        and settings.pickup_window_y[0] <= position[1] <= settings.pickup_window_y[1]
    )
```

**Thay Đổi 5**: Thêm method cho adaptive speed (dòng ~480 sau `update_speed()`):
```python
def update_adaptive_speed(self, now: float) -> None:
    """
    Cập nhật tốc độ băng tự động dựa trên số lượng hàng đợi.
    Gọi 1 lần/cycle từ main scheduler loop.
    """
    if not self.settings.adaptive_speed_enabled:
        return
    
    # Đếm sản phẩm upstream & downstream từ detection history
    n_upstream = len([d for d in self.recent_detections if d.y < self.settings.queue_detection_upstream_y_threshold])
    n_downstream = len([p for p in self.planned_picks if -65 <= p.predicted_pick_position_2d[1] <= 65])
    
    if self.settings.adaptive_speed_strategy == "dual_zone":
        v_up = self.settings.a_upstream_mm_per_product * max(1, n_upstream) + self.settings.v_min_mm_s
        v_down = self.settings.a_downstream_mm_per_product * max(1, n_downstream) + self.settings.v_min_mm_s
        
        # Cân bằng áp lực
        if n_upstream > 5 and n_downstream < 2:
            v_up = min(v_up * 0.7, self.settings.v_max_mm_s)
        
        v_up = self._clamp_speed(v_up, self.last_speed_up if hasattr(self, 'last_speed_up') else v_up, 0.05)
        v_down = self._clamp_speed(v_down, self.last_speed_down if hasattr(self, 'last_speed_down') else v_down, 0.05)
        
        self.last_speed_up = v_up
        self.last_speed_down = v_down
        
        # Gửi lên Siemens (implementation ở RealRobotExecutor)
        # executor.send_conveyor_speed(v_up, v_down)

def _clamp_speed(self, v_requested: float, v_last: float, dt: float) -> float:
    """Clamp tốc độ theo rate limit, hysteresis, low-pass."""
    dv_dt_max = self.settings.dv_dt_max_mm_s2 * dt
    v_rate_limited = max(v_last - dv_dt_max, min(v_requested, v_last + dv_dt_max))
    
    if abs(v_rate_limited - v_last) < self.settings.hysteresis_mm_s:
        v_rate_limited = v_last
    
    v_filtered = self.settings.low_pass_alpha * v_rate_limited + (1 - self.settings.low_pass_alpha) * v_last
    return max(self.settings.v_min_mm_s, min(v_filtered, self.settings.v_max_mm_s))
```

---

### C.3. `modules/EthernetCom.py`

**Thay Đổi**: Cấu trúc Siemens để hỗ trợ tốc độ dual-zone (nếu Siemens có 2 stepper riêng).

**Option 1** (Nếu Siemens vẫn nhận scalar speed):
```python
# Không thay đổi SiemensSendPacket, vẫn dùng 1 trường "speed"
# PC sẽ phải tính average hoặc dominant speed để gửi
```

**Option 2** (Nếu Siemens hỗ trợ 2 stepper):
```python
class SiemensSendPacket(ctypes.BigEndianStructure):
    _fields_ = [
        ("CommandID", ctypes.c_int32),   # int (4 bytes)
        ("rotate", ctypes.c_float),      # float (4 bytes)
        ("speed_upstream", ctypes.c_float),   # float (4 bytes) — NEW
        ("speed_downstream", ctypes.c_float), # float (4 bytes) — NEW
    ]
    # Total: 16 bytes (cập nhật SIEMENS_DB_WRITE_SIZE)
```

**Lưu Ý**: Phải **đồng bộ với TIA Portal** — nếu thay đổi struct, phải cập nhật DB1 trong PLC cũng.

---

### C.4. `modules/image_processing.py`

**Thay Đổi**: Không cần thay đổi logic chính, nhưng có thể mở rộng để theo dõi lịch sử detection cho adaptive speed.

**Optional Enhancement** (nếu cần đếm queue upstream):
```python
class SimulatedImageProcessing:
    def __init__(self, ...):
        # ... existing ...
        self.detection_history: list[ObjectDetection] = []  # Lưu 10 detection gần nhất
    
    def poll(self, now: float) -> list[ObjectDetection]:
        detections = [...]  # ... existing code ...
        self.detection_history.extend(detections)
        self.detection_history = self.detection_history[-10:]  # Keep last 10
        return detections
    
    def get_upstream_queue_size(self, y_threshold: float) -> int:
        """Đếm detection trong vùng upstream."""
        return len([d for d in self.detection_history if d.y < y_threshold])
```

---

### C.5. `modules/cli.py` (Optional)

**Thay Đổi**: Thêm command để kiểm tra / điều chỉnh tốc độ băng trong mode CLI.

```python
# Thêm vào command parser
if cmd_str.startswith("speed"):
    parts = cmd_str.split()
    if len(parts) >= 2:
        v_belt = float(parts[1])
        # Send to Siemens
        print(f"[CLI] Setting belt speed to {v_belt} mm/s")
        # executor.send_conveyor_speed(v_belt, v_belt)
    else:
        print(f"[CLI] Current belt speed: {self.executor.last_speed_up} mm/s (upstream)")
```

---

## Phần D — 8 Câu Hỏi Xác Nhận Từ User

Trước khi implement, user cần xác nhận:

1. **Siemens Stepper Configuration**: 
   - Hiện tại Siemens có bao nhiêu motor stepper điều khiển băng tải? 1 hay 2?
   - Nếu 2, có thể điều khiển tốc độ độc lập cho mỗi cái không?
   - *Ảnh hưởng*: Quyết định dùng Dual-Zone (2 stepper) hay đơn (1 scalar).

2. **Encoder Feedback Format**:
   - Siemens đọc encoder, tính tốc độ là scalar (mm/s) hay vector (vx, vy)?
   - *Ảnh hưởng*: Nếu scalar, phải thêm bước chuyển đổi dùng θ trong scheduler.

3. **Conveyor Orientation**:
   - Băng tải hiện nằm theo hướng nào? Giả định song song với −Y (θ = 0°)?
   - Nếu xoay, θ = bao nhiêu độ?
   - *Ảnh hưởng*: Config `conveyor.theta_deg` trong config.json.

4. **Adaptive Speed Strategy**:
   - Chọn Phương Án 1 (Dual-Zone ⭐), 2 (PI Control), hay tắt adaptive speed?
   - *Ảnh hưởng*: Độ phức tạp implement & tuning parameters.

5. **Queue Detection Method**:
   - Sẽ dùng camera đếm sản phẩm upstream hay dùng robot position prediction?
   - *Ảnh hưởng*: Cách tính `n_upstream` & `n_downstream` trong `update_adaptive_speed()`.

6. **Safety & Rate Limit**:
   - Có chấp nhận rate limit dv/dt = 5.0 mm/s² không?
   - Giá trị `v_max_mm_s = 100` có phù hợp với specs cơ học?
   - *Ảnh hưởng*: Cập nhật giá trị trong config.json.

7. **Integration Timeline**:
   - Thực hiện Part A trước, Part B sau (vì Part B phụ thuộc Part A)?
   - Hay cả 2 cùng lúc?
   - *Ảnh hưởng*: Lên kế hoạch sprint & test scenarios.

8. **Testing & Validation**:
   - Có test scenario nào trong `modules/test_module.py` để verify adaptive speed không?
   - Hay cần thêm scenario `test_adaptive_speed_responsiveness`?
   - *Ảnh hưởng*: Phạm vi của `run_scheduler_scenario()` mở rộng.

---

## Phần E — Phụ Lục: Công Thức Toán & Tham Số

### E.1. Vận Tốc & Toạ Độ (Rotation Support)

$$\begin{align}
v_x &= -v_{\text{belt}} \times \cos(\theta) \\
v_y &= -v_{\text{belt}} \times \sin(\theta) \\
\end{align}$$

Khi $\theta = 0°$: $v_x = -v_{\text{belt}}, v_y = 0$ (di chuyển theo −X).

Khi $\theta = 90°$: $v_x = 0, v_y = -v_{\text{belt}}$ (di chuyển theo −Y, current setup).

### E.2. Công Thức Adaptive Speed (Dual-Zone)

$$v_{\text{up}}(t) = A_{\text{up}} \times N_{\text{queue\_up}}(t) + v_{\text{min}}$$
$$v_{\text{down}}(t) = A_{\text{down}} \times N_{\text{queue\_down}}(t) + v_{\text{min}}$$

Điều kiện cân bằng:
$$\text{if } N_{\text{up}} > \alpha \times N_{\text{down}} \text{ then } v_{\text{up}} \leftarrow v_{\text{up}} \times \beta$$

Giá trị đề xuất: $\alpha = 5, \beta = 0.7$.

### E.3. Rate Limiting

$$\Delta v_{\text{max}} = \text{dv\_dt\_max} \times \Delta t$$
$$v_{\text{clamped}} = \text{clamp}(v_{\text{requested}}, v_{\text{last}} - \Delta v_{\text{max}}, v_{\text{last}} + \Delta v_{\text{max}})$$

### E.4. Low-Pass Filter (First-Order)

$$v_{\text{filtered}}(t) = \alpha \times v_{\text{measured}}(t) + (1 - \alpha) \times v_{\text{filtered}}(t - \Delta t)$$

Giá trị $\alpha = 0.2$ → thời hằng RC ≈ 4.5 × Δt (khoảng 225 ms nếu Δt = 50 ms).

### E.5. Thời Gian Interception (Iterative Fix-Point)

$$t_{\text{pick}}^{(k+1)} = t_{\text{now}} + \sum_{i=1}^{N} \Delta t_i(\vec{P}_{\text{pick}}^{(k)}) + t_{\text{cmd\_delay}}$$

Hội tụ khi $|t_{\text{pick}}^{(k+1)} - t_{\text{pick}}^{(k)}| < 0.01$ s.

---

## Phần F — Tóm Tắt & Khuyến Nghị

### Tóm Tắt Thiết Kế

| Tính Năng | Phương Pháp | Trạng Thái | File Chính |
|-----------|-----------|-----------|----------|
| **Adaptive Speed** | Dual-Zone (2 hằng số A) | Sẵn sàng implement | `scheduler.py`, `config.json`, `EthernetCom.py` |
| **Dynamic Orientation** | Rotation matrix + vector transform | Sẵn sàng implement | `scheduler.py`, `config.json` |
| **Encoder Feedback** | Đọc từ Siemens, chuyển thành (vx, vy) | Phụ thuộc kiến trúc PLC | `RealSpeedSource` trong `scheduler.py` |

### Khuyến Nghị Thực Hiện

1. **Phase 1** (2–3 ngày): Implement Part A (Adaptive Speed), test trong simulation với `test_throughput`.
2. **Phase 2** (1–2 ngày): Implement Part B (Dynamic Orientation), test `_predict_pick_position()` với góc khác nhau.
3. **Phase 3** (1 ngày): Integration test với real Siemens/Omron, validate encoder feedback.
4. **Phase 4** (1 ngày): Tune hằng số (A_up, A_down, v_min, v_max, dv_dt_max) trên setup vật lý.

### Checklist Pre-Implementation

- [ ] Xác nhận 8 câu hỏi ở Phần D.
- [ ] Cập nhật `config.json` với các tham số mới.
- [ ] Sửa `SchedulerSettings.from_config()` để load conveyor config.
- [ ] Implement `_predict_pick_position()` với rotation support.
- [ ] Thêm `update_adaptive_speed()` method.
- [ ] Sửa `RealSpeedSource.sample()` để transform encoder scalar → vector.
- [ ] Test compile: `python3 -m py_compile modules/scheduler.py`.
- [ ] Run scenario: `python3 main.py --scheduler --scenario test_throughput --duration 1.0 --simulate-executor`.
- [ ] Validate pickup window bounds với `_within_workspace()`.

---

## Tham Khảo & Mở Rộng Tương Lai

### Các Cải Tiến Tương Lai
1. **PI Control** cho lưu lượng (Phương Án 2).
2. **MPC** tối ưu hóa toàn chuỗi (Phương Án 3).
3. **Vision-based lane detection** để self-calibrate θ.
4. **Multi-product tracking** để tránh va chạm.

### Tài Liệu Liên Quan
- `doc/system_reference.md` — Kiến trúc phần cứng chi tiết.
- `doc/plc_programing.md` — Cấu hình Siemens byte order.
- `doc/ai_context.md` — Command mapping & PLC data contract.
- `modules/scheduler.py` — Scheduler loop chính.
- `modules/EthernetCom.py` — Communication gateway.

---

**Phiên Bản**: 1.0 | **Ngày**: 2026-06-03 | **Tác Giả**: AI Code Agent
