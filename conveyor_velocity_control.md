# Điều khiển vận tốc băng tải qua EtherCAT (Omron NX1P2)

Băng tải chuyển từ S7-1200 (PTO + encoder HSC) sang trục servo EtherCAT `MC_Conveyor`
(Axis 3, MC1 – Primary periodic task 4 ms, Output device: Node 2 InoSV660N (E004)).

- PC gửi vận tốc mong muốn (mm/s, ≥ 0) bằng **commandID = 8** qua `pc_package`.
- Băng tải **chỉ chạy 1 chiều**, là chiều **âm** của servo; chiều này được cố định trong code PLC.
- Trục được cài đơn vị **mm**, nên PLC đưa thẳng giá trị mm/s vào `MC_MoveVelocity`
  (hoặc dùng `MC_Stop` khi vận tốc ≈ 0). Không cần hàm quy đổi hồi quy như bên Siemens.
- PLC trả về `MC_Conveyor.Act.Vel` (mm/s) và `MC_Conveyor.Act.Pos` (mm) qua `plc_package`.

---

## 1. Hiệu chuẩn Unit Conversion (đo mm/vòng motor)

Servo chạy vòng kín nên motor không mất bước. Vận tốc băng tải tỉ lệ thuận với tốc độ motor và đi qua gốc tọa độ:

$$v_{băng\,tải}\ [\text{mm/s}] = n\ [\text{vòng/s}] \times L\ [\text{mm/vòng}]$$

Chỉ cần đo đúng một hệ số $L$. Hệ số này đã bao gồm đường kính puly, độ dày dây đai và độ căng.

### 1.1 Cách đo $L$

1. Giữ Unit Conversion ở đơn vị **pulse** (100000 pulse/vòng motor, như `axis_setting3.png`).
2. Đánh dấu một điểm trên dây đai, chạy `MC_MoveRelative` đúng **N vòng**, ví dụ
   `Distance := 2000000` (20 vòng), tốc độ thấp (ví dụ `Velocity := 100000` = 1 vòng/s).
3. Đo quãng đường điểm đánh dấu dịch chuyển bằng thước: $S$ (mm).
4. $L = S / N$. Lặp lại 3 lần rồi lấy trung bình. Nên đo khi băng tải có độ căng và tải giống lúc vận hành.

Sai số của $L$ bằng sai số đo chia cho N. Ví dụ đo lệch ±1 mm trên 20 vòng (~2000 mm) thì $L$ chỉ sai ≈ 0.05 %.

### 1.2 Nạp vào Sysmac Studio (Unit Conversion Settings)

| Ô cài đặt | Giá trị |
| :--- | :--- |
| Unit of display | **mm** |
| Command pulse count per motor rotation | 8388608 (giữ nguyên, encoder 23-bit) |
| Gearbox | **Do not use gearbox** |
| Work travel distance per motor rotation | **$L$** (mm/rev, giá trị vừa đo, giữ phần thập phân) |

### 1.3 Sửa lại Operation Settings (các ô đang tính theo pulse sẽ tự đổi thành mm)

| Ô cài đặt | Đang để (pulse) | Sửa thành (mm) |
| :--- | :--- | :--- |
| Maximum velocity | 3000000 | $30 \times L$ mm/s (= 1800 rpm) |
| Maximum jog velocity | 1000000 | ví dụ 100 mm/s |
| Maximum acceleration / deceleration | 0 | giữ 0 (gia tốc do code quyết định) |
| In-position range | 10 | 0.1 mm (để nguyên sẽ thành 10 mm) |
| Zero position range | 10 | 0.1 mm |
| Actual velocity filter time constant | 0 ms | **20 ms** (làm mượt `Act.Vel`, vốn là vi phân vị trí mỗi 4 ms) |

Sau khi sửa: **Transfer to Controller** rồi khởi động lại PLC.

### 1.4 Kiểm tra lại

Gửi `setspeed 50` từ PC, đo thời gian băng tải đi hết một đoạn 500 mm → phải ≈ 10 s.
Lặp lại ở 2–3 mức tốc độ. Nếu vận tốc lệch, nguyên nhân là dây đai trượt trên puly
(servo không đo được hiện tượng này), không phải do $L$ hay servo.

---

## 2. Sửa Data Type trong Sysmac Studio

Thêm member vào **cuối** 2 struct đang dùng cho `pc_package` / `plc_package`
(member phải được Network Publish như các member cũ để pylogix đọc/ghi được):

| Struct | Member mới | Kiểu | Ý nghĩa |
| :--- | :--- | :--- | :--- |
| `pc_package` | `conveyor_speed` | `REAL` | Vận tốc đặt (mm/s). Dương = chiều băng tải chạy xuôi |
| `plc_package` | `conveyor_velocity` | `REAL` | Vận tốc thực `Act.Vel` (mm/s) |
| `plc_package` | `conveyor_position` | `REAL` | Vị trí thực `Act.Pos` (mm) |
| `plc_package` | `conveyor_state` | `INT` | 0 = Đứng yên, 1 = Đang tăng/giảm tốc, 2 = Đạt vận tốc, 3 = Lỗi, 4 = Servo OFF |

> Lệnh 8 **không** ghi vào `task_doing` / `task_state`, vì hai biến này đang được Python
> dùng để chờ robot chạy xong quỹ đạo. Nếu đổi tốc độ băng tải giữa lúc robot đang
> chạy lệnh 3 mà ghi đè `task_doing := 8` thì Python sẽ hiểu sai trạng thái robot.
> Trạng thái băng tải được báo riêng qua `conveyor_state`.

---

## 3. Biến nội bộ (Internal Variables của Program0)

| Tên | Kiểu | Giá trị đầu | Ghi chú |
| :--- | :--- | :--- | :--- |
| `MC_Power_Conv` | `MC_Power` | | |
| `MC_Reset_Conv` | `MC_Reset` | | |
| `MC_MoveVelocity_Conv` | `MC_MoveVelocity` | | |
| `MC_Stop_Conv` | `MC_Stop` | | |
| `Conv_Power_En` | `BOOL` | FALSE | Nối với cờ Servo ON đang dùng cho 3 trục robot |
| `Conv_Reset_Req` | `BOOL` | FALSE | Nối với nút Reset lỗi đang dùng |
| `Conv_Target_Vel` | `LREAL` | 0.0 | Vận tốc PC gửi xuống (mm/s, ≥ 0; giá trị âm bị ép về 0) |
| `Conv_Cmd_Vel` | `LREAL` | 0.0 | Độ lớn vận tốc đưa vào `MC_MoveVelocity` (mm/s) |
| `Conv_Cmd_New` | `BOOL` | FALSE | Cờ báo có lệnh 8 mới (set ở Section0) |
| `Conv_Mv_Exe` | `BOOL` | FALSE | Execute của `MC_MoveVelocity` |
| `Conv_Mv_Pending` | `BOOL` | FALSE | Chờ 1 chu kỳ để tạo sườn lên cho Execute |
| `Conv_Stop_Exe` | `BOOL` | FALSE | Execute của `MC_Stop` |
| `Conv_State` | `INT` | 4 | |
| `Conv_Act_Vel` | `LREAL` | 0.0 | |
| `Conv_Act_Pos` | `LREAL` | 0.0 | |
| `Conv_Vel_Max` | `LREAL` (Constant) | 300.0 | Giới hạn vận tốc (mm/s), phải ≤ Maximum velocity của trục |
| `Conv_Vel_Deadband` | `LREAL` (Constant) | 0.5 | Nhỏ hơn giá trị này → dừng hẳn bằng `MC_Stop` (mm/s) |
| `Conv_Acc` | `LREAL` (Constant) | 500.0 | Gia tốc (mm/s²) |
| `Conv_Dec` | `LREAL` (Constant) | 500.0 | Giảm tốc (mm/s²) |

> Chiều chạy được **cố định** trong code: băng tải chỉ chạy theo chiều **âm** của servo
> (`_mcNegativeDirection`), nên không có biến chọn chiều.

---

## 4. Thêm lệnh 8 vào Section0 (PC_Receive and Dispatch)

Chèn vào trong `CASE pc_package.commandID OF ... END_CASE;` của Rung 4:

```pascal
    8:  // Lệnh: CONVEYOR SPEED (mm/s, >= 0; băng tải chỉ chạy 1 chiều)
        Conv_Target_Vel := REAL_TO_LREAL(pc_package.conveyor_speed);
        Conv_Cmd_New := TRUE;           // Section_Conveyor sẽ xử lý và tự xóa cờ
        pc_package.commandID := -1;     // Xóa lệnh (Handshake)
        // Không ghi task_doing / task_state để không đè trạng thái của robot
```

---

## 5. Rung mới: Section_Conveyor (ST inline)

Đặt rung này **sau** Section0 (Rung 4) và **trước** Section4 (Telemetry).

```pascal
//Main_Section_Conveyor
// Nhiệm vụ: Bật servo băng tải, chạy MC_MoveVelocity theo vận tốc PC gửi (mm/s),
//           dừng bằng MC_Stop khi vận tốc ~ 0, báo trạng thái về PC.
// Trục MC_Conveyor cài đơn vị mm (Work travel distance = L mm/vòng đo thực nghiệm)
// nên vận tốc PC gửi được đưa thẳng vào MC_MoveVelocity, không cần quy đổi.
// Băng tải CHỈ chạy theo chiều ÂM của servo (_mcNegativeDirection), không bao giờ đảo chiều.

// =========================================================================
// PHẦN 1: SERVO ON & RESET LỖI
// =========================================================================
MC_Power_Conv(
    Axis   := MC_Conveyor,
    Enable := Conv_Power_En
);

MC_Reset_Conv(
    Axis    := MC_Conveyor,
    Execute := Conv_Reset_Req
);

// Servo OFF thì hủy mọi lệnh đang chờ
IF NOT MC_Power_Conv.Status THEN
    Conv_Mv_Exe     := FALSE;
    Conv_Mv_Pending := FALSE;
    Conv_Stop_Exe   := FALSE;
END_IF;

// =========================================================================
// PHẦN 2: NHẬN LỆNH MỚI TỪ PC
// =========================================================================
IF Conv_Cmd_New THEN
    Conv_Cmd_New := FALSE;

    // Kẹp giới hạn vận tốc. Băng tải chỉ chạy 1 chiều:
    // lệnh âm bị ép về 0 -> dừng, không bao giờ quay theo chiều dương của servo.
    IF Conv_Target_Vel > Conv_Vel_Max THEN
        Conv_Target_Vel := Conv_Vel_Max;
    ELSIF Conv_Target_Vel < 0.0 THEN
        Conv_Target_Vel := 0.0;
    END_IF;

    IF Conv_Target_Vel < Conv_Vel_Deadband THEN
        // Vận tốc ~ 0: dừng hẳn bằng MC_Stop
        Conv_Mv_Exe     := FALSE;
        Conv_Mv_Pending := FALSE;
        Conv_Stop_Exe   := TRUE;
    ELSE
        Conv_Cmd_Vel := Conv_Target_Vel;   // mm/s, đưa thẳng vào MC_MoveVelocity

        // Hạ Execute 1 chu kỳ để lần sau tạo sườn lên -> MC_MoveVelocity nhận vận tốc mới
        // (BufferMode = Aborting nên đổi tốc độ ngay cả khi đang chạy)
        Conv_Mv_Exe     := FALSE;
        Conv_Stop_Exe   := FALSE;
        Conv_Mv_Pending := TRUE;
    END_IF;

// Chu kỳ sau: bật lại Execute, nhưng phải đợi MC_Stop giảm tốc xong (nếu có)
ELSIF Conv_Mv_Pending AND NOT MC_Stop_Conv.Busy AND MC_Power_Conv.Status THEN
    Conv_Mv_Exe     := TRUE;
    Conv_Mv_Pending := FALSE;
END_IF;

// =========================================================================
// PHẦN 3: GỌI KHỐI CHUYỂN ĐỘNG
// =========================================================================
MC_MoveVelocity_Conv(
    Axis         := MC_Conveyor,
    Execute      := Conv_Mv_Exe,
    Velocity     := Conv_Cmd_Vel,      // mm/s
    Acceleration := Conv_Acc,          // mm/s^2
    Deceleration := Conv_Dec,          // mm/s^2
    Jerk         := 0.0,
    Direction    := _mcNegativeDirection,  // CỐ ĐỊNH: chỉ chạy chiều âm của servo
    BufferMode   := _mcAborting
);

MC_Stop_Conv(
    Axis         := MC_Conveyor,
    Execute      := Conv_Stop_Exe,
    Deceleration := Conv_Dec,
    Jerk         := 0.0
);

// Dừng xong thì nhả Execute của MC_Stop, nếu không trục bị khóa ở trạng thái Stopping
IF MC_Stop_Conv.Done OR MC_Stop_Conv.Error THEN
    Conv_Stop_Exe := FALSE;
END_IF;

// =========================================================================
// PHẦN 4: TRẠNG THÁI BĂNG TẢI
// 0 = Đứng yên, 1 = Đang tăng/giảm tốc, 2 = Đạt vận tốc, 3 = Lỗi, 4 = Servo OFF
// =========================================================================
IF MC_Power_Conv.Error OR MC_MoveVelocity_Conv.Error OR MC_Stop_Conv.Error
   OR MC_Conveyor.MFaultLvl.Active THEN
    Conv_State := 3;
ELSIF NOT MC_Power_Conv.Status THEN
    Conv_State := 4;
ELSIF Conv_Mv_Pending OR MC_Stop_Conv.Busy
   OR (MC_MoveVelocity_Conv.Busy AND NOT MC_MoveVelocity_Conv.InVelocity) THEN
    Conv_State := 1;
ELSIF MC_MoveVelocity_Conv.InVelocity THEN
    Conv_State := 2;
ELSE
    Conv_State := 0;
END_IF;
```

---

## 6. Thêm vào Section4 (Telemetry)

Nối vào cuối Section4 hiện tại:

```pascal
// 4. Băng tải: vận tốc & vị trí thực đọc từ encoder servo (qua EtherCAT)
//    Trục cài đơn vị mm nên Act.Vel đã là mm/s, Act.Pos đã là mm
//    Servo chạy chiều âm nên Act.Vel < 0 và Act.Pos giảm dần.
//    Đổi dấu để PC thấy vận tốc dương và vị trí tăng dần theo chiều băng tải chạy.
Conv_Act_Vel := -MC_Conveyor.Act.Vel;   // mm/s
Conv_Act_Pos := -MC_Conveyor.Act.Pos;   // mm

plc_package.conveyor_velocity := LREAL_TO_REAL(Conv_Act_Vel);
plc_package.conveyor_position := LREAL_TO_REAL(Conv_Act_Pos);
plc_package.conveyor_state    := Conv_State;
```

---

## 7. Lưu ý

1. **Trượt dây đai** là nguồn sai số duy nhất còn lại: servo chỉ biết vòng quay motor.
   Với tải nhẹ (PCB) thì gần như không đáng kể.
2. **Count mode Linear**: `Act.Pos` tăng mãi khi băng tải chạy. Kiểu `REAL` vẫn chính xác
   dưới 0.1 mm tới khoảng 1 000 000 mm (~3 giờ ở 100 mm/s). Python chỉ dùng hiệu vị trí,
   nên có thể reset về 0 bằng `MC_SetPosition` mỗi lần khởi động nếu cần.
3. Thanh ghi vị trí 0x6064 của servo InoSV660N là 32-bit (±2³¹ pulse encoder ≈ 256 vòng motor).
   Motion Control của NX1P2 tự mở rộng khi tràn, nên không cần xử lý trong code.
4. Nếu sau này đo lại $L$, chỉ cần sửa ô Work travel distance trong Sysmac, không phải sửa code PLC hay Python.
5. **Băng tải chỉ chạy 1 chiều = chiều âm của servo** (cố định trong code):
   - PC gửi vận tốc dương v, PLC chạy `MC_MoveVelocity` với `Direction := _mcNegativeDirection`.
   - Lệnh âm từ PC bị ép về 0 (dừng) ở PHẦN 2. Python `setspeed` cũng từ chối số âm.
   - `Act.Vel` / `Act.Pos` (âm theo servo) luôn được đổi dấu ở Section4, nên PC thấy vận tốc
     dương và vị trí tăng dần.
   - Khi đo $L$ ở mục 1.1, chạy `MC_MoveRelative` với `Distance` **âm** (ví dụ `-2000000`),
     $L$ vẫn lấy giá trị dương.
   - **Không** đảo chiều quay trong tham số của servo InoSV660N. Nếu đảo, băng tải sẽ chạy ngược.

---

## 8. Phía Python (đã sửa)

| File | Thay đổi |
| :--- | :--- |
| `modules/EthernetCom.py` | `pc_package.conveyor_speed` được ghi từ key `speed`; đọc thêm `conveyor_velocity`, `conveyor_position`, `conveyor_state`; `SiemensGateway` bỏ `speed_current`, `conveyor_position` |
| `modules/scheduler.py` | `ConveyorSpeedSource` dùng `conveyor_velocity` từ PLC thay cho đạo hàm vị trí |
| `modules/cli.py` | `plan_siemen` tách thành lệnh 7 (Siemens) + lệnh 8 (Omron) |
| `modules/test_module.py` | Mock: lệnh 8 xử lý ở nhánh Omron, status Omron trả thêm dữ liệu băng tải |
| `main.py` (không có trong workspace) | **Tự sửa**: route lệnh 8 sang `PLCGateway` (Omron); khi gộp status, lấy dữ liệu băng tải từ Omron |
