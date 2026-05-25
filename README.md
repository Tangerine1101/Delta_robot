# Delta Robot Pick-and-Place Project

This repository contains Python-side control tooling for a Delta Robot sorting system communicating with PLCs (Omron NX1P2 and Siemens S7-1200) over Ethernet. It supports an interactive CLI mode for direct hardware commands and an offline scheduler simulation/benchmark tool.

---

## 1. Quickstart & Usage

### 1.1. Setup & Environment
Ensure you have the required packages installed. Pylogix is used for communicating with the Omron PLC.
```bash
pip install pylogix
```
Settings are loaded from `modules/config.json`. Check `ip_address`, `port`, and `scheduler` geometry values before executing commands on real hardware.

### 1.2. Run the Offline Scheduler Simulation
Simulates the pick scheduler, simulated object detections, and conveyor speed streams without hitting physical hardware:
```bash
# Run throughput scenario
python3 main.py --scheduler --scenario test_throughput --duration 10.0 --simulate-executor

# Run accuracy tracking scenario
python3 main.py --scheduler --scenario test_accuracy --duration 5.0 --simulate-executor
```

### 1.3. Run the Fake PLC TCP Server
Useful to test Python communication interfaces and telemetry log output without real controllers:
```bash
python3 -m modules.test_module --port 1502 --self-test --duration 1.0
```

### 1.4. Run the Real CLI or Auto-Scheduler
Execute these commands once connected to real PLCs:
```bash
# Start interactive CLI mode
python3 main.py --cli

# Run auto-scheduler with Omron RealRobotExecutor
python3 main.py --scheduler --scenario test_throughput
```

---

## 2. Basic Logic & Architecture

```
                  ┌──────────────────────┐
                  │      main.py         │
                  └──────────┬───────────┘
            ┌────────────────┴────────────────┐
            ▼                                 ▼
   ┌─────────────────┐               ┌─────────────────┐
   │ CLI Interactive │               │  Scheduler Loop │
   │   (cli.py)      │               │ (scheduler.py)  │
   └────────┬────────┘               └────────┬────────┘
            ▼                                 ▼
   ┌──────────────────────────────────────────────────┐
   │             EthernetCom.py (Gateway)             │
   └────────────────────────┬─────────────────────────┘
                            ▼
   ┌──────────────────────────────────────────────────┐
   │                PLC Hardware Layer                │
   └──────────────────────────────────────────────────┘
```

* **Threading Model**: 
  - Main Process: CLI Parser / Auto-Scheduler planning loop.
  - Worker Process (`multiprocessing` queue): PLCGateway communication to eliminate network latency blocking.
* **PLC Package Contract**: Fixed 4-slot coordinate arrays sent to the `pc_package` tag on the Omron PLC. Unused elements are zero-padded.
* **Interception Math**: Predicts conveyor interception using the object's initial position, dynamic 2D speed vector `[vx, vy]`, and a fixed-point iteration search. The default simulated conveyor moves along positive Y while X stays fixed per lane.
* **4-Point/2-Phase Trajectory**: Moves in a `goto` phase followed by a `pick` phase. `B_goto -> C_goto` and `B_pick -> C_pick` are mandatory 3D slope segments, not flat-then-vertical moves.
* **Timing Compensation**: Command is dispatched ahead of interception to account for mechanics and communication:
  $$t_{\text{dispatch}} = t_{\text{pick}} - t_{\text{robot\_movement\_delay}} - t_{\text{ethernet\_delay}}$$

---

## 3. Configuration Variables (Giải thích các biến cấu hình)

Các cấu hình hệ thống được lưu trữ trong tệp [config.json](file:///home/tangerine/Share/Global%20Share/Documents/Delta_robot/modules/config.json). Dưới đây là giải thích chi tiết cho từng tham số:

### 3.1. Cấu hình Kết nối & PLC (Connection & PLC Settings)
* `ip_address` (string): Địa chỉ IP của PLC Omron NX1P2 (mặc định: `192.168.250.1`).
* `port` (int): Cổng TCP kết nối với PLC Omron (mặc định: `44818` cho EtherNet/IP).
* `siemens_ip` (string): Địa chỉ IP của PLC Siemens S7-1200 (mặc định: `192.168.250.2`).
* `siemens_port` (int): Cổng TCP kết nối với PLC Siemens (mặc định: `1502`).
* `period_s` (float): Chu kỳ lấy mẫu trạng thái hoặc chu kỳ cập nhật dữ liệu (giây).
* `interpolar_points` (int): Số lượng điểm quỹ đạo tối đa được gửi trong gói tin PC (mặc định: `4` điểm).
* `object_types` (object): Bản đồ ánh xạ giữa định danh loại vật thể (ví dụ: `object_A`) với tên khay phân loại tương ứng.
* `object_A` (array): Tọa độ 3D `[x, y, z]` (mm) của khay phân loại dành cho sản phẩm `object_A`.

### 3.2. Cấu hình Bộ lập lịch & Hình học Robot (`scheduler` block)
* `home_position` (array): Tọa độ 3D `[x, y, z]` của vị trí nghỉ mặc định (Home) của robot Delta.
* `clearance_height` (float): Độ cao an toàn (tọa độ Z âm) để di chuyển ngang giữa các khay và băng tải (ví dụ: `-290.0`).
* `slope_transition_height` (float): Độ cao bắt đầu chuyển tiếp quỹ đạo nghiêng 3D để tối ưu chuyển động cơ học (ví dụ: `-295.0`).
* `pickup_height` (float): Độ cao gắp vật thể trên băng tải (ví dụ: `-310.0`).
* `pre_pick_height` (float): Độ cao tiếp cận phía trên vật thể trước khi thực hiện cú gắp (ví dụ: `-300.0`).
* `place_height` (float): Độ cao thả vật thể tại khay phân loại (ví dụ: `-290.0`).
* `corner_blend_xy` (float): Bán kính vê góc (blend) tại các góc quỹ đạo để robot di chuyển mượt mà hơn.
* `intercept_lead_time_s` (float): Khoảng thời gian ước lượng tối thiểu ban đầu để tính điểm hội tụ gắp vật thể.
* `release_descent_time_s` (float): Thời gian chờ xả giác hút tại điểm thả (giây).
* `nominal_xy_speed` (float): Tốc độ di chuyển định mức của robot theo phương ngang XY (mm/s).
* `nominal_z_speed` (float): Tốc độ di chuyển định mức của robot theo phương đứng Z (mm/s).
* `stale_timeout_s` (float): Thời gian tối đa để lưu vết vật thể trước khi loại bỏ khỏi hàng đợi (giây).
* `speed_timeout_s` (float): Thời gian hết hạn của dữ liệu tốc độ băng tải nếu không nhận được mẫu mới (giây).
* `poll_interval_s` (float): Tần suất lặp lại chu kỳ lập lịch (giây).
* `default_speed` (array): Vector vận tốc mặc định của băng tải `[vx, vy]` (mm/s) khi chạy mô phỏng hoặc mất kết nối PLC.
* `robot_movement_delay_s` (float): Độ trễ phản hồi cơ học và gia tốc thực tế của robot (giây).
* `ethernet_delay_s` (float): Độ trễ truyền thông mạng Ethernet một chiều (giây).
* `pickup_window_x` (array): Phạm vi giới hạn trục X `[xmin, xmax]` của vùng gắp vật thể.
* `pickup_window_y` (array): Phạm vi giới hạn trục Y `[ymin, ymax]` của vùng gắp vật thể.
* `throughput_object_types` (array): Danh sách các loại vật thể sinh ra trong kịch bản mô phỏng Throughput.
* `throughput_lanes` (array): Tọa độ trục X của các làn băng tải nơi sản phẩm được sinh ra.
* `throughput_spawn_x` & `throughput_spawn_y` (float): Điểm xuất phát của sản phẩm giả lập ở thượng nguồn băng tải.
* `throughput_emit_interval_s` (float): Khoảng thời gian sinh vật thể tuần tự trong kịch bản Throughput.
* `accuracy_emit_interval_s` (float): Khoảng thời gian sinh vật thể trong kịch bản kiểm tra Accuracy.
* `execution_margin_s` (float): Khoảng thời gian an toàn cộng thêm trước khi hết hạn lệnh quỹ đạo.
* `accuracy_points` (array): Danh sách các tọa độ điểm tĩnh dùng để kiểm tra sai số bám quỹ đạo.
* `log_path` (string): Đường dẫn lưu trữ dữ liệu log quỹ đạo bám vật thể.

---

## 4. Documentation Index

Detailed documentation files are available in the `doc/` directory:
* [system_reference.md](file:///home/tangerine/Share/Global%20Share/Documents/Delta_robot/doc/system_reference.md): Detailed specifications, coordinate constraints, trajectory math formulas, and code logic descriptions.
* [human_ideas.md](file:///home/tangerine/Share/Global%20Share/Documents/Delta_robot/doc/human_ideas.md): Human research notes, academic thesis topics, database schemas, and future ideas (AI should avoid editing this file).
* [ai_context.md](file:///home/tangerine/Share/Global%20Share/Documents/Delta_robot/doc/ai_context.md): Compact summary of codebase facts, command maps, and verification scripts for quick AI context updates.

---

## 5. Updates & Roadmap

### Recent Updates (23/5)
* **4 DOF and Siemens PLC Integration**: Added support for 4th degree of freedom (end-effector suction rotation via stepper) and conveyor speed adjustments handled by a secondary Siemens S7-1200 PLC. Defined new command IDs: `rotate_absolute` (7), `change_speed` (8), and `plan_siemen` (9).
* **2D Speed Vectors**: Updated conveyor speed calculations from a scalar speed to a 2D velocity vector `[vx, vy]` in `modules/scheduler.py` and `modules/config.json`.
* **Config Safety Constraints**: Added automated safety verification in `modules/scheduler.py` to assert:
  $$\text{clearance\_height} > \text{slope\_transition\_height} > \text{pre\_pick\_height} > \text{pickup\_height}$$

### Future Roadmap
1. **Profile Smoothing**: Add jerk/acceleration-limited profiles on top of the mandatory 3D slope waypoints.
2. **Calibration Utility**: Fix and integrate `modules/calibration.py` to auto-profile Ethernet round-trip latency and mechanical movement delays.
3. **Vision Integration**: Connect simulated perception queues to real camera segmentation streams.

---

## 6. Known Bugs & Limitations (Các lỗi đã biết & Hạn chế)

### 6.1. Lỗi logic & Thuật toán
1. **Lỗi thoát sớm trong dự đoán vị trí gắp (`_predict_pick_position` trong `scheduler.py`)** - **[ĐÃ KHẮC PHỤC]**:
   - *Chi tiết*: Phép kiểm tra vùng làm việc nằm bên trong vòng lặp tìm điểm hội tụ. Đã được sửa bằng cách giới hạn chu kỳ hội tụ dựa trên thời điểm vật đi vào vùng gắp `t_enter`.
2. **Thời gian phân đoạn gắp/thả bị gán cứng (`_build_pick_timing` trong `scheduler.py`)** - **[ĐÃ KHẮC PHỤC]**:
   - *Chi tiết*: Phân đoạn thời gian gắp/thả đã được tính toán lại động theo quãng đường thực tế di chuyển chéo (`blend`) và tốc độ định mức.
3. **Rò rỉ bộ nhớ (Memory Leak) trong bộ lập lịch** - **[ĐÃ KHẮC PHỤC]**:
   - *Chi tiết*: Tập hợp `self.seen_object_ids` lưu trữ vô hạn đã được chuyển thành kiểu từ điển và dọn dẹp định kỳ các ID cũ hơn `stale_timeout_s`.
4. **Chỉ số thống kê bị bỏ sót** - **[ĐÃ KHẮC PHỤC]**:
   - *Chi tiết*: Bộ đếm `skipped_outside_workspace` đã được bổ sung và tăng lên chính xác khi vật trôi qua khỏi giới hạn dưới của vùng làm việc.

### 6.2. Hạn chế tích hợp PLC & Giả lập
1. **Mảng `argument_time` vô dụng trên thực tế**:
   - *Chi tiết*: Code PLC hiện tại chưa quy hoạch thời gian thực hiện quỹ đạo, tốc độ chuyển động thực tế của robot không bị ảnh hưởng bởi tham số thời gian gửi từ PC.
2. **Lệnh di chuyển tương đối chưa được hỗ trợ**:
   - *Chi tiết*: Mã lệnh `goto_relative` (ID = 1) chưa được lập trình dưới PLC thực tế.
3. **Lỗi kiểm thử hiệu chuẩn cơ học (`calibration.py`)**:
   - *Chi tiết*: Hàm hiệu chuẩn mechanical delay gửi lệnh `stop` (ID = 0) và đợi phản hồi `task_doing == 1`. Tuy nhiên, mock PLC cập nhật `task_doing` bằng chính ID lệnh (là 0 cho lệnh `stop`). Do đó, bước hiệu chuẩn này luôn bị timeout.
5. **Gán cứng đường dẫn biểu đồ trong `run_test.py`**:
   - *Chi tiết*: Đường dẫn lưu trữ biểu đồ của `generate_plots()` bị chỉ định cố định vào một phiên làm việc cũ không tồn tại hoặc sai quyền ghi.
