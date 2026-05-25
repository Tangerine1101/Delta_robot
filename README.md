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

## 3. Documentation Index

Detailed documentation files are available in the `doc/` directory:
* [system_reference.md](file:///home/tangerine/Share/Global%20Share/Documents/Delta_robot/doc/system_reference.md): Detailed specifications, coordinate constraints, trajectory math formulas, and code logic descriptions.
* [human_ideas.md](file:///home/tangerine/Share/Global%20Share/Documents/Delta_robot/doc/human_ideas.md): Human research notes, academic thesis topics, database schemas, and future ideas (AI should avoid editing this file).
* [ai_context.md](file:///home/tangerine/Share/Global%20Share/Documents/Delta_robot/doc/ai_context.md): Compact summary of codebase facts, command maps, and verification scripts for quick AI context updates.

---

## 4. Updates & Roadmap

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

## 5. Known Bugs & Limitations (Các lỗi đã biết & Hạn chế)

Dưới đây là danh sách các lỗi và hạn chế đã được xác định trong hệ thống, bao gồm cả các thông tin thực tế từ PLC:

### 5.1. Lỗi logic & Thuật toán
1. **Lỗi thoát sớm trong dự đoán vị trí gắp (`_predict_pick_position` trong `scheduler.py`)**:
   - *Chi tiết*: Phép kiểm tra vùng làm việc `if not self._within_workspace(pick_position)` nằm bên trong vòng lặp tìm điểm hội tụ. Khi vật thể ở quá xa phía thượng nguồn băng tải (ngoài vùng làm việc lúc bắt đầu tìm kiếm), hàm sẽ trả về `None` ngay lập tức thay vì tiếp tục lặp để tìm điểm đón vật khi nó đi vào vùng gắp.
2. **Thời gian phân đoạn gắp/thả bị gán cứng (`_build_pick_timing` trong `scheduler.py`)**:
   - *Chi tiết*: Phân đoạn `C_pick -> D_pick` bị gán cứng thời gian chạy bằng `release_descent_time_s` (0.14s) bất kể khoảng cách đi ngang (`blend` lên đến 35mm). Phân đoạn `D_goto -> A_pick` cũng bị gán cứng $0.05$ giây. Mặc dù mảng `argument_time` hiện tại chưa được PLC dùng để quy hoạch thời gian thực tế, lỗi này vẫn làm sai lệch kết quả chạy mô phỏng (`SimulatedExecutor`).
3. **Rò rỉ bộ nhớ (Memory Leak) trong bộ lập lịch**:
   - *Chi tiết*: Tập hợp `self.seen_object_ids` trong `PickScheduler` lưu trữ tất cả ID vật thể đã từng phát hiện để tránh gắp trùng nhưng không bao giờ được giải phóng hay giới hạn kích thước, gây rò rỉ RAM khi chạy liên tục.
4. **Chỉ số thống kê bị bỏ sót**:
   - *Chi tiết*: Biến đếm `self.metrics.skipped_outside_workspace` không bao giờ được tăng lên khi bỏ qua vật thể ngoài vùng làm việc.

### 5.2. Hạn chế tích hợp PLC & Giả lập
1. **Mảng `argument_time` vô dụng trên thực tế**:
   - *Chi tiết*: Code PLC hiện tại chưa quy hoạch thời gian thực hiện quỹ đạo, tốc độ chuyển động thực tế của robot không bị ảnh hưởng bởi tham số thời gian gửi từ PC. Điều này gây ra sự không đồng nhất về mặt thời gian giữa dự đoán mô phỏng và chạy thực tế.
2. **Lệnh di chuyển tương đối chưa được hỗ trợ**:
   - *Chi tiết*: Mã lệnh `goto_relative` (ID = 1) chưa được lập trình dưới PLC thực tế. Đồng thời, trình giả lập PLC (`test_module.py`) cũng chưa viết logic xử lý cho lệnh này (robot giả lập sẽ đứng yên khi nhận lệnh).
3. **Lỗi kiểm thử hiệu chuẩn cơ học (`calibration.py`)**:
   - *Chi tiết*: Hàm hiệu chuẩn mechanical delay gửi lệnh `stop` (ID = 0) và đợi phản hồi `task_doing == 1`. Tuy nhiên, mock PLC cập nhật `task_doing` bằng chính ID lệnh (là 0 cho lệnh `stop`). Do đó, bước hiệu chuẩn này luôn bị timeout.
4. **Thiếu tag trạng thái cơ cấu chấp hành**:
   - *Chi tiết*: `PLCGateway` không truy vấn tag `end_effector` từ PLC trong danh sách `_status_tags()`, làm mất trạng thái bật/tắt của đầu hút trên CLI và log.
5. **Gán cứng đường dẫn biểu đồ trong `run_test.py`**:
   - *Chi tiết*: Đường dẫn lưu trữ biểu đồ của `generate_plots()` bị chỉ định cố định vào một phiên làm việc cũ không tồn tại hoặc sai quyền ghi.
