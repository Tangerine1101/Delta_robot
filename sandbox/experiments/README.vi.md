# Thí nghiệm — cách chạy và cách đọc kết quả

English version: [README.md](README.md).

Một thí nghiệm là một file YAML trong thư mục này. Chạy bằng một lệnh, kết quả (bảng và biểu đồ)
nằm trong `sandbox/results/<thí nghiệm>-<thời điểm>/`. Không cần robot.

## 1. Chạy

```bash
bash sandbox/run_experiments.sh phase0           # Pha 0: sandbox có tái hiện được cell không?   (~5 phút)
bash sandbox/run_experiments.sh phase1           # Pha 1: bản đồ W theo tải                      (~30 phút)
bash sandbox/run_experiments.sh phase0 --quick   # bản thô của bất kỳ thí nghiệm nào (vài phút)
bash sandbox/run_experiments.sh phase1 --dry-run # chỉ đếm số lần chạy và ước lượng thời gian
```

Tương đương bằng Python (luôn dùng `.venv` của repo; Python hệ thống thiếu `ruamel`):

```bash
.venv/bin/python -m sandbox exp --list           # liệt kê thí nghiệm kèm mô tả một dòng
.venv/bin/python -m sandbox exp w_map [--quick] [--dry-run] [--workers N]
.venv/bin/python -m sandbox report sandbox/results/<thư mục>   # vẽ lại bảng/biểu đồ của một sweep đã xong
```

Chương trình dùng mọi lõi trừ một. Kết quả không được git theo dõi; `manifest.json` trong mỗi
thư mục kết quả ghi lại revision git, spec và toàn bộ cấu hình đã giải.

## 2. Kết quả gồm gì

| File | Nội dung |
|---|---|
| `report.md` | Mở file này trước: biểu đồ, mức tốt nhất của từng yếu tố, bảng mọi tổ hợp (trung bình ± độ lệch chuẩn qua các seed) |
| `chart_*.png` | Các biểu đồ của báo cáo |
| `summary.csv` | Bảng mọi tổ hợp, để tự vẽ |
| `runs.csv` | Mỗi dòng một lần mô phỏng (mọi yếu tố và mọi chỉ số) — dữ liệu thô |
| `manifest.json` | Nguồn gốc: revision git, spec, cấu hình đã giải |
| `<run>/parts.csv`, `flow.csv` | (chỉ replay) bản ghi vật và dòng chảy của từng lần chạy sandbox, cùng cột với log của cell |

Các chỉ số cần biết: `pick_share` = số vật gắp được / số vật đến; `covered_share` = số lần gắp hụt
vào vật đang bị vật khác đè / số vật đến; `lost_share` = vật rời workspace mà không được gắp;
`mean_belt_mm_s` = tốc độ băng tải trung bình; `dip_s` = số giây mà thông lượng 60 s thấp hơn đầu vào
hơn 8 vật/phút ("dip" của dashboard); `bypassed_lost` = vật bị mất sau khi tay máy chờ một vật khác ở
xa hơn (open-issues S2).

## 3. Các thí nghiệm

### Pha 0 — `phase0_replay` (kind `replay`)

*Câu hỏi:* sandbox có tái hiện được cell không? Chỉ nên chạy sweep trên sandbox nếu câu trả lời là có.

*Cách làm:* mỗi run đã log của cell (`log/<session>/runNN_*_simulate_feeder/`) được chạy lại với đúng
chuỗi vật đến (`arrivals.csv`) và đúng cấu hình (`config.yaml` của session cộng các override của run),
rồi so sánh hai bản ghi vật theo thứ tự đến.

*Cách đọc:* `report.md` có, cho mỗi run, bảng kiểm tra theo ngưỡng (số vật gắp lệch ≤ 5 %, tốc độ
băng tải trung bình lệch ≤ 10 %, ≥ 85 % vật có cùng kết cục), số đếm kết cục, ma trận nhầm lẫn và
biểu đồ ba khung: thông lượng vào/ra, tốc độ băng tải, và các vật thất bại — cell so với sandbox.
Mục **Verdict** cuối file ghi pass hay FAIL cho từng run. Mục `_cal` dùng plant đã nhận dạng từ cell
(`sim.goto_extra_s`, `sim.pick_extra_s`); mục `_uncal` cộng độ trễ lệnh của mô hình vào mỗi chặng, như
sandbox đã làm trước khi nhận dạng. Từng vật riêng lẻ có thể khác (planner mang tính hỗn loạn); hãy
đánh giá theo số liệu tổng hợp. Có hai bộ: `phase0_replay` (seed 110, ba run ngày 07–08/10) và
`phase0_replay_1001` (seed 1001, bốn run ngày 10/10: W = 15, 25, 50 s và hằng 20 mm/s).

*Quyết định:* nếu replay ở giá trị W bạn quan tâm bị FAIL, sửa sandbox trước (open-issues S3);
không tin kết quả sweep ở vùng đó.

`phase0_noise` chạy lại mỗi run trong năm run của cell bảy lần với plant không đổi và đồng hồ mô phỏng
lệch vài mili giây, để đo bao nhiêu phần của độ lệch so với cell là độ phân tán của chính vòng kín: hãy
đánh giá một replay theo việc giá trị của cell có nằm trong khoảng phân tán đó hay không. Báo cáo còn cho,
mỗi replay, RMSE tốc độ băng tải, RMSE thông lượng 60 s và khoảng cách KS của phân bố khoảng gắp (chỉ để
tham khảo, không có ngưỡng).

### Nghiên cứu dp — `p1_select`, `p1_confirm`, `p1_regime`, `solve_time`

`bash sandbox/run_experiments.sh p1` (1–2 giờ): 20 thiết lập của `predictive_rank` (W) và `constant` (tốc
độ) cho dp trên các seed chọn, cùng lưới cho dp / KIM / SPT trên các seed xác nhận, một tải biến thiên, và
thời gian quyết định theo `max_jobs` của dp và bước của predictive rank; sau đó là các phân tích chọn rồi
xác nhận (`python3 -m sandbox confirm`). Mục đích từng thí nghiệm, các nhóm seed và kết quả:
`doc/experiment_notes.md`.

### Pha 1 — `w_map`, `w_map_dp` (kind `sweep`)

*Câu hỏi:* cửa sổ chấm điểm W của `predictive_rank` bao nhiêu là tốt, và nó có phụ thuộc tải và
stack-aware không?

*Cách làm:* lưới W × tốc độ vật đến × stack-aware, năm seed, mỗi lần chạy một giờ (`w_map`, KIM), và
cùng bản đồ cho `dp` ở tải của cell (`w_map_dp`). W = 1000 s được in là `inf`.

*Cách đọc:* trong `report.md`, `pick_share` theo W (mỗi tải một khung): đường phẳng nghĩa là W không
quan trọng ở tải đó, đường đi xuống về bên phải là "vách đá". Bảng **Best horizon_s** cho W tốt nhất
theo tải và vùng bình nguyên (các W kém không quá 0,5 điểm). Bản đồ nhiệt là cùng dữ liệu nhìn một lượt.

## 4. Viết thí nghiệm mới

Sao chép một spec, sửa, rồi chạy `exp <tên>`. Sweep không cần viết Python.

```yaml
description: một dòng, hiện trong `exp --list`
cell_config: modules/config.yaml        # tùy chọn: scheduling / speed / feeder lấy từ file cấu hình của cell
base:                                   # override dạng dotted của sandbox/config.yaml, áp cho mọi lần chạy
  sim.duration_s: 3600
  sim.stacking: true
seeds: [110, 111, 112]
grid:                                   # mọi tổ hợp là một case ...
  scheduling.planner: [kim, dp]
  feeder.poisson.rate_per_min: [16, 24]
cases:                                  # ... và/hoặc các case đặt tên
  - {name: slow belt, set: {speed.law: constant, speed.static_mm_s: 20}}
quick:                                  # --quick đổi gì: base được gộp, seeds/grid/cases bị thay
  base: {sim.duration_s: 600}
  seeds: [110]
report:
  title: Planner theo tải
  metrics: [pick_share, covered_share, mean_belt_mm_s]   # cột của bảng (xem sandbox/report.py)
  labels: {horizon_s: {1000: inf}}                        # cách in một mức
  charts:
    - {kind: line, x: rate_per_min, y: pick_share, series: planner}      # còn có: heatmap, bar
  best:
    - {over: planner, metric: pick_share, group: [rate_per_min], tolerance: 0.005}
```

* Khóa là khóa của `sandbox/config.yaml` (`python3 -m sandbox run --set key=value` dùng cùng khóa).
  Khóa lạ là lỗi, nên gõ sai không thể âm thầm chạy nhầm thí nghiệm.
* Một **yếu tố** (thiết lập khác nhau giữa các case) trở thành cột của `runs.csv`, đặt tên theo phần
  cuối của khóa (`rate_per_min`, `horizon_s`, `stack_aware`); hai khóa trùng phần cuối thì giữ tên
  đầy đủ. Dùng đúng các tên đó trong `report:`.
* Thứ không phải lưới — như Pha 0 chạy lại log — có `kind:` riêng. Một kind mới là một hàm cùng chữ ký
  với `_sweep` trong `sandbox/exp.py`, đăng ký vào `KINDS`.
* Để so với cell, `cell_config:` nhận `modules/config.yaml` hoặc `log/<session>/config.yaml`;
  `sandbox/cellrun.py` là nơi đọc log của cell.
* Chi phí: `exp <tên> --dry-run` ước lượng thời gian từ các lần chạy trước. Một lần chạy một giờ tốn
  5–100 s; tải và planner (`dp` chậm nhất) quyết định.

## 5. Giới hạn đã biết của sandbox

Đó là lý do Pha 0 tồn tại; tình trạng hiện tại nằm trong `doc/open-issues.md` (S2, S3). Kết quả Pha 0
mới nhất và quy tắc khi nào được tin một sweep nằm ở S3.
