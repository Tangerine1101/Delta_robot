> **NOTE — scratch file, NOT a reference source.** These are rough hand-noted figures
> from ad-hoc bench trials (e.g. "pick cycle 1.6s", "lệch 1mm"). Do not cite this file in
> the thesis or treat any number here as authoritative (see `report/writing_guideline.md` §0.1).
> The 1.6 s pick cycle here is where the abstract's ~38 picks/min figure was derived.

# độ võng z theo xy
tại các điểm khác nhau trong workspace(R-frame), đo khoảng cách giữa cơ cấu(z= -290) so với băng tải
|x, y|offset| description |
|---|---|---|
|0.000, 0.000| 10| 
|-78.552, 28.452| 6| top left
|80.723, -79.396| 10| bottom right
|1.258, -121.649| 10|bottom left
|0.913, 70.705| 10| top right

# test_accuracy
sau 300 cycle:
- lệch 1mm (sai số 1mm)
- plc_roundtrip ~50ms
- pick cycle 1.6s 

# test lệch vị trí cơ cấu theo phương dọc/ngang
phương pháp: đặt thước lên băng tải song song với trục u hoặc v và cho cơ cấu di chuyển đến từng điểm trên thước để quan sát độ chính xác (sai số 1mm)

|R-frame|C-frame| e_x | e_y|
|---|---|---|---|
|0,0|298,66| - | - |
|9.39, -17.66| 278.00, 66.00| 1mm | - |
|4.695, -8.83| 288.00, 66.00| 1mm | - |
|   | 283,66| 1mm | - |
|   | 298, 61| - | 0mm |
|   | 298, 56| - | 0mm |