# Giải Thích Siêu Chi Tiết Các Script Python Trong Project Delta Robot

## Ghi chú cập nhật

Tài liệu này ban đầu được viết để giải thích luồng **CLI + PLC worker** là chính.

Codebase hiện tại đã thay đổi thêm:

- `interpolar_points` mặc định bây giờ là `6`, không còn là `4`
- có thêm mode `--scheduler`
- có thêm [modules/image_processing.py](/home/tangerine/Share/Global%20Share/Documents/Delta_robot/modules/image_processing.py:1)
- có thêm [modules/scheduler.py](/home/tangerine/Share/Global%20Share/Documents/Delta_robot/modules/scheduler.py:1)

Vì tài liệu này rất dài, phần thân bên dưới vẫn tập trung mạnh vào luồng CLI/PLC cốt lõi. Khi đọc các ví dụ cũ có nhắc `4` slot, hãy hiểu rằng **giá trị hiện tại trong code là 6 slot**. Nguồn tham chiếu đúng nhất hiện tại là:

- [modules/config.json](/home/tangerine/Share/Global%20Share/Documents/Delta_robot/modules/config.json:1)
- [doc/system_configuration.md](/home/tangerine/Share/Global%20Share/Documents/Delta_robot/doc/system_configuration.md:1)

Tài liệu này dành cho người mới học Python rất ngắn ngày.

Mục tiêu của tài liệu:

1. Giải thích từng script đang có trong project này làm gì.
2. Giải thích cú pháp Python xuất hiện trong các script đó.
3. Giải thích các thư viện được import và các hàm quan trọng của chúng.
4. Giải thích luồng chạy thật của chương trình từ lúc bạn gõ lệnh đến lúc PLC nhận package.

Tài liệu này tập trung vào 3 file:

- [main.py](/home/tangerine/Share/Global%20Share/Documents/Delta_robot/main.py:1)
- [modules/cli.py](/home/tangerine/Share/Global%20Share/Documents/Delta_robot/modules/cli.py:1)
- [modules/EthernetCom.py](/home/tangerine/Share/Global%20Share/Documents/Delta_robot/modules/EthernetCom.py:1)

Ngoài ra có thêm file liên quan:

- [modules/config.json](/home/tangerine/Share/Global%20Share/Documents/Delta_robot/modules/config.json:1)
- [Algorithm.md](/home/tangerine/Share/Global%20Share/Documents/Delta_robot/Algorithm.md:1)

---

## 1. Bức tranh lớn: chương trình này đang làm gì?

Nói kiểu cực đơn giản:

- Bạn chạy `python3 main.py --cli`
- `main.py` tạo một giao diện dòng lệnh đơn giản
- Bạn gõ lệnh như `stop`, `go 1 2 3`, `goto 100 0 -200`
- `modules/cli.py` đọc câu lệnh đó và biến nó thành một `package`
- `modules/EthernetCom.py` lấy `package` đó và ghi từng trường vào tag PLC
- PLC đọc đúng struct đã khai báo sẵn trong Sysmac Studio

Ý cực quan trọng của project này:

- Struct package bên PLC đã cố định
- Vì vậy phía Python không được thay đổi cấu trúc package một cách tùy tiện
- Kể cả khi không dùng hết phần tử trong mảng, vẫn phải gửi đủ số phần tử
- Phần tử không dùng thì điền `0.0`

Đó là lý do bạn sẽ thấy code rất hay làm việc kiểu:

```python
[0.0] * slots
```

Nghĩa là:

- tạo một list
- có `slots` phần tử
- mỗi phần tử là `0.0`

Ví dụ nếu `slots = 4` thì kết quả là:

```python
[0.0, 0.0, 0.0, 0.0]
```

---

## 2. Cấu trúc package mà PLC đang mong đợi

Theo [Algorithm.md](/home/tangerine/Share/Global%20Share/Documents/Delta_robot/Algorithm.md:9), phía PC gửi `pc_package` có dạng:

```python
{
    "commandID": 0,
    "argument_number": 0,
    "argument_x": [...],
    "argument_y": [...],
    "argument_z": [...],
    "argument_e": [...],
    "argument_time": [...],
}
```

Ý nghĩa:

- `commandID`: mã lệnh
- `argument_number`: số phần tử thật sự đang dùng
- `argument_x`, `argument_y`, `argument_z`, `argument_time`: các mảng dữ liệu quỹ đạo gửi cho PLC
- `argument_e`: mảng trạng thái cơ cấu chấp hành dọc theo quỹ đạo, `0` là mở, `1` là pick

Ví dụ:

- lệnh `stop` không cần dữ liệu tọa độ
- nhưng vẫn phải gửi đủ `argument_x`, `argument_y`, `argument_z`, `argument_time`
- và cũng phải gửi đủ `argument_e`
- chỉ khác là mọi phần tử đều là `0.0`

Ví dụ package hợp lệ cho `stop` khi `slots = 4`:

```python
{
    "commandID": 0,
    "argument_number": 0,
    "argument_x": [0.0, 0.0, 0.0, 0.0],
    "argument_y": [0.0, 0.0, 0.0, 0.0],
    "argument_z": [0.0, 0.0, 0.0, 0.0],
    "argument_e": [0.0, 0.0, 0.0, 0.0],
    "argument_time": [0.0, 0.0, 0.0, 0.0],
}
```

---

## 3. Python cực cơ bản cần hiểu trước khi đọc code

Phần này là từ điển mini để bạn đỡ bị choáng.

### 3.1. Biến

```python
x = 10
name = "robot"
```

Nghĩa là gán giá trị cho biến.

### 3.2. Hàm

```python
def hello():
    print("hi")
```

Nghĩa là:

- `def` dùng để định nghĩa hàm
- `hello` là tên hàm
- phần thụt vào là thân hàm

### 3.3. Gọi hàm

```python
hello()
```

### 3.4. Kiểu dữ liệu hay gặp

- `int`: số nguyên, ví dụ `1`, `20`, `-5`
- `float`: số thực, ví dụ `0.5`, `-220.0`
- `str`: chuỗi, ví dụ `"stop"`
- `bool`: đúng/sai, `True` hoặc `False`
- `list`: danh sách, ví dụ `[1, 2, 3]`
- `dict`: từ điển key-value, ví dụ `{"x": 10, "y": 20}`
- `None`: không có giá trị

### 3.5. `if`

```python
if x > 0:
    print("duong")
```

Nghĩa là nếu điều kiện đúng thì chạy phần bên trong.

### 3.6. `for`

```python
for item in [1, 2, 3]:
    print(item)
```

Nghĩa là lặp qua từng phần tử.

### 3.7. `while`

```python
while True:
    print("lap vo han")
```

Nghĩa là lặp mãi mãi cho đến khi có `break` hoặc `return`.

### 3.8. `return`

```python
def add(a, b):
    return a + b
```

Nghĩa là trả kết quả ra khỏi hàm.

### 3.9. `break` và `continue`

- `break`: thoát khỏi vòng lặp
- `continue`: bỏ phần còn lại của lượt lặp hiện tại, chuyển sang lượt tiếp theo

### 3.10. `try / except / finally`

```python
try:
    do_something()
except ValueError:
    print("co loi")
finally:
    print("luon chay")
```

Ý nghĩa:

- `try`: thử chạy
- `except`: nếu lỗi thì xử lý ở đây
- `finally`: dù có lỗi hay không vẫn chạy đoạn này

### 3.11. `class`

```python
class Dog:
    pass
```

Class là khuôn để tạo object.

### 3.12. `self`

Trong method của class:

```python
class Dog:
    def bark(self):
        print("gau")
```

`self` nghĩa là object hiện tại.

### 3.13. Type hint

Ví dụ:

```python
def add(a: int, b: int) -> int:
    return a + b
```

Ý nghĩa:

- `a: int`: biến `a` được mong đợi là số nguyên
- `-> int`: hàm dự kiến trả về số nguyên

Type hint chủ yếu để người đọc dễ hiểu hơn và tool kiểm tra code tốt hơn.

Nó không phải là luật cứng tuyệt đối của Python runtime.

### 3.14. `| None`

Ví dụ:

```python
def get_name() -> str | None:
    ...
```

Nghĩa là hàm có thể trả về:

- `str`
- hoặc `None`

### 3.15. `dict[str, Any]`

Nghĩa là:

- đây là một `dict`
- key là `str`
- value có thể là bất cứ thứ gì (`Any`)

### 3.16. `*` trong khai báo hàm

Ví dụ:

```python
def f(a, b, *, c=1):
    ...
```

Thì `c` phải truyền theo dạng tên:

```python
f(1, 2, c=3)
```

Không được viết:

```python
f(1, 2, 3)
```

Trong project này:

```python
def build_command(self, command_name: str, *, x=None, y=None, z=None, t=None, argument_number: int = 0)
```

Nghĩa là `x`, `y`, `z`, `t`, `argument_number` nên truyền bằng tên cho rõ ràng.

### 3.17. F-string

Ví dụ:

```python
name = "PLC"
print(f"Hello {name}")
```

Kết quả:

```python
Hello PLC
```

### 3.18. List multiplication

```python
[0.0] * 4
```

Kết quả:

```python
[0.0, 0.0, 0.0, 0.0]
```

### 3.19. `enumerate`

```python
for index, item in enumerate(["a", "b"]):
    print(index, item)
```

Kết quả:

```python
0 a
1 b
```

### 3.20. `isinstance`

```python
isinstance(5, int)
```

Kết quả là `True`.

Nó dùng để kiểm tra kiểu dữ liệu.

### 3.21. `getattr`

```python
getattr(obj, "name", "default")
```

Nghĩa là:

- lấy thuộc tính `name` của `obj`
- nếu không có thì trả `"default"`

### 3.22. `with ... as ...`

```python
with open("a.txt") as f:
    data = f.read()
```

Nghĩa là mở tài nguyên rồi tự đóng khi xong.

---

## 4. Giải thích các thư viện xuất hiện trong code

Phần này chỉ nói các thư viện thực sự được import trong 3 script.

### 4.1. `__future__`

Trong các file đều có:

```python
from __future__ import annotations
```

Ý nghĩa đơn giản:

- cho phép xử lý type hint theo cách hiện đại hơn
- giúp tránh một số lỗi khi type hint tham chiếu đến kiểu chưa được định nghĩa ngay lúc parse file
- trong project nhỏ này bạn có thể hiểu nó là "bật chế độ annotation thân thiện hơn"

Bạn chưa cần đào sâu hơn ở giai đoạn 3 ngày học Python.

### 4.2. `argparse`

Dùng trong `main.py`.

Nó dùng để đọc tham số dòng lệnh, ví dụ:

```bash
python3 main.py --cli --ip 192.168.250.1 --port 502
```

Các phần quan trọng:

- `argparse.ArgumentParser(...)`: tạo bộ parser
- `add_argument(...)`: khai báo một đối số
- `parse_args()`: đọc đối số thật người dùng truyền vào
- `error(...)`: in lỗi và thoát chương trình

Ví dụ:

```python
parser.add_argument("--port", type=int, default=502, help="PLC port")
```

Ý nghĩa:

- có một option tên `--port`
- ép kiểu thành `int`
- nếu người dùng không truyền thì mặc định là `502`
- `help` là dòng mô tả khi chạy `--help`

### 4.3. `multiprocessing`

Được import thành tên ngắn:

```python
import multiprocessing as mp
```

`as mp` nghĩa là:

- từ giờ thay vì viết `multiprocessing.Queue()`
- code sẽ viết ngắn hơn là `mp.Queue()`

Thư viện này dùng để tạo process riêng.

Trong project này:

- process chính chạy CLI
- process phụ giữ kết nối PLC

Các thứ quan trọng:

- `mp.get_context("spawn")`
- `ctx.Queue()`
- `ctx.Process(...)`
- `worker.start()`
- `worker.join(timeout=...)`
- `worker.is_alive()`
- `worker.terminate()`

Giải thích:

#### `mp.get_context("spawn")`

Tạo context cho multiprocessing.

`"spawn"` nghĩa là tạo process mới sạch sẽ hơn, dễ đoán hơn, đặc biệt ổn trên nhiều hệ điều hành.

#### `Queue`

`Queue` là hàng đợi.

Hãy tưởng tượng như một cái hộp để process A bỏ tin nhắn vào, process B lấy ra đọc.

Trong project:

- `command_queue`: main gửi lệnh cho worker
- `response_queue`: worker gửi phản hồi lại cho main

#### `Process`

Tạo process mới.

Ví dụ:

```python
worker = ctx.Process(target=_worker, args=(...))
```

Ý nghĩa:

- `target=_worker`: hàm `_worker` sẽ chạy trong process phụ
- `args=(...)`: truyền tham số cho hàm đó

#### `start()`

Bắt đầu chạy process.

#### `join(timeout=...)`

Đợi process kết thúc.

Nếu có `timeout=5.0` thì đợi tối đa 5 giây.

#### `is_alive()`

Kiểm tra process còn sống không.

#### `terminate()`

Bắt buộc dừng process.

### 4.4. `queue.Empty`

Trong `main.py` có:

```python
from queue import Empty
```

Nó là một loại exception.

Nó thường xuất hiện khi bạn chờ lấy dữ liệu từ queue mà hết thời gian chờ nhưng không có dữ liệu.

Ví dụ:

```python
try:
    x = response_queue.get(timeout=5.0)
except Empty:
    return None
```

Nghĩa là:

- thử chờ 5 giây để lấy dữ liệu
- nếu không có gì trả về thì bắt lỗi `Empty`
- rồi trả `None`

### 4.5. `typing`

Trong code có:

- `Any`
- `Callable`
- `Iterable`

Đây chủ yếu là type hint.

#### `Any`

Có nghĩa là "bất cứ kiểu gì cũng được".

#### `Callable`

Nghĩa là một thứ có thể gọi như hàm.

Ví dụ:

```python
dispatch: Callable[[dict[str, Any]], dict[str, Any] | None]
```

Hiểu đơn giản:

- `dispatch` là một hàm
- nó nhận vào 1 `dict`
- trả về `dict` hoặc `None`

#### `Iterable`

Là một thứ có thể lặp qua được bằng `for`.

Ví dụ:

- `list`
- `tuple`
- generator

### 4.6. `shlex`

Dùng trong `modules/cli.py`.

Hàm quan trọng:

- `shlex.split(line)`

Nó tách một chuỗi lệnh thành các token gần giống shell.

Ví dụ:

```python
shlex.split('goto 100 0 -200')
```

Cho kết quả gần như:

```python
["goto", "100", "0", "-200"]
```

Lợi ích:

- tách lệnh sạch hơn `line.split()`
- xử lý chuỗi có dấu nháy tốt hơn

### 4.7. `dataclasses`

Trong code có:

- `@dataclass`
- `field`

#### `@dataclass`

Đây là decorator.

Nó giúp class dữ liệu bớt phải tự viết mã lặp lại.

Ví dụ:

```python
@dataclass
class CommandPlan:
    packages: list[dict[str, Any]]
    show_status: bool = False
```

Python sẽ tự hỗ trợ tạo `__init__` cơ bản, để bạn có thể viết:

```python
plan = CommandPlan(packages=[], show_status=True)
```

#### `field(default_factory=list)`

Dùng trong `RobotPacket`.

Vì sao không viết trực tiếp `argument_x: list[float] = []`?

Vì list là mutable object.

Nếu dùng trực tiếp `[]` trong nhiều object, rất dễ dính lỗi dùng chung list.

Cho nên `field(default_factory=list)` nghĩa là:

- mỗi object mới sẽ được tạo một list riêng

### 4.8. `json`

Dùng trong `modules/EthernetCom.py`.

Hàm chính:

- `json.load(file_handle)`

Nó đọc nội dung JSON từ file và chuyển thành object Python, thường là `dict`.

Ví dụ:

```json
{
  "ip_address": "192.168.250.1",
  "port": 502
}
```

Sau khi `json.load(...)` sẽ thành gần như:

```python
{"ip_address": "192.168.250.1", "port": 502}
```

### 4.9. `pathlib.Path`

`Path` giúp làm việc với đường dẫn file rõ ràng hơn.

Ví dụ trong code:

```python
PARENT_DIR = Path(__file__).parent
CONFIG_FILE = PARENT_DIR / "config.json"
```

Giải thích:

- `__file__` là đường dẫn tới file Python hiện tại
- `Path(__file__)` biến nó thành object `Path`
- `.parent` lấy thư mục chứa file
- toán tử `/` nối đường dẫn

Nghĩa là `CONFIG_FILE` trỏ tới file `modules/config.json`.

### 4.10. `types.SimpleNamespace`

Đây là object đơn giản cho phép truy cập kiểu chấm.

Ví dụ:

```python
config = SimpleNamespace(ip_address="192.168.250.1", port=502)
```

Sau đó dùng:

```python
config.ip_address
config.port
```

thay vì:

```python
config["ip_address"]
config["port"]
```

### 4.11. `pylogix`

Đây là thư viện giao tiếp với PLC.

Trong code:

```python
from pylogix import PLC
```

Class chính là `PLC`.

Các thứ được dùng:

- `PLC()`
- `plc.IPAddress = ...`
- `plc.Port = ...`
- `plc.Write(tag, value)`
- `plc.Read(tags)`
- `plc.Close()`

Hiểu đơn giản:

- `Write`: ghi dữ liệu xuống PLC
- `Read`: đọc dữ liệu từ PLC
- `Close`: đóng kết nối

Code hiện tại đang dùng pylogix như một gateway đọc/ghi tag.

### 4.12. Một số built-in Python quan trọng trong code

#### `list(...)`

Biến thứ gì đó thành list.

#### `dict(...)`

Tạo dict hoặc copy dict.

#### `len(...)`

Lấy số phần tử.

#### `int(...)`, `float(...)`

Ép kiểu số.

#### `print(...)`

In ra màn hình.

#### `sorted(...)`

Sắp xếp.

#### `str(...)`

Ép sang chuỗi.

---

## 5. Script 1: `modules/EthernetCom.py`

Đây là file nền tảng nhất cho chuyện giao tiếp PLC.

Nếu ví project như một nhà hàng:

- `cli.py` là nhân viên nhận order
- `main.py` là người điều phối
- `EthernetCom.py` là người thực sự mang order vào bếp

---

## 6. Mổ xẻ `modules/EthernetCom.py` từ trên xuống

### 6.1. Phần import

```python
from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path
from types import SimpleNamespace
from typing import Any, Iterable
```

Ý nghĩa:

- `json`: đọc file `config.json`
- `dataclass`, `field`: tạo class dữ liệu gọn hơn
- `Path`: làm việc với đường dẫn file
- `SimpleNamespace`: biến config thành object truy cập kiểu chấm
- `Any`, `Iterable`: type hint

Sau đó:

```python
try:
    from pylogix import PLC
except ImportError:
    PLC = None
```

Ý nghĩa:

- thử import thư viện `pylogix`
- nếu máy chưa cài thì không cho chương trình nổ ngay
- thay vào đó gán `PLC = None`

Sau này lúc cần kết nối, code sẽ báo lỗi rõ ràng hơn.

### 6.2. Khai báo đường dẫn config

```python
PARENT_DIR = Path(__file__).parent
CONFIG_FILE = PARENT_DIR / "config.json"
```

Nghĩa là:

- lấy thư mục chứa `EthernetCom.py`
- nối với `"config.json"`

Vậy file config chính là:

- `modules/config.json`

### 6.3. `DEFAULT_CONFIG`

```python
DEFAULT_CONFIG = {
    "ip_address": "192.168.250.1",
    "port": 502,
    "period_s": 0.1,
    "interpolar_points": 4,
}
```

Đây là cấu hình mặc định.

Nếu file JSON không có hoặc hỏng thì dùng cái này.

Lưu ý:

- `interpolar_points = 4` rất quan trọng
- vì nó quyết định số phần tử mảng trong package

### 6.4. `COMMAND_ID`

```python
COMMAND_ID = {
    "stop": 0,
    "goto_relative": 1,
    "goto_absolute": 2,
    "go_trajectory": 3,
    "calibrate": 4,
    "pick": 5,
    "release": 6,
}
```

Ý nghĩa:

- PLC không đọc chữ `"stop"` trực tiếp
- PLC đọc số nguyên

Vậy Python phải map:

- `"stop"` thành `0`
- `"pick"` thành `5`

### 6.5. `COMMAND_NAME`

```python
COMMAND_NAME = {value: key for key, value in COMMAND_ID.items()}
```

Đây là dictionary đảo ngược.

Nếu `COMMAND_ID` là:

```python
{"stop": 0, "pick": 5}
```

thì `COMMAND_NAME` sẽ giống:

```python
{0: "stop", 5: "pick"}
```

Trong file hiện tại nó chưa được dùng nhiều, nhưng hữu ích nếu sau này cần đổi ngược từ số sang tên lệnh.

### 6.6. `ARRAY_FIELDS`

```python
ARRAY_FIELDS = ("argument_x", "argument_y", "argument_z", "argument_time")
```

Đây là tuple chứa tên các field kiểu mảng.

Nó giúp code lặp qua các field này thay vì viết lại 4 lần.

### 6.7. Hàm `load_config()`

Xem tại [modules/EthernetCom.py](/home/tangerine/Share/Global%20Share/Documents/Delta_robot/modules/EthernetCom.py:40)

```python
def load_config() -> SimpleNamespace:
```

Hàm này làm gì:

1. Copy cấu hình mặc định
2. Cố đọc `config.json`
3. Nếu đọc được thì đè giá trị mới lên giá trị mặc định
4. Trả kết quả dưới dạng `SimpleNamespace`

Đoạn quan trọng:

```python
data = dict(DEFAULT_CONFIG)
```

Ý nghĩa:

- copy `DEFAULT_CONFIG`
- để không sửa trực tiếp bản gốc

Đoạn:

```python
with CONFIG_FILE.open("r", encoding="utf-8") as handle:
    raw = json.load(handle)
```

Ý nghĩa:

- mở file config ở chế độ đọc (`"r"`)
- dùng UTF-8
- `json.load(...)` đọc JSON thành object Python

Đoạn:

```python
except FileNotFoundError:
    raw = {}
```

Nếu file không tồn tại thì dùng dict rỗng.

Đoạn:

```python
except json.JSONDecodeError as exc:
    print(...)
    raw = {}
```

Nếu JSON sai cú pháp:

- in cảnh báo
- rồi vẫn tiếp tục chạy bằng default config

Đoạn:

```python
if isinstance(raw, dict):
    data.update(raw)
```

Ý nghĩa:

- chỉ merge nếu `raw` thật sự là dict
- `update` dùng giá trị trong file JSON đè lên default

Cuối cùng:

```python
return SimpleNamespace(**data)
```

`**data` nghĩa là bung dict thành keyword arguments.

Ví dụ:

```python
{"ip_address": "192.168.250.1", "port": 502}
```

gần giống:

```python
SimpleNamespace(ip_address="192.168.250.1", port=502)
```

### 6.8. Hàm `_coerce_list(...)`

Xem tại [modules/EthernetCom.py](/home/tangerine/Share/Global%20Share/Documents/Delta_robot/modules/EthernetCom.py:58)

```python
def _coerce_list(values: Iterable[Any], size: int, fill_value: Any = 0.0) -> list[Any]:
```

Mục tiêu:

- ép một thứ có thể lặp thành list
- nếu dài hơn `size` thì cắt bớt
- nếu ngắn hơn `size` thì thêm `fill_value` cho đủ

Ví dụ:

```python
_coerce_list([1, 2], 4, 0)
```

ra:

```python
[1, 2, 0, 0]
```

Đây là một hàm cực quan trọng vì nó giữ đúng số phần tử mà PLC cần.

### 6.9. Hàm `_zero_package(slots)`

Xem tại [modules/EthernetCom.py](/home/tangerine/Share/Global%20Share/Documents/Delta_robot/modules/EthernetCom.py:65)

Nó tạo package rỗng hoàn chỉnh.

Điểm chính:

- `commandID` mặc định là `stop`
- `argument_number = 0`
- mọi mảng đều dài đúng `slots`
- mọi phần tử là `0.0`

### 6.10. Class `RobotPacket`

Xem tại [modules/EthernetCom.py](/home/tangerine/Share/Global%20Share/Documents/Delta_robot/modules/EthernetCom.py:76)

Đây là class dữ liệu mô tả một package robot.

```python
@dataclass
class RobotPacket:
    commandID: int
    argument_number: int = 0
    argument_x: list[float] = field(default_factory=list)
    argument_y: list[float] = field(default_factory=list)
    argument_z: list[float] = field(default_factory=list)
    argument_time: list[float] = field(default_factory=list)
```

Ý nghĩa:

- object này chứa đúng các field của package
- `commandID` là bắt buộc
- các field khác có default

Ví dụ tạo object:

```python
packet = RobotPacket(
    commandID=3,
    argument_number=2,
    argument_x=[100.0, 120.0],
)
```

### 6.11. Method `to_dict(self, slots)`

Xem tại [modules/EthernetCom.py](/home/tangerine/Share/Global%20Share/Documents/Delta_robot/modules/EthernetCom.py:87)

Đây là method cực quan trọng.

Nó chuyển object `RobotPacket` thành `dict` đúng format PLC cần.

Luồng làm việc:

1. Tạo package rỗng bằng `_zero_package(slots)`
2. Ghi `commandID`
3. Ghi `argument_number`
4. Dùng `_coerce_list` để ép các mảng về đúng số phần tử

Tại sao không trả thẳng dữ liệu đang có?

Vì nếu làm vậy:

- list có thể thiếu phần tử
- hoặc thừa phần tử
- hoặc không đúng kiểu

PLC rất ghét chuyện cấu trúc không ổn định.

### 6.12. Class `PLCGateway`

Đây là lớp giao tiếp trực tiếp với PLC bằng `pylogix`.

Tên class:

```python
class PLCGateway:
```

`Gateway` hiểu nôm na là "cổng trung gian".

### 6.13. Hàm khởi tạo `__init__`

Xem tại [modules/EthernetCom.py](/home/tangerine/Share/Global%20Share/Documents/Delta_robot/modules/EthernetCom.py:101)

```python
def __init__(self, ip: str | None = None, port: int | None = None, interpolar_points: int | None = None, tag_write: str = "pc_package", tag_read: str = "plc_package") -> None:
```

Nếu bạn mới học Python, đây nhìn đáng sợ nhưng thật ra chỉ là:

- có thể truyền IP
- có thể truyền port
- có thể truyền số slot
- nếu không truyền thì lấy từ config/default

Các dòng chính:

```python
self.config = load_config()
```

`self` là object hiện tại.

Nghĩa là object `PLCGateway` tự nạp config cho mình.

```python
self.ip = ip or self.config.ip_address
```

Ý nghĩa:

- nếu `ip` được truyền vào và có giá trị thật thì dùng nó
- nếu không thì dùng `self.config.ip_address`

```python
self.port = port or getattr(self.config, "port", DEFAULT_CONFIG["port"])
```

`getattr` dùng để lấy thuộc tính `port`, nếu không có thì lấy default.

```python
self.interpolar_points = interpolar_points or getattr(
    self.config, "interpolar_points", DEFAULT_CONFIG["interpolar_points"]
)
```

Đây là chỗ quyết định số phần tử mảng trong package.

```python
self.plc = PLC() if PLC is not None else None
```

Đây là toán tử điều kiện 1 dòng.

Nghĩa là:

- nếu import được `PLC` thì tạo object `PLC()`
- không thì gán `None`

### 6.14. `connect()`

Xem tại [modules/EthernetCom.py](/home/tangerine/Share/Global%20Share/Documents/Delta_robot/modules/EthernetCom.py:125)

Việc chính:

- kiểm tra `pylogix` có tồn tại không
- nếu không có thì báo lỗi
- nếu có thì đánh dấu `connected = True`

Điểm cần hiểu:

```python
raise ImportError(...)
```

`raise` nghĩa là ném ra lỗi.

Nếu không cài `pylogix`, chương trình sẽ báo rõ lý do.

### 6.15. `disconnect()`

Xem tại [modules/EthernetCom.py](/home/tangerine/Share/Global%20Share/Documents/Delta_robot/modules/EthernetCom.py:136)

Việc chính:

- nếu `self.plc` tồn tại thì gọi `Close()`
- nếu lỗi khi close thì in cảnh báo
- đánh dấu `connected = False`

Chú ý phần:

```python
try:
    self.plc.Close()
except Exception as exc:
    ...
```

Nghĩa là:

- cố đóng kết nối
- nếu đóng lỗi thì không làm chương trình chết ngay

### 6.16. `_normalize_package(...)`

Xem tại [modules/EthernetCom.py](/home/tangerine/Share/Global%20Share/Documents/Delta_robot/modules/EthernetCom.py:145)

Mục tiêu:

- dù đầu vào là `RobotPacket`
- hay là `dict`
- cuối cùng cũng biến thành `dict` chuẩn hóa đúng cấu trúc PLC

Logic:

#### Trường hợp 1: đầu vào là `RobotPacket`

```python
if isinstance(package, RobotPacket):
    package = package.to_dict(self.interpolar_points)
```

Nghĩa là gọi `to_dict()` luôn.

#### Trường hợp 2: đầu vào là `dict`

Code tạo một package rỗng chuẩn trước, rồi mới chép dữ liệu của người dùng vào.

Cách này an toàn hơn kiểu lấy dict người dùng đưa vào rồi tin tưởng hoàn toàn.

### 6.17. `_write_result_ok(...)`

Xem tại [modules/EthernetCom.py](/home/tangerine/Share/Global%20Share/Documents/Delta_robot/modules/EthernetCom.py:160)

Mục tiêu:

- kiểm tra kết quả trả về từ `pylogix.Write(...)`
- xem có thành công không

Nó khá "phòng thủ":

- nếu `result is None` thì coi như tạm ổn
- nếu không có thuộc tính `Status` thì cũng coi như tạm ổn
- nếu có `Status` thì phải là `"success"`

### 6.18. `_write_tag(tag_name, value)`

Xem tại [modules/EthernetCom.py](/home/tangerine/Share/Global%20Share/Documents/Delta_robot/modules/EthernetCom.py:169)

Nó gọi:

```python
result = self.plc.Write(tag_name, value)
```

Nếu không OK thì:

```python
raise RuntimeError(...)
```

Điều này tốt vì:

- nếu ghi PLC thất bại
- code không im lặng bỏ qua
- nó báo lỗi ngay

### 6.19. `_read_tags(tags)`

Xem tại [modules/EthernetCom.py](/home/tangerine/Share/Global%20Share/Documents/Delta_robot/modules/EthernetCom.py:174)

Nó gọi:

```python
result = self.plc.Read(tags)
```

Sau đó:

- nếu `None` thì trả list rỗng
- nếu có dữ liệu thì ép thành list

### 6.20. `send_package(...)`

Xem tại [modules/EthernetCom.py](/home/tangerine/Share/Global%20Share/Documents/Delta_robot/modules/EthernetCom.py:180)

Đây là một trong những hàm quan trọng nhất của toàn project.

Luồng:

1. Nếu chưa `connected` thì gọi `connect()`
2. Chuẩn hóa package
3. Lặp qua từng key-value trong package
4. Nếu value là list thì ghi từng phần tử vào từng tag mảng
5. Nếu value là scalar thì ghi trực tiếp vào tag

Ví dụ package:

```python
{
    "commandID": 2,
    "argument_number": 1,
    "argument_x": [10.0, 0.0, 0.0, 0.0],
    ...
}
```

Thì code sẽ ghi kiểu:

- `pc_package.commandID = 2`
- `pc_package.argument_number = 1`
- `pc_package.argument_x[0] = 10.0`
- `pc_package.argument_x[1] = 0.0`
- ...

Đây là lý do tại sao code phải giữ đúng số phần tử.

Nếu PLC khai báo mảng 4 phần tử thì Python cũng phải gửi đủ 4 phần tử.

### 6.21. `get_package()`

Xem tại [modules/EthernetCom.py](/home/tangerine/Share/Global%20Share/Documents/Delta_robot/modules/EthernetCom.py:193)

Nó đọc trạng thái từ PLC.

Hiện tại chỉ đọc:

- `plc_package.task_doing`
- `plc_package.task_state`

Luồng:

1. Nếu chưa kết nối thì kết nối
2. Tạo list tên tag cần đọc
3. Gọi `_read_tags(tags)`
4. Duyệt từng item trả về
5. Nếu item thành công thì lấy:
   - `TagName`
   - `Value`
6. Chuyển thành dict gọn

Ví dụ kết quả có thể là:

```python
{"task_doing": 3, "task_state": 1}
```

### 6.22. `build_command(...)`

Xem tại [modules/EthernetCom.py](/home/tangerine/Share/Global%20Share/Documents/Delta_robot/modules/EthernetCom.py:218)

Đây là helper để build package từ tên lệnh và các mảng dữ liệu.

Ví dụ:

```python
gateway.build_command(
    "go_trajectory",
    x=[100.0, 120.0],
    y=[0.0, 20.0],
    z=[-220.0, -220.0],
    t=[0.5, 0.5],
    argument_number=2,
)
```

Nó sẽ trả về dict đã pad/cắt đúng số phần tử.

### 6.23. `build_zero_command(...)`

Xem tại [modules/EthernetCom.py](/home/tangerine/Share/Global%20Share/Documents/Delta_robot/modules/EthernetCom.py:231)

Dùng cho những lệnh không cần dữ liệu như:

- `stop`
- `pick`
- `release`
- `calibrate`

### 6.24. Khối `if __name__ == "__main__":`

Xem tại [modules/EthernetCom.py](/home/tangerine/Share/Global%20Share/Documents/Delta_robot/modules/EthernetCom.py:239)

Đây là cú pháp rất quan trọng trong Python.

Nếu file được chạy trực tiếp:

```bash
python3 modules/EthernetCom.py
```

thì đoạn này chạy.

Nếu file chỉ được import từ file khác thì đoạn này không chạy.

Trong file này nó đóng vai trò smoke test nhẹ:

- load config
- tạo gateway
- connect
- build package demo
- gửi package
- đọc status
- disconnect

---

## 7. Script 2: `modules/cli.py`

File này chịu trách nhiệm:

- nhận lệnh text từ người dùng
- hiểu người dùng muốn gì
- biến lệnh đó thành package đúng chuẩn PLC

Nó không trực tiếp nói chuyện với PLC.

Nó chỉ build package và giao package cho nơi khác gửi đi.

---

## 8. Mổ xẻ `modules/cli.py` từ trên xuống

### 8.1. Import

```python
import shlex
from dataclasses import dataclass
from typing import Any, Callable, Iterable
from modules.EthernetCom import COMMAND_ID, RobotPacket
```

Ý nghĩa:

- `shlex`: tách chuỗi lệnh
- `dataclass`: tạo class dữ liệu `CommandPlan`
- `Callable`: type hint cho các hàm callback
- `Iterable`: type hint cho `_pad`
- `COMMAND_ID`, `RobotPacket`: tái sử dụng định nghĩa package từ `EthernetCom.py`

### 8.2. `INTERPOLAR_POINTS = 4`

Đây là default local cho file CLI.

Nó chỉ là giá trị mặc định.

Khi chạy thật, `main.py` sẽ truyền `interpolar_points=args.interpolar_points` vào.

### 8.3. `CommandPlan`

Xem tại [modules/cli.py](/home/tangerine/Share/Global%20Share/Documents/Delta_robot/modules/cli.py:13)

```python
@dataclass
class CommandPlan:
    packages: list[dict[str, Any]]
    show_status: bool = False
    quit_requested: bool = False
```

Đây là "kế hoạch hành động" sau khi parse một dòng lệnh.

Ví dụ:

- `status` thì không có package để gửi, nhưng `show_status=True`
- `quit` thì `quit_requested=True`
- `goto 10 20 -30` thì có 1 package `goto_absolute`

### 8.4. `PRESET_TRAJECTORIES`

Đây là các quỹ đạo demo hard-code sẵn.

Ví dụ `demo`, `square`, `home`.

Khi người dùng gõ:

```bash
go_trajectory demo
```

CLI sẽ lấy danh sách điểm từ đây.

Mỗi điểm là một dict:

```python
{"x": 100.0, "y": 0.0, "z": -220.0, "time": 0.5}
```

### 8.5. `_format_bool(token)`

Xem tại [modules/cli.py](/home/tangerine/Share/Global%20Share/Documents/Delta_robot/modules/cli.py:39)

Mục tiêu:

- hiểu chữ người dùng nhập có nghĩa là "đúng" hay không

Ví dụ các token được coi là đúng:

- `"1"`
- `"on"`
- `"pick"`
- `"true"`
- `"yes"`

Điều khiển cơ cấu chấp hành không đi kèm trong `goto`.

Muốn kẹp hoặc nhả thì dùng lệnh riêng:

- `pick`
- `release`

### 8.6. `_pad(values, slots)`

Xem tại [modules/cli.py](/home/tangerine/Share/Global%20Share/Documents/Delta_robot/modules/cli.py:43)

Mục tiêu:

- ép một chuỗi giá trị về đúng số phần tử `slots`

Ví dụ:

```python
_pad([10.0], 4)
```

ra:

```python
[10.0, 0.0, 0.0, 0.0]
```

Đây là bản đơn giản hóa rất rõ cho nhu cầu package PLC.

### 8.7. `_zero_command(command_name, slots)`

Xem tại [modules/cli.py](/home/tangerine/Share/Global%20Share/Documents/Delta_robot/modules/cli.py:50)

Nó tạo package "toàn số 0" cho một command.

Ví dụ:

```python
_zero_command("pick", 4)
```

ra package với:

- `commandID = 5`
- `argument_number = 0`
- mọi mảng dài 4 và đều là `0.0`

### 8.8. `_trajectory_command(name, points, slots)`

Xem tại [modules/cli.py](/home/tangerine/Share/Global%20Share/Documents/Delta_robot/modules/cli.py:61)

Mục tiêu:

- biến một quỹ đạo gồm nhiều điểm thành một package `go_trajectory`

Đoạn:

```python
if len(points) > slots:
    raise ValueError(...)
```

Rất quan trọng.

Nó chặn trường hợp:

- quỹ đạo có nhiều điểm hơn số phần tử PLC cho phép
- vì nếu cứ gửi bừa sẽ sai cấu trúc hoặc mất dữ liệu

Đoạn:

```python
argument_x=_pad((point["x"] for point in points), slots)
```

Đây là generator expression.

Hiểu chậm như sau:

- duyệt từng `point` trong `points`
- lấy `point["x"]`
- gom chúng thành một chuỗi giá trị
- `_pad` sẽ biến chuỗi đó thành list đủ độ dài

Ví dụ nếu các điểm là:

```python
[
    {"x": 100.0, "y": 0.0, "z": -220.0, "time": 0.5},
    {"x": 120.0, "y": 20.0, "z": -220.0, "time": 0.5},
]
```

thì:

- `argument_x` thành `[100.0, 120.0, 0.0, 0.0]`
- `argument_y` thành `[0.0, 20.0, 0.0, 0.0]`

### 8.9. `_joint_command(...)`

Xem tại [modules/cli.py](/home/tangerine/Share/Global%20Share/Documents/Delta_robot/modules/cli.py:81)

Dùng cho lệnh:

```bash
go <theta1> <theta2> <theta3>
```

Trong code, lệnh này map sang `goto_relative`.

Nó tạo package kiểu:

- `argument_number = 1`
- `argument_x[0] = theta1`
- `argument_y[0] = theta2`
- `argument_z[0] = theta3`
- `argument_time` toàn `0.0`

### 8.10. `_cartesian_command(...)`

Xem tại [modules/cli.py](/home/tangerine/Share/Global%20Share/Documents/Delta_robot/modules/cli.py:98)

Dùng cho lệnh:

```bash
goto <x> <y> <z>
```

Trong code, lệnh này map sang `goto_absolute`.

Package tạo ra tương tự `_joint_command`, chỉ khác ý nghĩa của dữ liệu là tọa độ Cartesian thay vì góc khớp.

### 8.11. `_parse_plan(line, slots)`

Xem tại [modules/cli.py](/home/tangerine/Share/Global%20Share/Documents/Delta_robot/modules/cli.py:115)

Đây là bộ não chính của CLI.

Nó đọc một dòng lệnh text và quyết định phải làm gì.

#### Bước 1: tách token

```python
tokens = shlex.split(line)
```

Ví dụ:

```python
"goto 10 20 -30"
```

thành:

```python
["goto", "10", "20", "-30"]
```

#### Bước 2: lệnh rỗng

Nếu không có token thì trả plan rỗng.

#### Bước 3: lấy command

```python
command = tokens[0].lower()
```

`lower()` biến chữ hoa thành chữ thường.

Ví dụ `"STOP"` thành `"stop"`.

#### Bước 4: xử lý từng lệnh

##### `quit` hoặc `exit`

Trả plan với `quit_requested=True`.

##### `help` hoặc `?`

Trả plan rỗng.

Việc in help được xử lý ở chỗ khác.

##### `status`

Trả plan với `show_status=True`.

##### `stop`

Tạo 1 package zero command cho `stop`.

##### `go`

Yêu cầu đúng 3 số.

Nếu không đúng:

```python
raise ValueError("go expects 3 numbers: go <theta1> <theta2> <theta3>")
```

Nếu đúng thì build package `goto_relative`.

##### `goto`

Cho phép đúng 3 tham số số.

Ví dụ:

```bash
goto 100 0 -200
```

thì chỉ có package `goto_absolute`.

Muốn điều khiển cơ cấu chấp hành thì dùng lệnh riêng:

- `pick`
- `release`

##### `go_trajectory`

Phải có đúng 1 tên preset.

Ví dụ:

```bash
go_trajectory demo
```

Nếu preset không tồn tại thì code báo lỗi và liệt kê các preset hợp lệ.

##### `calib`

Map sang `calibrate`.

##### `pick`

Tạo zero command `pick`.

##### `release`

Tạo zero command `release`.

##### Lệnh lạ

Nếu không khớp gì cả:

```python
raise ValueError(f"Unknown command: {command}")
```

### 8.12. `_print_help()`

Xem tại [modules/cli.py](/home/tangerine/Share/Global%20Share/Documents/Delta_robot/modules/cli.py:174)

Chỉ đơn giản là in danh sách lệnh.

### 8.13. `format_status(status)`

Xem tại [modules/cli.py](/home/tangerine/Share/Global%20Share/Documents/Delta_robot/modules/cli.py:190)

Mục tiêu:

- biến dict status thành chuỗi dễ đọc

Ví dụ:

```python
{"task_doing": 3, "task_state": 1}
```

thành:

```python
[INFO] PLC status: task_doing=3, task_state=1
```

### 8.14. `run_interactive(...)`

Xem tại [modules/cli.py](/home/tangerine/Share/Global%20Share/Documents/Delta_robot/modules/cli.py:196)

Đây là vòng lặp CLI thật sự.

Tham số:

- `dispatch`: hàm gửi package đi đâu đó
- `request_status`: hàm hỏi trạng thái PLC
- `slots`: số phần tử package
- `prompt`: chữ hiện trước con trỏ nhập lệnh

Luồng:

1. In tiêu đề CLI
2. Lặp vô hạn
3. Đợi người dùng nhập lệnh bằng `input(prompt)`
4. Nếu Ctrl+C hoặc EOF thì thoát
5. Nếu dòng rỗng thì bỏ qua
6. Parse lệnh bằng `_parse_plan`
7. Nếu parse lỗi thì in lỗi rồi lặp tiếp
8. Nếu người dùng muốn quit thì `return`
9. Nếu là help thì in help
10. Nếu là status thì gọi `request_status()`
11. Nếu là lệnh gửi package thì gọi `dispatch(package)` cho từng package

Điểm hay của thiết kế này:

- `cli.py` không cần biết PLC cụ thể hoạt động ra sao
- nó chỉ biết gọi callback `dispatch`

Đây là kiểu tách trách nhiệm khá tốt:

- CLI lo hiểu lệnh
- phần khác lo giao tiếp PLC

---

## 9. Script 3: `main.py`

Đây là entrypoint hiện tại của project.

Bạn chạy nó như sau:

```bash
python3 main.py --cli
```

Vai trò của file này:

- parse argument dòng lệnh
- tạo worker process giữ PLC connection
- nối CLI với worker đó
- đảm bảo shutdown sạch sẽ

---

## 10. Mổ xẻ `main.py` từ trên xuống

### 10.1. Import

```python
import argparse
import multiprocessing as mp
from queue import Empty
from typing import Any

from modules.EthernetCom import PLCGateway, load_config
from modules.cli import run_interactive
```

Ý nghĩa:

- `argparse`: đọc đối số dòng lệnh
- `multiprocessing`: tạo worker process
- `Empty`: bắt lỗi timeout từ queue
- `Any`: type hint
- `PLCGateway`, `load_config`: giao tiếp PLC và đọc config
- `run_interactive`: chạy CLI

### 10.2. Hàm `_worker(...)`

Xem tại [main.py](/home/tangerine/Share/Global%20Share/Documents/Delta_robot/main.py:12)

Đây là hàm chạy trong process phụ.

Bạn có thể hiểu nó là:

- một người trực điện thoại PLC
- ngồi chờ main process gửi yêu cầu
- main bảo gì thì làm đó

Tham số:

- `command_queue`: nhận yêu cầu từ main
- `response_queue`: gửi phản hồi về main
- `ip`, `port`, `slots`: thông tin PLC và cấu trúc package

Đầu tiên:

```python
gateway = PLCGateway(ip=ip, port=port, interpolar_points=interpolar_points)
```

Tạo gateway để nói chuyện với PLC.

Sau đó:

```python
gateway.connect()
```

Rồi báo về main:

```python
response_queue.put(
    {
        "ok": True,
        "type": "connected",
        "ip": ip,
        "port": port,
    }
)
```

`put(...)` nghĩa là bỏ một message vào queue.

Tiếp theo là vòng lặp vô hạn:

```python
while True:
    message = command_queue.get()
```

`get()` nghĩa là lấy một message từ queue.

Process worker sẽ đứng chờ ở đây cho tới khi có message.

#### Message type `shutdown`

Nếu main gửi:

```python
{"type": "shutdown"}
```

worker sẽ:

- gửi ack shutdown
- `break` khỏi vòng lặp
- sau đó `finally` sẽ đóng kết nối PLC

#### Message type `status`

worker gọi:

```python
status = gateway.get_package()
```

rồi trả về:

```python
{"ok": True, "type": "status", "data": status}
```

Nếu lỗi thì:

```python
{"ok": False, "type": "error", "error": "..."}
```

#### Message type `send`

worker gọi:

```python
package = gateway.send_package(message["package"])
status = gateway.get_package()
```

Nghĩa là:

1. gửi package
2. đọc status sau khi gửi
3. trả kết quả về main

#### Message type lạ

Nếu `type` không phải `shutdown/status/send` thì worker trả lỗi.

#### `finally`

Cuối hàm:

```python
finally:
    gateway.disconnect()
```

Nghĩa là dù worker thoát kiểu gì thì vẫn cố đóng PLC.

### 10.3. `_wait_for_response(...)`

Xem tại [main.py](/home/tangerine/Share/Global%20Share/Documents/Delta_robot/main.py:75)

```python
def _wait_for_response(response_queue: mp.Queue, timeout: float = 5.0) -> dict[str, Any] | None:
```

Mục tiêu:

- chờ message trả về từ worker
- nhưng không chờ mãi mãi

Code:

```python
try:
    return response_queue.get(timeout=timeout)
except Empty:
    return None
```

Ý nghĩa:

- chờ tối đa `timeout` giây
- nếu không có gì thì trả `None`

### 10.4. `main()`

Xem tại [main.py](/home/tangerine/Share/Global%20Share/Documents/Delta_robot/main.py:82)

Đây là hàm chính.

#### Bước 1: tạo parser

```python
parser = argparse.ArgumentParser(description="Delta robot command line entrypoint")
```

#### Bước 2: đọc config

```python
config = load_config()
default_interpolar_points = int(getattr(config, "interpolar_points", 4))
```

Nếu `config.json` thiếu `interpolar_points` thì vẫn fallback về `4`.

#### Bước 3: khai báo đối số CLI

Các option hiện có:

- `--cli`
- `--ip`
- `--port`
- `--package-slots`
- `--prompt`

#### `--cli`

```python
parser.add_argument("--cli", action="store_true", help="Run the interactive CLI mode")
```

`action="store_true"` nghĩa là:

- nếu người dùng ghi `--cli` thì `args.cli = True`
- nếu không ghi thì `args.cli = False`

#### `--ip`

Cho phép override IP PLC.

#### `--port`

Cho phép override port PLC.

#### `--package-slots`

Cho phép chọn số slot package.

Nhưng nhớ rằng trong thực tế nó phải khớp với struct PLC.

Không được thích gì đặt nấy.

#### `--prompt`

Đổi text prompt CLI.

Ví dụ:

```bash
python3 main.py --cli --prompt "delta> "
```

#### Bước 4: parse args

```python
args = parser.parse_args()
```

#### Bước 5: chặn mode chưa hỗ trợ

```python
if not args.cli:
    parser.error("Only --cli mode is implemented right now.")
```

Tức là hiện tại:

- bắt buộc phải chạy với `--cli`

#### Bước 6: kiểm tra slots

```python
if args.interpolar_points <= 0:
    parser.error("--package-slots must be a positive integer.")
```

#### Bước 7: tạo multiprocessing context

```python
ctx = mp.get_context("spawn")
command_queue = ctx.Queue()
response_queue = ctx.Queue()
```

Tạo 2 queue nói chuyện giữa main và worker.

#### Bước 8: tạo worker process

```python
worker = ctx.Process(
    target=_worker,
    args=(command_queue, response_queue, args.ip, args.port, args.interpolar_points),
    daemon=True,
)
```

`daemon=True` nghĩa là process phụ phụ thuộc vào process chính.

#### Bước 9: chạy worker

```python
worker.start()
```

#### Bước 10: chờ worker báo đã sẵn sàng

```python
startup = _wait_for_response(response_queue, timeout=10.0)
```

Rồi chia 3 trường hợp:

##### Trường hợp 1: không có phản hồi

In cảnh báo.

##### Trường hợp 2: phản hồi OK

In:

```python
[INFO] Worker connected to ...
```

##### Trường hợp 3: phản hồi lỗi

Main:

- in lỗi
- gửi `shutdown`
- `join`
- nếu chưa chết thì `terminate`
- rồi `return`

### 10.5. Hàm lồng `dispatch(...)`

Xem tại [main.py](/home/tangerine/Share/Global%20Share/Documents/Delta_robot/main.py:139)

Đây là hàm main truyền cho CLI.

Nó làm cầu nối:

- CLI gọi `dispatch(package)`
- `dispatch` bỏ package vào `command_queue`
- worker lấy package ra và gửi PLC
- worker trả status về `response_queue`
- `dispatch` nhận status đó rồi trả về cho CLI

Code:

```python
command_queue.put({"type": "send", "package": package})
```

Rồi chờ response:

```python
response = _wait_for_response(response_queue, timeout=10.0)
```

Nếu timeout:

- in cảnh báo
- trả `None`

Nếu worker báo lỗi:

- in lỗi
- trả `None`

Nếu OK:

- trả `response.get("status")`

Tức là CLI cuối cùng chỉ quan tâm phần status của PLC.

### 10.6. Hàm lồng `request_status()`

Xem tại [main.py](/home/tangerine/Share/Global%20Share/Documents/Delta_robot/main.py:150)

Tương tự `dispatch`, nhưng chỉ hỏi status.

Nó gửi:

```python
{"type": "status"}
```

Rồi đợi worker phản hồi.

### 10.7. Gọi `run_interactive(...)`

```python
run_interactive(
    dispatch,
    request_status,
    interpolar_points=args.interpolar_points,
    prompt=args.prompt,
)
```

Đây là lúc main nói với CLI:

- đây là hàm để gửi package
- đây là hàm để hỏi status
- đây là số slot
- đây là prompt

Từ đây trở đi, CLI bắt đầu nhận lệnh từ người dùng.

### 10.8. `finally` trong `main()`

Sau khi CLI kết thúc:

```python
finally:
    command_queue.put({"type": "shutdown"})
    _wait_for_response(response_queue, timeout=5.0)
    worker.join(timeout=5.0)
    if worker.is_alive():
        worker.terminate()
        worker.join(timeout=5.0)
```

Ý nghĩa:

1. bảo worker tự shutdown
2. đợi worker trả lời
3. đợi worker chết tử tế
4. nếu vẫn lì thì terminate cưỡng bức

Đây là cleanup khá chuẩn.

### 10.9. `if __name__ == "__main__":`

Xem tại [main.py](/home/tangerine/Share/Global%20Share/Documents/Delta_robot/main.py:177)

Nếu chạy trực tiếp `main.py` thì gọi `main()`.

---

## 11. Luồng chạy thực tế từ đầu đến cuối

Phần này là phần quan trọng nhất nếu bạn muốn hiểu "chương trình sống như thế nào".

### 11.1. Ví dụ: chạy chương trình

Bạn gõ:

```bash
python3 main.py --cli
```

### 11.2. `main.py` bắt đầu chạy

- tạo parser
- đọc config
- kiểm tra `--cli`
- tạo 2 queue
- tạo worker process

### 11.3. Worker process khởi động

Worker chạy hàm `_worker(...)`.

Nó:

- tạo `PLCGateway`
- connect PLC
- gửi message `"connected"` về main

### 11.4. Main nhận message `"connected"`

Main in:

```text
[INFO] Worker connected to 192.168.250.1:502
```

### 11.5. Main gọi `run_interactive(...)`

CLI hiện lên:

```text
Delta Robot CLI
Type 'help' for available commands.
robot>
```

### 11.6. Bạn gõ lệnh

Ví dụ:

```text
goto 100 0 -200
```

### 11.7. `cli.py` parse lệnh

`_parse_plan(...)` tạo ra 1 package:

1. package `goto_absolute`
2. package `pick`

### 11.8. `run_interactive(...)` gọi `dispatch(package)` cho package đầu

Main bỏ message vào `command_queue`:

```python
{"type": "send", "package": {...}}
```

### 11.9. Worker nhận message `send`

Worker gọi:

```python
gateway.send_package(...)
```

### 11.10. `EthernetCom.py` chuẩn hóa package

Nó đảm bảo:

- đúng `commandID`
- đúng `argument_number`
- đúng đủ `argument_x/y/z/time`
- đúng số phần tử mỗi mảng

### 11.11. `EthernetCom.py` ghi xuống PLC

Ví dụ:

- `pc_package.commandID`
- `pc_package.argument_number`
- `pc_package.argument_x[0]`
- ...

### 11.12. Worker đọc status

Sau khi gửi xong, worker gọi `gateway.get_package()`.

### 11.13. Worker trả status về main

Main nhận được status rồi đưa lại cho CLI.

### 11.14. CLI in status

Ví dụ:

```text
[INFO] PLC status: task_doing=2, task_state=1
```

### 11.15. CLI gửi package thứ hai: `pick`

Lặp lại quy trình tương tự.

### 11.16. Bạn gõ `quit`

CLI trả về khỏi `run_interactive(...)`.

### 11.17. `main.py` cleanup

- gửi `shutdown` cho worker
- worker disconnect PLC
- main join worker

Chương trình kết thúc.

---

## 12. Giải nghĩa vài dòng Python "nhìn đáng sợ"

### 12.1. `response.get("ok", False)`

`dict.get(key, default)` nghĩa là:

- lấy value của key
- nếu key không tồn tại thì lấy default

Ví dụ:

```python
response.get("ok", False)
```

nghĩa là:

- nếu dict có key `"ok"` thì lấy nó
- không có thì coi như `False`

### 12.2. `message["package"]`

Khác với `.get(...)`.

`message["package"]` nghĩa là:

- bắt buộc key `"package"` phải tồn tại
- nếu không sẽ lỗi `KeyError`

### 12.3. `status_dict or None`

Trong Python, dict rỗng là falsey.

Nên:

```python
return status_dict or None
```

nghĩa là:

- nếu `status_dict` có dữ liệu thì trả nó
- nếu rỗng thì trả `None`

### 12.4. `ip or self.config.ip_address`

Trong Python:

- nếu `ip` là giá trị thật thì lấy `ip`
- nếu `ip` là `None` hoặc rỗng thì lấy vế sau

### 12.5. `package = package.to_dict(self.interpolar_points)`

Nghĩa là:

- object `package` ban đầu là `RobotPacket`
- sau dòng này nó trở thành `dict`

Python cho phép cùng một biến đổi sang giá trị kiểu khác.

---

## 13. Vì sao thiết kế hiện tại dùng worker process thay vì gửi PLC trực tiếp trong CLI?

Lý do dễ hiểu:

1. Tách UI với giao tiếp PLC.
2. Nếu chỗ giao tiếp PLC bị treo, CLI chính vẫn dễ kiểm soát hơn.
3. Main và CLI không cần giữ trực tiếp object PLC.
4. Dễ mở rộng sau này nếu muốn thêm scheduler hoặc xử lý ảnh.

Bạn có thể xem nó như:

- process chính: "ông quản lý"
- process phụ: "ông kỹ thuật nói chuyện với PLC"

---

## 14. Các lệnh CLI hiện hỗ trợ

### 14.1. `stop`

Tạo package lệnh dừng.

### 14.2. `go <theta1> <theta2> <theta3>`

Ví dụ:

```text
go 1 2 3
```

Tạo package `goto_relative`.

### 14.3. `goto <x> <y> <z>`

Ví dụ:

```text
goto 100 0 -200
```

Tạo package `goto_absolute`.

### 14.4. `pick`

Tạo package `pick`.

### 14.5. `release`

Tạo package `release`.

### 14.6. `go_trajectory <preset>`

Ví dụ:

```text
go_trajectory demo
```

Tạo package quỹ đạo từ preset.

### 14.7. `calib`

Tạo package `calibrate`.

### 14.8. `status`

Không gửi package mới.

Chỉ hỏi trạng thái hiện tại từ PLC.

### 14.9. `help` hoặc `?`

In trợ giúp.

### 14.10. `quit` hoặc `exit`

Thoát CLI.

---

## 15. Những chỗ dễ nhầm cho người mới

### 15.1. `go` không phải là Python keyword ở đây

Nó chỉ là chuỗi người dùng nhập vào CLI.

### 15.2. `goto_absolute` và `goto_relative` không phải hàm

Chúng chỉ là tên command được map sang `COMMAND_ID`.

### 15.3. `argument_number` không phải là độ dài thật của các list

Các list vẫn phải luôn dài bằng `slots`.

`argument_number` chỉ nói bao nhiêu phần tử đầu là có ý nghĩa thật.

Ví dụ:

```python
{
    "argument_number": 1,
    "argument_x": [10.0, 0.0, 0.0, 0.0]
}
```

Nghĩa là:

- chỉ phần tử đầu tiên có nghĩa
- nhưng list vẫn phải đủ 4 phần tử

### 15.4. `type` trong message queue không phải `type()` built-in

Nó chỉ là key trong dict message:

```python
{"type": "send", "package": ...}
```

### 15.5. `RobotPacket` không tự gửi được lên PLC

Nó chỉ là object dữ liệu.

Người thực sự gửi là `PLCGateway.send_package(...)`.

### 15.6. `connect()` hiện tại không phải là kiểm tra sâu mọi thứ

Trong code hiện tại, `connect()` chủ yếu:

- đảm bảo có `pylogix`
- set trạng thái `connected`

Việc read/write thật sự mới là lúc giao tiếp tag rõ hơn.

---

## 16. Ví dụ cụ thể để tự trace bằng đầu

### 16.1. Ví dụ `stop`

Bạn gõ:

```text
stop
```

CLI build package:

```python
{
    "commandID": 0,
    "argument_number": 0,
    "argument_x": [0.0, 0.0, 0.0, 0.0],
    "argument_y": [0.0, 0.0, 0.0, 0.0],
    "argument_z": [0.0, 0.0, 0.0, 0.0],
    "argument_time": [0.0, 0.0, 0.0, 0.0],
}
```

Worker gửi package này.

### 16.2. Ví dụ `go 1 2 3`

CLI build:

```python
{
    "commandID": 1,
    "argument_number": 1,
    "argument_x": [1.0, 0.0, 0.0, 0.0],
    "argument_y": [2.0, 0.0, 0.0, 0.0],
    "argument_z": [3.0, 0.0, 0.0, 0.0],
    "argument_time": [0.0, 0.0, 0.0, 0.0],
}
```

### 16.3. Ví dụ `goto 10 20 -30`

CLI build package 1:

```python
{
    "commandID": 2,
    "argument_number": 1,
    "argument_x": [10.0, 0.0, 0.0, 0.0],
    "argument_y": [20.0, 0.0, 0.0, 0.0],
    "argument_z": [-30.0, 0.0, 0.0, 0.0],
    "argument_time": [0.0, 0.0, 0.0, 0.0],
}
```

### 16.4. Ví dụ `go_trajectory demo`

Nếu preset `demo` có 4 điểm thì package sẽ có:

- `commandID = 3`
- `argument_number = 4`
- `argument_x` chứa 4 giá trị x
- `argument_y` chứa 4 giá trị y
- `argument_z` chứa 4 giá trị z
- `argument_time` chứa 4 giá trị time

---

## 17. Nếu muốn tự đọc code hiệu quả thì đọc theo thứ tự nào?

Thứ tự dễ nhất cho người mới:

1. Đọc [Algorithm.md](/home/tangerine/Share/Global%20Share/Documents/Delta_robot/Algorithm.md:9) để biết package phải trông như thế nào.
2. Đọc [modules/EthernetCom.py](/home/tangerine/Share/Global%20Share/Documents/Delta_robot/modules/EthernetCom.py:65) để hiểu package chuẩn được tạo ra ra sao.
3. Đọc [modules/cli.py](/home/tangerine/Share/Global%20Share/Documents/Delta_robot/modules/cli.py:115) để hiểu câu lệnh text được đổi thành package thế nào.
4. Cuối cùng đọc [main.py](/home/tangerine/Share/Global%20Share/Documents/Delta_robot/main.py:82) để hiểu chương trình nối các phần này lại với nhau ra sao.

---

## 18. Tóm tắt một câu cho từng file

### `modules/EthernetCom.py`

Tạo và chuẩn hóa package, rồi đọc/ghi package sang PLC qua `pylogix`.

### `modules/cli.py`

Đổi lệnh text người dùng nhập thành package đúng chuẩn PLC.

### `main.py`

Khởi động chương trình CLI, tạo worker process để giao tiếp PLC, rồi nối CLI với worker đó.

---

## 19. Tóm tắt siêu ngắn nếu đầu đã quá tải

Nếu bạn chỉ nhớ 5 điều, hãy nhớ 5 điều này:

1. PLC cần một struct cố định, nên Python luôn phải gửi đủ số phần tử.
2. `cli.py` biến lệnh text thành package.
3. `EthernetCom.py` biến package thành các lệnh ghi tag PLC.
4. `main.py` là dây nối giữa CLI và PLC worker.
5. `argument_number` là số dữ liệu thật dùng, nhưng các mảng vẫn phải đủ độ dài `slots`.

---

## 20. Cách tự luyện sau khi đọc xong

Bạn có thể tự thử mấy bài này:

1. Tự viết ra package mà lệnh `pick` sẽ tạo.
2. Tự giải thích bằng miệng luồng `goto 100 0 -200`.
3. Tự sửa `PRESET_TRAJECTORIES["home"]` rồi đoán package mới.
4. Tự tìm trong code xem chỗ nào đảm bảo list luôn đủ 4 phần tử.
5. Tự chỉ ra chỗ nào trong code chịu trách nhiệm gửi `shutdown` cho worker.

Nếu bạn làm được 5 bài đó mà không nhìn lời giải, nghĩa là bạn đã hiểu code này ở mức khá ổn cho người mới.
