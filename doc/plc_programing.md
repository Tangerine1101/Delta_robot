# Siemens S7-1200 PLC Configuration Guide for PC Communication (Python — Snap7)

This document guides PLC engineers on how to configure hardware, set up Data Blocks (DBs), and define data formats in **TIA Portal** to be compatible with the [EthernetCom.py](file:///home/tangerine/Share/Global%20Share/Documents/Delta_robot/modules/EthernetCom.py) communication gateway using the `python-snap7` library.

---

## 1. CPU Hardware Configuration (TIA Portal)

To allow the PC to connect and exchange data with the PLC via the Snap7 library, you must enable PUT/GET communication on the S7-1200 CPU.

1. Open the project in **TIA Portal**.
2. Select **Device configuration** → Double-click the **CPU S7-1200** image.
3. In the properties panel below, navigate to **Properties** → **General** → **Protection & Security** → **Connection mechanisms**.
4. Check the box: **"Permit access with PUT/GET communication from remote partner"**.
5. Compile and download the hardware configuration to the PLC.

---

## 2. Data Block (DB) Properties Configuration

By default, S7-1200/S7-1500 PLCs use **Optimized block access**, which hides the byte offset addresses of variables and prevents Snap7 from accessing them directly by byte address.

You must disable this feature for the communication DBs:

1. Right-click the target Data Block (e.g. `DB1`, `DB2`) → Select **Properties**.
2. Go to the **Attributes** tab.
3. Uncheck **"Optimized block access"**.
4. Click **OK**.
5. **Compile** the Data Block. The **Offset** column will now appear showing the byte address of each variable (`0.0`, `4.0`, `8.0`, …).

---

## 3. Data Structure Definitions in the DB

For raw byte array reads/writes between Python and the PLC to be correct, the variables declared in the PLC DB must exactly match the Python structs in **order**, **data type**, and **size**.

### 3.1. Command Write DB (PC → PLC)
* **Default DB number:** `1` (corresponds to `SIEMENS_DB_WRITE = 1` in Python)
* **Write start offset:** `0` (corresponds to `SIEMENS_DB_WRITE_OFFSET = 0`)
* **Structure declaration in TIA Portal:**

| Variable Name (PLC) | Data Type (PLC) | Offset | Description | Python Field |
| :--- | :--- | :--- | :--- | :--- |
| **CommandID** | DINT (Double Integer) | `0.0` (4 bytes) | Control command ID | `CommandID` (ctypes.c_int32) |
| **rotate** | REAL (Floating Point) | `4.0` (4 bytes) | Absolute rotation angle (4th DOF) | `rotate` (ctypes.c_float) |
| **speed** | REAL (Floating Point) | `8.0` (4 bytes) | Requested conveyor speed | `speed` (ctypes.c_float) |

> [!IMPORTANT]
> Total packet size sent to the PLC is **12 bytes**. Do not declare any additional variables between or after these three within the offset range `0.0` to `11.0`.

### 3.2. Status Read DB (PLC → PC)
* **Default DB number:** `2` (corresponds to `SIEMENS_DB_READ = 2` in Python)
* **Read start offset:** `0` (corresponds to `SIEMENS_DB_READ_OFFSET = 0`)
* **Structure declaration in TIA Portal:**

| Variable Name (PLC) | Data Type (PLC) | Offset | Description | Python Field |
| :--- | :--- | :--- | :--- | :--- |
| **rotate_current** | REAL (Floating Point) | `0.0` (4 bytes) | Current suction cup rotation angle | `rotate_current` (ctypes.c_float) |
| **speed_current** | REAL (Floating Point) | `4.0` (4 bytes) | Current actual conveyor speed | `speed_current` (ctypes.c_float) |
| **task_doing** | DINT (Double Integer) | `8.0` (4 bytes) | Command currently executing | `task_doing` (ctypes.c_int32) |
| **task_state** | DINT (Double Integer) | `12.0` (4 bytes) | Command state (0: Idle, 1: Running, 2: Done…) | `task_state` (ctypes.c_int32) |

> [!IMPORTANT]
> Total packet size read by the PC is **16 bytes**, covering offsets `0.0` through `15.0`.

---

## 4. Important Notes on Byte Order (Endianness)

* **System difference:**
  * PC (x86/x64 architecture) stores data as **Little-Endian** (low byte first).
  * Siemens S7-1200 PLC stores data as **Big-Endian** (high byte first).
* **Impact:** Transmitting raw byte arrays via Snap7 without handling endianness causes `REAL` and `DINT` values to appear byte-reversed and numerically incorrect.
* **Fix options:**
  * **Option A (Recommended — PLC side):** In the PLC program, after receiving a packet from the PC or before sending data to the PC, use the byte-swap instruction (**SWAP** or **CAD/CAW**) on all DINT and REAL variables to ensure the correct values are read.
  * **Option B (Python side):** If the PLC transmits raw Big-Endian bytes without swapping, change the Python struct from `ctypes.Structure` to `ctypes.BigEndianStructure` so the PC handles the byte reversal automatically. *(This is the approach currently implemented in `EthernetCom.py`.)*
