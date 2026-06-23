# EtherCAT Network Configuration Summary

This document summarizes the EtherCAT network configuration for the current Sysmac Studio project, based on the provided hardware tree and slave device settings.

## 1. Network Topology Overview
The system is configured with an Omron EtherCAT Master controlling three slave devices.

* **Master:** Configured as the root node of the EtherCAT network that is the PLC.
* **Slaves (Nodes):**
    * **Node 47:** Device E002 (MADLN05BE)
    * **Node 72:** Device E001 (MADLN05BE)
    * **Node 73:** Device E003 (MADLN05BE)

## 2. Master Configuration Settings
The EtherCAT Master is configured with the following communication parameters:

| Parameter | Value |
| :--- | :--- |
| **PDO Communication Cycle Time** | 4000 µs (4 ms) | 
* It means that the PLC will send a frame to all the slave at the same time and wait for the response from them.  
| **Reference Clock** | Exist |
* It means that the slave will synchronize their internal clock with the master's clock. This is critical for the synchronized motion control required by the Delta Robot application.
| **Total Cable Length** | 1000 m |
* It means that the total length of the EtherCAT cable is 1000 meters or less
| **Slave Startup Wait Time** | 30 s |
* It means that the maximum wait time for the slaves to start up is 30 seconds. If the slaves do not start up within this time, the system will generate an error.
| **Fail-soft Operation** | Enabled |
* It means that the system will continue to operate even if one of the slaves fails. This is a safety feature that is critical for the Delta Robot application.
| **Revision Check Method** | Setting <= Actual |
* It means that the system will check the revision of the slaves and compare it with the revision set in the configuration. If the revision is not the same, the system will generate an error.

## 3. Slave Device Details (Typical Configuration: E002)
Each slave (MADLN05BE servo drive) shares a similar configuration profile. Below are the key settings for **Node 47 (E002)**:

* **Device Name:** E002
* **Revision:** 0x00010000 -> download from omron official catalog website
* **Distributed Clocks (DC):** Enabled (DC SYNC) for synchronized motion control required by the Delta Robot application
* **PDO Map Settings:**
    * **Receive PDOs:** 0x6040, 0x6060, 0x607A, 0x60B8, 0x60B9, 0x60BA
    * **Transmit PDOs:** 0x603F, 0x6041, 0x6061, 0x6064, 0x60F4, 0x60FD

---

### Technical Notes for Troubleshooting
1.  **Synchronization:** Distributed Clocks (DC) are enabled for the slaves, which is critical for the synchronized motion control required by the Delta Robot application.
    *   **How Synchronization in EtherCAT Works:** In an EtherCAT network, Distributed Clocks (DC) synchronize the internal clocks of all slave devices (MADLN05BE servo drives) with a master reference clock (typically the first DC-capable slave, E002/E001, which locks to the PLC master clock). The propagation delay between each slave is measured, and local clocks are adjusted with sub-microsecond precision (jitter < 1 µs).
    *   **Why DC is Critical for Delta Robots:** A Delta Robot consists of a parallel kinematics mechanism where three separate arms are mechanically connected to a single moving platform (end-effector).
        *   **Preventing Mechanical Binding:** If the three axes are not precisely synchronized in execution, they will pull/push against each other, causing mechanical vibration, high current spikes in the servo motors, or physical binding (structural damage to the carbon fiber rods or joints).
        *   **Path Accuracy:** The controller calculates trajectory paths in Cartesian coordinates and maps them to joint angles for each axis. A timing difference of even a few microseconds between the motors executing their target coordinates will cause the end-effector to deviate from the planned path, resulting in poor tracking accuracy or failed pick-and-place operations.
2.  **Communication Cycle:** The cycle time of 4ms suggests a moderate-speed motion application. If jitter or synchronization errors occur, verify that the task cycle in the PLC program is synchronized with this EtherCAT cycle.
3.  **Device Revision:** The "Setting <= Actual" revision check method provides flexibility if you replace a drive with a newer hardware revision, preventing unnecessary configuration errors during startup.

---

## 4. Understanding Process Data Objects (PDO) in EtherCAT
In EtherCAT communication, data exchange is divided into two primary types of mechanisms: **PDO (Process Data Objects)** and **SDO (Service Data Objects)**.

*   **PDO (Process Data Objects):** Used for **real-time, cyclic** data transfer. PDOs are configured during startup and transfer operational data (like target positions, actual positions, control words, and status words) at every communication cycle (e.g., every 4ms in this system).
*   **SDO (Service Data Objects):** Used for **non-real-time, acyclic** parameter access (mailbox communication). SDOs are used for configuring parameters, reading diagnostic error codes, or changing device settings that do not require cyclic real-time updates.

### RxPDO vs. TxPDO
*   **RxPDO (Receive PDO - Master to Slave):** Process data sent by the EtherCAT Master (PLC) and received by the Slave (Servo Drive). These are commands such as target positions or control commands.
*   **TxPDO (Transmit PDO - Slave to Master):** Process data sent (transmitted) by the Slave (Servo Drive) and received by the EtherCAT Master (PLC). These are feedback values such as actual positions, status words, or sensor inputs.

---

## 5. Axis Variable to PDO Mapping Configuration (Axis Basic Settings)
Based on the **Axis Basic Settings** configuration in Sysmac Studio (shown in [PDO_config.jpg](file:///d:/1_Uni_ute/Graduation_Project/Delta_All/1.sysmac_code/Ethercat_Config/ethercat_config_img/PDO_config.jpg)), specific variables in the PLC's Motion Control (MC) Function Module are mapped directly to the PDOs of **Node 47: MADLN05BE (E002)**:

| Axis Basic Settings Function | I/O Direction | Slave Device Parameter (CoE Object) | Description |
| :--- | :--- | :--- | :--- |
| **1. Controlword** | Output (RxPDO) | `6040h-00.0` (Receive PDO mapping 1) | Controls the drive state machine (e.g., servo ON/OFF, quick stop, fault reset) using the CiA 402 drive profile. |
| **3. Target position** | Output (RxPDO) | `607Ah-00.0` (Receive PDO mapping 1) | Command position value generated by the PLC's motion control engine for each 4ms cycle. |
| **22. Statusword** | Input (TxPDO) | `6041h-00.0` (Transmit PDO mapping 1) | Displays the current operating state of the servo drive (e.g., Ready to Switch On, Switched On, Fault). |
| **23. Position actual value** | Input (TxPDO) | `6064h-00.0` (Transmit PDO mapping 1) | Feedback position from the motor's encoder, used by the PLC to monitor position and close the control loop. |

### Configuration Details & Mode of Operation
*   **Unassigned Fields:** Other functions such as *Target velocity* (`60FFh`), *Target torque* (`6071h`), *Velocity actual value*, *Torque actual value*, and *Modes of operation* (`6060h`) are set to `<Not assigned>`.
*   **CSP Control Mode (Cyclic Synchronous Position):** This configuration indicates that the Delta Robot's axes are set up in **CSP** mode. In CSP mode, the PLC's Motion Control module dynamically generates path profiles and sends only the target position (`607Ah`) and Controlword (`6040h`) cyclically. The servo drive's internal cascade controller handles speed and current loops internally to track the target position.
*   **Performance Optimization:** Leaving unnecessary parameters unassigned minimizes the EtherCAT frame size. This reduces bandwidth consumption and network overhead, ensuring stable, jitter-free real-time communication within the 4ms cycle time.