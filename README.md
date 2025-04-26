# LiDAR-Scanner
# 360° Indoor Spatial Mapping System

This project is a low-cost embedded system designed for indoor 3D spatial scanning and visualization. Built using a VL53L1X Time-of-Flight sensor, stepper motor, and TM4C1294 microcontroller, the device collects distance measurements across a 360° field and transmits them via UART to a PC for real-time visualization using Python and Open3D.

## 🔧 Features
- 360° scanning using a stepper motor with 11.25° resolution (32 total steps)
- Distance measurement using VL53L1X ToF sensor (up to 4 meters)
- UART communication with PC at 115200 baud
- Real-time 2D/3D point cloud rendering with Open3D in Python
- LED indicators for scan status and step feedback
- Push-button scan trigger with sensor reset logic via XSHUT

## 🧰 Hardware Used
- **Microcontroller:** TM4C1294 (TI MSP-EXP432E401Y LaunchPad)
- **Distance Sensor:** VL53L1X ToF (I2C interface)
- **Stepper Motor:** 28BYJ-48 + ULN2003 driver
- **Estimated Cost:** ~$65 CAD

## 💻 Software Components
- **Microcontroller Code:** C (Keil / CCS), includes motor control, I2C read, UART output
- **PC Visualization Code:** Python + PySerial + Open3D

## 🔌 Pin Mapping
| Component     | Pin Connections (TM4C1294) |
|---------------|----------------------------|
| I2C SDA       | PB3                        |
| I2C SCL       | PB2                        |
| XSHUT         | PG0                        |
| Stepper IN1-4 | PH0, PH1, PH2, PH3         |
| LED (status)  | PF4                        |
| LED (step)    | PN1                        |
| Button        | PJ0                        |

##  How to Run

1. **Set Up Hardware**
   - Connect VL53L1X to I2C pins
   - Connect stepper motor via ULN2003 to GPIO
   - Connect TM4C1294 via USB

2. **Flash Microcontroller**
   - Load and run the C code (e.g., `main.c`) onto the TM4C1294 board.

3. **Start Scan**
   - Press onboard button (PJ0) to trigger a 360° scan.
   - LEDs indicate scan progress.

4. **Visualize**
   - Run `visualizer.py` in Python.
   - Scan data is parsed and displayed using Open3D.
   - Data is saved as `tof_radar.xyz`.

## 📸 Output Example
The point cloud accurately captures indoor layouts such as hallways, identifying wall edges, doors, and open space.

## Applications
- Indoor navigation & mapping
- Accessibility scanning for public buildings
- Mobile robotics & SLAM prototypes

## Limitations
- Max range: ~4m; sensitive to surface reflectivity
- Sensor read time (~140ms) is the primary speed bottleneck
- Minor floating-point error due to trigonometric calculations

## 📚 References
- [Texas Instruments TM4C1294 Datasheet](https://www.ti.com/product/TM4C1294NCPDT)
- [VL53L1X Datasheet - STMicroelectronics](https://www.st.com/en/imaging-and-photonics-solutions/vl53l1x.html)
- [Open3D Documentation](http://www.open3d.org/docs/latest/)

---

> Developed by Viha Tripathi (Winter 2025)
