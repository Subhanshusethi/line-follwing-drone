# 🛩️ UAV Vision-Based Lane Tracking and Control System

This project implements a **real-time UAV vision-based lane-following system** using OpenCV and DroneKit-Python. It processes video feeds to detect lanes using LAB color space segmentation and dynamically adjusts drone velocity through PID-tuned control signals based on the centroid of the detected lane region.

---

## 📦 Features

* 🧠 **Color-based Lane Detection** using LAB color space
* 🎯 **Centroid Extraction** from detected lane contours
* 🔁 **PID Control** logic to calculate error and generate tuned directional movement
* 🚁 **Drone Control with DroneKit** — GUIDED mode, takeoff, velocity setpoint commands
* 📸 **Live Camera Feed Processing** with sharpening, erosion, dilation
* 🔧 **LAB Tuner Utility** for pixel threshold selection with image masking and visualization

---

## 🚀 System Architecture

1. **Camera Feed (USB/Webcam)**
2. **Image Preprocessing** (resize, sharpen, LAB thresholding)
3. **Contour Detection** to locate lane region
4. **Centroid Calculation** using image moments
5. **PID-based Angle Tuning**
6. **Velocity Control** sent to the drone via MAVLink
7. **Live Telemetry Feedback**: Roll, Pitch, Yaw

---

## 🧪 Key Components

### `main.py` (Core Vision + Control)

* Captures live video, sharpens and thresholds image.
* Extracts contours and computes centroid.
* Calculates velocity using PID controller and sends it to drone.
* Handles drone takeoff and landing using DroneKit.

### `lab_tuner.py` (LAB Threshold Calibration)

* Interactive tool to find optimal LAB values for lane color segmentation.
* Displays original, masked, and segmented images in real-time.
* LAB values can be copied directly into the main script.


## 🎮 Usage

### Run LAB Color Tuner:

```bash
python lab_tuner.py
```

Use trackbars to find color thresholds for the lane.

### Run Main Control Script:

```bash
python main.py
```

* UAV takes off to 12 meters and begins lane-following.
* Press `q` to stop the loop and trigger safe landing.

---

## 🧠 PID Parameters

* `kp`, `ki`, `kd`: Used to tune lane-following smoothness.
* You can dynamically adjust gains in the code or add trackbars for real-time tuning.

---

## 📂 Project Structure

```
📁 UAV_Lane_Tracking/
│
├── main.py              # Vision + control loop
├── lab_tuner.py         # LAB segmentation calibration tool
└── README.md            # Project documentation
```

---

## 🎥 Demo (Optional)

📌 *Videos and additional test footage available upon request.*

---

## 💬 Notes

* Ensure camera is connected at index `2` or change `cv2.VideoCapture(2)` accordingly.
* UAV must be in GUIDED mode and have sufficient GPS lock before takeoff.


