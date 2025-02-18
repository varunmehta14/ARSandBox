# **Augmented Reality (AR) Sandbox using Orbbec Femto Depth Sensor**  

## **Overview**  
This project integrates an **Orbbec Femto depth sensor** with **Rhino and Grasshopper** to create an **interactive AR sandbox**. The system captures **real-time depth data**, processes it into a **3D point cloud**, and generates **dynamic terrain models**. Additionally, it simulates **water flow dynamics** and **rainfall effects**, allowing for interactive real-world physics modeling.  

## **Features**  
✅ **Real-time depth sensing** using Orbbec Femto Bolt sensor  
✅ **3D point cloud processing** and conversion to **meshes and contours**  
✅ **Dynamic terrain modeling** based on real-world surface changes  
✅ **Water simulation** including **rainfall effects and fluid flow**  
✅ **Depth image visualization** using custom **color mapping**  

## **Technology Stack**  

### **Hardware**  
- **Orbbec Femto Bolt**: Captures **depth data** with high precision  

### **Software & Libraries**  
- **C# (Grasshopper Plugin for Rhino3D)**: Main programming language  
- **K4AdotNet.Sensor**: Interfaces with Orbbec Femto depth sensor  
- **Rhino.Geometry**: Handles **point cloud processing** and **3D transformations**  
- **Grasshopper**: Provides **visual programming environment** for real-time AR processing  
- **System.Drawing & OpenCV**: Used for **depth image visualization** and **color mapping**  

## **Technical Implementation**  

### **1. Depth Data Acquisition & Processing**  
- Uses the **Orbbec Femto SDK** to capture **real-time depth frames**.  
- Converts depth maps into **3D point clouds** using **camera intrinsics** (focal length, principal point).  

### **2. Point Cloud & Mesh Generation**  
- Converts raw **depth data** into **3D points** using **sensor calibration parameters**.  
- Applies **A* pathfinding and contour extraction** for **terrain visualization**.  

### **3. Water Simulation & Rainfall Effects**  
- Implements a **grid-based water heightmap**, simulating **fluid flow**.  
- Adds **raindrop simulation**, affecting **water distribution dynamically**.  
- Uses **basic physics-based propagation** to simulate **natural water movement**.  

### **4. Visualization & Interaction**  
- **Depth images are color-coded** to represent **elevation changes**.  
- **Mesh and contour outputs** allow for **real-time AR visualization** in Rhino.  

## **Performance Optimization**  
- **Optimized depth image processing** using **multi-threading**.  
- **Memory-efficient data structures** for handling **large point clouds**.  
- **Dynamic framerate adjustments** to maintain **real-time responsiveness**.  

## **Future Enhancements**  
- **Enhancing fluid simulation** with **Navier-Stokes-based water flow**.  
- **Adding AI-driven terrain recognition** to classify different surface types.  
- **Real-time machine learning integration** for **automatic feature detection**.  

---


---
This project enables **real-time augmented reality experiences**, bridging the gap between **physical interaction and digital simulation**. 🚀  
