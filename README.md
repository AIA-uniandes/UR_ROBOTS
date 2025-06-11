# UR_ROBOTS

This repository provides a Python-based console interface for controlling a UR (Universal Robots) arm using the `urx` library and socket programming. The system includes functionalities for pose movement, coordinate transformation, and digital output control for end-effectors such as vacuum grippers (VG).

## Features

- Move robot to predefined home and manipulation poses  
- Send the robot to custom target points with automatic coordinate transformation  
- Enable and disable vacuum gripper outputs  
- Apply rotation and translation transformations using Euler angles  
- Menu-driven command-line interface  

## Project Structure

- `main.py`: Core script for controlling the UR robot, includes all movement and control logic  
- `euler_transform.py`: Contains Euler angle transformation functions  
- `transformada_puntos.py`: Implements 3D point transformation logic using rotation matrices  

## Getting Started

### Prerequisites

- Python 3.7+
- `urx` library
- `numpy`
- UR robot with IP connectivity (e.g., UR5, UR10e)
- Ensure port `30002` (for URScript commands) is open

### Installation

```bash
pip install urx numpy
```

### Running the Interface

```bash
python main.py
```

Use the numbered menu to control robot movements, execute point transformations, or activate/deactivate the gripper.

## Example Use Case

- Transform a 3D point from a local to global coordinate frame using Euler angles  
- Move the robot linearly to a transformed point  
- Toggle vacuum gripper outputs during object pickup  

## 📡 Network Configuration

Ensure your robot is reachable via the IP address set in:

```python
robot_ip = '157.253.197.147'
```

Update this value to match your local network configuration.

## Safety Warning

Always verify transformed coordinates and use low speeds/accelerations during development to prevent unexpected collisions. Run the robot in **Simulation Mode** or with **Protective Stop Zones** when possible.

## License

This project is provided for academic and prototyping purposes. Use at your own risk.

---

**Developed by:**  
[Your Name or Lab Name]  
TUM / Universidad de los Andes – Robotics Research Project
