<p align="center">

  <a href="https://vector-wangel.github.io/XLeRobot-Web/"><img src="https://github.com/user-attachments/assets/f6880e23-efad-4590-8feb-bbeaf18616b6" href></a>
  <a href="https://vector-wangel.github.io/XLeRobot-Web/"><img src="https://github.com/user-attachments/assets/8c4577fd-515f-4512-9c79-ba2091cd5820" href></a>
</p>



## Run XLeRobot in MuJoCo in your Browser.

Load and Run MuJoCo 3.3.8 Models using JavaScript and the official MuJoCo WebAssembly Bindings.

### [See the Live Demo Here](https://vector-wangel.github.io/XLeRobot-Web/)

<table>
  <tr>
    <td><img src="https://github.com/user-attachments/assets/e4dedf6d-8ec0-4a11-aaa4-5e4fd29a41a3" width="250"/></td>
    <td><img src="https://github.com/user-attachments/assets/4d398904-d2c3-4f95-b8dc-57d225317f8a" width="250"/></td>
    <td><img src="https://github.com/user-attachments/assets/6f1e9b1e-f7b8-45d0-b563-1cc90d290a65" width="250"/></td>
  </tr>
</table>

## Features

- **MuJoCo 3.3.8 WebAssembly**: MuJoCo running entirely in the browser

- **Hybrid Rendering**: Combine MuJoCo with photorealistic 3D Gaussian Splatting scenes imported from [World Labs](https://worldlabs.ai/)

- **Toggle Mode**: Switch between pure physics rendering and hybrid 3DGS+physics mode

- **Proper Occlusion**: Physics objects correctly occlude and interact with 3DGS environments

- **Mouse Interaction**: Click and drag any physics object in the scene


### 🤖 Supported Robots

#### **XLeRobot** - Dual-Arm Mobile Manipulator
- **DOF**: 2 (base) + 12 (dual 6-DOF arms) + 2 (grippers) + 2 (head)
- **Control Method**: Inverse kinematics with keyboard teleoperation
- **Actuators**: STS3215 servos (SO-Arm101 specification)
- **Keyboard Mapping**:
  - Base: `W/S` (forward/backward), `A/D` (turn)
  - Left arm: `7/Y` (rotation), `8/U` (EE Y), `9/I` (EE X), `0/O` (pitch), `-/P` (wrist roll)
  - Right arm: `H/N` (rotation), `J/M` (EE Y), `K/,` (EE X), `L/.` (pitch), `;//` (wrist roll)
  - Grippers: `V` (left), `B` (right)
  - Head: `R/T` (pan), `F/G` (tilt)
  - Reset: `X`

#### **SO101** - Single-Arm Manipulator
- **DOF**: 6 (shoulder rotation, shoulder pitch, elbow, wrist pitch, wrist roll, gripper)
- **Control Method**: 2-link inverse kinematics
- **Actuators**: STS3215 servos with high-precision position control
- **Keyboard Mapping**:
  - Shoulder: `A/D` (rotation)
  - End effector: `W/S` (Y axis), `Q/E` (X axis)
  - Orientation: `R/F` (pitch adjustment)
  - Wrist: `Z/C` (roll)
  - Gripper: `V` (toggle)
  - Reset: `X`

#### **Franka Emika Panda** - 7-DOF Research Arm
- **DOF**: 7 (redundant arm) + 2 (parallel gripper)
- **Control Method**: Damped Least Squares inverse kinematics with gravity compensation
- **Special Features**:
  - 6-DOF pose control (position + orientation)
  - Adaptive damping for stability
  - Multi-iteration IK solver
  - High-force gripper (100N)
- **Keyboard Mapping**:
  - Position: `W/S` (X), `A/D` (Y), `Q/E` (Z)
  - Orientation: `Z/C` (roll), `R/F` (pitch), `T/G` (yaw)
  - Gripper: `V` (open), `B` (close)
  - Reset: `X`

#### **Humanoid**
- **DOF**: Full humanoid with torso, arms, and legs
- **Control Method**: Joint-level PD control
- **Interaction**: Mouse drag support for direct manipulation


## JavaScript API

```javascript
import load_mujoco from "./dist/mujoco_wasm.js";

// Load the MuJoCo Module
const mujoco = await load_mujoco();

// Set up Emscripten's Virtual File System
mujoco.FS.mkdir('/working');
mujoco.FS.mount(mujoco.MEMFS, { root: '.' }, '/working');
mujoco.FS.writeFile("/working/humanoid.xml", await (await fetch("./assets/scenes/humanoid.xml")).text());

// Load model and create data
let model = mujoco.MjModel.loadFromXML("/working/humanoid.xml");
let data  = new mujoco.MjData(model);

// Access model properties directly
let timestep = model.opt.timestep;
let nbody = model.nbody;

// Access data buffers (typed arrays)
let qpos = data.qpos;  // Joint positions
let qvel = data.qvel;  // Joint velocities
let ctrl = data.ctrl;  // Control inputs
let xpos = data.xpos;  // Body positions

// Step the simulation
mujoco.mj_step(model, data);

// Run forward kinematics
mujoco.mj_forward(model, data);

// Reset simulation
mujoco.mj_resetData(model, data);

// Apply forces (force, torque, point, body, qfrc_target)
mujoco.mj_applyFT(model, data, [fx, fy, fz], [tx, ty, tz], [px, py, pz], bodyId, data.qfrc_applied);

// Clean up
data.delete();
model.delete();
```

