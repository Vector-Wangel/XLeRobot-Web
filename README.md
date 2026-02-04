## [Run MuJoCo + 3DGS in your Browser.](https://vector-wangel.github.io/MuJoCo-GS-Web/)

Load and Run MuJoCo 3.3.8 Models using JavaScript and the official MuJoCo WebAssembly Bindings.

<table>
  <tr>
    <td><img src="https://github.com/user-attachments/assets/85466388-9868-4319-90bb-e40d331bcdfd" width="250"/></td>
    <td><img src="https://github.com/user-attachments/assets/4d398904-d2c3-4f95-b8dc-57d225317f8a" width="250"/></td>
    <td><img src="https://github.com/user-attachments/assets/b41f6159-16c4-4c64-a104-a0ac65586b09" width="250"/></td>
  </tr>
  <tr>
    <td><img src="https://github.com/user-attachments/assets/17a240fe-b234-44a5-a5ab-c31875b1567c" width="250"/></td>
    <td><img src="https://github.com/user-attachments/assets/1327e33a-af17-4b73-9b65-f6b531110563" width="250"/></td>
    <td><img src="https://github.com/user-attachments/assets/10bc5d06-a1f9-4e50-98b8-5df2dffb448f" width="250"/></td>
  </tr>
</table>




## Features

- **MuJoCo 3.3.8 WebAssembly**: MuJoCo running entirely in the browser, works on any devices, even your phone.

- **Hybrid Rendering**: Combine MuJoCo with photorealistic 3D Gaussian Splatting scenes, with physics objects correctly occlude and interact with 3DGS environments

- **Toggle Mode**: Switch between pure physics rendering and hybrid 3DGS+physics mode

- **Mouse Interaction**: Click and drag any physics object in the scene
  
- **Keyboard teleop**: End-effector teleop controller for SO101 (analytical) and Franka Panda (numerical, SGD)

- **Import any robot**: By uploading the whole folder
  - The folder should only contain a single .xml, along with /assets folder
  - Find available robots at [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie))

<img width="1737" height="732" alt="image" src="https://github.com/user-attachments/assets/fba08231-0dfd-4bc6-bec6-458b6abc58d5" />
  
- **Import any scene**: use any 3DGS scene you want
  - .spz file, recommend: [Marble from World Labs](https://marble.worldlabs.ai/)
  - If you only upload the .spz file, there will not be any collision other than the floor
  - You can add a 3DGS scene along with an .xml file for collision setup
  
<img width="1659" height="558" alt="image" src="https://github.com/user-attachments/assets/50807972-784d-4fa0-a4b3-a2920dff86c9" />



Easiest way to find collision boxes' coordinate: adding boxes in [the studio in Marble](https://marble.worldlabs.ai/projects) and give the bounding boxes info to AI to generate a collision.xml

<img width="2220" height="837" alt="image" src="https://github.com/user-attachments/assets/55ad2873-8e1a-4eef-9c84-2df4e2b82951" />



### 🤖 Supported Robots (& keyboard control)

#### **XLeRobot** - Dual-Arm Mobile Manipulator
- **DOF**: 2 (base) + 10 (dual 5-DOF arms) + 2 (grippers) + 2 (head)
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

## TODO

- Add simple RL policy deployments (humanoid/dog walking/running, arms/mobile robot picking)

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

## 🙏 Acknowledgements

This project is built based on [mujoco-wasm](https://github.com/zalo/mujoco_wasm), [sparkjs](https://sparkjs.dev/), and [human policy viewer](https://github.com/Axellwppr/humanoid-policy-viewer).




















