<p align="center">

  <a href="https://vector-wangel.github.io/XLeRobot-Web/"><img src="https://github.com/user-attachments/assets/f6880e23-efad-4590-8feb-bbeaf18616b6" href></a>
  <a href="https://vector-wangel.github.io/XLeRobot-Web/"><img src="https://github.com/user-attachments/assets/9f6faa1c-6f4e-40ea-a28d-795b0f3323d2" href></a>
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











