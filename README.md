<p align="center">

  <a href="https://vector-wangel.github.io/XLeRobot-Web/"><img src="https://github.com/user-attachments/assets/f6880e23-efad-4590-8feb-bbeaf18616b6" href></a>
</p>




## Run XLeRobot in MuJoCo in your Browser.

Load and Run MuJoCo 3.3.8 Models using JavaScript and the official MuJoCo WebAssembly Bindings.

### [See the Live Demo Here](https://vector-wangel.github.io/XLeRobot-Web/)
<img width="1271" height="975" alt="image" src="https://github.com/user-attachments/assets/e79e8659-d663-4157-ab38-abd24dde9151" />


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







