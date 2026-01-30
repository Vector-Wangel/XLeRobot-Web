/**
 * Keyboard Control Module for MuJoCo Scenes
 *
 * Provides configurable keyboard controls for different scenes.
 * Supports both simple control mappings and complex controllers with IK.
 */

// ============================================================================
// Inverse Kinematics for 2-Link Planar Arm
// ============================================================================

/**
 * Calculate inverse kinematics for a 2-link robotic arm
 * @param {number} x - End effector x coordinate
 * @param {number} y - End effector y coordinate
 * @param {number} l1 - Upper arm length (default 0.1159 m)
 * @param {number} l2 - Lower arm length (default 0.1350 m)
 * @returns {[number, number]} - [joint2, joint3] angles in radians
 */
function inverseKinematics(x, y, l1 = 0.1159, l2 = 0.1350) {
  // Calculate joint offsets based on arm geometry
  const theta1Offset = Math.atan2(0.028, 0.11257);  // theta1 offset when joint2=0
  const theta2Offset = Math.atan2(0.0052, 0.1349) + theta1Offset;  // theta2 offset when joint3=0

  // Calculate distance from origin to target point
  let r = Math.sqrt(x * x + y * y);
  const rMax = l1 + l2;  // Maximum reachable distance
  const rMin = Math.abs(l1 - l2);  // Minimum reachable distance

  // Clamp to workspace boundaries
  if (r > rMax) {
    const scale = rMax / r;
    x *= scale;
    y *= scale;
    r = rMax;
  }
  if (r < rMin && r > 0) {
    const scale = rMin / r;
    x *= scale;
    y *= scale;
    r = rMin;
  }

  // Use law of cosines to calculate theta2 (elbow angle)
  const cosTheta2 = -(r * r - l1 * l1 - l2 * l2) / (2 * l1 * l2);
  const clampedCos = Math.max(-1, Math.min(1, cosTheta2));
  const theta2 = Math.PI - Math.acos(clampedCos);

  // Calculate theta1 (shoulder angle)
  const beta = Math.atan2(y, x);
  const gamma = Math.atan2(l2 * Math.sin(theta2), l1 + l2 * Math.cos(theta2));
  const theta1 = beta + gamma;

  // Convert to joint angles with offsets
  let joint2 = theta1 + theta1Offset;
  let joint3 = theta2 + theta2Offset;

  // Clamp to URDF limits
  joint2 = Math.max(-0.1, Math.min(3.45, joint2));
  joint3 = Math.max(-0.2, Math.min(Math.PI, joint3));

  return [joint2, joint3];
}

// ============================================================================
// XLeRobot Controller
// ============================================================================

/**
 * Actuator index mapping for xlerobot
 * Index | Name          | Type     | Description
 * ------|---------------|----------|-------------
 * 0     | forward       | motor    | Base forward/backward
 * 1     | turn          | motor    | Base rotation
 * 2     | Rotation_L    | position | Left arm shoulder rotation
 * 3     | Pitch_L       | position | Left arm shoulder pitch (IK)
 * 4     | Elbow_L       | position | Left arm elbow (IK)
 * 5     | Wrist_Pitch_L | position | Left arm wrist pitch (compensation)
 * 6     | Wrist_Roll_L  | position | Left arm wrist roll
 * 7     | Jaw_L         | position | Left gripper
 * 8     | Rotation_R    | position | Right arm shoulder rotation
 * 9     | Pitch_R       | position | Right arm shoulder pitch (IK)
 * 10    | Elbow_R       | position | Right arm elbow (IK)
 * 11    | Wrist_Pitch_R | position | Right arm wrist pitch (compensation)
 * 12    | Wrist_Roll_R  | position | Right arm wrist roll
 * 13    | Jaw_R         | position | Right gripper
 * 14    | head_pan      | position | Head pan (left/right)
 * 15    | head_tilt     | position | Head tilt (up/down)
 */

class XLeRobotController {
  constructor() {
    // Control constants (1/8 of original for finer control)
    this.JOINT_STEP = 0.002;    // 0.01 / 8
    this.EE_STEP = 0.0002;      // 0.005 / 8
    this.PITCH_STEP = 0.0025;     // 0.02 / 8
    this.TIP_LENGTH = 0.108;      // Length from wrist to end effector tip
    this.BASE_SPEED = 2;

    // Gripper positions
    this.GRIPPER_OPEN = 1.5;
    this.GRIPPER_CLOSED = -0.25;
    this.GRIPPER_COOLDOWN_FRAMES = 60;  // Frames to wait between toggles

    // Initial end effector position
    this.INITIAL_EE_POS = [0.162, 0.118];

    // State
    this.state = null;
    this.initialized = false;
  }

  /**
   * Initialize the controller state
   */
  _initState() {
    // Calculate initial IK
    const [j2, j3] = inverseKinematics(this.INITIAL_EE_POS[0], this.INITIAL_EE_POS[1]);

    this.state = {
      // Target joint positions (16 actuators)
      targetJoints: new Float64Array(16),

      // End effector positions [x, y]
      eePos1: [...this.INITIAL_EE_POS],  // Left arm
      eePos2: [...this.INITIAL_EE_POS],  // Right arm

      // Pitch adjustments for wrist orientation
      pitch1: 0.0,  // Left arm
      pitch2: 0.0,  // Right arm

      // Gripper toggle cooldown counters
      gripperCooldown: [0, 0],

      // Gripper states (false = closed, true = open)
      gripperOpen: [false, false],
    };

    // Set initial joint positions
    // Left arm
    this.state.targetJoints[2] = 1.5708;  // Rotation_L (-90 degrees)
    this.state.targetJoints[3] = j2;   // Pitch_L
    this.state.targetJoints[4] = j3;   // Elbow_L
    this.state.targetJoints[5] = j2 - j3;  // Wrist_Pitch_L (compensation)
    this.state.targetJoints[6] = 1.57;  // Wrist_Roll_L

    // Right arm
    this.state.targetJoints[8] = -1.5708;   // Rotation_R (+90 degrees)
    this.state.targetJoints[9] = j2;    // Pitch_R
    this.state.targetJoints[10] = j3;   // Elbow_R
    this.state.targetJoints[11] = j2 - j3;  // Wrist_Pitch_R (compensation)
    this.state.targetJoints[12] = 1.57;  // Wrist_Roll_R

    // Grippers closed
    this.state.targetJoints[7] = this.GRIPPER_CLOSED;   // Jaw_L
    this.state.targetJoints[13] = this.GRIPPER_CLOSED;  // Jaw_R
  }

  /**
   * Initialize the controller with model and data
   * @param {object} model - MuJoCo model
   * @param {object} data - MuJoCo data
   */
  initialize(model, data) {
    this._initState();

    // Write initial control values to position actuators
    for (let i = 2; i < 16; i++) {
      data.ctrl[i] = this.state.targetJoints[i];
    }

    this.initialized = true;
  }

  /**
   * Reset all positions to initial state
   * @param {object} data - MuJoCo data
   */
  reset(data) {
    this._initState();

    // Write reset values
    data.ctrl[0] = 0;  // forward
    data.ctrl[1] = 0;  // turn
    for (let i = 2; i < 16; i++) {
      data.ctrl[i] = this.state.targetJoints[i];
    }
  }

  /**
   * Update controls based on keyboard state
   * @param {object} keyStates - Current keyboard states
   * @param {object} model - MuJoCo model
   * @param {object} data - MuJoCo data
   */
  update(keyStates, model, data) {
    if (!this.initialized || !this.state) {
      this.initialize(model, data);
    }

    // Decrement gripper cooldowns
    if (this.state.gripperCooldown[0] > 0) this.state.gripperCooldown[0]--;
    if (this.state.gripperCooldown[1] > 0) this.state.gripperCooldown[1]--;

    // ========================================
    // Base Control (motor actuators - velocity)
    // ========================================

    // Forward/Backward (W/S)
    if (keyStates['KeyS']) {
      data.ctrl[0] = this.BASE_SPEED;
    } else if (keyStates['KeyW']) {
      data.ctrl[0] = -this.BASE_SPEED;
    } else {
      data.ctrl[0] = 0;
    }

    // Turn Left/Right (A/D)
    if (keyStates['KeyA']) {
      data.ctrl[1] = this.BASE_SPEED;
    } else if (keyStates['KeyD']) {
      data.ctrl[1] = -this.BASE_SPEED;
    } else {
      data.ctrl[1] = 0;
    }

    // ========================================
    // Left Arm Control (indices 2-7)
    // ========================================

    // Shoulder rotation (7/Y)
    if (keyStates['Digit7']) {
      this.state.targetJoints[2] += this.JOINT_STEP;
    }
    if (keyStates['KeyY']) {
      this.state.targetJoints[2] -= this.JOINT_STEP;
    }

    // End effector Y position (8/U)
    if (keyStates['Digit8']) {
      this.state.eePos1[1] += this.EE_STEP;
    }
    if (keyStates['KeyU']) {
      this.state.eePos1[1] -= this.EE_STEP;
    }

    // End effector X position (9/I)
    if (keyStates['Digit9']) {
      this.state.eePos1[0] += this.EE_STEP;
    }
    if (keyStates['KeyI']) {
      this.state.eePos1[0] -= this.EE_STEP;
    }

    // Pitch adjustment (0/O)
    if (keyStates['Digit0']) {
      this.state.pitch1 += this.PITCH_STEP;
    }
    if (keyStates['KeyO']) {
      this.state.pitch1 -= this.PITCH_STEP;
    }

    // Wrist roll (Minus/P)
    if (keyStates['Minus']) {
      this.state.targetJoints[6] += this.JOINT_STEP * 3;
    }
    if (keyStates['KeyP']) {
      this.state.targetJoints[6] -= this.JOINT_STEP * 3;
    }

    // Calculate IK for left arm with pitch compensation
    const compensatedY1 = this.state.eePos1[1] + this.TIP_LENGTH * Math.sin(this.state.pitch1);
    const [j2_1, j3_1] = inverseKinematics(this.state.eePos1[0], compensatedY1);
    this.state.targetJoints[3] = j2_1;  // Pitch_L
    this.state.targetJoints[4] = j3_1;  // Elbow_L

    // Wrist pitch compensation to maintain end effector orientation
    this.state.targetJoints[5] = j2_1 - j3_1 + this.state.pitch1;

    // ========================================
    // Right Arm Control (indices 8-13)
    // ========================================

    // Shoulder rotation (H/N)
    if (keyStates['KeyH']) {
      this.state.targetJoints[8] += this.JOINT_STEP;
    }
    if (keyStates['KeyN']) {
      this.state.targetJoints[8] -= this.JOINT_STEP;
    }

    // End effector Y position (J/M)
    if (keyStates['KeyJ']) {
      this.state.eePos2[1] += this.EE_STEP;
    }
    if (keyStates['KeyM']) {
      this.state.eePos2[1] -= this.EE_STEP;
    }

    // End effector X position (K/Comma)
    if (keyStates['KeyK']) {
      this.state.eePos2[0] += this.EE_STEP;
    }
    if (keyStates['Comma']) {
      this.state.eePos2[0] -= this.EE_STEP;
    }

    // Pitch adjustment (L/Period)
    if (keyStates['KeyL']) {
      this.state.pitch2 += this.PITCH_STEP;
    }
    if (keyStates['Period']) {
      this.state.pitch2 -= this.PITCH_STEP;
    }

    // Wrist roll (Semicolon/Slash)
    if (keyStates['Semicolon']) {
      this.state.targetJoints[12] += this.JOINT_STEP * 3;
    }
    if (keyStates['Slash']) {
      this.state.targetJoints[12] -= this.JOINT_STEP * 3;
    }

    // Calculate IK for right arm with pitch compensation
    const compensatedY2 = this.state.eePos2[1] + this.TIP_LENGTH * Math.sin(this.state.pitch2);
    const [j2_2, j3_2] = inverseKinematics(this.state.eePos2[0], compensatedY2);
    this.state.targetJoints[9] = j2_2;   // Pitch_R
    this.state.targetJoints[10] = j3_2;  // Elbow_R

    // Wrist pitch compensation
    this.state.targetJoints[11] = j2_2 - j3_2 + this.state.pitch2;

    // ========================================
    // Gripper Control (toggle with cooldown)
    // ========================================

    // Left gripper (V)
    if (keyStates['KeyV'] && this.state.gripperCooldown[0] === 0) {
      this.state.gripperOpen[0] = !this.state.gripperOpen[0];
      this.state.targetJoints[7] = this.state.gripperOpen[0] ? this.GRIPPER_OPEN : this.GRIPPER_CLOSED;
      this.state.gripperCooldown[0] = this.GRIPPER_COOLDOWN_FRAMES;
    }

    // Right gripper (B)
    if (keyStates['KeyB'] && this.state.gripperCooldown[1] === 0) {
      this.state.gripperOpen[1] = !this.state.gripperOpen[1];
      this.state.targetJoints[13] = this.state.gripperOpen[1] ? this.GRIPPER_OPEN : this.GRIPPER_CLOSED;
      this.state.gripperCooldown[1] = this.GRIPPER_COOLDOWN_FRAMES;
    }

    // ========================================
    // Head Control (indices 14-15)
    // ========================================

    // Head pan (R/T)
    if (keyStates['KeyR']) {
      this.state.targetJoints[14] += this.JOINT_STEP * 2;
    }
    if (keyStates['KeyT']) {
      this.state.targetJoints[14] -= this.JOINT_STEP * 2;
    }

    // Head tilt (F/G)
    if (keyStates['KeyF']) {
      this.state.targetJoints[15] += this.JOINT_STEP * 2;
    }
    if (keyStates['KeyG']) {
      this.state.targetJoints[15] -= this.JOINT_STEP * 2;
    }

    // ========================================
    // Reset (X)
    // ========================================
    if (keyStates['KeyX']) {
      this.reset(data);
      return;
    }

    // ========================================
    // Apply target positions to actuators
    // ========================================
    for (let i = 2; i < 16; i++) {
      data.ctrl[i] = this.state.targetJoints[i];
    }
  }

  /**
   * Get the list of keys this controller uses
   * @returns {string[]}
   */
  getControlKeys() {
    return [
      // Base
      'KeyW', 'KeyS', 'KeyA', 'KeyD',
      // Left arm
      'Digit7', 'KeyY', 'Digit8', 'KeyU', 'Digit9', 'KeyI',
      'Digit0', 'KeyO', 'Minus', 'KeyP',
      // Right arm
      'KeyH', 'KeyN', 'KeyJ', 'KeyM', 'KeyK', 'Comma',
      'KeyL', 'Period', 'Semicolon', 'Slash',
      // Grippers
      'KeyV', 'KeyB',
      // Head
      'KeyR', 'KeyT', 'KeyF', 'KeyG',
      // Reset
      'KeyX'
    ];
  }

  /**
   * Get description for GUI display
   * @returns {string}
   */
  getDescription() {
    return [
      'Base: W/S Forward | A/D Turn',
      'Arm1: 7/Y Rotate | 8/U Y | 9/I X | 0/O Pitch | -/P Roll',
      'Arm2: H/N Rotate | J/M Y | K/, X | L/. Pitch | ;/? Roll',
      'Gripper: V/B Toggle | Head: R/T Pan, F/G Tilt | X Reset'
    ].join('\n');
  }
}

// ============================================================================
// Scene Configuration
// ============================================================================

// Scene-specific control configurations
const SCENE_CONFIGS = {
  'xlerobot/scene.xml': {
    type: 'custom',
    controller: new XLeRobotController(),
    description: 'XLeRobot Dual-Arm Control'
  }
  // Add more scenes here as needed
};

// ============================================================================
// Keyboard Controller Class
// ============================================================================

export class KeyboardController {
  constructor() {
    this.enabled = false;
    this.config = null;
    this.model = null;
    this.data = null;
    this.keyStates = {};
    this.actuatorIndices = {};
    this.customController = null;

    // Bind event handlers
    this._onKeyDown = this._onKeyDown.bind(this);
    this._onKeyUp = this._onKeyUp.bind(this);
  }

  /**
   * Get configuration for a scene
   * @param {string} sceneName - The scene filename
   * @returns {object|null} - Configuration object or null if not configured
   */
  getConfig(sceneName) {
    return SCENE_CONFIGS[sceneName] || null;
  }

  /**
   * Check if a scene has keyboard control configured
   * @param {string} sceneName - The scene filename
   * @returns {boolean}
   */
  hasConfig(sceneName) {
    return sceneName in SCENE_CONFIGS;
  }

  /**
   * Enable keyboard control for a scene
   * @param {string} sceneName - The scene filename
   * @param {object} model - MuJoCo model
   * @param {object} data - MuJoCo data
   * @returns {boolean} - Whether enabling was successful
   */
  enable(sceneName, model, data) {
    // Disable any existing control first
    this.disable();

    const config = this.getConfig(sceneName);
    if (!config) {
      return false;
    }

    this.config = config;
    this.model = model;
    this.data = data;

    // Handle custom controller
    if (config.type === 'custom' && config.controller) {
      this.customController = config.controller;
      this.customController.initialize(model, data);

      // Initialize key states for custom controller
      this.keyStates = {};
      for (const key of this.customController.getControlKeys()) {
        this.keyStates[key] = false;
      }
    } else {
      // Handle simple control mapping (legacy support)
      this.customController = null;

      // Build actuator name to index mapping
      const textDecoder = new TextDecoder("utf-8");
      const nullChar = textDecoder.decode(new ArrayBuffer(1));

      this.actuatorIndices = {};
      for (let i = 0; i < model.nu; i++) {
        const name = textDecoder.decode(
          model.names.subarray(model.name_actuatoradr[i])
        ).split(nullChar)[0];
        this.actuatorIndices[name] = i;
      }

      // Initialize key states
      this.keyStates = {};
      if (config.controls) {
        for (const ctrl of config.controls) {
          this.keyStates[ctrl.key] = false;
        }
      }
    }

    // Add event listeners
    document.addEventListener('keydown', this._onKeyDown);
    document.addEventListener('keyup', this._onKeyUp);

    this.enabled = true;
    return true;
  }

  /**
   * Disable keyboard control
   */
  disable() {
    if (!this.enabled) return;

    document.removeEventListener('keydown', this._onKeyDown);
    document.removeEventListener('keyup', this._onKeyUp);

    this.enabled = false;
    this.config = null;
    this.model = null;
    this.data = null;
    this.keyStates = {};
    this.actuatorIndices = {};
    this.customController = null;
  }

  /**
   * Update actuator controls based on current key states
   * Call this in the render loop
   */
  update() {
    if (!this.enabled || !this.config || !this.data) return;

    // Use custom controller if available
    if (this.customController) {
      this.customController.update(this.keyStates, this.model, this.data);
      return;
    }

    // Legacy: simple control mapping
    if (!this.config.controls) return;

    // Group controls by actuator to handle opposing keys
    const actuatorValues = {};

    for (const ctrl of this.config.controls) {
      const actuatorIdx = this.actuatorIndices[ctrl.actuator];
      if (actuatorIdx === undefined) continue;

      if (!(ctrl.actuator in actuatorValues)) {
        actuatorValues[ctrl.actuator] = 0;
      }

      if (this.keyStates[ctrl.key]) {
        actuatorValues[ctrl.actuator] += ctrl.value;
      }
    }

    // Apply values to data.ctrl
    for (const [actuatorName, value] of Object.entries(actuatorValues)) {
      const idx = this.actuatorIndices[actuatorName];
      if (idx !== undefined) {
        let clampedValue = value;
        if (this.model.actuator_ctrllimited[idx]) {
          const min = this.model.actuator_ctrlrange[2 * idx];
          const max = this.model.actuator_ctrlrange[2 * idx + 1];
          clampedValue = Math.max(min, Math.min(max, value));
        }
        this.data.ctrl[idx] = clampedValue;
      }
    }
  }

  /**
   * Get current control description for GUI display
   * @returns {string}
   */
  getDescription() {
    if (this.customController) {
      return this.customController.getDescription();
    }
    return this.config ? this.config.description : '';
  }

  _onKeyDown(event) {
    if (!this.enabled) return;

    // Ignore if typing in an input field
    if (event.target.tagName === 'INPUT' || event.target.tagName === 'TEXTAREA') {
      return;
    }

    if (event.code in this.keyStates) {
      this.keyStates[event.code] = true;
      event.preventDefault();
    }
  }

  _onKeyUp(event) {
    if (!this.enabled) return;

    if (event.code in this.keyStates) {
      this.keyStates[event.code] = false;
      event.preventDefault();
    }
  }
}

// Singleton instance for easy access
export const keyboardController = new KeyboardController();
