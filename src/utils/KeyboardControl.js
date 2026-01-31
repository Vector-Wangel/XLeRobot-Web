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
   * @param {object} _model - MuJoCo model (unused)
   * @param {object} data - MuJoCo data
   * @param {object} _mujoco - MuJoCo WASM module (unused)
   */
  initialize(_model, data, _mujoco) {
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
   * @param {object} _mujoco - MuJoCo WASM module (unused)
   */
  update(keyStates, model, data, _mujoco) {
    if (!this.initialized || !this.state) {
      this.initialize(model, data, _mujoco);
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
// Panda Controller (DLS IK with mj_jac)
// ============================================================================

/**
 * Quaternion multiplication helper
 * @param {number[]} q1 - First quaternion [w, x, y, z]
 * @param {number[]} q2 - Second quaternion [w, x, y, z]
 * @returns {number[]} - Result quaternion [w, x, y, z]
 */
function quatMultiply(q1, q2) {
  return [
    q1[0]*q2[0] - q1[1]*q2[1] - q1[2]*q2[2] - q1[3]*q2[3],
    q1[0]*q2[1] + q1[1]*q2[0] + q1[2]*q2[3] - q1[3]*q2[2],
    q1[0]*q2[2] - q1[1]*q2[3] + q1[2]*q2[0] + q1[3]*q2[1],
    q1[0]*q2[3] + q1[1]*q2[2] - q1[2]*q2[1] + q1[3]*q2[0]
  ];
}

/**
 * Create quaternion from axis-angle
 * @param {number[]} axis - Rotation axis [x, y, z]
 * @param {number} angle - Rotation angle in radians
 * @returns {number[]} - Quaternion [w, x, y, z]
 */
function quatFromAxisAngle(axis, angle) {
  const halfAngle = angle / 2;
  const s = Math.sin(halfAngle);
  return [Math.cos(halfAngle), axis[0] * s, axis[1] * s, axis[2] * s];
}

/**
 * Normalize a quaternion
 * @param {number[]} q - Quaternion [w, x, y, z]
 * @returns {number[]} - Normalized quaternion
 */
function quatNormalize(q) {
  const norm = Math.sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
  if (norm < 1e-10) return [1, 0, 0, 0];
  return [q[0]/norm, q[1]/norm, q[2]/norm, q[3]/norm];
}

/**
 * Panda robot controller using Damped Least Squares IK with mj_jac
 *
 * Controls:
 * - Position: W/S (X forward/back), A/D (Y left/right), Q/E (Z up/down)
 * - Orientation: Z/C (Roll), R/F (Pitch), T/G (Yaw)
 * - Gripper: V (Open), B (Close)
 * - Reset: X
 */
class PandaController {
  constructor() {
    // Movement speed constants
    this.POS_STEP = 0.002;
    this.ROT_STEP = 0.02;

    // DLS IK parameters
    this.DAMPING_MIN = 0.01;     // Minimum damping (for small errors)
    this.DAMPING_MAX = 0.2;      // Maximum damping (for large errors)
    this.IK_ITERATIONS = 5;      // IK iterations per frame
    this.IK_STEP_SIZE = 0.3;     // Step size for each iteration

    // Convergence thresholds
    this.POS_THRESHOLD = 0.001;  // 1mm position tolerance
    this.ROT_THRESHOLD = 0.01;   // ~0.5 degree rotation tolerance

    // Error weighting for 6-DOF IK
    this.POS_WEIGHT = 1.0;       // Position error weight
    this.ROT_WEIGHT = 0.3;       // Rotation error weight (less aggressive)

    // Joint velocity limits (rad/frame)
    this.MAX_JOINT_VEL = 0.05;

    // Gripper settings (actuator8 has ctrlrange 0-255)
    this.GRIPPER_OPEN = 255;
    this.GRIPPER_CLOSED = 0;

    // Panda joint indices (7 DOF arm)
    this.ARM_JOINT_INDICES = [0, 1, 2, 3, 4, 5, 6];
    this.GRIPPER_ACTUATOR_IDX = 7;

    // Panda joint limits (radians) - from URDF
    this.JOINT_LIMITS_MIN = [-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973];
    this.JOINT_LIMITS_MAX = [2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973];

    // Workspace limits
    this.POS_MIN = [0.2, -0.5, 0.02];
    this.POS_MAX = [0.8, 0.5, 0.8];

    // State
    this.state = null;
    this.mujoco = null;
    this.handBodyId = -1;
    this.initialized = false;

    // Previous joint positions for velocity smoothing
    this.prevQpos = null;
  }

  /**
   * Get body ID by name
   */
  _getBodyId(model, name) {
    for (let i = 0; i < model.nbody; i++) {
      const nameAddr = model.name_bodyadr[i];
      let bodyName = '';
      for (let j = nameAddr; model.names[j] !== 0; j++) {
        bodyName += String.fromCharCode(model.names[j]);
      }
      if (bodyName === name) return i;
    }
    return -1;
  }

  /**
   * Rotate a vector by a quaternion
   */
  _rotateVector(v, q) {
    const [w, qx, qy, qz] = q;
    const [vx, vy, vz] = v;
    const t = [
      2 * (qy * vz - qz * vy),
      2 * (qz * vx - qx * vz),
      2 * (qx * vy - qy * vx)
    ];
    return [
      vx + w * t[0] + qy * t[2] - qz * t[1],
      vy + w * t[1] + qz * t[0] - qx * t[2],
      vz + w * t[2] + qx * t[1] - qy * t[0]
    ];
  }

  /**
   * Get current end-effector position (fingertip)
   */
  _getEEPosition(data) {
    const idx = this.handBodyId * 3;
    const handPos = [data.xpos[idx], data.xpos[idx + 1], data.xpos[idx + 2]];
    const handQuat = this._getEEQuaternion(data);
    // Offset from hand to fingertip
    const tipOffset = this._rotateVector([0, 0, 0.103], handQuat);
    return [
      handPos[0] + tipOffset[0],
      handPos[1] + tipOffset[1],
      handPos[2] + tipOffset[2]
    ];
  }

  /**
   * Get current end-effector quaternion
   */
  _getEEQuaternion(data) {
    const idx = this.handBodyId * 4;
    return [
      data.xquat[idx],
      data.xquat[idx + 1],
      data.xquat[idx + 2],
      data.xquat[idx + 3]
    ];
  }

  /**
   * Compute position error
   */
  _posError(targetPos, data) {
    const eePos = this._getEEPosition(data);
    return [
      targetPos[0] - eePos[0],
      targetPos[1] - eePos[1],
      targetPos[2] - eePos[2]
    ];
  }

  /**
   * Compute orientation error (angle-axis)
   */
  _rotError(targetQuat, data) {
    const eeQuat = this._getEEQuaternion(data);
    // q_error = q_target * q_current^(-1)
    const qInv = [eeQuat[0], -eeQuat[1], -eeQuat[2], -eeQuat[3]];
    const qErr = quatMultiply(targetQuat, qInv);
    // Convert to scaled axis-angle
    if (qErr[0] < 0) {
      return [-2 * qErr[1], -2 * qErr[2], -2 * qErr[3]];
    }
    return [2 * qErr[1], 2 * qErr[2], 2 * qErr[3]];
  }

  /**
   * Solve linear system using Gaussian elimination
   */
  _solveLinear(A, b) {
    const n = b.length;
    const aug = A.map((row, i) => [...row, b[i]]);

    // Forward elimination with partial pivoting
    for (let i = 0; i < n; i++) {
      let maxRow = i;
      for (let k = i + 1; k < n; k++) {
        if (Math.abs(aug[k][i]) > Math.abs(aug[maxRow][i])) {
          maxRow = k;
        }
      }
      [aug[i], aug[maxRow]] = [aug[maxRow], aug[i]];

      if (Math.abs(aug[i][i]) < 1e-10) continue;

      for (let k = i + 1; k < n; k++) {
        const factor = aug[k][i] / aug[i][i];
        for (let j = i; j <= n; j++) {
          aug[k][j] -= factor * aug[i][j];
        }
      }
    }

    // Back substitution
    const x = new Array(n).fill(0);
    for (let i = n - 1; i >= 0; i--) {
      if (Math.abs(aug[i][i]) < 1e-10) continue;
      x[i] = aug[i][n];
      for (let j = i + 1; j < n; j++) {
        x[i] -= aug[i][j] * x[j];
      }
      x[i] /= aug[i][i];
    }
    return x;
  }

  /**
   * Compute adaptive damping based on error magnitude (Levenberg-Marquardt style)
   */
  _computeAdaptiveDamping(errorNorm) {
    // Larger errors -> more damping (more stable but slower)
    // Smaller errors -> less damping (faster convergence)
    const t = Math.min(1, errorNorm / 0.1);  // Normalize by 10cm
    return this.DAMPING_MIN + t * (this.DAMPING_MAX - this.DAMPING_MIN);
  }

  /**
   * Damped Least Squares: dq = J^T * (J * J^T + λ²I)^(-1) * error
   * With adaptive damping for better stability
   */
  _solveDLS(J, error) {
    const m = error.length;
    const n = this.ARM_JOINT_INDICES.length;

    // Compute error magnitude for adaptive damping
    const errorNorm = Math.sqrt(error.reduce((sum, e) => sum + e * e, 0));
    const damping = this._computeAdaptiveDamping(errorNorm);

    // Compute J * J^T + λ²I
    const JJT = [];
    for (let i = 0; i < m; i++) {
      JJT[i] = [];
      for (let j = 0; j < m; j++) {
        let sum = 0;
        for (let k = 0; k < n; k++) {
          sum += J[i][k] * J[j][k];
        }
        if (i === j) {
          sum += damping * damping;
        }
        JJT[i][j] = sum;
      }
    }

    // Solve (J*J^T + λ²I) * y = error
    const y = this._solveLinear(JJT, error);

    // Compute dq = J^T * y
    const dq = new Array(n).fill(0);
    for (let i = 0; i < n; i++) {
      for (let j = 0; j < m; j++) {
        dq[i] += J[j][i] * y[j];
      }
    }
    return dq;
  }

  /**
   * Clamp joint positions to limits
   */
  _clampJoints(data) {
    for (let i = 0; i < this.ARM_JOINT_INDICES.length; i++) {
      const jointIdx = this.ARM_JOINT_INDICES[i];
      const min = this.JOINT_LIMITS_MIN[i];
      const max = this.JOINT_LIMITS_MAX[i];
      data.qpos[jointIdx] = Math.max(min, Math.min(max, data.qpos[jointIdx]));
    }
  }

  /**
   * Clamp joint velocity to prevent jerky motion
   */
  _clampJointVelocity(dq) {
    for (let i = 0; i < dq.length; i++) {
      dq[i] = Math.max(-this.MAX_JOINT_VEL, Math.min(this.MAX_JOINT_VEL, dq[i]));
    }
    return dq;
  }

  /**
   * Convert quaternion difference to axis-angle (for numerical Jacobian)
   */
  _quatToAxisAngle(q1, q2) {
    // q_error = q2 * q1^(-1)
    const q1Inv = [q1[0], -q1[1], -q1[2], -q1[3]];
    const qErr = quatMultiply(q2, q1Inv);

    // For small angles: axis * angle ≈ 2 * (x, y, z)
    if (qErr[0] < 0) {
      return [-2 * qErr[1], -2 * qErr[2], -2 * qErr[3]];
    }
    return [2 * qErr[1], 2 * qErr[2], 2 * qErr[3]];
  }

  /**
   * Compute full 6-DOF Jacobian using numerical differentiation
   * Returns [Jp (3x7), Jr (3x7)] stacked as 6x7
   */
  _computeFullJacobian(model, data) {
    const epsilon = 1e-6;
    const nj = this.ARM_JOINT_INDICES.length;

    // Save current state
    const savedQpos = new Float64Array(model.nq);
    savedQpos.set(data.qpos);

    // Get current EE pose
    this.mujoco.mj_forward(model, data);
    const currentPos = this._getEEPosition(data);
    const currentQuat = this._getEEQuaternion(data);

    // Jacobian: 6 rows (3 position + 3 orientation), 7 columns (joints)
    const J = [];
    for (let row = 0; row < 6; row++) {
      J.push(new Array(nj).fill(0));
    }

    for (let j = 0; j < nj; j++) {
      const jointIdx = this.ARM_JOINT_INDICES[j];

      // Perturb joint
      data.qpos[jointIdx] = savedQpos[jointIdx] + epsilon;
      this.mujoco.mj_forward(model, data);

      const perturbedPos = this._getEEPosition(data);
      const perturbedQuat = this._getEEQuaternion(data);

      // Position Jacobian (rows 0-2)
      for (let row = 0; row < 3; row++) {
        J[row][j] = (perturbedPos[row] - currentPos[row]) / epsilon;
      }

      // Orientation Jacobian (rows 3-5)
      const deltaRot = this._quatToAxisAngle(currentQuat, perturbedQuat);
      for (let row = 0; row < 3; row++) {
        J[row + 3][j] = deltaRot[row] / epsilon;
      }

      // Restore joint
      data.qpos[jointIdx] = savedQpos[jointIdx];
    }

    // Restore all qpos
    data.qpos.set(savedQpos);
    this.mujoco.mj_forward(model, data);

    return J;
  }

  /**
   * Multi-iteration 6-DOF IK for position and orientation
   */
  _solveIK(targetPos, targetQuat, model, data) {
    const totalDq = new Array(this.ARM_JOINT_INDICES.length).fill(0);

    for (let iter = 0; iter < this.IK_ITERATIONS; iter++) {
      // Forward kinematics
      this.mujoco.mj_forward(model, data);

      // Compute errors
      const posError = this._posError(targetPos, data);
      const rotError = this._rotError(targetQuat, data);

      const posNorm = Math.sqrt(posError[0]**2 + posError[1]**2 + posError[2]**2);
      const rotNorm = Math.sqrt(rotError[0]**2 + rotError[1]**2 + rotError[2]**2);

      // Check convergence
      if (posNorm < this.POS_THRESHOLD && rotNorm < this.ROT_THRESHOLD) {
        break;
      }

      // Compute full 6-DOF Jacobian
      const J = this._computeFullJacobian(model, data);

      // Combined error with weighting (position more important)
      const error = [
        posError[0] * this.POS_WEIGHT,
        posError[1] * this.POS_WEIGHT,
        posError[2] * this.POS_WEIGHT,
        rotError[0] * this.ROT_WEIGHT,
        rotError[1] * this.ROT_WEIGHT,
        rotError[2] * this.ROT_WEIGHT
      ];

      // Solve DLS
      const dq = this._solveDLS(J, error);

      // Clamp velocity
      this._clampJointVelocity(dq);

      // Apply joint deltas
      for (let i = 0; i < this.ARM_JOINT_INDICES.length; i++) {
        const jointIdx = this.ARM_JOINT_INDICES[i];
        const delta = dq[i] * this.IK_STEP_SIZE;
        data.qpos[jointIdx] += delta;
        totalDq[i] += delta;
      }

      // Clamp to joint limits
      this._clampJoints(data);
    }

    return totalDq;
  }

  /**
   * Initialize the controller state
   */
  _initState(data) {
    const eePos = this._getEEPosition(data);
    const eeQuat = this._getEEQuaternion(data);
    this.state = {
      targetPos: [...eePos],
      targetQuat: [...eeQuat],
      gripperOpen: false,
    };
  }

  /**
   * Clamp position to workspace limits
   */
  _clampPosition() {
    for (let i = 0; i < 3; i++) {
      this.state.targetPos[i] = Math.max(this.POS_MIN[i], Math.min(this.POS_MAX[i], this.state.targetPos[i]));
    }
  }

  /**
   * Initialize the controller with model and data
   */
  initialize(model, data, mujoco) {
    this.mujoco = mujoco;
    this.handBodyId = this._getBodyId(model, 'hand');

    if (this.handBodyId < 0) {
      console.error('PandaController: hand body not found');
      return;
    }

    // Do forward kinematics to get initial EE position
    mujoco.mj_forward(model, data);

    this._initState(data);
    this.initialized = true;

    console.log('PandaController initialized with DLS IK');
    console.log('Initial EE position:', this.state.targetPos);
  }

  /**
   * Reset to initial state
   */
  reset(model, data) {
    // Reset to keyframe if available
    if (model.nkey > 0) {
      data.qpos.set(model.key_qpos.slice(0, model.nq));
      this.mujoco.mj_forward(model, data);
    }
    this._initState(data);
  }

  /**
   * Update controls based on keyboard state
   */
  update(keyStates, model, data, mujoco) {
    if (!this.initialized || !this.state) {
      this.initialize(model, data, mujoco);
      return;
    }

    // Store mujoco reference in case it wasn't set during init
    if (!this.mujoco && mujoco) {
      this.mujoco = mujoco;
    }

    // ========================================
    // Position Control
    // ========================================
    if (keyStates['KeyW']) this.state.targetPos[0] += this.POS_STEP;
    if (keyStates['KeyS']) this.state.targetPos[0] -= this.POS_STEP;
    if (keyStates['KeyA']) this.state.targetPos[1] += this.POS_STEP;
    if (keyStates['KeyD']) this.state.targetPos[1] -= this.POS_STEP;
    if (keyStates['KeyQ']) this.state.targetPos[2] += this.POS_STEP;
    if (keyStates['KeyE']) this.state.targetPos[2] -= this.POS_STEP;

    // ========================================
    // Orientation Control
    // ========================================
    if (keyStates['KeyZ']) {
      const dq = quatFromAxisAngle([1, 0, 0], this.ROT_STEP);
      this.state.targetQuat = quatNormalize(quatMultiply(this.state.targetQuat, dq));
    }
    if (keyStates['KeyC']) {
      const dq = quatFromAxisAngle([1, 0, 0], -this.ROT_STEP);
      this.state.targetQuat = quatNormalize(quatMultiply(this.state.targetQuat, dq));
    }
    if (keyStates['KeyR']) {
      const dq = quatFromAxisAngle([0, 1, 0], this.ROT_STEP);
      this.state.targetQuat = quatNormalize(quatMultiply(this.state.targetQuat, dq));
    }
    if (keyStates['KeyF']) {
      const dq = quatFromAxisAngle([0, 1, 0], -this.ROT_STEP);
      this.state.targetQuat = quatNormalize(quatMultiply(this.state.targetQuat, dq));
    }
    if (keyStates['KeyT']) {
      const dq = quatFromAxisAngle([0, 0, 1], this.ROT_STEP);
      this.state.targetQuat = quatNormalize(quatMultiply(this.state.targetQuat, dq));
    }
    if (keyStates['KeyG']) {
      const dq = quatFromAxisAngle([0, 0, 1], -this.ROT_STEP);
      this.state.targetQuat = quatNormalize(quatMultiply(this.state.targetQuat, dq));
    }

    // ========================================
    // Gripper Control
    // ========================================
    if (keyStates['KeyV']) this.state.gripperOpen = true;
    if (keyStates['KeyB']) this.state.gripperOpen = false;

    // ========================================
    // Reset
    // ========================================
    if (keyStates['KeyX']) {
      this.reset(model, data);
      return;
    }

    // ========================================
    // Apply IK (6-DOF: position + orientation)
    // ========================================
    this._clampPosition();

    // Solve IK with position and orientation
    this._solveIK(this.state.targetPos, this.state.targetQuat, model, data);

    // Set control targets to solved joint positions
    for (let i = 0; i < this.ARM_JOINT_INDICES.length; i++) {
      const jointIdx = this.ARM_JOINT_INDICES[i];
      data.ctrl[i] = data.qpos[jointIdx];
    }

    // ========================================
    // Gravity Compensation
    // ========================================
    // Apply feedforward torque to counteract gravity
    // qfrc_bias contains gravity + Coriolis forces
    for (let i = 0; i < this.ARM_JOINT_INDICES.length; i++) {
      const jointIdx = this.ARM_JOINT_INDICES[i];
      // Add gravity compensation torque directly
      data.qfrc_applied[jointIdx] = data.qfrc_bias[jointIdx];
    }

    // Gripper control
    data.ctrl[this.GRIPPER_ACTUATOR_IDX] = this.state.gripperOpen ? this.GRIPPER_OPEN : this.GRIPPER_CLOSED;
  }

  /**
   * Get the list of keys this controller uses
   */
  getControlKeys() {
    return [
      'KeyW', 'KeyS', 'KeyA', 'KeyD', 'KeyQ', 'KeyE',
      'KeyZ', 'KeyC', 'KeyR', 'KeyF', 'KeyT', 'KeyG',
      'KeyV', 'KeyB', 'KeyX'
    ];
  }

  /**
   * Get description for GUI display
   */
  getDescription() {
    return [
      'Position: W/S (X) | A/D (Y) | Q/E (Z)',
      'Rotation: Z/C (Roll) | R/F (Pitch) | T/G (Yaw)',
      'Gripper: V Open | B Close | X Reset'
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
  },
  'franka_emika_panda/scene.xml': {
    type: 'custom',
    controller: new PandaController(),
    description: 'Panda DLS IK Control'
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
   * @param {object} mujoco - MuJoCo WASM module (optional, needed for IK)
   * @returns {boolean} - Whether enabling was successful
   */
  enable(sceneName, model, data, mujoco = null) {
    // Disable any existing control first
    this.disable();

    const config = this.getConfig(sceneName);
    if (!config) {
      return false;
    }

    this.config = config;
    this.model = model;
    this.data = data;
    this.mujoco = mujoco;

    // Handle custom controller
    if (config.type === 'custom' && config.controller) {
      this.customController = config.controller;
      this.customController.initialize(model, data, mujoco);

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
    this.mujoco = null;
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
      this.customController.update(this.keyStates, this.model, this.data, this.mujoco);
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
