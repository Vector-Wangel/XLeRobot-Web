/**
 * Panda Controller
 *
 * Franka Emika Panda robot controller using Damped Least Squares IK.
 *
 * Controls:
 * - Position: W/S (X forward/back), A/D (Y left/right), Q/E (Z up/down)
 * - Orientation: Z/C (Roll), R/F (Pitch), T/G (Yaw)
 * - Gripper: V (Open), B (Close)
 * - Reset: X
 */

import { BaseController } from './BaseController.js';
import { quatMultiply, quatFromAxisAngle, quatNormalize } from '../math/quaternion.js';

export class PandaController extends BaseController {
  constructor() {
    super();

    // Movement speed constants
    this.POS_STEP = 0.001;
    this.ROT_STEP = 0.004;

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

    // Gripper settings (two finger actuators, each with ctrlrange 0-0.04)
    this.GRIPPER_OPEN = 0.04;
    this.GRIPPER_CLOSED = 0;

    // Panda joint indices (7 DOF arm)
    this.ARM_JOINT_INDICES = [0, 1, 2, 3, 4, 5, 6];
    // Two separate finger actuators (indices 7 and 8)
    this.GRIPPER_ACTUATOR_IDX_1 = 7;
    this.GRIPPER_ACTUATOR_IDX_2 = 8;

    // Panda joint limits (radians) - from URDF
    this.JOINT_LIMITS_MIN = [-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973];
    this.JOINT_LIMITS_MAX = [2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973];

    // Workspace limits (Panda raised by 0.8m)
    this.POS_MIN = [-0.8, -0.8, 0.82];
    this.POS_MAX = [0.8, 0.8, 1.6];

    // IK frequency control - reduce computation load
    this.IK_SKIP_FRAMES = 10;  // Only compute IK every N physics steps
    this.frameCounter = 0;
    this.cachedCtrl = new Array(7).fill(0);  // Cache arm control values

    // State
    this.state = null;
    this.mujoco = null;
    this.handBodyId = -1;

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
  async initialize(model, data, mujoco) {
    this.mujoco = mujoco;
    this.handBodyId = this._getBodyId(model, 'hand');

    if (this.handBodyId < 0) {
      console.error('PandaController: hand body not found');
      return;
    }

    // Do forward kinematics to get initial EE position
    mujoco.mj_forward(model, data);

    this._initState(data);

    // Initialize cached control values from current joint positions
    for (let i = 0; i < this.ARM_JOINT_INDICES.length; i++) {
      const jointIdx = this.ARM_JOINT_INDICES[i];
      this.cachedCtrl[i] = data.qpos[jointIdx];
    }
    this.frameCounter = 0;

    this.initialized = true;

    console.log('PandaController initialized with DLS IK (skip frames:', this.IK_SKIP_FRAMES, ')');
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

    // Reset cached control values
    for (let i = 0; i < this.ARM_JOINT_INDICES.length; i++) {
      const jointIdx = this.ARM_JOINT_INDICES[i];
      this.cachedCtrl[i] = data.qpos[jointIdx];
    }
    this.frameCounter = 0;

    // Reset cube ("box") to initial position [0.1, -0.4, 1]
    const boxBodyId = this._getBodyId(model, 'box');
    if (boxBodyId >= 0) {
      const jntAdr = model.body_jntadr[boxBodyId];
      if (jntAdr >= 0) {
        const qposAdr = model.jnt_qposadr[jntAdr];
        // Set position (x, y, z)
        data.qpos[qposAdr] = 0.1;
        data.qpos[qposAdr + 1] = -0.4;
        data.qpos[qposAdr + 2] = 1.0;
        // Set orientation to identity quaternion (w, x, y, z)
        data.qpos[qposAdr + 3] = 1.0;
        data.qpos[qposAdr + 4] = 0.0;
        data.qpos[qposAdr + 5] = 0.0;
        data.qpos[qposAdr + 6] = 0.0;
        // Reset velocities
        const dofAdr = model.jnt_dofadr[jntAdr];
        for (let i = 0; i < 6; i++) {
          data.qvel[dofAdr + i] = 0.0;
        }
        this.mujoco.mj_forward(model, data);
      }
    }
  }

  /**
   * Check if any position/orientation keyboard control is active
   * @param {object} keyStates - Current keyboard states
   * @returns {boolean}
   */
  _isKeyControllingPose(keyStates) {
    return keyStates['KeyW'] || keyStates['KeyS'] || keyStates['KeyA'] || keyStates['KeyD'] ||
           keyStates['KeyQ'] || keyStates['KeyE'] || keyStates['KeyZ'] || keyStates['KeyC'] ||
           keyStates['KeyR'] || keyStates['KeyF'] || keyStates['KeyT'] || keyStates['KeyG'];
  }

  /**
   * 异步控制步进 - 每个控制周期调用一次
   */
  async step(keyStates, model, data, mujoco) {
    if (!this.initialized || !this.state) {
      await this.initialize(model, data, mujoco);
      return;
    }

    // Store mujoco reference in case it wasn't set during init
    if (!this.mujoco && mujoco) {
      this.mujoco = mujoco;
    }

    // ========================================
    // SYNC: If no keyboard control is active, sync cached control from sliders
    // ========================================
    if (!this._isKeyControllingPose(keyStates)) {
      // Read current control values from sliders
      for (let i = 0; i < this.ARM_JOINT_INDICES.length; i++) {
        this.cachedCtrl[i] = data.ctrl[i];
      }
    }

    // ========================================
    // Position Control
    // ========================================
    if (keyStates['KeyA']) this.state.targetPos[0] += this.POS_STEP;
    if (keyStates['KeyD']) this.state.targetPos[0] -= this.POS_STEP;
    if (keyStates['KeyS']) this.state.targetPos[1] += this.POS_STEP;
    if (keyStates['KeyW']) this.state.targetPos[1] -= this.POS_STEP;
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
    // Sync gripper state from slider if not using keyboard
    if (!keyStates['KeyV'] && !keyStates['KeyB']) {
      // If slider moved the gripper, update our state to match
      const currentGripperValue = data.ctrl[this.GRIPPER_ACTUATOR_IDX_1];
      this.state.gripperOpen = currentGripperValue > (this.GRIPPER_OPEN + this.GRIPPER_CLOSED) / 2;
    }
    
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
    // Only compute every N frames to reduce load
    // ========================================
    this.frameCounter++;

    if (this.frameCounter % this.IK_SKIP_FRAMES === 0) {
      this._clampPosition();

      // Solve IK with position and orientation (expensive: ~40 mj_forward calls)
      this._solveIK(this.state.targetPos, this.state.targetQuat, model, data);

      // Cache control values
      for (let i = 0; i < this.ARM_JOINT_INDICES.length; i++) {
        const jointIdx = this.ARM_JOINT_INDICES[i];
        this.cachedCtrl[i] = data.qpos[jointIdx];
      }
    }

    // Apply cached control values every frame
    for (let i = 0; i < this.ARM_JOINT_INDICES.length; i++) {
      data.ctrl[i] = this.cachedCtrl[i];
    }

    // ========================================
    // Gravity Compensation (every frame)
    // ========================================
    for (let i = 0; i < this.ARM_JOINT_INDICES.length; i++) {
      const jointIdx = this.ARM_JOINT_INDICES[i];
      data.qfrc_applied[jointIdx] = data.qfrc_bias[jointIdx];
    }

    // ========================================
    // Gripper control (every frame - responsive)
    // ========================================
    const gripperValue = this.state.gripperOpen ? this.GRIPPER_OPEN : this.GRIPPER_CLOSED;
    data.ctrl[this.GRIPPER_ACTUATOR_IDX_1] = gripperValue;
    data.ctrl[this.GRIPPER_ACTUATOR_IDX_2] = gripperValue;
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
