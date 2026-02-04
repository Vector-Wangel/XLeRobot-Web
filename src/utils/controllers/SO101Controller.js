/**
 * SO101 Controller
 *
 * Single-arm SO101 robot controller with inverse kinematics.
 *
 * Actuator index mapping:
 * Index | Name          | Type     | Description
 * ------|---------------|----------|-------------
 * 0     | Rotation      | position | Shoulder rotation
 * 1     | Pitch         | position | Shoulder pitch (IK)
 * 2     | Elbow         | position | Elbow (IK)
 * 3     | Wrist_Pitch   | position | Wrist pitch (compensation)
 * 4     | Wrist_Roll    | position | Wrist roll
 * 5     | Jaw           | position | Gripper
 */

import { BaseController } from './BaseController.js';
import { inverseKinematics2Link } from '../math/inverseKinematics.js';

export class SO101Controller extends BaseController {
  constructor() {
    super();

    // Control constants
    this.JOINT_STEP = 0.002;    // Fine joint control
    this.EE_STEP = 0.0002;      // End effector step size
    this.PITCH_STEP = 0.0025;   // Pitch adjustment step
    this.TIP_LENGTH = 0.108;    // Length from wrist to end effector tip

    // Gripper positions
    this.GRIPPER_OPEN = 1.5;
    this.GRIPPER_CLOSED = -0.25;
    this.GRIPPER_COOLDOWN_FRAMES = 60;  // Frames to wait between toggles

    // Initial end effector position
    this.INITIAL_EE_POS = [0.162, 0.118];

    // State
    this.state = null;
  }

  /**
   * Initialize the controller state
   */
  _initState() {
    // Calculate initial IK
    const [j2, j3] = inverseKinematics2Link(this.INITIAL_EE_POS[0], this.INITIAL_EE_POS[1]);

    this.state = {
      // Target joint positions (6 actuators)
      targetJoints: new Float64Array(6),

      // End effector position [x, y]
      eePos: [...this.INITIAL_EE_POS],

      // Pitch adjustment for wrist orientation
      pitch: 0.0,

      // Gripper toggle cooldown counter
      gripperCooldown: 0,

      // Gripper state (false = closed, true = open)
      gripperOpen: false,
    };

    // Set initial joint positions
    this.state.targetJoints[0] = 0;        // Rotation (center)
    this.state.targetJoints[1] = j2;       // Pitch
    this.state.targetJoints[2] = j3;       // Elbow
    this.state.targetJoints[3] = j2 - j3;  // Wrist_Pitch (compensation)
    this.state.targetJoints[4] = 0;        // Wrist_Roll (center)
    this.state.targetJoints[5] = this.GRIPPER_CLOSED;  // Jaw (closed)
  }

  /**
   * Initialize the controller with model and data
   * @param {object} _model - MuJoCo model (unused)
   * @param {object} data - MuJoCo data
   * @param {object} _mujoco - MuJoCo WASM module (unused)
   */
  async initialize(_model, data, _mujoco) {
    this._initState();

    // Write initial control values to position actuators
    for (let i = 0; i < 6; i++) {
      data.ctrl[i] = this.state.targetJoints[i];
    }

    this.initialized = true;
  }

  /**
   * Reset all positions to initial state
   * @param {object} _model - MuJoCo model (unused)
   * @param {object} data - MuJoCo data
   */
  reset(_model, data) {
    this._initState();

    // Write reset values
    for (let i = 0; i < 6; i++) {
      data.ctrl[i] = this.state.targetJoints[i];
    }
  }

  /**
   * Check if an actuator is currently being controlled by keyboard input
   * @param {number} actuatorIdx - Actuator index
   * @param {object} keyStates - Current keyboard states
   * @returns {boolean}
   */
  _isKeyControlling(actuatorIdx, keyStates) {
    switch (actuatorIdx) {
      case 0: return keyStates['KeyA'] || keyStates['KeyD'];  // Rotation
      case 1: case 2: case 3:  // IK joints (controlled together)
        return keyStates['KeyW'] || keyStates['KeyS'] ||  // X position
               keyStates['KeyQ'] || keyStates['KeyE'] ||  // Y position
               keyStates['KeyR'] || keyStates['KeyF'];     // Pitch
      case 4: return keyStates['KeyZ'] || keyStates['KeyC'];  // Wrist roll
      case 5: return keyStates['KeyV'];  // Gripper
      default: return false;
    }
  }

  /**
   * 异步控制步进 - 每个控制周期调用一次
   * @param {object} keyStates - Current keyboard states
   * @param {object} model - MuJoCo model
   * @param {object} data - MuJoCo data
   * @param {object} _mujoco - MuJoCo WASM module (unused)
   */
  async step(keyStates, model, data, _mujoco) {
    if (!this.initialized || !this.state) {
      await this.initialize(model, data, _mujoco);
    }

    // ========================================
    // SYNC: Read external control inputs (from sliders) for non-active controls
    // ========================================
    for (let i = 0; i < 6; i++) {
      if (!this._isKeyControlling(i, keyStates)) {
        this.state.targetJoints[i] = data.ctrl[i];
      }
    }

    // Decrement gripper cooldown
    if (this.state.gripperCooldown > 0) this.state.gripperCooldown--;

    // ========================================
    // Arm Control
    // ========================================

    // Shoulder rotation (A/D) - A: left, D: right
    if (keyStates['KeyA']) {
      this.state.targetJoints[0] -= this.JOINT_STEP;
    }
    if (keyStates['KeyD']) {
      this.state.targetJoints[0] += this.JOINT_STEP;
    }

    // End effector X position (W/S) - W: forward, S: backward
    if (keyStates['KeyW']) {
      this.state.eePos[0] += this.EE_STEP;
    }
    if (keyStates['KeyS']) {
      this.state.eePos[0] -= this.EE_STEP;
    }

    // End effector Y position (Q/E) - Q: up, E: down
    if (keyStates['KeyQ']) {
      this.state.eePos[1] += this.EE_STEP;
    }
    if (keyStates['KeyE']) {
      this.state.eePos[1] -= this.EE_STEP;
    }

    // Pitch adjustment (R/F)
    if (keyStates['KeyR']) {
      this.state.pitch += this.PITCH_STEP;
    }
    if (keyStates['KeyF']) {
      this.state.pitch -= this.PITCH_STEP;
    }

    // Wrist roll (Z/C)
    if (keyStates['KeyZ']) {
      this.state.targetJoints[4] += this.JOINT_STEP * 3;
    }
    if (keyStates['KeyC']) {
      this.state.targetJoints[4] -= this.JOINT_STEP * 3;
    }

    // Calculate IK with pitch compensation
    const compensatedY = this.state.eePos[1] + this.TIP_LENGTH * Math.sin(this.state.pitch);
    const [j2, j3] = inverseKinematics2Link(this.state.eePos[0], compensatedY);
    this.state.targetJoints[1] = j2;  // Pitch
    this.state.targetJoints[2] = j3;  // Elbow

    // Wrist pitch compensation to maintain end effector orientation
    this.state.targetJoints[3] = j2 - j3 + this.state.pitch;

    // ========================================
    // Gripper Control (toggle with cooldown)
    // ========================================

    // Gripper toggle (V)
    if (keyStates['KeyV'] && this.state.gripperCooldown === 0) {
      this.state.gripperOpen = !this.state.gripperOpen;
      this.state.targetJoints[5] = this.state.gripperOpen ? this.GRIPPER_OPEN : this.GRIPPER_CLOSED;
      this.state.gripperCooldown = this.GRIPPER_COOLDOWN_FRAMES;
    }

    // ========================================
    // Reset (X)
    // ========================================
    if (keyStates['KeyX']) {
      this.reset(model, data);
      return;
    }

    // ========================================
    // Apply target positions to actuators
    // ========================================
    for (let i = 0; i < 6; i++) {
      data.ctrl[i] = this.state.targetJoints[i];
    }
  }

  /**
   * Get the list of keys this controller uses
   * @returns {string[]}
   */
  getControlKeys() {
    return [
      // Arm control
      'KeyA', 'KeyD',          // Shoulder rotation
      'KeyW', 'KeyS',          // EE Y position
      'KeyQ', 'KeyE',          // EE X position
      'KeyR', 'KeyF',          // Pitch adjustment
      'KeyZ', 'KeyC',          // Wrist roll
      // Gripper
      'KeyV',
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
      'Rotation: A/D | Forward/Back: W/S | Up/Down: Q/E',
      'Pitch: R/F | Wrist Roll: Z/C',
      'Gripper: V Toggle | Reset: X'
    ].join('\n');
  }
}
