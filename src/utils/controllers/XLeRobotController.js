/**
 * XLeRobot Controller
 *
 * Dual-arm robot controller with inverse kinematics.
 *
 * Actuator index mapping:
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

import { BaseController } from './BaseController.js';
import { inverseKinematics2Link } from '../math/inverseKinematics.js';

export class XLeRobotController extends BaseController {
  constructor() {
    super();

    // Control constants (1/8 of original for finer control)
    this.JOINT_STEP = 0.002;    // 0.01 / 8
    this.EE_STEP = 0.0002;      // 0.005 / 8
    this.PITCH_STEP = 0.0025;   // 0.02 / 8
    this.TIP_LENGTH = 0.108;    // Length from wrist to end effector tip
    this.BASE_SPEED = 2;

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
   * @param {object} _model - MuJoCo model (unused)
   * @param {object} data - MuJoCo data
   */
  reset(_model, data) {
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
    const [j2_1, j3_1] = inverseKinematics2Link(this.state.eePos1[0], compensatedY1);
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
    const [j2_2, j3_2] = inverseKinematics2Link(this.state.eePos2[0], compensatedY2);
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
      this.reset(model, data);
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
