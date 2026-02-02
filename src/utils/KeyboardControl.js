/**
 * Keyboard Control Module for MuJoCo Scenes
 *
 * Provides configurable keyboard controls for different robots.
 * Supports both simple control mappings and complex controllers with IK.
 */

import { XLeRobotController } from './controllers/XLeRobotController.js';
import { PandaController } from './controllers/PandaController.js';
import { SO101Controller } from './controllers/SO101Controller.js';

// ============================================================================
// Robot Control Configurations
// ============================================================================

// Robot-specific control configurations (keyed by robot name)
const ROBOT_CONFIGS = {
  'xlerobot': {
    type: 'custom',
    controller: new XLeRobotController(),
    description: 'XLeRobot Dual-Arm Control'
  },
  'SO101': {
    type: 'custom',
    controller: new SO101Controller(),
    description: 'SO101 Single-Arm Control'
  },
  'panda': {
    type: 'custom',
    controller: new PandaController(),
    description: 'Panda DLS IK Control'
  }
  // humanoid has no keyboard control
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
   * Get configuration for a robot
   * @param {string} robotName - The robot name (e.g., 'xlerobot', 'SO101', 'panda')
   * @returns {object|null} - Configuration object or null if not configured
   */
  getConfig(robotName) {
    return ROBOT_CONFIGS[robotName] || null;
  }

  /**
   * Check if a robot has keyboard control configured
   * @param {string} robotName - The robot name
   * @returns {boolean}
   */
  hasConfig(robotName) {
    return robotName in ROBOT_CONFIGS;
  }

  /**
   * Enable keyboard control for a robot
   * @param {string} robotName - The robot name
   * @param {object} model - MuJoCo model
   * @param {object} data - MuJoCo data
   * @param {object} mujoco - MuJoCo WASM module (optional, needed for IK)
   * @returns {boolean} - Whether enabling was successful
   */
  enable(robotName, model, data, mujoco = null) {
    // Disable any existing control first
    this.disable();

    const config = this.getConfig(robotName);
    if (!config) {
      console.log(`No keyboard control configured for robot: ${robotName}`);
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
    console.log(`Keyboard control enabled for robot: ${robotName}`);
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
