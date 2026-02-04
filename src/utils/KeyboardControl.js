/**
 * Keyboard Control Module for MuJoCo Scenes
 *
 * Provides configurable keyboard controls for different robots.
 * 统一使用异步 step() 模式，每个控制周期调用一次。
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
    controller: XLeRobotController,
    description: 'XLeRobot Dual-Arm Control'
  },
  'SO101': {
    controller: SO101Controller,
    description: 'SO101 Single-Arm Control'
  },
  'panda': {
    controller: PandaController,
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
    this.mujoco = null;
    this.keyStates = {};
    this.customController = null;

    // Bind event handlers
    this._onKeyDown = this._onKeyDown.bind(this);
    this._onKeyUp = this._onKeyUp.bind(this);
    this._onBlur = this._onBlur.bind(this);
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
   * @param {object} mujoco - MuJoCo WASM module
   * @returns {Promise<boolean>} - Whether enabling was successful
   */
  async enable(robotName, model, data, mujoco) {
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

    // Create and initialize controller
    if (config.controller) {
      const ControllerClass = config.controller;
      this.customController = new ControllerClass();
      await this.customController.initialize(model, data, mujoco);

      // Initialize key states for controller
      this.keyStates = {};
      for (const key of this.customController.getControlKeys()) {
        this.keyStates[key] = false;
      }
    } else {
      console.warn(`No controller class for robot: ${robotName}`);
      return false;
    }

    // Add event listeners
    document.addEventListener('keydown', this._onKeyDown);
    document.addEventListener('keyup', this._onKeyUp);
    window.addEventListener('blur', this._onBlur);

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
    window.removeEventListener('blur', this._onBlur);

    this.enabled = false;
    this.config = null;
    this.model = null;
    this.data = null;
    this.mujoco = null;
    this.keyStates = {};
    this.customController = null;
  }

  /**
   * 异步控制步进 - 每个控制周期调用一次
   * @returns {Promise<void>}
   */
  async step() {
    if (!this.enabled || !this.customController || !this.data) return;

    await this.customController.step(this.keyStates, this.model, this.data, this.mujoco);
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

  _onBlur() {
    if (!this.enabled) return;
    
    // Reset all key states when window loses focus
    // This prevents stuck keys when user clicks outside the window
    for (const key in this.keyStates) {
      this.keyStates[key] = false;
    }
  }
}

// Singleton instance for easy access
export const keyboardController = new KeyboardController();
