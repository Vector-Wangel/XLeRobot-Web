/**
 * Base Controller Interface
 *
 * All robot controllers should extend this class.
 */

export class BaseController {
  constructor() {
    this.initialized = false;
  }

  /**
   * Initialize the controller with model and data
   * @param {object} model - MuJoCo model
   * @param {object} data - MuJoCo data
   * @param {object} mujoco - MuJoCo WASM module
   */
  initialize(model, data, mujoco) {
    throw new Error('initialize() must be implemented by subclass');
  }

  /**
   * Reset the controller to initial state
   * @param {object} model - MuJoCo model
   * @param {object} data - MuJoCo data
   */
  reset(model, data) {
    throw new Error('reset() must be implemented by subclass');
  }

  /**
   * Update controls based on keyboard state
   * @param {object} keyStates - Current keyboard states
   * @param {object} model - MuJoCo model
   * @param {object} data - MuJoCo data
   * @param {object} mujoco - MuJoCo WASM module
   */
  update(keyStates, model, data, mujoco) {
    throw new Error('update() must be implemented by subclass');
  }

  /**
   * Get the list of keys this controller uses
   * @returns {string[]}
   */
  getControlKeys() {
    throw new Error('getControlKeys() must be implemented by subclass');
  }

  /**
   * Get description for GUI display
   * @returns {string}
   */
  getDescription() {
    throw new Error('getDescription() must be implemented by subclass');
  }
}
