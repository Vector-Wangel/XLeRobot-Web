/**
 * Scene Manager
 *
 * Manages loading of modular scenes using MuJoCo's include mechanism.
 * Scene files use <include file="robot.xml"/> to include robot definitions.
 */

import { RobotLoader, SceneConfigManager } from './RobotLoader.js';
import { downloadRobotAssets, isRobotDownloaded } from '../mujocoUtils.js';

export class SceneManager {
  /**
   * Robot configurations - centralized robot metadata
   * All robots are in assets/robots/{robotDir}/
   */
  static ROBOT_CONFIGS = {
    'xlerobot': {
      xmlPath: './assets/robots/xlerobot/xlerobot.xml',
      objectsPath: './assets/robots/xlerobot/objects.xml',
      robotDir: 'xlerobot',
      meshDir: 'assets',
      description: 'XLeRobot Dual-Arm Mobile Robot'
    },
    'SO101': {
      xmlPath: './assets/robots/xlerobot/SO101.xml',
      objectsPath: './assets/robots/xlerobot/objects_SO101.xml',
      robotDir: 'xlerobot',  // shares assets with xlerobot
      meshDir: 'assets',
      description: 'SO101 Single Arm'
    },
    'panda': {
      xmlPath: './assets/robots/panda/panda.xml',
      objectsPath: './assets/robots/panda/objects.xml',
      robotDir: 'panda',
      meshDir: 'assets',
      description: 'Franka Emika Panda'
    },
    'humanoid': {
      xmlPath: './assets/robots/humanoid/humanoid.xml',
      objectsPath: null,  // humanoid has no objects
      robotDir: 'humanoid',
      meshDir: null,  // humanoid uses primitive shapes, no assets
      description: 'DeepMind Humanoid'
    }
  };

  /**
   * Environment configurations
   * All environments are in assets/environments/{envName}/
   */
  static ENV_CONFIGS = {
    'tabletop': {
      xmlPath: './assets/environments/tabletop/scene.xml',
      spzPath: './assets/environments/tabletop/scene.spz',
      description: 'Tabletop manipulation environment'
    },
    'basic': {
      xmlPath: './assets/environments/basic.xml',
      spzPath: null,
      description: 'Basic environment with light and floor only'
    }
  };

  constructor(mujoco) {
    this.mujoco = mujoco;
    this.robotLoader = new RobotLoader(mujoco);
    this.configManager = new SceneConfigManager();
    this.currentEnv = null;
    this.currentRobot = null;
    this.scenePath = null;
    this.customSpzData = null;  // Store custom SPZ data from user upload
    this.customCollisionXml = null;  // Store custom collision XML content
  }

  /**
   * Load a scene using MuJoCo's include mechanism.
   * Creates a scene XML that uses <include> to bring in robot and objects.
   *
   * @param {string} envName - Environment name (e.g., 'tabletop')
   * @param {string} robotName - Robot name (e.g., 'xlerobot', 'SO101')
   * @returns {Promise<string>} - Path to the scene XML in virtual filesystem
   */
  async loadModularScene(envName, robotName) {
    console.log(`Loading modular scene: env=${envName}, robot=${robotName}`);

    try {
      if (!SceneManager.ROBOT_CONFIGS[robotName]) {
        throw new Error(`Unknown robot: ${robotName}`);
      }

      const envConfig = SceneManager.ENV_CONFIGS[envName];
      if (!envConfig) {
        throw new Error(`Unknown environment: ${envName}`);
      }

      const vfsSceneDir = `/working/scenes/${robotName}`;
      this._ensureDir('/working/scenes');
      this._ensureDir(vfsSceneDir);

      // Copy robot files to scene directory
      const hasObjects = await this._copyRobotToDir(robotName, vfsSceneDir);

      // Load environment XML and create scene
      const envResponse = await fetch(envConfig.xmlPath);
      if (!envResponse.ok) {
        throw new Error(`Environment XML not found: ${envConfig.xmlPath}`);
      }
      const envXml = await envResponse.text();

      const sceneName = `${envName}_${robotName}`;
      const sceneXml = this._createSceneXml(envXml, robotName, hasObjects, sceneName);
      this._writeToFS(`${vfsSceneDir}/scene.xml`, sceneXml);

      this.currentEnv = envName;
      this.currentRobot = robotName;
      this.scenePath = `scenes/${robotName}/scene.xml`;

      console.log(`Scene loaded: ${this.scenePath}`);
      return this.scenePath;

    } catch (error) {
      console.error('Scene loading failed:', error);
      throw error;
    }
  }

  /**
   * Create scene XML by inserting include statements into environment XML
   */
  _createSceneXml(envXml, robotName, hasObjects, sceneName) {
    // Parse environment XML
    const parser = new DOMParser();
    const doc = parser.parseFromString(envXml, 'text/xml');

    if (doc.querySelector('parsererror')) {
      throw new Error('Failed to parse environment XML');
    }

    // Update model name
    const mujoco = doc.documentElement;
    mujoco.setAttribute('model', sceneName);

    // Create include element for robot
    const robotInclude = doc.createElement('include');
    robotInclude.setAttribute('file', `${robotName}.xml`);

    // Insert include at the beginning (after mujoco tag)
    mujoco.insertBefore(robotInclude, mujoco.firstChild);

    // Add objects include if exists
    if (hasObjects) {
      const objectsInclude = doc.createElement('include');
      objectsInclude.setAttribute('file', 'objects.xml');
      // Insert after robot include
      mujoco.insertBefore(objectsInclude, robotInclude.nextSibling);
    }

    // Serialize back to string
    const serializer = new XMLSerializer();
    let result = serializer.serializeToString(doc);

    // Clean up XML
    result = result.replace(/(<\?xml[^?]*\?>)/g, '');
    result = result.replace(/\s+xmlns="[^"]*"/g, '');
    result = result.replace(/\s+xmlns:[a-z]+="[^"]*"/g, '');
    result = '<?xml version="1.0" encoding="UTF-8"?>\n' + result.trim();

    return result;
  }

  /**
   * Write content to MuJoCo virtual filesystem
   */
  _writeToFS(path, content) {
    // Try to remove existing file
    try {
      this.mujoco.FS.unlink(path);
    } catch (e) {
      // File doesn't exist, that's fine
    }

    // Write new file
    if (typeof content === 'string') {
      this.mujoco.FS.writeFile(path, content);
    } else {
      this.mujoco.FS.writeFile(path, content);
    }
    console.log(`Written to VFS: ${path}`);
  }

  /**
   * Ensure directory exists in virtual filesystem
   */
  _ensureDir(path) {
    try {
      if (!this.mujoco.FS.analyzePath(path).exists) {
        this.mujoco.FS.mkdir(path);
      }
    } catch (e) {
      // Directory might already exist
    }
  }

  /**
   * Load user-uploaded robot files
   * @param {FileList} files - Files from input[webkitdirectory]
   * @param {string} envName - Environment to load robot into
   * @returns {Promise<string>} - Path to scene XML
   */
  async loadUploadedRobot(files, envName = 'tabletop') {
    // Parse uploaded files
    const { robotXml, objectsXml, meshFiles, robotName } =
      await this.robotLoader.loadUploadedRobot(files);

    const vfsSceneDir = `/working/scenes/uploaded_${robotName}`;

    // Create directories
    this._ensureDir('/working/scenes');
    this._ensureDir(vfsSceneDir);
    this._ensureDir(`${vfsSceneDir}/assets`);

    // Write asset files
    for (const [name, buffer] of meshFiles) {
      const meshPath = `${vfsSceneDir}/assets/${name}`;
      this.mujoco.FS.writeFile(meshPath, new Uint8Array(buffer));
    }

    // Fix meshdir and write robot XML
    let fixedRobotXml = robotXml.replace(
      /meshdir="[^"]*"/g,
      `meshdir="./assets/"`
    );
    this._writeToFS(`${vfsSceneDir}/robot.xml`, fixedRobotXml);

    // Write objects XML if exists
    const hasObjects = !!objectsXml;
    if (hasObjects) {
      this._writeToFS(`${vfsSceneDir}/objects.xml`, objectsXml);
    }

    // Load environment XML
    // If in custom_spz mode, use 'basic' environment for the XML
    const effectiveEnv = (envName === 'custom_spz') ? 'basic' : envName;
    const envConfig = SceneManager.ENV_CONFIGS[effectiveEnv];
    const envResponse = await fetch(envConfig.xmlPath);
    const envXml = await envResponse.text();

    // Create scene XML with include
    const sceneName = `${effectiveEnv}_uploaded_${robotName}`;
    const sceneXml = this._createSceneXml(envXml, 'robot', hasObjects, sceneName);

    // Write scene XML
    this._writeToFS(`${vfsSceneDir}/scene.xml`, sceneXml);

    // Keep custom_spz as currentEnv if that was the original environment
    this.currentEnv = envName;
    this.currentRobot = `uploaded_${robotName}`;
    this.scenePath = `scenes/uploaded_${robotName}/scene.xml`;

    return this.scenePath;
  }

  /**
   * Get current scene info
   */
  getCurrentSceneInfo() {
    return {
      environment: this.currentEnv,
      robot: this.currentRobot,
      scenePath: this.scenePath
    };
  }

  /**
   * Get 3DGS path for current environment
   * @returns {string|null}
   */
  getSpzPath() {
    if (!this.currentEnv) return null;
    const envConfig = SceneManager.ENV_CONFIGS[this.currentEnv];
    return envConfig ? envConfig.spzPath : null;
  }

  /**
   * Get custom SPZ data if available
   * @returns {ArrayBuffer|null}
   */
  getCustomSpzData() {
    return this.customSpzData;
  }

  /**
   * Check if using custom SPZ
   * @returns {boolean}
   */
  hasCustomSpz() {
    return this.customSpzData !== null;
  }

  /**
   * Clear custom SPZ data
   */
  clearCustomSpz() {
    this.customSpzData = null;
  }

  /**
   * Set custom collision XML content
   * @param {string} xmlContent - The collision XML content
   */
  setCustomCollision(xmlContent) {
    this.customCollisionXml = xmlContent;
    console.log('Custom collision XML set');
  }

  /**
   * Clear custom collision XML
   */
  clearCustomCollision() {
    this.customCollisionXml = null;
    console.log('Custom collision XML cleared');
  }

  /**
   * Check if custom collision XML is available
   * @returns {boolean}
   */
  hasCustomCollision() {
    return this.customCollisionXml !== null;
  }

  /**
   * Get custom collision XML content
   * @returns {string|null}
   */
  getCustomCollisionXml() {
    return this.customCollisionXml;
  }

  /**
   * Load a custom SPZ file with basic environment
   * @param {File} spzFile - The SPZ file to load
   * @param {string} robotName - Robot to use (default: none)
   * @returns {Promise<string>} - Path to scene XML
   */
  async loadCustomSpz(spzFile, robotName = null) {
    console.log(`Loading custom SPZ: ${spzFile.name}`);
    this.customSpzData = await spzFile.arrayBuffer();
    return this._setupCustomSpzScene(robotName);
  }

  /**
   * Reload custom SPZ scene with a different robot
   * Uses the already stored customSpzData
   * @param {string} robotName - Robot to use (null for no robot)
   * @returns {Promise<string>} - Path to scene XML
   */
  async loadCustomSpzWithRobot(robotName = null) {
    if (!this.customSpzData) {
      throw new Error('No custom SPZ data stored');
    }
    console.log(`Reloading custom SPZ with robot: ${robotName}`);
    return this._setupCustomSpzScene(robotName);
  }

  /**
   * Internal helper to setup custom SPZ scene with optional robot
   * @param {string} robotName - Robot to use (null for no robot)
   * @returns {Promise<string>} - Path to scene XML
   */
  async _setupCustomSpzScene(robotName) {
    // Load basic environment XML
    const envConfig = SceneManager.ENV_CONFIGS['basic'];
    const envResponse = await fetch(envConfig.xmlPath);
    const envXml = await envResponse.text();

    // Handle uploaded robots - use their existing scene directory
    if (this.isUploadedRobot(robotName)) {
      const uploadedSceneDir = `/working/scenes/${robotName}`;

      // Check if uploaded robot files still exist
      try {
        this.mujoco.FS.readFile(`${uploadedSceneDir}/robot.xml`);
      } catch (e) {
        throw new Error('Uploaded robot files not found. Please re-upload the robot.');
      }

      // Check if objects.xml exists
      let hasObjects = false;
      try {
        this.mujoco.FS.readFile(`${uploadedSceneDir}/objects.xml`);
        hasObjects = true;
      } catch (e) {
        // No objects file
      }

      // Write collision.xml if custom collision is set
      if (this.customCollisionXml) {
        this._writeToFS(`${uploadedSceneDir}/collision.xml`, this.customCollisionXml);
      }

      // Create scene XML with environment, collision, and uploaded robot
      const sceneXml = this._createSceneXmlWithCollision(envXml, 'robot', hasObjects, `custom_spz_${robotName}`, this.hasCustomCollision());
      this._writeToFS(`${uploadedSceneDir}/scene.xml`, sceneXml);

      this.currentRobot = robotName;
      this.currentEnv = 'custom_spz';
      this.scenePath = `scenes/${robotName}/scene.xml`;

      console.log(`Custom SPZ scene with uploaded robot ready: ${this.scenePath}`);
      return this.scenePath;
    }

    // Handle built-in robots
    const vfsSceneDir = `/working/scenes/custom_spz`;
    this._ensureDir('/working/scenes');
    this._ensureDir(vfsSceneDir);

    // Write collision.xml if custom collision is set
    if (this.customCollisionXml) {
      this._writeToFS(`${vfsSceneDir}/collision.xml`, this.customCollisionXml);
    }

    if (robotName && SceneManager.ROBOT_CONFIGS[robotName]) {
      const hasObjects = await this._copyRobotToDir(robotName, vfsSceneDir);
      const sceneXml = this._createSceneXmlWithCollision(envXml, robotName, hasObjects, `custom_spz_${robotName}`, this.hasCustomCollision());
      this._writeToFS(`${vfsSceneDir}/scene.xml`, sceneXml);
      this.currentRobot = robotName;
    } else {
      // No robot - still need to handle collision XML
      if (this.customCollisionXml) {
        const sceneXml = this._createSceneXmlWithCollision(envXml, null, false, 'custom_spz', true);
        this._writeToFS(`${vfsSceneDir}/scene.xml`, sceneXml);
      } else {
        this._writeToFS(`${vfsSceneDir}/scene.xml`, envXml);
      }
      this.currentRobot = null;
    }

    this.currentEnv = 'custom_spz';
    this.scenePath = `scenes/custom_spz/scene.xml`;

    console.log(`Custom SPZ scene ready: ${this.scenePath}`);
    return this.scenePath;
  }

  /**
   * Create scene XML with optional collision include
   * @param {string} envXml - Base environment XML
   * @param {string} robotName - Robot name (or null)
   * @param {boolean} hasObjects - Whether objects.xml exists
   * @param {string} sceneName - Scene name for model attribute
   * @param {boolean} hasCollision - Whether to include collision.xml
   * @returns {string} - Generated scene XML
   */
  _createSceneXmlWithCollision(envXml, robotName, hasObjects, sceneName, hasCollision) {
    // Parse environment XML
    const parser = new DOMParser();
    const doc = parser.parseFromString(envXml, 'text/xml');

    if (doc.querySelector('parsererror')) {
      throw new Error('Failed to parse environment XML');
    }

    // Update model name
    const mujoco = doc.documentElement;
    mujoco.setAttribute('model', sceneName);

    // Insert includes at the beginning (after mujoco tag)
    // Order: collision -> robot -> objects
    let insertPoint = mujoco.firstChild;

    // Add collision include if available
    if (hasCollision) {
      const collisionInclude = doc.createElement('include');
      collisionInclude.setAttribute('file', 'collision.xml');
      mujoco.insertBefore(collisionInclude, insertPoint);
      insertPoint = collisionInclude.nextSibling;
    }

    // Add robot include if specified
    if (robotName) {
      const robotInclude = doc.createElement('include');
      robotInclude.setAttribute('file', `${robotName}.xml`);
      mujoco.insertBefore(robotInclude, insertPoint);
      insertPoint = robotInclude.nextSibling;

      // Add objects include if exists
      if (hasObjects) {
        const objectsInclude = doc.createElement('include');
        objectsInclude.setAttribute('file', 'objects.xml');
        mujoco.insertBefore(objectsInclude, insertPoint);
      }
    }

    // Serialize back to string
    const serializer = new XMLSerializer();
    let result = serializer.serializeToString(doc);

    // Clean up XML
    result = result.replace(/(<\?xml[^?]*\?>)/g, '');
    result = result.replace(/\s+xmlns="[^"]*"/g, '');
    result = result.replace(/\s+xmlns:[a-z]+="[^"]*"/g, '');
    result = '<?xml version="1.0" encoding="UTF-8"?>\n' + result.trim();

    return result;
  }

  /**
   * Copy robot files (XML, objects, meshes) to target directory
   * @param {string} robotName - Robot name
   * @param {string} targetDir - Target VFS directory
   * @returns {Promise<boolean>} - Whether objects file exists
   */
  async _copyRobotToDir(robotName, targetDir) {
    console.log(`_copyRobotToDir: robotName=${robotName}, targetDir=${targetDir}`);
    const robotConfig = SceneManager.ROBOT_CONFIGS[robotName];
    const robotDir = robotConfig.robotDir;

    // Ensure robot assets are downloaded (lazy loading)
    if (!isRobotDownloaded(robotDir)) {
      console.log(`Robot ${robotDir} not yet downloaded, downloading now...`);
      await downloadRobotAssets(this.mujoco, robotDir);
      console.log(`Robot ${robotDir} download completed`);
    } else {
      console.log(`Robot ${robotDir} already downloaded`);
    }

    if (robotConfig.meshDir) {
      this._ensureDir(`${targetDir}/assets`);
      console.log(`Created assets directory: ${targetDir}/assets`);
    }

    // Copy robot XML
    console.log(`Fetching robot XML from: ${robotConfig.xmlPath}`);
    const robotXmlResponse = await fetch(robotConfig.xmlPath);
    let robotXml = await robotXmlResponse.text();
    if (robotConfig.meshDir) {
      robotXml = robotXml.replace(/meshdir="[^"]*"/g, `meshdir="./assets/"`);
    }
    this._writeToFS(`${targetDir}/${robotName}.xml`, robotXml);

    // Copy objects if exists
    let hasObjects = false;
    if (robotConfig.objectsPath) {
      try {
        const objectsResponse = await fetch(robotConfig.objectsPath);
        if (objectsResponse.ok) {
          const objectsXml = await objectsResponse.text();
          this._writeToFS(`${targetDir}/objects.xml`, objectsXml);
          hasObjects = true;
          console.log(`Objects XML written: ${targetDir}/objects.xml`);
        }
      } catch (e) {
        console.log('No objects file for robot:', robotName);
      }
    }

    // Copy mesh files
    if (robotConfig.meshDir) {
      const srcMeshDir = `/working/robots/${robotDir}/assets`;
      console.log(`Copying mesh files from: ${srcMeshDir}`);
      this._copyMeshFiles(srcMeshDir, `${targetDir}/assets`);
    } else {
      console.log(`Robot ${robotName} has no mesh directory (uses primitives)`);
    }

    return hasObjects;
  }

  /**
   * Copy mesh files from source to destination directory
   * @param {string} srcDir - Source directory in VFS
   * @param {string} dstDir - Destination directory in VFS
   */
  _copyMeshFiles(srcDir, dstDir) {
    try {
      // Check if source directory exists
      const srcExists = this.mujoco.FS.analyzePath(srcDir).exists;
      if (!srcExists) {
        console.error(`Source mesh directory does not exist: ${srcDir}`);
        return;
      }

      const meshFiles = this.mujoco.FS.readdir(srcDir);
      console.log(`Copying ${meshFiles.length - 2} mesh files from ${srcDir} to ${dstDir}`);

      let copiedCount = 0;
      for (const file of meshFiles) {
        if (file === '.' || file === '..') continue;
        try {
          const content = this.mujoco.FS.readFile(`${srcDir}/${file}`);
          this.mujoco.FS.writeFile(`${dstDir}/${file}`, content);
          copiedCount++;
        } catch (e) {
          console.warn(`Failed to copy mesh file ${file}:`, e.message);
        }
      }
      console.log(`Copied ${copiedCount} mesh files to ${dstDir}`);
    } catch (e) {
      console.error('Failed to read mesh directory:', srcDir, e);
    }
  }

  /**
   * List available environments
   */
  listEnvironments() {
    return Object.keys(SceneManager.ENV_CONFIGS);
  }

  /**
   * List available robots
   */
  listRobots() {
    return Object.keys(SceneManager.ROBOT_CONFIGS);
  }

  /**
   * Check if robot is an uploaded custom robot
   * @param {string} robotName - Robot name to check
   * @returns {boolean}
   */
  isUploadedRobot(robotName) {
    return robotName && robotName.startsWith('uploaded_');
  }

  /**
   * Reload uploaded robot scene
   * Used when reloading a scene with an uploaded robot
   * @param {string} envName - Environment name
   * @returns {Promise<string>} - Path to scene XML
   */
  async reloadUploadedRobot(envName) {
    if (!this.currentRobot || !this.isUploadedRobot(this.currentRobot)) {
      throw new Error('No uploaded robot to reload');
    }

    const vfsSceneDir = `/working/scenes/${this.currentRobot}`;

    // Check if scene files still exist
    try {
      this.mujoco.FS.readFile(`${vfsSceneDir}/scene.xml`);
    } catch (e) {
      throw new Error('Uploaded robot scene files not found. Please re-upload the robot.');
    }

    // If environment changed, recreate scene XML with new environment
    const effectiveEnv = (envName === 'custom_spz') ? 'basic' : envName;
    const envConfig = SceneManager.ENV_CONFIGS[effectiveEnv];
    if (!envConfig) {
      throw new Error(`Unknown environment: ${envName}`);
    }

    const envResponse = await fetch(envConfig.xmlPath);
    const envXml = await envResponse.text();

    // Check if objects.xml exists
    let hasObjects = false;
    try {
      this.mujoco.FS.readFile(`${vfsSceneDir}/objects.xml`);
      hasObjects = true;
    } catch (e) {
      // No objects file
    }

    // Recreate scene XML with environment
    const sceneName = `${effectiveEnv}_${this.currentRobot}`;
    const sceneXml = this._createSceneXml(envXml, 'robot', hasObjects, sceneName);
    this._writeToFS(`${vfsSceneDir}/scene.xml`, sceneXml);

    this.currentEnv = envName;
    this.scenePath = `scenes/${this.currentRobot}/scene.xml`;

    return this.scenePath;
  }
}

// Singleton instance
let sceneManagerInstance = null;

export function getSceneManager(mujoco) {
  if (!sceneManagerInstance && mujoco) {
    sceneManagerInstance = new SceneManager(mujoco);
  }
  return sceneManagerInstance;
}
