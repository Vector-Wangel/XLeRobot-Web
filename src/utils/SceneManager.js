/**
 * Scene Manager
 *
 * Manages loading of modular scenes (environment + robot + objects).
 * Provides both legacy scene loading and new modular loading support.
 */

import { MujocoXmlMerger } from './XmlMerger.js';
import { RobotLoader, SceneConfigManager } from './RobotLoader.js';

export class SceneManager {
  /**
   * Robot configurations - centralized robot metadata
   * All robots are now in assets/robots/{robotDir}/
   */
  static ROBOT_CONFIGS = {
    'xlerobot': {
      xmlPath: './assets/robots/xlerobot/xlerobot.xml',
      objectsPath: './assets/robots/xlerobot/objects.xml',
      robotDir: 'xlerobot',
      meshDir: 'meshes',
      description: 'XLeRobot Dual-Arm Mobile Robot'
    },
    'SO101': {
      xmlPath: './assets/robots/xlerobot/SO101.xml',
      objectsPath: './assets/robots/xlerobot/objects_SO101.xml',
      robotDir: 'xlerobot',  // shares meshes with xlerobot
      meshDir: 'meshes',
      description: 'SO101 Single Arm'
    },
    'panda': {
      xmlPath: './assets/robots/panda/panda.xml',
      objectsPath: './assets/robots/panda/objects.xml',
      robotDir: 'panda',
      meshDir: 'meshes',
      description: 'Franka Emika Panda'
    },
    'humanoid': {
      xmlPath: './assets/robots/humanoid/humanoid.xml',
      objectsPath: null,  // humanoid has no objects
      robotDir: 'humanoid',
      meshDir: null,  // humanoid uses primitive shapes, no meshes
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
    }
  };

  constructor(mujoco) {
    this.mujoco = mujoco;
    this.robotLoader = new RobotLoader(mujoco);
    this.configManager = new SceneConfigManager();
    this.currentEnv = null;
    this.currentRobot = null;
    this.mergedScenePath = null;
  }

  /**
   * Load a scene using the modular approach (env + robot + objects)
   * Falls back to legacy loading if modular files don't exist.
   *
   * @param {string} envName - Environment name (e.g., 'tabletop')
   * @param {string} robotName - Robot name (e.g., 'xlerobot', 'SO101')
   * @returns {Promise<string>} - Path to the scene XML in virtual filesystem
   */
  async loadModularScene(envName, robotName) {
    console.log(`Loading modular scene: env=${envName}, robot=${robotName}`);

    try {
      // 1. Load environment XML
      const envPath = `./assets/environments/${envName}/scene.xml`;
      const envResponse = await fetch(envPath);
      if (!envResponse.ok) {
        throw new Error(`Environment not found: ${envPath}`);
      }
      const envXml = await envResponse.text();

      // 2. Load robot XML
      const robotXmlPath = this._getRobotXmlPath(robotName);
      const robotResponse = await fetch(robotXmlPath);
      if (!robotResponse.ok) {
        throw new Error(`Robot not found: ${robotXmlPath}`);
      }
      const robotXml = await robotResponse.text();

      // 3. Load objects XML (optional)
      const objectsXmlPath = this._getObjectsXmlPath(robotName);
      let objectsXml = null;
      if (objectsXmlPath) {
        try {
          const objectsResponse = await fetch(objectsXmlPath);
          if (objectsResponse.ok) {
            objectsXml = await objectsResponse.text();
          }
        } catch (e) {
          // Objects file is optional
        }
      }

      // 4. Merge XMLs
      let mergedXml = MujocoXmlMerger.merge(envXml, robotXml, objectsXml);

      // 5. Fix mesh paths - meshdir needs to point to the robot folder in VFS
      // Robot files are downloaded to /working/robots/{robotDir}/
      const robotDir = this._getRobotDir(robotName);
      mergedXml = MujocoXmlMerger.fixMeshPaths(mergedXml, `/working/robots/${robotDir}`);

      // 6. Set model name
      const sceneName = `${envName}_${robotName}`;
      mergedXml = MujocoXmlMerger.setModelName(mergedXml, sceneName);

      // 7. Write merged XML to virtual filesystem
      const mergedPath = `/working/merged_${sceneName}.xml`;
      this._writeToFS(mergedPath, mergedXml);

      this.currentEnv = envName;
      this.currentRobot = robotName;
      this.mergedScenePath = `merged_${sceneName}.xml`;

      console.log(`Modular scene loaded: ${mergedPath}`);
      return this.mergedScenePath;

    } catch (error) {
      console.error('Modular scene loading failed:', error);
      throw error;
    }
  }

  /**
   * Get robot XML path based on robot name
   */
  _getRobotXmlPath(robotName) {
    const config = SceneManager.ROBOT_CONFIGS[robotName];
    return config ? config.xmlPath : `./assets/robots/${robotName}/${robotName}.xml`;
  }

  /**
   * Get robot objects XML path
   */
  _getObjectsXmlPath(robotName) {
    const config = SceneManager.ROBOT_CONFIGS[robotName];
    return config ? config.objectsPath : `./assets/robots/${robotName}/objects.xml`;
  }

  /**
   * Get robot directory path for mesh resolution
   */
  _getRobotDir(robotName) {
    const config = SceneManager.ROBOT_CONFIGS[robotName];
    return config ? config.robotDir : robotName;
  }

  /**
   * Get mesh directory name for a robot
   */
  _getMeshDir(robotName) {
    const config = SceneManager.ROBOT_CONFIGS[robotName];
    return config ? config.meshDir : 'meshes';
  }

  /**
   * Write content to MuJoCo virtual filesystem
   */
  _writeToFS(path, content) {
    const filename = path.split('/').pop();
    const dir = path.substring(0, path.lastIndexOf('/'));

    // Try to remove existing file
    try {
      this.mujoco.FS.unlink(path);
    } catch (e) {
      // File doesn't exist, that's fine
    }

    // Write new file
    this.mujoco.FS.writeFile(path, content);
    console.log(`Written to VFS: ${path}`);
  }

  /**
   * Load user-uploaded robot files
   * @param {FileList} files - Files from input[webkitdirectory]
   * @param {string} envName - Environment to load robot into
   * @returns {Promise<string>} - Path to merged scene
   */
  async loadUploadedRobot(files, envName = 'tabletop') {
    // Parse uploaded files
    const { robotXml, objectsXml, meshFiles, robotName } =
      await this.robotLoader.loadUploadedRobot(files);

    // Write mesh files to virtual filesystem
    const robotDir = `/working/uploaded_${robotName}`;
    this._ensureDir(robotDir);
    this._ensureDir(`${robotDir}/meshes`);

    for (const [name, buffer] of meshFiles) {
      const meshPath = `${robotDir}/meshes/${name}`;
      this.mujoco.FS.writeFile(meshPath, new Uint8Array(buffer));
    }

    // Fix mesh paths in robot XML
    const fixedRobotXml = MujocoXmlMerger.fixMeshPaths(robotXml, robotDir);

    // Load environment
    const envPath = `./assets/environments/${envName}/scene.xml`;
    const envResponse = await fetch(envPath);
    const envXml = await envResponse.text();

    // Merge
    let mergedXml = MujocoXmlMerger.merge(envXml, fixedRobotXml, objectsXml);
    const sceneName = `${envName}_uploaded_${robotName}`;
    mergedXml = MujocoXmlMerger.setModelName(mergedXml, sceneName);

    // Write merged scene
    const mergedPath = `/working/merged_${sceneName}.xml`;
    this._writeToFS(mergedPath, mergedXml);

    this.currentEnv = envName;
    this.currentRobot = `uploaded_${robotName}`;
    this.mergedScenePath = `merged_${sceneName}.xml`;

    return this.mergedScenePath;
  }

  /**
   * Ensure directory exists in virtual filesystem
   */
  _ensureDir(path) {
    if (!this.mujoco.FS.analyzePath(path).exists) {
      this.mujoco.FS.mkdir(path);
    }
  }

  /**
   * Get current scene info
   */
  getCurrentSceneInfo() {
    return {
      environment: this.currentEnv,
      robot: this.currentRobot,
      scenePath: this.mergedScenePath
    };
  }

  /**
   * Get 3DGS path for current environment
   * @returns {string|null}
   */
  getSpzPath() {
    if (!this.currentEnv) return null;
    return `./assets/environments/${this.currentEnv}/scene.spz`;
  }

  /**
   * List available environments
   */
  listEnvironments() {
    return ['tabletop'];
  }

  /**
   * List available robots
   */
  listRobots() {
    return ['xlerobot', 'SO101', 'panda', 'humanoid'];
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
