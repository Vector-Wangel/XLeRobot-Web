/**
 * Quaternion Math Utilities
 *
 * Quaternion format: [w, x, y, z]
 */

/**
 * Quaternion multiplication
 * @param {number[]} q1 - First quaternion [w, x, y, z]
 * @param {number[]} q2 - Second quaternion [w, x, y, z]
 * @returns {number[]} - Result quaternion [w, x, y, z]
 */
export function quatMultiply(q1, q2) {
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
export function quatFromAxisAngle(axis, angle) {
  const halfAngle = angle / 2;
  const s = Math.sin(halfAngle);
  return [Math.cos(halfAngle), axis[0] * s, axis[1] * s, axis[2] * s];
}

/**
 * Normalize a quaternion
 * @param {number[]} q - Quaternion [w, x, y, z]
 * @returns {number[]} - Normalized quaternion
 */
export function quatNormalize(q) {
  const norm = Math.sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
  if (norm < 1e-10) return [1, 0, 0, 0];
  return [q[0]/norm, q[1]/norm, q[2]/norm, q[3]/norm];
}
