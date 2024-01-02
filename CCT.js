function createDHMatrix(theta, d, a, alpha) {
    // Convert angles from degrees to radians
    theta = theta * Math.PI / 180;
    alpha = alpha * Math.PI / 180;

    let cosTheta = Math.cos(theta);
    let sinTheta = Math.sin(theta);
    let cosAlpha = Math.cos(alpha);
    let sinAlpha = Math.sin(alpha);

    // Creating the transformation matrix
    let matrix = [
        [cosTheta, -sinTheta * cosAlpha, sinTheta * sinAlpha, a * cosTheta],
        [sinTheta, cosTheta * cosAlpha, -cosTheta * sinAlpha, a * sinTheta],
        [0, sinAlpha, cosAlpha, d],
        [0, 0, 0, 1]
    ];

    return matrix;
}

function multiplyMatrices(m1, m2) {
    const result = [];
    for (let i = 0; i < m1.length; i++) {
        result[i] = [];
        for (let j = 0; j < m2[0].length; j++) {
            let sum = 0;
            for (let k = 0; k < m1[0].length; k++) {
                sum += m1[i][k] * m2[k][j];
            }
            result[i][j] = sum;
        }
    }
    return result;
}

function printMatrix(matrix) {
    matrix.forEach(row => {
        const formattedRow = row.map(element => element.toFixed(4)).join('\t');
        console.log(formattedRow);
    });
}

function subtractVectors(v1, v2) {
    return v1.map((k, i) => k - v2[i]);
}

function crossProduct(v1, v2) {
    return [
        v1[1] * v2[2] - v1[2] * v2[1],
        v1[2] * v2[0] - v1[0] * v2[2],
        v1[0] * v2[1] - v1[1] * v2[0]
    ];
}

function dotProduct(v1, v2) {
    return v1.reduce((acc, k, i) => acc + k * v2[i], 0);
}

function normalize(v) {
    const norm = Math.sqrt(v.reduce((acc, val) => acc + val * val, 0));
    return v.map(k => k / norm);
}

function computeAngle(a, b) {
    return Math.atan2(a, b);
}

function vectorFromMatrix(m) {
    // Assuming m is a 4x4 matrix, return the translation vector
    return [m[0][3], m[1][3], m[2][3]];
}

function vectorMagnitude(v) {
    return Math.sqrt(v.reduce((sum, component) => sum + component * component, 0));
}

function normalizeAngle(angle) {
    while (angle <= -180) {
        angle += 360;
    }
    while (angle > 180) {
        angle -= 360;
    }
    return angle;
}

function updateTransformationMatrices() {
    const T = [];
    const T_cumulative = [];
    for (let i = 0; i < thetas.length; i++) {
        const T_i = createDHMatrix(thetas[i], d[i], a[i], alpha[i]);
        T[i] = T_i;
        if (i === 0) {
            T_cumulative[i] = T_i;
        } else {
            T_cumulative[i] = multiplyMatrices(T_cumulative[i - 1], T_i);
        }
    }
    return { T, T_cumulative };
}

function performCCD(goalvector, maxIterations = 1000, threshold = 0.001) {
    const jointIndicesToUpdate = [15, 13, 11, 10, 8, 7, 5, 3, 1]; // Indices of thetas to be updated, in reverse order
    let counter = 0;

    for (let iteration = 0; iteration < maxIterations; iteration++) {
        for (let j = 0; j < jointIndicesToUpdate.length; j++) {
            const jointIndex = jointIndicesToUpdate[j];
            const { T, T_cumulative } = updateTransformationMatrices(); // Update matrices at each iteration

            const T_end = vectorFromMatrix(T_cumulative[T_cumulative.length - 1]); // End-effector's current position
            const T_joint = vectorFromMatrix(T[jointIndex]); // Current joint's position

            const V_tcp_joint = subtractVectors(T_end, T_joint); // Vector from current joint to end-effector
            const V_goal_joint = subtractVectors(goalvector, T_joint); // Vector from current joint to goal

            if (vectorMagnitude(V_tcp_joint) < 1e-6) continue; // Skip to avoid singularity

            // Calculate the rotation required
            const rotation_axis = crossProduct(V_tcp_joint, V_goal_joint);
            const cos_theta = dotProduct(normalize(V_tcp_joint), normalize(V_goal_joint));
            const sin_theta = vectorMagnitude(rotation_axis) / (vectorMagnitude(V_tcp_joint) * vectorMagnitude(V_goal_joint));
            const rotation_angle = Math.atan2(sin_theta, cos_theta);

            // Update the joint angle
            thetas[jointIndex] += rotation_angle * 180 / Math.PI; // Convert radian to degree
            // Optionally: Clamp the joint angle within its limits
        }

        // Check if end-effector is close enough to the goal
        const { T_cumulative } = updateTransformationMatrices(); // Update matrices after processing all joints
        const T_end = vectorFromMatrix(T_cumulative[T_cumulative.length - 1]);
        if (vectorMagnitude(subtractVectors(T_end, goalvector)) < threshold) {
            console.log("GOAL | " + " Goaldistance: " + vectorMagnitude(subtractVectors(T_end, goalvector)));
            console.log(T_end);
            const variableThetas = {
                "Theta 2": (normalizeAngle(thetas[1]) - theta2_offset).toFixed(2),
                "Theta 4": normalizeAngle(thetas[3]).toFixed(2),
                "Theta 6": normalizeAngle(thetas[5]).toFixed(2),
                "Theta 8": (normalizeAngle(thetas[7]) - theta8_offset).toFixed(2),
                "Theta 9": (normalizeAngle(thetas[8]) - theta9_offset).toFixed(2),
                "Theta 11": normalizeAngle(thetas[10]).toFixed(2),
                "Theta 12": normalizeAngle(thetas[11]).toFixed(2),
                "Theta 14": normalizeAngle(thetas[13]).toFixed(2),
                "Theta 16": (normalizeAngle(thetas[15]) - theta16_offset).toFixed(2),
            };

            for (const key in variableThetas) {
                console.log(`${key}: ${variableThetas[key]} degrees`);
            }
            break; // Exit if within threshold
        }
        counter++;
        console.log("Iteration: " + counter + " Goaldistance: " + vectorMagnitude(subtractVectors(T_end, goalvector)));
        console.log(T_end);
    }
}

// Constants
const l = 1;
const theta2_offset = -90;
const theta8_offset = -90;
const theta9_offset = -90;
const theta16_offset = -90;

const d = [l, l, l, l, l, l, l, 0, -l, l, l, l, l, l, l, l];
const alpha = [0, 90, 0, -90, 0, 90, 0, 180, -90, 0, 0, 90, -90, -90, 90, 0];
const a = [0, 0, 0, 0, 0, 0, 0, -3 * l, 0, 0, 0, 0, 0, 0, 0, 0];
const goal = [6, 5, 7];

// Variables
const thetas = [0, theta2_offset, 0, 0, 0, 0, 0, theta8_offset, theta9_offset, 0, 0, 0, 0, 0, 0, theta16_offset];

// Initialization and running CCD
performCCD(goal);