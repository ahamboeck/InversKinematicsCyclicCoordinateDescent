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
    let result = [];
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
        let formattedRow = row.map(element => element.toFixed(4)).join('\t');
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
    let norm = Math.sqrt(v.reduce((acc, val) => acc + val * val, 0));
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

// Function to normalize an angle to be within -180 to 180 degrees
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

    // Update individual transformation matrices
    T1 = createDHMatrix(thetas[0], d[0], a[0], alpha[0]);
    T2 = createDHMatrix(thetas[1], d[1], a[1], alpha[1]);
    T3 = createDHMatrix(thetas[2], d[2], a[2], alpha[2]);
    T4 = createDHMatrix(thetas[3], d[3], a[3], alpha[3]);
    T5 = createDHMatrix(thetas[4], d[4], a[4], alpha[4]);
    T6 = createDHMatrix(thetas[5], d[5], a[5], alpha[5]);
    T7 = createDHMatrix(thetas[6], d[6], a[6], alpha[6]);
    T8 = createDHMatrix(thetas[7], d[7], a[7], alpha[7]);
    T9 = createDHMatrix(thetas[8], d[8], a[8], alpha[8]);
    T10 = createDHMatrix(thetas[9], d[9], a[9], alpha[9]);
    T11 = createDHMatrix(thetas[10], d[10], a[10], alpha[10]);
    T12 = createDHMatrix(thetas[11], d[11], a[11], alpha[11]);
    T13 = createDHMatrix(thetas[12], d[12], a[12], alpha[12]);
    T14 = createDHMatrix(thetas[13], d[13], a[13], alpha[13]);
    T15 = createDHMatrix(thetas[14], d[14], a[14], alpha[14]);
    T16 = createDHMatrix(thetas[15], d[15], a[15], alpha[15]);

    // Update cumulative transformation matrices
    T02 = multiplyMatrices(T1, T2);
    T03 = multiplyMatrices(T02, T3);
    T04 = multiplyMatrices(T03, T4);
    T05 = multiplyMatrices(T04, T5);
    T06 = multiplyMatrices(T05, T6);
    T07 = multiplyMatrices(T06, T7);
    T08 = multiplyMatrices(T07, T8);
    T09 = multiplyMatrices(T08, T9);
    T010 = multiplyMatrices(T09, T10);
    T011 = multiplyMatrices(T010, T11);
    T012 = multiplyMatrices(T011, T12);
    T013 = multiplyMatrices(T012, T13);
    T014 = multiplyMatrices(T013, T14);
    T015 = multiplyMatrices(T014, T15);
    T016 = multiplyMatrices(T015, T16);
}

function performCCD(goalvector, maxIterations = 1000, threshold = 0.001) {
    const jointIndicesToUpdate = [15, 13, 11, 10, 8, 7, 5, 3, 1]; // Indices of thetas to be updated, in reverse order
    counter = 0;

    for (let iteration = 0; iteration < maxIterations; iteration++) {
        for (let j = 0; j < jointIndicesToUpdate.length; j++) {
            let jointIndex = jointIndicesToUpdate[j];
            updateTransformationMatrices(); // Update matrices at each iteration

            let T_end = vectorFromMatrix(T016); // End-effector's current position
            let T_joint = vectorFromMatrix(eval('T0' + (jointIndex + 1))); // Current joint's position

            let V_tcp_joint = subtractVectors(T_end, T_joint); // Vector from current joint to end-effector
            let V_goal_joint = subtractVectors(goalvector, T_joint); // Vector from current joint to goal

            if (vectorMagnitude(V_tcp_joint) < 1e-6) continue; // Skip to avoid singularity

            // Calculate the rotation required
            let rotation_axis = crossProduct(V_tcp_joint, V_goal_joint);
            let cos_theta = dotProduct(normalize(V_tcp_joint), normalize(V_goal_joint));
            let sin_theta = vectorMagnitude(rotation_axis) / (vectorMagnitude(V_tcp_joint) * vectorMagnitude(V_goal_joint));
            let rotation_angle = Math.atan2(sin_theta, cos_theta);

            // Update the joint angle
            thetas[jointIndex] += rotation_angle * 180 / Math.PI; // Convert radian to degree
            // Optionally: Clamp the joint angle within its limits
        }

        // Check if end-effector is close enough to the goal
        updateTransformationMatrices(); // Update matrices after processing all joints
        let T_end = vectorFromMatrix(T016);
        if (vectorMagnitude(subtractVectors(T_end, goalvector)) < threshold) {
            console.log("GOAL | " + " Goaldistance: " + vectorMagnitude(subtractVectors(T_end, goal)));
            console.log(T_end)
            // Define the variable names and values with offsets applied
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

            // Print the normalized variable thetas with offsets
            for (const key in variableThetas) {
                console.log(`${key}: ${variableThetas[key]} degrees`);
            }
            break; // Exit if within threshold
        }
        counter = ++counter;
        console.log("Iteration: " + counter + " Goaldistance: " + vectorMagnitude(subtractVectors(T_end, goal)));
        console.log(T_end)
    }
}

// Konstanten

const l = 1;

const theta1 = 0;
const theta3 = 0;
const theta5 = 0;
const theta7 = 0;
const theta10 = 0;
const theta13 = 0;
const theta15 = 0;

const theta2_offset = -90;
const theta8_offset = -90;
const theta9_offset = -90;
const theta16_offset = -90;

const d1 = l;
const d2 = l;
const d3 = l;
const d4 = l;
const d5 = l;
const d6 = l;
const d7 = l;
const d8 = 0;
const d9 = -l;
const d10 = l;
const d11 = l;
const d12 = l;
const d13 = l;
const d14 = l;
const d15 = l;
const d16 = l;
const d = [d1, d2, d3, d4, d5, d6, d7, d8, d9, d10, d11, d12, d13, d14, d15, d16];

const alpha1 = 0;
const alpha2 = 90;
const alpha3 = 0;
const alpha4 = -90;
const alpha5 = 0;
const alpha6 = 90;
const alpha7 = 0;
const alpha8 = 180;
const alpha9 = -90;
const alpha10 = 0;
const alpha11 = 0;
const alpha12 = 90;
const alpha13 = -90;
const alpha14 = -90;
const alpha15 = 90;
const alpha16 = 0;
const alpha = [alpha1, alpha2, alpha3, alpha4, alpha5, alpha6, alpha7, alpha8, alpha9, alpha10, alpha11, alpha12, alpha13, alpha14, alpha15, alpha16];

const a1 = 0;
const a2 = 0;
const a3 = 0;
const a4 = 0;
const a5 = 0;
const a6 = 0;
const a7 = 0;
const a8 = -3 * l;
const a9 = 0;
const a10 = 0;
const a11 = 0;
const a12 = 0;
const a13 = 0;
const a14 = 0;
const a15 = 0;
const a16 = 0;
const a = [a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16];

const goalx = 4;
const goaly = 4;
const goalz = 4;

const goal = [goalx, goaly, goalz]

// Variable

let theta2 = 0 + theta2_offset;
let theta4 = 0;
let theta6 = 0;
let theta8 = 0 + theta8_offset;
let theta9 = 0 + theta9_offset;
let theta11 = 0;
let theta12 = 0;
let theta14 = 0;
let theta16 = 0 + theta16_offset;
let thetas = [
    theta1, theta2, theta3, theta4, theta5,
    theta6, theta7, theta8, theta9, theta10,
    theta11, theta12, theta13, theta14, theta15, theta16
];


// Transformationsmatritzen
let T1, T2, T3, T4, T5, T6, T7, T8, T9, T10, T11, T12, T13, T14, T15, T16;
let T02, T03, T04, T05, T06, T07, T08, T09, T010, T011, T012, T013, T014, T015, T016;
// let T1 = createDHMatrix(theta1, d1, a1, alpha1);
// let T2 = createDHMatrix(theta2, d2, a2, alpha2);
// let T3 = createDHMatrix(theta3, d3, a3, alpha3);
// let T4 = createDHMatrix(theta4, d4, a4, alpha4);
// let T5 = createDHMatrix(theta5, d5, a5, alpha5);
// let T6 = createDHMatrix(theta6, d6, a6, alpha6);
// let T7 = createDHMatrix(theta7, d7, a7, alpha7);
// let T8 = createDHMatrix(theta8, d8, a8, alpha8);
// let T9 = createDHMatrix(theta9, d9, a9, alpha9);
// let T10 = createDHMatrix(theta10, d10, a10, alpha10);
// let T11 = createDHMatrix(theta11, d11, a11, alpha11);
// let T12 = createDHMatrix(theta12, d12, a12, alpha12);
// let T13 = createDHMatrix(theta13, d13, a13, alpha13);
// let T14 = createDHMatrix(theta14, d14, a14, alpha14);
// let T15 = createDHMatrix(theta15, d15, a15, alpha15);
// let T16 = createDHMatrix(theta16, d16, a16, alpha16);

// let T02 = multiplyMatrices(T1, T2);
// let T03 = multiplyMatrices(T02, T3);
// let T04 = multiplyMatrices(T03, T4);
// let T05 = multiplyMatrices(T04, T5);
// let T06 = multiplyMatrices(T05, T6);
// let T07 = multiplyMatrices(T06, T7);
// let T08 = multiplyMatrices(T07, T8);
// let T09 = multiplyMatrices(T08, T9);
// let T010 = multiplyMatrices(T09, T10);
// let T011 = multiplyMatrices(T010, T11);
// let T012 = multiplyMatrices(T011, T12);
// let T013 = multiplyMatrices(T012, T13);
// let T014 = multiplyMatrices(T013, T14);
// let T015 = multiplyMatrices(T014, T15);
// let T016 = multiplyMatrices(T015, T16);

// let T16t = vectorFromMatrix(T016);
// let T_vektor_var = [T2, T4, T6, T8, T9, T11, T12, T14, T16];
// let T_vektor_joint_pos = [T1, T3, T5, T7, T8, T10, T11, T13, T15];

// let V_tcp_joint = subtractVectors(T16t, vectorFromMatrix(T_vektor_joint_pos[2]));

// let V_goal_joint = subtractVectors(goalvector, vectorFromMatrix(T_vektor_joint_pos[2]));

// let rotation_axis = crossProduct(V_tcp_joint, V_goal_joint);

// let cos_theta = dotProduct(V_tcp_joint, V_goal_joint)/(dotProduct(normalize(V_tcp_joint),(normalize(V_goal_joint))));

// let sin_theta = normalize(rotation_axis)/(dotProduct(normalize(V_tcp_joint), normalize(V_goal_joint)));

// let rotation_angle = atan2(sin_theta, cos_theta)

// theta_2 = rotation_angle;

// console.log(T16t);
// console.log("Done");



// Initialization and running CCD
updateTransformationMatrices(); // Initialize transformation matrices
performCCD(goal);
