function ALL_DH = DH_ALL(t1, t2, t3, t4, t5)


% Matrix 1
matrix1 = [0, 0, 0, 0];
% Matrix for the midpoint of link 1
matrixMidpointLink1 = [
    30, 0, 28.025, t1+pi/2
];
% Matrix for Joint 2
matrixJoint2 = [
    0, pi/2, 56.05, t1;
    0, 0, 60, 0
];
% Matrix for the midpoint of link 2
matrixMidpointLink2 = [
    0, pi/2, 56.05, t1;
    161.15, 0, 103.55, t2
];
% Matrix for Joint 3
matrixJoint3 = [
    0, pi/2, 56.05, t1;
    330.3, pi, 67.025, t2
];
% Matrix for the midpoint of link 3
matrixMidpointLink3 = [
    0, pi/2, 56.05, t1;
    330.3, pi, 67.025, t2;
    127.05, 0, 36.525, t3
];
% Matrix for Joint 4
matrixJoint4 = [
    0, pi/2, 56.05, t1;
    330.3, pi, 67.025, t2;
    254.1, pi, 0, t3
];
% Matrix for the midpoint of link 4
matrixMidpointLink4 = [
    0, pi/2, 56.05, t1;
    330.3, pi, 67.025, t2;
    254.1, pi, 0, t3;
    29.525, 0, 27.2375, t4
];
% Matrix for Joint 5
matrixJoint5 = [
    0, pi/2, 56.05, t1;
    330.3, pi, 67.025, t2;
    254.1, pi, 0, t3;
    0, pi/2, 54.475, t4 + pi/2;
    0, 0, 59.05, 0
];
% Matrix for the midpoint of link 5
matrixMidpointLink5 = [
    0, pi/2, 56.05, t1;
    330.3, pi, 67.025, t2;
    254.1, pi, 0, t3;
    0, pi/2, 54.475, t4 + pi/2;
    0, 0, 136.4, t5
];

MatrixEE = [0 pi/2 56.05 t1; 330.3 pi 67.025 t2; 254.1 pi 0 t3; 0 pi/2 54.475 t4+pi/2; 0 0 213.75 t5];


ALL_DH = {matrix1; matrixMidpointLink1; matrixJoint2; matrixMidpointLink2; 
    matrixJoint3; matrixMidpointLink3; matrixJoint4; matrixMidpointLink4; matrixJoint5; matrixMidpointLink5; MatrixEE};


