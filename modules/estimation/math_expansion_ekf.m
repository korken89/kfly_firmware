clear all;
clc;
syms theta_x theta_y theta_z dt real;
syms Sp00 Sp01 Sp02 Sp03 Sp04 Sp05 Sp11 Sp12 Sp13 Sp14 Sp15 Sp22 Sp23 Sp24 Sp25 Sp33 Sp34 Sp35 Sp44 Sp45 Sp55 real;
syms R00 R01 R02 R10 R11 R12 R20 R21 R22 vx vy vz real;
syms Ss00 Ss01 Ss02 Ss11 Ss12 Ss22 real;
syms T200 T201 T202 T203 T204 T205 T210 T211 T212 T213 T214 T215 T220 T221 T222 T223 T224 T225 real
syms K00 K01 K02 K10 K11 K12 K20 K21 K22 K30 K31 K32 K40 K41 K42 K50 K51 K52 yx yy yz real
syms T100 T101 T102 T103 T104 T105 T111 T112 T113 T114 T115 T122 T123 T124 T125 T133 T134 T135 T144 T145 T155 real
syms T300 T301 T302 T310 T311 T312 T320 T321 T322 real

% How the two covariance matrices are stored in memory
% They are stored as transposes to not need to do transpose when doing the
% QR decomposition

Sp = [Sp00, Sp01, Sp02, Sp03, Sp04, Sp05
         0, Sp11, Sp12, Sp13, Sp14, Sp15;
         0,    0, Sp22, Sp23, Sp24, Sp25;
         0,    0,    0, Sp33, Sp34, Sp35;
         0,    0,    0,    0, Sp44, Sp45;
         0,    0,    0,    0,    0, Sp55];

Ss = [Ss00, Ss01, Ss02;
         0, Ss11, Ss12;
         0,    0, Ss22];
     
% Make them lower triangular as the equations require
Sp = Sp';
Ss = Ss';

% Rotation matrix
R = [R00 R01 R02;
     R10 R11 R12;
     R20 R21 R22];

% Jacobian of the state prediction function
F = [        0,  theta_z, -theta_y, -dt,   0,   0;
       -theta_z,       0,  theta_x,   0, -dt,   0;
        theta_y, -theta_x,       0,   0,   0, -dt;
             0,       0,       0,   0,   0,   0;
             0,       0,       0,   0,   0,   0;
             0,       0,       0,   0,   0,   0];
         

% The first part of the first QR Decomposition (without the lower part)
fprintf('The first part of the first QR Decomposition')
[F * Sp]'


% Jacobian of the two cases of the measurement prediction. With and without
% magnetometer measurements
H1 = [[1, 0, 0] * R', 0, 0, 0;
      [0, 1, 0] * R', 0, 0, 0;
      [0, 0, 1] * R', 0, 0, 0];

H2 = [[1, 0, 0] * R', 0, 0, 0;
      [0, 1, 0] * R', 0, 0, 0];

% The first part of the second QR Decomposition (without the lower part)
fprintf('The first part of the second QR Decomposition')
[H1 * Sp]'
 
% Calculation of the inv(Ss) * H * Sp (used in the Kalman gain and the
% Cholesky downdate
fprintf('Calculation of the inv(Ss) * H * Sp')
Ss * H1 * Sp

% The form of the previous equation, used to calculate the Kalman gain
T = [T300 T301 T302 0 0 0;
     T310 T311 T312 0 0 0;
     T320 T321 T322 0 0 0];
 
% Calculate the Kalman gain
fprintf('Calculate the Kalman gain')
Sp*T'*Ss

% Form of the Kalman gain
K = [K00 K01 K02;
     K10 K11 K12;
     K20 K21 K22;
     K30 K31 K32;
     K40 K41 K42;
     K50 K51 K52];
 
% Apply Kalman gain
fprintf('Apply Kalman gain')
y = [yx yy yz]'
K*y
 
% The form of the Cholesky downdate
W = [T100, T101, T102, T103, T104, T105
       0,  T111, T112, T113, T114, T115;
       0,     0, T122, T123, T124, T125;
       0,     0,    0, T133, T134, T135;
       0,     0,    0,    0, T144, T145;
       0,     0,    0,    0,    0, T155];

fprintf('Apply the Cholesky Downdate to the covariance matrix')
% Apply the Downdate to the covariance matrix
Sp*W'

A =[-0.0071414    0.0000004    -0.0000004    ;
-0.0000004    -0.0071414    0.0000004    ;
0.0000004    -0.0000004    -0.0071414    ;
10.0000000    0.0000000    0.0000000    ;
0.0000000    10.0000000    0.0000000    ;   
0.0000000    0.0000000    31.6227760 ]