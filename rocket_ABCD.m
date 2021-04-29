% State Space Representation
A = [0 1 0 0    0                  0;
     0 0 0 0    Fe/m               0;
     0 0 0 1    0                  0;
     0 0 0 0    -(Fe*ph+Fs)/m      0;
     0 0 0 0    0                  1;
     0 0 0 0    0                  0];

B = [0              0        0; 
    (th+ph)/m       1/m     Fe/m;
     0              0       0; 
    (1-th*ph)/m    -th/m    -Fe*th/m;
     0              0       0;
    -ph*L/(2*J)     L/(2*J) -Fe*L/(2*J)];

Bw = [0;1;0;1;0;1]; 

% C = [1 0 1 0 1 0];
C = [1 0 0 0 0 0;
     0 0 1 0 0 0;
     0 0 0 0 1 0];

%D = 0*eye(3);
D=0;
Dwd= [1 1 1]';


% (A,B,C,D) matrices
% states=[𝑥,𝑥̇,𝑧,𝑧̇,𝜃,𝜃̇
% 𝑢=[𝐹e,𝐹s,𝜑]