M = [0 -1  0 0.39;
     0  0 -1 0.401;
     1  0  0 0.2155;
     0  0  0 1];
S = [0  0  0  0  1  0;
     0  1  1  1  0  1;
     1  0  0  0  0  0;
     0.15  -0.162  -0.162  -0.162  0  -0.162;
     0.15  0  0  0  0.162  0;
     0  0.15  0.094  0.307  -0.26  0.39];
 syms theta1 theta2 theta3 theta4 theta5 theta6;
 T = [1 0 0 0;
      0 1 0 0;
      0 0 1 0;
      0 0 0 1];
 theta = [theta1 theta2 theta3 theta4 theta5 theta6];
 for i = 1:6
     SS = [0,  -S(3,i),  S(2,i),  S(4,i);
      S(3,i),  0,  -S(1,i),  S(5,i);
      -S(2,i),  S(1,i),  0,  S(6,i);
      0,  0,  0,  0];
     T = T*(expm(SS*theta(i)));
 end
 T = T*M
 d = T(1:3,4)