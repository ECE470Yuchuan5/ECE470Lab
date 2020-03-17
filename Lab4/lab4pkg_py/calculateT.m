M = [0 -1  0 540;
     0  0 -1 192;
     1  0  0 152;
     0  0  0 1];
S = [0  0  0  0  1  0;
     0  1  1  1  0  1;
     1  0  0  0  0  0;
     0  -152  -152  -152  0  -152;
     0  0  0  0  152  0;
     0  0  244  457  110  540];
 syms theta1 theta2 theta3 theta4 theta5 theta6;
 T = [1 0 0 0;
      0 1 0 0;
      0 0 1 0;
      0 0 0 1];
 theta = [theta1,theta2,theta3,theta4,theta5,theta6];
 for i = 1:6
     SS = [0,  -S(3,i),  S(2,i),  S(4,i);
      S(3,i),  0,  -S(1,i),  S(5,i);
      -S(2,i),  S(1,i),  0,  S(6,i);
      0,  0,  0,  0];
     T = T*(expm(SS*theta(i)));
 end
 T = T*M