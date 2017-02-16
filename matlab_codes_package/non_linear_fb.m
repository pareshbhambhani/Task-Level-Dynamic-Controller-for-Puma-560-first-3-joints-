%non linear feedback
%tau = D(q)*J^(-1)*u - D(q)*J^(-1)J'q' + C(q,q')q' + G(q)

function tau = non_linear_fb(u,q,q_dot)

q1 = q(1);
q2 = q(2);
q3 = q(3);
q1_dot = q_dot(1);
q2_dot = q_dot(2);
q3_dot = q_dot(3);


%J
J11 = 0.02032*cos(q2 + q3)*sin(q1) - 0.4318*cos(q2)*sin(q1) - 0.14909*cos(q1) - 0.43307*sin(q2 + q3)*sin(q1);
J12 = 0.43307*cos(q2 + q3)*cos(q1) - 0.4318*cos(q1)*sin(q2) + 0.02032*sin(q2 + q3)*cos(q1);
J13 = 0.43307*cos(q2 + q3)*cos(q1) + 0.02032*sin(q2 + q3)*cos(q1);
J21 = 0.4318*cos(q1)*cos(q2) - 0.14909*sin(q1) - 0.02032*cos(q2 + q3)*cos(q1) + 0.43307*sin(q2 + q3)*cos(q1);
J22 = 0.43307*cos(q2 + q3)*sin(q1) - 0.4318*sin(q1)*sin(q2) + 0.02032*sin(q2 + q3)*sin(q1);
J23 = 0.43307*cos(q2 + q3)*sin(q1) + 0.02032*sin(q2 + q3)*sin(q1);
J31 = 0;
J32 = 0.02032*cos(q2 + q3) - 0.43307*sin(q2 + q3) - 0.4318*cos(q2);
J33 = 0.02032*cos(q2 + q3) - 0.43307*sin(q2 + q3);
J = [J11 J12 J13; J21 J22 J23; J31 J32 J33];

%J' = d/dt(J)
J_dot_11 = 0.02032*cos(q2 + q3)*cos(q1)*q1_dot - 0.02032*sin(q2 + q3)*sin(q1)*(q2_dot+q3_dot) + 0.4318*sin(q2)*sin(q1)*q2_dot - 0.4318*cos(q2)*cos(q1)*q1_dot + 0.14909*sin(q1)*q1_dot - 0.43307*cos(q2 + q3)*sin(q1)*(q2_dot+q3_dot) - 0.43307*sin(q2 + q3)*cos(q1)*q1_dot;
J_dot_12 = -0.43307*sin(q2 + q3)*cos(q1)*(q2_dot + q3_dot) - 0.43307*cos(q2 + q3)*sin(q1)*q1_dot + 0.4318*sin(q1)*sin(q2)*q1_dot - 0.4318*cos(q1)*cos(q2)*q2_dot + 0.02032*cos(q2 + q3)*cos(q1)*(q2_dot + q3_dot) - 0.02032*sin(q2 + q3)*sin(q1)*q1_dot;
J_dot_13 = -0.43307*sin(q2 + q3)*cos(q1)*(q2_dot + q3_dot) - 0.43307*cos(q2 + q3)*sin(q1)*q1_dot  + 0.02032*cos(q2 + q3)*cos(q1)*(q2_dot + q3_dot) - 0.02032*sin(q2 + q3)*sin(q1)*q1_dot;
J_dot_21 = -0.4318*sin(q1)*cos(q2)*q1_dot - 0.4318*cos(q1)*sin(q2)*q2_dot - 0.14909*cos(q1)*q1_dot + 0.02032*sin(q2 + q3)*cos(q1)*(q2_dot + q3_dot) + 0.02032*cos(q2 + q3)*sin(q1)*q1_dot  + 0.43307*cos(q2 + q3)*cos(q1)*(q2_dot + q3_dot)  - 0.43307*sin(q2 + q3)*sin(q1)*q1_dot;
J_dot_22 = -0.43307*sin(q2 + q3)*sin(q1)*(q2_dot + q3_dot) + 0.43307*cos(q2 + q3)*cos(q1)*q1_dot  - 0.4318*cos(q1)*sin(q2)*q1_dot  - 0.4318*sin(q1)*cos(q2)*q2_dot + 0.02032*cos(q2 + q3)*sin(q1)*(q2_dot + q3_dot) + 0.02032*sin(q2 + q3)*cos(q1)*q1_dot;
J_dot_23 = -0.43307*sin(q2 + q3)*sin(q1)*(q2_dot + q3_dot) + 0.43307*cos(q2 + q3)*cos(q1)*q1_dot + 0.02032*cos(q2 + q3)*sin(q1)*(q2_dot + q3_dot) + 0.02032*sin(q2 + q3)*cos(q1)*q1_dot;
J_dot_31 = 0;
J_dot_32 =  -0.02032*sin(q2 + q3)*(q2_dot + q3_dot) - 0.43307*cos(q2 + q3)*(q2_dot + q3_dot) + 0.4318*sin(q2)*q2_dot;
J_dot_33 = -0.02032*sin(q2 + q3)*(q2_dot + q3_dot) - 0.43307*cos(q2 + q3)*(q2_dot + q3_dot);

J_dot = [J_dot_11 J_dot_12 J_dot_13; J_dot_21 J_dot_22 J_dot_23; J_dot_31 J_dot_32 J_dot_33];


%D(q) matrix
d11 = 2.4574 + 1.7181*cos(q2)*cos(q2) + 0.4430*sin(q2+q3)*sin(q2+q3) - 0.0324*cos(q2)*cos(q2+q3) - 0.0415*cos(q2+q3)*sin(q2+q3) + 0.9378*cos(q2)*sin(q2+q3);
d12 = 2.2312*sin(q2) - 0.0068*sin(q2+q3) - 0.1634*cos(q2+q3);
d13 = -0.0068*sin(q2+q3) - 0.1634*cos(q2+q3);
d21 = d12;
d22 = 5.1285 + 0.9378*sin(q3) - 0.0324*cos(q3);
d23 = 0.4424 + 0.4689*sin(q3) - 0.0162*cos(q3);
d31 = d13;
d32 = d23;
d33 = 1.0236;
D = [d11 d12 d13; d21 d22 d23; d31 d32 d33];

%C(q,q') Matrix
c111 = 0;
c121 = 0.0207 - 1.2752*cos(q2)*sin(q2) + 0.4429*cos(q3)*sin(q3) - 0.8859*sin(q2)*sin(q3)*sin(q2+q3) + 0.0325*cos(q2)*sin(q2+q3) + 0.4689*cos(q2)*cos(q2+q3) - 0.4689*sin(q2)*sin(q2+q3) - 0.0461*cos(q2+q2) - 0.0415*cos(q2+q3)*cos(q2+q3) - 0.0163*sin(q3);
c131 = 0.0207 + 0.4429*cos(q2)*sin(q2) + 0.4429*cos(q3)*sin(q3) - 0.8859*sin(q2)*sin(q3)*sin(q2+q3) + 0.0163*cos(q2)*sin(q2+q3) + 0.4689*cos(q2)*cos(q2+q3) - 0.0415*cos(q2+q3)*cos(q2+q3);
c211 = c121;
c221 = 1.8181*cos(q2) + 0.1634*sin(q2+q3) - 0.0068*cos(q2+q3);
c231 = 0.1634*sin(q2+q3) - 0.0068*cos(q2+q3);
c311 = c131;
c321 = c231;
c331 = 0.1634*sin(q2+q3) - 0.0068*cos(q2+q3);
c112 = - c121;
c122 = 0;
c132 = 0;
c212 = c122;
c222 = 0;
c232 = 0.4689*cos(q3) + 0.0162*sin(q3);
c312 = 0;
c322 = c232;
c332 = 0.4689*cos(q3) + 0.0162*sin(q3);
c113 = - c131;
c123 = - c132;
c133 = 0;
c213 = c123;
c223 = - c232;
c233 = 0;
c313 = c133;
c323 = c233;
c333 = 0;

c11 = c111*q1_dot + c211*q2_dot + c311*q3_dot;
c12 = c121*q1_dot + c221*q2_dot + c321*q3_dot;
c13 = c131*q1_dot + c231*q2_dot + c331*q3_dot;
c21 = c112*q1_dot + c212*q2_dot + c312*q3_dot;
c22 = c122*q1_dot + c222*q2_dot + c322*q3_dot;
c23 = c132*q1_dot + c232*q2_dot + c332*q3_dot;
c31 = c113*q1_dot + c213*q2_dot + c313*q3_dot;
c32 = c123*q1_dot + c223*q2_dot + c323*q3_dot;
c33 = c133*q1_dot + c233*q2_dot + c333*q3_dot;

C = [c11 c12 c13; c21 c22 c23; c31 c32 c33];

%G matrix
g1 = 0;
g2 = - 48.5564*cos(q2) + 1.0462*sin(q2) + 0.3683*cos(q2+q3) - 10.6528*sin(q2+q3);
g3 = 0.3683*cos(q2+q3) - 10.6528*sin(q2+q3);

G = [g1 ; g2 ; g3];

tau = D*(inv(J))*u - D*(inv(J))*J_dot*q_dot + C*q_dot + G;