
yalmip('clear');
clear all;

% 1. Parameters
% 1.1. Quadruped Robots
tf = 40;

I_B = [0.41   0     0;
       0      2.1   0;
       0      0     2.1];

I_B_inv = [1/0.41   0     0;
           0      1/2.1   0;
           0      0     1/2.1];
       
psi = 0;% yaw
% 1.2. size of the robot
Length = 0.52;
Width = 0.26;
Height = 0.2;
%1.3. hip of the robot
hip = [Length/2   Length/2  -Length/2  -Length/2;
       -Width/2   Width/2   -Width/2    Width/2;
       0          0          0          0 ];
Rz = [cos(psi) sin(psi) 0;
     -sin(psi) cos(psi) 0;
      0        0        1];

% 1.4. 机器人常数
m = 30;
l = 0.4;% leg length
a = 1.4;% cycle weight
g = -9.8;
mu = 0.6;% friction weight
f_min = 5;
f_max = 650;

R_1 = 1;
R_2 = 100;
N_p = 11;
% 1.5. Gait pattern
Per = [5/10 6/10 7/10 8/10];

S1 = [1 1 0 0; % phase 1
      1 1 0 0; % phase 2
      1 1 0 0; % phase 3
      1 1 0 0; % phase 4
      1 1 0 0; % phase 5
      0 0 1 1; % phase 6
      0 0 1 1; % phase 7
      0 0 1 1; % phase 8
      0 0 1 1; % phase 9
      0 0 1 1; % phase 10
      1 1 0 0; % phase 1
      1 1 0 0]; % phase 2
  
S2 = [1 1 1 1; % phase 1
      1 1 0 0; % phase 2
      1 1 0 0; % phase 3
      1 1 0 0; % phase 4
      1 1 0 0; % phase 5
      1 1 1 1; % phase 6
      0 0 1 1; % phase 7
      0 0 1 1; % phase 8
      0 0 1 1; % phase 9
      0 0 1 1; % phase 10
      1 1 1 1; % phase 1
      1 1 0 0]; % phase 2
    
S3 = [1 1 1 1; % phase 1
      1 1 1 1; % phase 2
      1 1 0 0; % phase 3
      1 1 0 0; % phase 4
      1 1 0 0; % phase 5
      1 1 1 1; % phase 6
      1 1 1 1; % phase 7
      0 0 1 1; % phase 8
      0 0 1 1; % phase 9
      0 0 1 1; % phase 10
      1 1 1 1; % phase 1
      1 1 1 1]; % phase 2
  
S4 = [1 1 1 1; % phase 1
      1 1 1 1; % phase 2
      1 1 1 1; % phase 3
      1 1 1 1; % phase 4
      1 1 1 1; % phase 5
      1 1 1 1; % phase 6
      1 1 0 0; % phase 7
      1 1 0 0; % phase 8
      0 0 1 1; % phase 9
      0 0 1 1; % phase 10
      1 1 1 1; % phase 1
      1 1 1 1]; % phase 2

% 2. Initialization

% 2.1. 参考轨迹
load Time_des.mat;
load Velocity_des.mat;
load Position_des.mat;

% 2.2. 初始机体位置和姿态
Phi(1)   = 0;       % pitch
Theta(1) = 0;       % roll
% Psi(:,1) = 0;     % yaw
P(:,1) = [0; 0; 0.3];   % 初始CoM位置
W(:,1) = [0; 0; 0];  % 初始角速度
V(:,1) = [1.65; 0; 0];  % 初始线速度

% 2.3. 计时器 
Time(1) = 1;
T = a*l/V(1,1);
Delta = T/(N_p-1);

X_S1 = [0.26  0.26  -0.26  -0.26;
       -0.13  0.13  -0.13   0.13;
        0     0     0       0];
X_S2 = [0.26  0.26  -0.26  -0.26;
       -0.13  0.13  -0.13   0.13;
        0     0     0       0];
X_S3 = [0.26  0.26  -0.26  -0.26;
       -0.13  0.13  -0.13   0.13;
        0     0     0       0];
X_S4 = [0.26  0.26  -0.26  -0.26;
       -0.13  0.13  -0.13   0.13;
        0     0     0       0];

% 2.4. Variables
phi = sdpvar(1,N_p);% pitch
theta = sdpvar(1,N_p);% roll

p = sdpvar(3,N_p);% body position
w = sdpvar(3,N_p);% body angular velocity
v = sdpvar(3,N_p);% body velocity

% footend force
f_1 = sdpvar(3,N_p-1);
f_2 = sdpvar(3,N_p-1);
f_3 = sdpvar(3,N_p-1);
f_4 = sdpvar(3,N_p-1);
% footend position
x_1 = sdpvar(3,N_p);
x_2 = sdpvar(3,N_p);
x_3 = sdpvar(3,N_p);
x_4 = sdpvar(3,N_p);

s = sdpvar(N_p,4);
dt = sdpvar(1,1);

v_ref = sdpvar(3,N_p);
p_ref = sdpvar(3,N_p);
% z_ref = 0.3;

% 3. Controller
constraints = [];
objective_1 = 0;
objective_2 = 0;
objective_3 = 0;
objective_4 = 0;

for k = 1:N_p-1
    % 3.1. Objective
    objective_1 = objective_1 + ...
        R_1*(v(:,k+1) - v_ref(:,k+1))'*(v(:,k+1) - v_ref(:,k+1))+...
        R_2*(p(:,k+1) - p_ref(:,k+1))'*(p(:,k+1) - p_ref(:,k+1));
    objective_2 = objective_2 + ...
        R_1*(v(:,k+1) - v_ref(:,k+1))'*(v(:,k+1) - v_ref(:,k+1))+...
        R_2*(p(:,k+1) - p_ref(:,k+1))'*(p(:,k+1) - p_ref(:,k+1));
    objective_3 = objective_3 + ...
        R_1*(v(:,k+1) - v_ref(:,k+1))'*(v(:,k+1) - v_ref(:,k+1))+...
        R_2*(p(:,k+1) - p_ref(:,k+1))'*(p(:,k+1) - p_ref(:,k+1));
    objective_4 = objective_4 + ...
        R_1*(v(:,k+1) - v_ref(:,k+1))'*(v(:,k+1) - v_ref(:,k+1))+...
        R_2*(p(:,k+1) - p_ref(:,k+1))'*(p(:,k+1) - p_ref(:,k+1));
    % 3.2. Transit State
    A = [zeros(3) zeros(3) Rz      zeros(3);
         zeros(3) zeros(3) zeros(3) eye(3);
         zeros(3) zeros(3) zeros(3) zeros(3);
         zeros(3) zeros(3) zeros(3) zeros(3)];

    r_1 = [0                    -(x_1(3,k)-p_ref(3,k))    (x_1(2,k)-p_ref(2,k));
          (x_1(3,k)-p_ref(3,k))   0                      -(x_1(1,k)-p_ref(1,k));
         -(x_1(2,k)-p_ref(2,k))  (x_1(1,k)-p_ref(1,k))     0              ];

    r_2 = [0                    -(x_2(3,k)-p_ref(3,k))    (x_2(2,k)-p_ref(2,k));
          (x_2(3,k)-p_ref(3,k))   0                      -(x_2(1,k)-p_ref(1,k));
         -(x_2(2,k)-p_ref(2,k))  (x_2(1,k)-p_ref(1,k))     0              ];

    r_3 = [0                    -(x_3(3,k)-p_ref(3,k))    (x_3(2,k)-p_ref(2,k));
          (x_3(3,k)-p_ref(3,k))   0                      -(x_3(1,k)-p_ref(1,k));
         -(x_3(2,k)-p_ref(2,k))  (x_3(1,k)-p_ref(1,k))     0              ];

    r_4 = [0                    -(x_4(3,k)-p_ref(3,k))    (x_4(2,k)-p_ref(2,k));
          (x_4(3,k)-p_ref(3,k))   0                      -(x_4(1,k)-p_ref(1,k));
         -(x_4(2,k)-p_ref(2,k))  (x_4(1,k)-p_ref(1,k))     0              ];


    B = [zeros(3)      zeros(3)      zeros(3)      zeros(3);
         zeros(3)      zeros(3)      zeros(3)      zeros(3);
         I_B_inv*r_1   I_B_inv*r_2   I_B_inv*r_3   I_B_inv*r_4;
         eye(3)./m     eye(3)./m     eye(3)./m     eye(3)./m ];
     
    phase = kron(diag(s(k,:)),eye(3));
    % 3.4. Dynamics
    constraints = [constraints,...
        [phi(k+1);theta(k+1);psi;p(:,k+1);w(:,k+1);v(:,k+1)]...
        == [phi(k);theta(k);psi;p(:,k);w(:,k);v(:,k)] + ...
        (A*[phi(k);theta(k);psi;p(:,k);w(:,k);v(:,k)] + ...
        B*phase*[f_1(:,k);f_2(:,k);f_3(:,k);f_4(:,k)] + [zeros(11,1);g])*dt];
    
    % 3.5. Physical Constraints
    constraints = [constraints,  s(k,1)*f_min <= f_1(3,k) <= s(k,1)*f_max];
    constraints = [constraints, -mu*f_1(3,k) <=  f_1(1,k) <= mu*f_1(3,k)];
    constraints = [constraints, -mu*f_1(3,k) <= -f_1(1,k) <= mu*f_1(3,k)];
    constraints = [constraints, -mu*f_1(3,k) <=  f_1(2,k) <= mu*f_1(3,k)];
    constraints = [constraints, -mu*f_1(3,k) <= -f_1(2,k) <= mu*f_1(3,k)];

    constraints = [constraints,  s(k,2)*f_min <= f_2(3,k) <= s(k,2)*f_max];
    constraints = [constraints, -mu*f_2(3,k) <=  f_2(1,k) <= mu*f_2(3,k)];
    constraints = [constraints, -mu*f_2(3,k) <= -f_2(1,k) <= mu*f_2(3,k)];
    constraints = [constraints, -mu*f_2(3,k) <=  f_2(2,k) <= mu*f_2(3,k)];
    constraints = [constraints, -mu*f_2(3,k) <= -f_2(2,k) <= mu*f_2(3,k)];

    constraints = [constraints,  s(k,3)*f_min <= f_3(3,k) <= s(k,3)*f_max];
    constraints = [constraints, -mu*f_3(3,k) <=  f_3(1,k) <= mu*f_3(3,k)];
    constraints = [constraints, -mu*f_3(3,k) <= -f_3(1,k) <= mu*f_3(3,k)];
    constraints = [constraints, -mu*f_3(3,k) <=  f_3(2,k) <= mu*f_3(3,k)];
    constraints = [constraints, -mu*f_3(3,k) <= -f_3(2,k) <= mu*f_3(3,k)];

    constraints = [constraints,  s(k,4)*f_min <= f_4(3,k) <= s(k,4)*f_max];
    constraints = [constraints, -mu*f_4(3,k) <=  f_4(1,k) <= mu*f_4(3,k)];
    constraints = [constraints, -mu*f_4(3,k) <= -f_4(1,k) <= mu*f_4(3,k)];
    constraints = [constraints, -mu*f_4(3,k) <=  f_4(2,k) <= mu*f_4(3,k)];
    constraints = [constraints, -mu*f_4(3,k) <= -f_4(2,k) <= mu*f_4(3,k)];
end
    constraints = [constraints,  w(3,N_p) == 0];
% 3.7. Input/Output
solutions_out_1 = {[phi(:)],[theta(:)],[p(:)],[w(:)],[v(:)],...
    [f_1(:)],[f_2(:)],[f_3(:)],[f_4(:)],objective_1};
solutions_out_2 = {[phi(:)],[theta(:)],[p(:)],[w(:)],[v(:)],...
    [f_1(:)],[f_2(:)],[f_3(:)],[f_4(:)],objective_2};
solutions_out_3 = {[phi(:)],[theta(:)],[p(:)],[w(:)],[v(:)],...
    [f_1(:)],[f_2(:)],[f_3(:)],[f_4(:)],objective_3};
solutions_out_4 = {[phi(:)],[theta(:)],[p(:)],[w(:)],[v(:)],...
    [f_1(:)],[f_2(:)],[f_3(:)],[f_4(:)],objective_4};

% ops = sdpsettings('solver','ipopt');
ops = sdpsettings('solver','ipopt','verbose',0);

ops.ipopt.max_iter = 50;
ops.ipopt.tol = 1e-3;
ops.ipopt.acceptable_tol = 1e-2;
ops.ipopt.acceptable_iter = 3;
ops.ipopt.constr_viol_tol = 1e-3;
ops.ipopt.print_level = 0;

controller_1 = optimizer(constraints,objective_1,ops,...
    {phi(1),theta(1),p(:,1),w(:,1),v(:,1),s(:),dt(:),...
    v_ref(:),p_ref(:),x_1(:),x_2(:),x_3(:),x_4(:)},solutions_out_1);
controller_2 = optimizer(constraints,objective_2,ops,...
    {phi(1),theta(1),p(:,1),w(:,1),v(:,1),s(:),dt(:),...
    v_ref(:),p_ref(:),x_1(:),x_2(:),x_3(:),x_4(:)},solutions_out_2);
controller_3 = optimizer(constraints,objective_3,ops,...
    {phi(1),theta(1),p(:,1),w(:,1),v(:,1),s(:),dt(:),...
    v_ref(:),p_ref(:),x_1(:),x_2(:),x_3(:),x_4(:)},solutions_out_3);
controller_4 = optimizer(constraints,objective_4,ops,...
    {phi(1),theta(1),p(:,1),w(:,1),v(:,1),s(:),dt(:),...
    v_ref(:),p_ref(:),x_1(:),x_2(:),x_3(:),x_4(:)},solutions_out_4);

% 4. Simulation
i = 1; 
P_ref(:,1) = Position_des(:,1);
V_ref(:,1) = Velocity_des(:,1);
h = waitbar(0,'please wait');

while Time(i) <= tf
    
    for j = 1:N_p-1
    Time(i+j) = Time(i)+j*Delta;
    P_ref(:,i+j) = interp1(Time_des,Position_des',Time(i+j))';
    V_ref(:,i+j) = interp1(Time_des,Velocity_des',Time(i+j))';
    end
    % 4.1. Phase setting
    for j = 1:N_p
    for leg = 1:4
        if S1(j,leg) == 0 && S1(j+1,leg) == 1

            X_S1(:,leg) = ...
                P_ref(:,i+j-1) + ...
                Rz*hip(:,leg) + ...
                V_ref(:,i+j-1)*T*Per(1)/2;
        end  
    end
        X_S1_1(:,j) = X_S1(:,1);
        X_S1_2(:,j) = X_S1(:,2);
        X_S1_3(:,j) = X_S1(:,3);
        X_S1_4(:,j) = X_S1(:,4);
    end
    X_S1_1(3,:) = 0;
    X_S1_2(3,:) = 0;
    X_S1_3(3,:) = 0;
    X_S1_4(3,:) = 0;
    
    for j = 1:N_p
    for leg = 1:4
        if S2(j,leg) == 0 && S2(j+1,leg) == 1

            X_S2(:,leg) = ...
                P_ref(:,i+j-1) + ...
                Rz*hip(:,leg) + ...
                V_ref(:,i+j-1)*T*Per(2)/2;
        end  
    end
        X_S2_1(:,j) = X_S2(:,1);
        X_S2_2(:,j) = X_S2(:,2);
        X_S2_3(:,j) = X_S2(:,3);
        X_S2_4(:,j) = X_S2(:,4);
    end
    X_S2_1(3,:) = 0;
    X_S2_2(3,:) = 0;
    X_S2_3(3,:) = 0;
    X_S2_4(3,:) = 0;
    
    for j = 1:N_p
    for leg = 1:4
        if S3(j,leg) == 0 && S3(j+1,leg) == 1

            X_S3(:,leg) = ...
                P_ref(:,i+j-1) + ...
                Rz*hip(:,leg) + ...
                V_ref(:,i+j-1)*T*Per(3)/2;
        end  
    end
        X_S3_1(:,j) = X_S3(:,1);
        X_S3_2(:,j) = X_S3(:,2);
        X_S3_3(:,j) = X_S3(:,3);
        X_S3_4(:,j) = X_S3(:,4);
    end
    X_S3_1(3,:) = 0;
    X_S3_2(3,:) = 0;
    X_S3_3(3,:) = 0;
    X_S3_4(3,:) = 0;
    
    for j = 1:N_p
    for leg = 1:4
        if S4(j,leg) == 0 && S4(j+1,leg) == 1

            X_S4(:,leg) = ...
                P_ref(:,i+j-1) + ...
                Rz*hip(:,leg) + ...
                V_ref(:,i+j-1)*T*Per(4)/2;
        end  
    end
        X_S4_1(:,j) = X_S4(:,1);
        X_S4_2(:,j) = X_S4(:,2);
        X_S4_3(:,j) = X_S4(:,3);
        X_S4_4(:,j) = X_S4(:,4);
    end
    X_S4_1(3,:) = 0;
    X_S4_2(3,:) = 0;
    X_S4_3(3,:) = 0;
    X_S4_4(3,:) = 0;
    
    % 4.2. Call controller
    solutions_all = cell(1,4);
    values = inf(1,4); 
    
    [solutions_all{1},diagnostics_1] = controller_1({Phi(i),Theta(i),...
        P(:,i),W(:,i),V(:,i),reshape(S1(1:N_p,:),4*N_p,1),Delta,...
        reshape(V_ref(:,i:i+N_p-1),3*N_p,1),reshape(P_ref(:,i:i+N_p-1),3*N_p,1),...
        X_S1_1(:),X_S1_2(:),X_S1_3(:),X_S1_4(:)});
   
    if diagnostics_1 == 0
        values(1) = solutions_all{1}{end};
    end
    
    [solutions_all{2},diagnostics_2] = controller_2({Phi(i),Theta(i),...
        P(:,i),W(:,i),V(:,i),reshape(S2(1:N_p,:),4*N_p,1),Delta,...
        reshape(V_ref(:,i:i+N_p-1),3*N_p,1),reshape(P_ref(:,i:i+N_p-1),3*N_p,1),...
        X_S2_1(:),X_S2_2(:),X_S2_3(:),X_S2_4(:)});
   
    if diagnostics_2 == 0
        values(2) = solutions_all{2}{end};
    end
    
    [solutions_all{3},diagnostics_3] = controller_3({Phi(i),Theta(i),...
        P(:,i),W(:,i),V(:,i),reshape(S3(1:N_p,:),4*N_p,1),Delta,...
        reshape(V_ref(:,i:i+N_p-1),3*N_p,1),reshape(P_ref(:,i:i+N_p-1),3*N_p,1),...
        X_S3_1(:),X_S3_2(:),X_S3_3(:),X_S3_4(:)});
   
    if diagnostics_3 == 0
        values(3) = solutions_all{3}{end};
    end
    
    [solutions_all{4},diagnostics_4] = controller_4({Phi(i),Theta(i),...
        P(:,i),W(:,i),V(:,i),reshape(S4(1:N_p,:),4*N_p,1),Delta,...
        reshape(V_ref(:,i:i+N_p-1),3*N_p,1),reshape(P_ref(:,i:i+N_p-1),3*N_p,1),...
        X_S4_1(:),X_S4_2(:),X_S4_3(:),X_S4_4(:)});
   
    if diagnostics_4 == 0
        values(4) = solutions_all{4}{end};
    end
    
    % 如果四个 gait 全部不可行，才停止
    if all(isinf(values))
    error('All gait candidates are infeasible');
    end
    % 在可行 gait 里面选择 objective 最小的
    [best_value,best_id] = min(values);
    solutions = solutions_all{best_id};
    
    % 4.3. Record
    Phi(i:i+N_p-1) = reshape(solutions{1},1,N_p);
    Theta(i:i+N_p-1) = reshape(solutions{2},1,N_p);
    P(:,i:i+N_p-1) = reshape(solutions{3},3,N_p);
    W(:,i:i+N_p-1) = reshape(solutions{4},3,N_p);
    V(:,i:i+N_p-1) = reshape(solutions{5},3,N_p);
    F_1(:,i:i+N_p-2) = reshape(solutions{6},3,N_p-1);
    F_2(:,i:i+N_p-2) = reshape(solutions{7},3,N_p-1);
    F_3(:,i:i+N_p-2) = reshape(solutions{8},3,N_p-1);
    F_4(:,i:i+N_p-2) = reshape(solutions{9},3,N_p-1);
    T = a*l/solutions{5}(3*N_p-2);
    Delta = T/(N_p-1);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    progress = min(Time(i)/tf,1);

    waitbar(progress,h,...
    sprintf('Simulation %.1f%% | Time %.2f / %.2f s',...
    100*progress,Time(i),tf));

    
    i = i + N_p-1;
end  
close(h);