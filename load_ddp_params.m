%       params - params used in ddp
params.horizon = 18;
params.iterations = 50;
params.dt = 1.0;

% Weight in Final State:
params.Q_f = 1 * eye(3,3);
params.Q_f(1,1) = 1;
params.Q_f(2,2) = 1;
params.Q_f(3,3) = 0;

% Weight in the Control:
params.R = 0.01;

% Learning Rate:
params.gamma = 0.05;



