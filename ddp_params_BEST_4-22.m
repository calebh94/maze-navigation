%       params - params used in ddp
params.horizon = 14;
params.iterations = 100;
params.dt = 1.0;

% Weight in Final State:
params.Q_f = 1 * eye(3,3);
params.Q_f(1,1) = 10;
params.Q_f(2,2) = 10;
params.Q_f(3,3) = 1;

% Weight in the Control:
params.R = 0.1;

% Learning Rate:
params.gamma = 0.01;



