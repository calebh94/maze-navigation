%       params - params used in ddp
params.horizon = 20;
params.iterations = 500;
params.dt = 1.0;

% Weight in Final State:
params.Q_f = 1 * eye(3,3);
params.Q_f(1,1) = 100;
params.Q_f(2,2) = 100;
params.Q_f(3,3) = 10;

% Weight in the Control:
params.R = 0.05;

% Learning Rate:
params.gamma = 0.001;



