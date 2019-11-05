%       params - params used in ddp
params.horizon = 30;
params.iterations = 1000;
params.dt = 1.0;

% Weight in Final State:
params.Q_f = 1 * eye(3,3);
params.Q_f(1,1) = 100;
params.Q_f(2,2) = 100;
params.Q_f(3,3) = 10;

% Weight in the Control:
params.R = 10;

% Learning Rate:
params.gamma = 0.005;



