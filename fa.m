

clc;
clear;
close all;

%% Problema Definition
model = CreateModel();

model.n = 3;  % number of Handle Points

CostFunction = @(x) MyCost(x, model);    % Cost Function

nVar = model.n;       % Number of Decision Variables

VarSize = [1 nVar];   % Size of Decision Variables Matrix

VarMin.x = model.xmin;           % Lower Bound of Variables
VarMax.x = model.xmax;           % Upper Bound of Variables
VarMin.y = model.ymin;           % Lower Bound of Variables
VarMax.y = model.ymax;           % Upper Bound of Variables

%% Firefly Algorithm Parameters
MaxIt = 500;         % Maximum Number of Iterations
nPop = 20;           % Number of Fireflies (Swarm Size)
gamma = 1;            % Light Absorption Coefficient
beta0 = 2;            % Attraction Coefficient Base Value
alpha = 0.2;          % Mutation Coefficient
alpha_damp = 0.98;    % Mutation Coefficient Damping Ratio
delta = 0.05;         % Uniform Mutation Range
m = 2;

%% Initialization
% Empty Firefly Structure
firefly.Position.x = [];
firefly.Position.y = [];
firefly.Cost = [];
firefly.Sol = [];
firefly.Best.Position.x = [];
firefly.Best.Position.y = [];
firefly.Best.Cost = [];
firefly.Best.Sol = [];

% Initialize Population Array
pop = repmat(firefly, nPop, 1);

% Initialize Best Solution Ever Found
BestSol.Position.x = [];
BestSol.Position.y = [];
BestSol.Cost = inf;
BestSol.Sol = [];

% Create Initial Fireflies
for i = 1:nPop
    if i > 1
        pop(i).Position = CreateRandomSolution(model);
    else
        % Straight line from source to destination
        xx = linspace(model.xs, model.xt, model.n + 2);
        yy = linspace(model.ys, model.yt, model.n + 2);
        pop(i).Position.x = xx(2:end-1);
        pop(i).Position.y = yy(2:end-1);
    end
    
    % Evaluation
    [pop(i).Cost, pop(i).Sol] = CostFunction(pop(i).Position);
    
    % Update Personal Best
    pop(i).Best.Position = pop(i).Position;
    pop(i).Best.Cost = pop(i).Cost;
    pop(i).Best.Sol = pop(i).Sol;
    
    % Update Global Best
    if pop(i).Best.Cost < BestSol.Cost
        BestSol = pop(i).Best;
    end
end

% Array to Hold Best Cost Values
BestCost = zeros(MaxIt, 1);

%% Firefly Algorithm Main Loop
for it = 1:MaxIt
    
    newpop = repmat(firefly, nPop, 1);
    
    for i = 1:nPop
        newpop(i).Cost = inf;
        
        for j = 1:nPop
            if pop(j).Cost < pop(i).Cost
                rij = sqrt(sum((pop(i).Position.x - pop(j).Position.x).^2 + (pop(i).Position.y - pop(j).Position.y).^2));
                beta = beta0 * exp(-gamma * rij^m);
                e.x = delta * unifrnd(-1, +1, VarSize);
                e.y = delta * unifrnd(-1, +1, VarSize);
                
                newsol = firefly; % Create a new solution structure
                newsol.Position.x = pop(i).Position.x + beta * rand(VarSize).*(pop(j).Position.x - pop(i).Position.x) + alpha * e.x;
                newsol.Position.y = pop(i).Position.y + beta * rand(VarSize).*(pop(j).Position.y - pop(i).Position.y) + alpha * e.y;
                
                newsol.Position.x = max(newsol.Position.x, VarMin.x);
                newsol.Position.x = min(newsol.Position.x, VarMax.x);
                
                newsol.Position.y = max(newsol.Position.y, VarMin.y);
                newsol.Position.y = min(newsol.Position.y, VarMax.y);
                
                [newsol.Cost, newsol.Sol] = CostFunction(newsol.Position);
                
                if newsol.Cost <= newpop(i).Cost
                    newpop(i) = newsol;
                    
                    % Update Personal Best
                    if newpop(i).Cost < pop(i).Best.Cost
                        pop(i).Best.Position = newpop(i).Position;
                        pop(i).Best.Cost = newpop(i).Cost;
                        pop(i).Best.Sol = newpop(i).Sol;
                    end
                    
                    % Update Global Best
                    if newpop(i).Cost < BestSol.Cost
                        BestSol = newpop(i);
                    end
                end
            end
        end
    end
    
    % Merge
    pop = [pop; newpop];  %#ok
    
    % Sort
    [~, SortOrder] = sort([pop.Cost]);
    pop = pop(SortOrder);
    
    % Truncate
    pop = pop(1:nPop);
    
    % Store Best Cost Ever Found
    BestCost(it) = BestSol.Cost;
    
    % Show Iteration Information
    disp(['Iteration ' num2str(it) ': Best Cost = ' num2str(BestCost(it))]);
    
    % Damp Mutation Coefficient
    alpha = alpha * alpha_damp;
    
    % Plot Solution
    figure(1);
    PlotSolution(BestSol.Sol, model);
    pause(0.01);
    
end

%% Results
figure;
plot(BestCost, 'LineWidth', 2);
xlabel('Iteration');
ylabel('Best Cost');
grid on;