# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program Term 2 project 5 MPC

---

# MPC Implementation Reflection

## Model
I chose Global Kinematic Model discussed in the course. 
1. State vector include 6 elements:
- x - vehicle position x co-ordinate
- y - vehicle position y co-ordinate
- psi - vehicle orientation
- v- speed
- cte - cross track error
- epsi - orientation error

2. Two acutors: 
- delta - turning angle
- a - acceleration 

3. State Update equation
- x[t+1] = x[t] + v[t] * dt * cos(psi[t])
- y[t+1] = y[t] + v[t] * dt * sin(psi[t])
- pis[t+1]= psi[t] + v[t]/Lf * delta[t] * dt
- v[t+1] = psi[t] + a[t] * dt
- cte[t+1] = (f(x[t]) - y[t]) + v[t] * sin(epsi[t]) * dt
- espi[t+1] = psi[t] - atan(f'(x[t])) + v[t]/Lf * delta[t]*dt

Lf is length from front to CoG and f is a polynomial function of x, which is the reference way point for vehicle In this project the polynomial order is 3: f(x) = coeffs[0] + coeffs[1] * x + coeffs[2] * pow(x, 2) + coeffs[3] * pow(x, 3). f'is the derivative of polynomial function: f'(x) = coeffs[1] + 2 * coeffs[2] * x + 3 * coeffs[3] * pow(x, 3). coeffs is the vector of polynomial coefficients.

## N (timestep length) and dt (elapsed duration between timesteps) 

- N is the number of timesteps. 
- dt is how much time elapses between actuations. 
- T =N * dt is the duration over which future predictions are made. 

For driving, T should be a few seconds, othereise the environment will change enough that it won't make sense to predict any further into the future.  The goal of Model Predictive Control is to optimize two accuators. Larger N means more variables and more complex cost function.  With larger N, optimization model may be more accurate result but calculation cost will be higher and it takes longer time to return solution result. So there should be a balance between accurancy and calculation cost.  MPC attempts to approximate a continues reference trajectory by means of discrete paths between actuations. Larger values of dt result in less frequent actuations, which makes it harder to accurately approximate a continuous reference trajectory.

In this project, I chose the T to be less or equal to 1 second. I started dt as 0.05 and N = 20 which makes T = 1 second. It actually gave a pretty good result. Cost reduce to blow 10 in about 40 cycles and stayed steady, so the car drove smoothly. I tried follwing combination:
| N       | dt     Cost converge | Drive smoothness | 
| -------:|: -----:|:-------------:|--------:|
| 20      | .05  | converge at value around 10 | smooth | 
| 40      | 0.05 | Not converge very well | eradicate | 
| 10      | 0.05 |  converge at value around 40  | Ok |






