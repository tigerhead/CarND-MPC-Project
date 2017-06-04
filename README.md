# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program Term 2 project 5 MPC

---

# MPC Implementation Reflection

##Model
I chose Global Kinematic Model discussed in the course. 
1. State vector include 6 elements:
-x - vehicle position x co-ordinate
-y - vehicle position y co-ordinate
-psi - vehicle orientation
-v- speed
-cte - cross track erro
-epsi - turn rate

2. Two acutors: 
-delta - turning angle
-a - acceleration 
3. State Update equation
-x[t+1] = x[t] + v[t] * dt * cos(psi[t])
-y[t+1] = y[t] + v[t] * dt * sin(psi[t])
-pis[t+1]= psi[t] + v[t]/Lf * delta[t] * dt
-v[t+1] = psi[t] + a[t] * dt
-cte[t+1] = (f(x[t]) - y[t]) + v[t] * sin(epsi[t]) * dt
-espi[t+1] = psi[t] - psides[t] + v[t]/Lf * delta[t]*dt

*Lf - length from front to CoG 
*f is a polynomial function of x, which is the reference way point for vehicle In this project the polynomial order is 3





