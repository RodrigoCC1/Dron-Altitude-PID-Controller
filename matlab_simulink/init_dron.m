% init_dron.m

% parametros fisicos
m = 1.5;
g = 9.81;
u_max = 35;
u_min = 0;

% parametros de diseño 
zeta = 1.2;
wn = 4.0;

% calculo de sintonizacion de kd y kp
kp = m * wn^2;
kd = m * 2 * zeta * wn;

% ganancia ki (aprox. 10% de su limite->192)
ki = 2;