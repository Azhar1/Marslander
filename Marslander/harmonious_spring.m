% Simulation of harmonious spring

%time variables
starttime = 0; 		%start at time 0
endtime = 100		%we don't want to wait forever. 1 = 1s
dt = 0.01			%has to be found empirically. We want it to be as big as possible while still producing stable results to enhance performance 1 = 1s
t = starttime;		%for first iteration
iterations = (endtime-starttime)/dt

%physical variables
mass = 20						%kg
x = 0							%meter. We assume the spring to be in stable position. Motion will be generated by applying a v0
v = 0.5							%meter/second. 
k = 1							%kg/s^2
a = 0
x_verlet = zeros(iterations,1);	%initialize vectors to store the values of v and x 
v_verlet = zeros(iterations,1);	%for later plotting
t_verlet = zeros(iterations,1);
x_euler = zeros(iterations,1);	%both for Euler and Verlet Integrations
v_euler = zeros(iterations,1);
t_euler = zeros(iterations,1);


%Euler Integration
for i= 1:iterations
	%update trajectories
	x_euler(i)=x;
	v_euler(i)=v;
	t_euler(i)=t;
	%update symbols
	a=-k*x/mass;
	x=x+dt*v;
	v=v+dt*a;
	t=t+dt;
end

%plot Euler
%plot(t_euler,[x_euler,v_euler],'-');

%reset variables%physical variables
mass = 20						%kg
x = 0;							%meter. We assume the spring to be in stable position. Motion will be generated by applying a v0
v = 0.5;						%meter/second. 
k = 1;							%kg/s^2
a = 0;

%Verlet Integration
%Verlet Integration relies on two value to create the next value
%We only know the initial values, so we need to calculate a second x,v value pair
%We can do this using the Taylor polynom of the second order, leaving us with only 
%a third-order error that only appears in the first timestep. Because we simulate many timesteps, this
%is neglectable
x_verlet(1)= x;
v_verlet(1)= v;
t_verlet(1)= 0;

a=-k*x/mass;
x_verlet(2)= x_verlet(1)+v_verlet(1)*dt+0.5*a*dt^2;
t_verlet(2)= dt;

%We use the mean value between two position points over time to get the speed. For accuracy, we use 
%the previous and next value. This means that we have to calculate the next x value first. Thus the
%speed calculation lags one td step behind.

for i= 3:iterations
	%calculate current acceleration
	a=-k*x_verlet(i-1)/mass;
	%calculate new x
	x_verlet(i)= 2*x_verlet(i-1)-x_verlet(i-2)+(dt^2)*a;
	%calculate previous v
	v_verlet(i-1)= (x_verlet(i)-x_verlet(i-2))/(2*dt);
	t_verlet(i)=t_verlet(i-1)+dt;
end

%Now calculate the last v point - we can of course only use the current value, the last point is thus slightly less accurate
v_verlet(iterations)=(x_verlet(iterations)-x_verlet(iterations-1))/(dt);

%plot Verlet
plot(t_verlet,[x_verlet,v_verlet, x_euler, v_euler],'-');


