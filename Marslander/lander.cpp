// Mars lander simulator
// Version 1.7
// Mechanical simulation functions
// Gabor Csanyi and Andrew Gee, March 2013

// Permission is hereby granted, free of charge, to any person obtaining
// a copy of this software and associated documentation, to make use of it
// for non-commercial purposes, provided that (a) its original authorship
// is acknowledged and (b) no modified versions of the source code are
// published. Restriction (b) is designed to protect the integrity of the
// exercise for future generations of students. 

#include "lander.h"

double const PI = 3.14159265;

//autopilot variables
enum phases { LEAVE_ORBIT,WAIT_300000,SLOW_60pph_800ms, Reach_7000_400, FINAL_DESCENT, TOUCHDOWN };
phases phase = LEAVE_ORBIT;
double rate_of_descent_change = 1;
double target_rate_of_desent = -0.5;
double climb_rate = 1;
double altitude_measurement=-1;
bool initialized_final_descent = false;
bool initialized_mid_descent = false;

void initialize_variables(void){
	phase = LEAVE_ORBIT;
	rate_of_descent_change = 1;
	target_rate_of_desent = 0.8;
	climb_rate = 1;
	altitude_measurement = -1;
	initialized_final_descent = false;
	initialized_mid_descent = false;
}

void autopilot(bool launch)
// Autopilot to adjust the engine throttle, parachute and attitude control
{
	// INSERT YOUR CODE HERE
	// Assignment 6 - Autopilot only vertical landing from 10 km
	// Adapted approach for fuelsaving: Use thruster to came down to 300 m/s at 6500m altitude
	// Then deploy parachute and use thruster to reach 100 m/s at 2000m altitude
	// Then use thruster to reach 0.5m/s at 0m altitude

	//linear decrease:
	//-0.5+h*dr
	//dr=-0.5/h
	//error=-0.5/h-d

	double old = altitude_measurement;
	if (old <0)
		old = position.abs() - MARS_RADIUS;
	altitude_measurement = position.abs() - MARS_RADIUS;
	climb_rate = (altitude_measurement - old)/delta_t;



	if (!launch){
		//deploy parachute under 7k and 400 m\s
		if (safe_to_deploy_parachute() && altitude_measurement<50000 && climb_rate<0){
			if (parachute_status == NOT_DEPLOYED)
			{
				parachute_status = DEPLOYED;
				phase = FINAL_DESCENT;
				//rate_of_descent_change = 1;
				throttle = 0;
			}
		}
		switch (phase)
		{
		case LEAVE_ORBIT:
		{
							if (fuel < 0.85 || get_ground_speed() < 800){
								phase = WAIT_300000;
								throttle = 0;
								stabilized_attitude_angle = 0;
								break;
							}
							stabilized_attitude_angle = 90;
							throttle = 1;
		}

		case WAIT_300000:
		{
							if (altitude_measurement < 300000)
								phase = SLOW_60pph_800ms;
							break;
		}

		case SLOW_60pph_800ms:
		{
								 if (fuel < 0.4 || get_ground_speed() < 800){
									 phase = Reach_7000_400;
									 throttle = 0;
									 stabilized_attitude_angle = 0;
									 break;
								 }
								 stabilized_attitude_angle = 90;
								 throttle = 1;
		}

		case Reach_7000_400:
		{
							   if (initialized_mid_descent == false && climb_rate < -250 && altitude_measurement < 260000){
								   rate_of_descent_change = (climb_rate + 200) / (altitude_measurement - 8000);
								   initialized_mid_descent = true;
							   }
							   if (!initialized_mid_descent)
								   break;
							   if (fuel < 0.15){
								   throttle = 0;
								   break;
							   }
							   double current_target_rate = rate_of_descent_change*(altitude_measurement - 8000) - 200;
							   double error = climb_rate - current_target_rate;
							   //negative error: use thruster
							   if (error < 0){
								   //100 m\s = full thrust
								   //gain: -0.1 below 40, -100 above 4000,
								   double gain = -100;
								   double temp_throttle = error / gain;
								   if (temp_throttle>1)
									   temp_throttle = 1;
								   throttle = temp_throttle;
							   }
							   else{
								   throttle = 0;
							   }
		}
			break;
		case FINAL_DESCENT:
		{
							  if (climb_rate > 0)
								  climb_rate = 0;
							  if (initialized_final_descent == false && climb_rate < 0 && altitude_measurement < 400){
								  double total_speed = sqrt(pow(climb_rate, 2.0) + pow(get_ground_speed(), 2.0));
								  rate_of_descent_change = (total_speed - target_rate_of_desent) / altitude_measurement;
								  initialized_final_descent = true;
							  }
							  if (!initialized_final_descent)
								  break;
							  double current_target_rate = rate_of_descent_change*altitude_measurement - 0.1;
							  double current_speed = sqrt(pow(climb_rate, 2.0) + pow(get_ground_speed(), 2.0));
							  double error = (current_speed - current_target_rate)*-1;

							  //angle
							  double ground_speed = get_ground_speed();
							  if (ground_speed < 1){
								  stabilized_attitude_angle = 0;
								  initialized_final_descent = false;
								  phase = TOUCHDOWN;
							  }
							  else
							  {
								  if (climb_rate > -1 && ground_speed > 1)
									  stabilized_attitude_angle = 90;
								  else
									  stabilized_attitude_angle = (int)(90.0 / (1 + ((-1 * climb_rate) / ground_speed)));
							  }
							  //negative error: use thruster
							  if (error < 0){
								  //100 m\s = full thrust
								  //gain: -0.1 below 40, -100 above 4000,
								  double gain = -0.025*altitude_measurement;
								  if (gain<-100)
									  gain = -100;
								  if (gain>-1)
									  gain = -1;
								  double temp_throttle = error / gain;
								  if (temp_throttle>1)
									  temp_throttle = 1;
								  throttle = temp_throttle;
							  }
							  else{
								  throttle = 0;
							  }
		}
			break;
		case TOUCHDOWN:
		{
						  if (initialized_final_descent == false && climb_rate < 0 && altitude_measurement < 400){
							  rate_of_descent_change = (climb_rate - target_rate_of_desent) / altitude_measurement;
							  initialized_final_descent = true;
						  }
						  if (!initialized_final_descent)
							  break;
						  double current_target_rate = rate_of_descent_change*altitude_measurement - 0.1;
						  double error = (climb_rate - current_target_rate);

						  //negative error: use thruster
						  if (error < 0){
							  //100 m\s = full thrust
							  //gain: -0.1 below 40, -100 above 4000,
							  double gain = -0.025*altitude_measurement;
							  if (gain<-100)
								  gain = -100;
							  if (gain>-1)
								  gain = -1;
							  double temp_throttle = error / gain;
							  if (temp_throttle>1)
								  temp_throttle = 1;
							  throttle = temp_throttle;
						  }
						  else{
							  throttle = 0;
						  }
		}
		default:
			break;
		}
	}
	else{

	}
}

void numerical_dynamics (void)
  // This is the function that performs the numerical integration to update the
  // lander's pose. The time step is delta_t (global variable).
{
  // INSERT YOUR CODE HERE
	double lander_mass = fuel*FUEL_CAPACITY*FUEL_DENSITY + UNLOADED_LANDER_MASS;
	
	//get the force acting on the lander through gravity
	vector3d gravitational_acceleration = (-GRAVITY*MARS_MASS / (position.abs2()))*position.norm();
	vector3d gravitational_force = gravitational_acceleration*lander_mass;

	//get the force acting on the lander through atmospheric drag
	vector3d atmospheric_drag_force = ((-0.5)*DRAG_COEF_LANDER*((pow(LANDER_SIZE, 2)*3.16) / 2)*atmospheric_density(position)*velocity.abs2())*velocity.norm();
	vector3d atmospheric_drag_force_parachute = ((-0.5)*DRAG_COEF_CHUTE*((pow(LANDER_SIZE, 2)*3.16) / 2)*atmospheric_density(position)*velocity.abs2())*velocity.norm();
	if 
	(
		parachute_status == DEPLOYED && 
		(
		atmospheric_drag_force_parachute.abs() > MAX_PARACHUTE_DRAG ||
		velocity.abs()>MAX_PARACHUTE_SPEED
		)
	)
		parachute_status = LOST;
	

	//get the force acting on the lander through thrust
	vector3d thruster_force = (throttle*MAX_THRUST)*thrust_wrt_world().norm();

	//generate wind force

	//sum up resulting forces
	vector3d resulting_force = atmospheric_drag_force + gravitational_force;
	if (fuel > 0)
		resulting_force += thruster_force;
	if (parachute_status == DEPLOYED)
		resulting_force += atmospheric_drag_force_parachute;

	//sum up resulting acceleration
	vector3d resulting_acceleration = resulting_force / lander_mass;

	//update position through verlet
	position += delta_t*(velocity + (delta_t*resulting_acceleration*0.5));

	//recalculate acceleration
	gravitational_acceleration = (-GRAVITY*MARS_MASS / (position.abs2()))*position.norm();
	gravitational_force = gravitational_acceleration*lander_mass;
	atmospheric_drag_force = ((-0.5)*DRAG_COEF_LANDER*((pow(LANDER_SIZE, 2)*PI) / 2)*atmospheric_density(position)*velocity.abs2())*velocity.norm();
	atmospheric_drag_force_parachute = -0.5*DRAG_COEF_CHUTE*atmospheric_density(position)*5.0*2.0*LANDER_SIZE*2.0*LANDER_SIZE*velocity.abs2()*velocity.norm();
	if
	(
		parachute_status == DEPLOYED &&
		(
			atmospheric_drag_force_parachute.abs() > MAX_PARACHUTE_DRAG ||
			velocity.abs()>MAX_PARACHUTE_SPEED
		)
	)
		parachute_status = LOST;

	if (fuel > 0)
		resulting_force += thruster_force;
	if (parachute_status == DEPLOYED)
		resulting_force += atmospheric_drag_force_parachute;

	vector3d resulting_acceleration_new = resulting_force / lander_mass;

	//update velocity through verlet
	velocity += delta_t*(resulting_acceleration + resulting_acceleration_new)*0.5;

	if
	(
		parachute_status == DEPLOYED &&
		(
			atmospheric_drag_force_parachute.abs() > MAX_PARACHUTE_DRAG ||
			velocity.abs()>MAX_PARACHUTE_SPEED
		)
	)
		parachute_status = LOST;

  // Here we can apply an autopilot to adjust the thrust, parachute and attitude
  if (autopilot_enabled) autopilot();

  // Here we can apply 3-axis stabilization to ensure the base is always pointing downwards
  if (stabilized_attitude) attitude_stabilization();
}

void initialize_simulation (void)
  // Lander pose initialization - selects one of 10 possible scenarios
{
  // The parameters to set are:
  // position - in Cartesian planetary coordinate system (m)
  // velocity - in Cartesian planetary coordinate system (m/s)
  // orientation - in lander coordinate system (xyz Euler angles, degrees)
  // delta_t - the simulation time step
  // boolean state variables - parachute_status, stabilized_attitude, autopilot_enabled
  // scenario_description - a descriptive string for the help screen

  scenario_description[0] = "circular orbit";
  scenario_description[1] = "descent from 10km";
  scenario_description[2] = "elliptical orbit, thrust changes orbital plane";
  scenario_description[3] = "polar launch at escape velocity (but drag prevents escape)";
  scenario_description[4] = "elliptical orbit that clips the atmosphere and decays";
  scenario_description[5] = "descent from 200km";
  scenario_description[6] = "geostationary orbit";
  scenario_description[7] = "";
  scenario_description[8] = "";
  scenario_description[9] = "";

  switch (scenario) {

  case 0:
    // a circular equatorial orbit
    position = vector3d(1.2*MARS_RADIUS, 0.0, 0.0);
    velocity = vector3d(0.0, -3247.087385863725, 0.0);
    orientation = vector3d(0.0, 90.0, 0.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = false;
    autopilot_enabled = false;

	stabilized_attitude_angle = 0;

	initialize_variables();
    break;

  case 1:
    // a descent from rest at 10km altitude
    position = vector3d(0.0, -(MARS_RADIUS + 10000.0), 0.0);
    velocity = vector3d(0.0, 0.0, 0.0);
    orientation = vector3d(0.0, 0.0, 90.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = true;
    autopilot_enabled = false;
	stabilized_attitude_angle = 0;
    
	initialize_variables();
	break;

  case 2:
    // an elliptical polar orbit
    position = vector3d(0.0, 0.0, 1.2*MARS_RADIUS);
    velocity = vector3d(3500.0, 0.0, 0.0);
    orientation = vector3d(0.0, 0.0, 90.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = false;
    autopilot_enabled = false;
	stabilized_attitude_angle = 0;

	initialize_variables();
	break;

  case 3:
    // polar surface launch at escape velocity (but drag prevents escape)
    position = vector3d(0.0, 0.0, MARS_RADIUS + LANDER_SIZE/2.0);
    velocity = vector3d(0.0, 0.0, 5027.0);
    orientation = vector3d(0.0, 0.0, 0.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = false;
    autopilot_enabled = false;
	stabilized_attitude_angle = 0;
    
	initialize_variables();
	break;

  case 4:
    // an elliptical orbit that clips the atmosphere each time round, losing energy
    position = vector3d(0.0, 0.0, MARS_RADIUS + 100000.0);
    velocity = vector3d(4000.0, 0.0, 0.0);
    orientation = vector3d(0.0, 90.0, 0.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = false;
    autopilot_enabled = false;
	stabilized_attitude_angle = 0;

	initialize_variables();
	break;

  case 5:
    // a descent from rest at the edge of the exosphere
    position = vector3d(0.0, -(MARS_RADIUS + EXOSPHERE), 0.0);
    velocity = vector3d(0.0, 0.0, 0.0);
    orientation = vector3d(0.0, 0.0, 90.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = true;
    autopilot_enabled = false;
	stabilized_attitude_angle = 0;
    
	initialize_variables();
	break;

  case 6:
	{
			double mars_omega = 2.0 * PI / MARS_DAY;
			double distance_geostat = pow(GRAVITY*MARS_MASS / pow(mars_omega, 2.0), 1.0 / 3.0);
			double speed_geostat = mars_omega*distance_geostat;
			position = vector3d(distance_geostat, 0.0, 0.0);
			velocity = vector3d(0.0, speed_geostat, 0.0);
			delta_t = 0.1;
			parachute_status = NOT_DEPLOYED;
			stabilized_attitude = false;
			autopilot_enabled = false;
			stabilized_attitude_angle = 0;

			initialize_variables();
			break;
	}
    break;

  case 7:
    break;

  case 8:
    break;

  case 9:
    break;

  }
}
