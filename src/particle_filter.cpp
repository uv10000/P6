/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h> 
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>

#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).
	cout << "entering init" << endl;
	num_particles=10;

	default_random_engine gen;
	double std_x=std[0], std_y=std[1], std_theta=std[2]; // Standard deviations for x, y, and theta

	normal_distribution<double> dist_x(x, std_x);
	normal_distribution<double> dist_y(y, std_y);
	normal_distribution<double> dist_theta(theta, std_theta);

	for (int i = 0;i < num_particles;i++) {	
		//cout << "yyy" << particles.size();//particles;
		double sample_x, sample_y, sample_theta;
		sample_x = dist_x(gen);sample_y = dist_y(gen);sample_theta = dist_theta(gen);
		Particle particle; 
		particle.id=i; particle.weight=1.0;
		particle.x=sample_x;particle.y=sample_y;particle.theta=sample_theta;
		particles.push_back(particle);
		weights.push_back(1.0);
		
		/*	 
		struct Particle {
		int id;
		double x;
		double y;
		double theta;
		double weight;
		std::vector<int> associations;
		std::vector<double> sense_x;
		std::vector<double> sense_y;
		*/
	}
	is_initialized = true;

}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/
	default_random_engine gen;
	double std_x=std_pos[0], std_y=std_pos[1], std_theta=std_pos[2]; // Standard deviations for x, y, and theta
	
	
	//std::vector<Particle> fresh_particles;
	/*
	for (int i = 0;i < num_particles;i++) {	
		// apply measurement update
		float x_0= particles[i].x; float y_0 = particles[i].y; float theta_0= particles[i].theta;
		particles[i].x= x_0 + velocity/yaw_rate*(    sin(theta_0+yaw_rate*delta_t) - sin(theta_0));
		particles[i].y= y_0 + velocity/yaw_rate*(0.0-cos(theta_0+yaw_rate*delta_t) + cos(theta_0));
		particles[i].theta=theta_0 + yaw_rate*delta_t;
		// now add random noise to particle position
		normal_distribution<double> dist_x(particles[i].x, std_x);
	    normal_distribution<double> dist_y(particles[i].y, std_y);
	    normal_distribution<double> dist_theta(particles[i].theta, std_theta);
		particles[i].x+= dist_x(gen);
		particles[i].y+= dist_y(gen);
		particles[i].theta+=dist_theta(gen);
	}  */
	for (auto & particle : particles) {
    	float x_0= particle.x; float y_0 = particle.y; float theta_0= particle.theta;
		particle.x= x_0 + velocity/yaw_rate*(    sin(theta_0+yaw_rate*delta_t) - sin(theta_0));
		particle.y= y_0 + velocity/yaw_rate*(0.0-cos(theta_0+yaw_rate*delta_t) + cos(theta_0));
		particle.theta=theta_0 + yaw_rate*delta_t;
		// now add random noise to particle position
		normal_distribution<double> dist_x(particle.x, std_x);
	    normal_distribution<double> dist_y(particle.y, std_y);
	    normal_distribution<double> dist_theta(particle.theta, std_theta);
		particle.x+= dist_x(gen);
		//cout << "new x: " << particle.x << endl;
		particle.y+= dist_y(gen);
		particle.theta+=dist_theta(gen);
	}

}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html
	for (auto & particle : particles) {
		double x_part=particle.x; double y_part=particle.y; double theta_part= particle.theta;
		
		/* struct Particle {
		int id;
		double x;double y;double theta;double weight;
		std::vector<int> associations;
		std::vector<double> sense_x;
		std::vector<double> sense_y;}; */
        // create empty lists in particle
		particle.associations.clear();particle.sense_x.clear(); particle.sense_y.clear(); particle.weight=1.0;

		for (auto & observation : observations) {
			double x_obs=observation.x; double y_obs = observation.y; int id_obs = observation.id;
			double x_map=  x_obs * cos(theta_part) - y_obs * sin(theta_part)+ x_part;
			double y_map=  x_obs * sin(theta_part) + y_obs * cos(theta_part)+ y_part;
			// prepare to find best estimate given (x_map,y_map)	
			double x_best; double y_best; double distance_best = 1.0e10; int id_best = 10000000;
			for (auto & map_landmark : map_landmarks.landmark_list) {
				double x_lm=map_landmark.x_f; double y_lm=map_landmark.y_f; int id_lm=map_landmark.id_i;
				double mydistance= dist(x_map,y_map,x_lm,y_lm);
				//cout <<"mydistance: " << mydistance<<  "xmap: "<<x_map << " ymap: " << y_map << " xlm: " << x_lm << " ylm: " << y_lm << endl;
				if ((mydistance<= distance_best) 
									&& dist(x_map,y_map,particle.x, particle.y) <= sensor_range) {
					x_best=x_lm; y_best= y_lm; id_best = id_lm;
					distance_best = mydistance;
					//cout << " distance_best: " << distance_best << endl;
				}
				//cout << " DISTANCE_BEST: " << distance_best << endl;
			}
			
			
			// at this point (x_best,y_best) are known to be the coordinates of lm number id_best,
			// that is the best estimate for the given observation. It remains to append this to particle's lists
			particle.associations.push_back(id_best);
			particle.sense_x.push_back(x_best); particle.sense_y.push_back(y_best); 
			particle.weight= particle.weight * my_mv_2d_gauss(std_landmark,x_best,y_best,x_map,y_map);
			//cout << "weight: " << particle.weight << endl;
		} 
	}
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
    std::random_device rd;
    std::mt19937 gen(rd());
	//extract vector of weights, not yet normalized to one 
	std::vector<double> ws_non_normalized;
	double w_sum=0.0;
	for (auto & particle : particles) {
		ws_non_normalized.push_back(particle.weight);
		w_sum+= particle.weight;
	}
	//normalize the vector of weights
	std::vector<double> ws_normalized;
	cout << "w_sum" << w_sum << endl;
	for (auto & w : ws_non_normalized) {
		ws_normalized.push_back(w/w_sum);
	}
	std::discrete_distribution<> dist(ws_normalized.begin(), ws_normalized.end());
	//cout << "probability distribution w"  <<  ws_normalized[0]  << endl;
	
	std::vector<Particle> new_particles;
	for (auto & particle : particles) {
		int rand_index=dist(gen);
		//cout << "rand_index: "<< rand_index << endl;
		new_particles.push_back(particles[rand_index]);  // select random index according to weights
	}
	particles=new_particles;
}

Particle ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations, 
                                     const std::vector<double>& sense_x, const std::vector<double>& sense_y)
{
    //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates

    particle.associations= associations;
    particle.sense_x = sense_x;
    particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best)
{
	vector<int> v = best.associations;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseX(Particle best)
{
	vector<double> v = best.sense_x;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseY(Particle best)
{
	vector<double> v = best.sense_y;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
