/**
 * particle_filter.cpp
 *
 * Created on: Dec 12, 2016
 * Author: Tiffany Huang
 */

#include "particle_filter.h"

#include <math.h>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <numeric>
#include <random>
#include <string>
#include <vector>

#include "helper_functions.h"

using std::string;
using std::vector;
using std::normal_distribution;
using std::uniform_int_distribution;
using std::uniform_real_distribution;
using std::cout;
using std::endl;
using std::numeric_limits;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  /**
   * TODO: Set the number of particles. Initialize all particles to 
   *   first position (based on estimates of x, y, theta and their uncertainties
   *   from GPS) and all weights to 1. 
   * TODO: Add random Gaussian noise to each particle.
   * NOTE: Consult particle_filter.h for more information about this method 
   *   (and others in this file).
   */
  num_particles = 100;  // TODO: Set the number of particles
  std::default_random_engine gen;

  // Creates normal (Gaussian) distributions (for x, y and theta)
  normal_distribution<double> dist_x(x, std[0]);
  normal_distribution<double> dist_y(y, std[1]);
  normal_distribution<double> dist_theta(theta, std[2]);
  
  // Initialization
  for (int i = 0; i < num_particles; ++i) {
    Particle p;

    // Initialize each parameter
    p.id = i;
    p.x = dist_x(gen);
    p.y = dist_y(gen);
    p.theta = dist_theta(gen);
    p.weight = 1.0;

    // Show initialized data of the first five particles
    if (debug == true) {
      if (p.id < 5) {
        cout << "Particle #" << p.id + 1 << endl;      
        cout << "p.x = " << p.x << ", p.y = " << p.y << ", p.theta = " << p.theta << endl;
      }
    }

    // Add the created particle to the vector
    particles.push_back(p);
  }

  // Enable the flag
  is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], 
                                double velocity, double yaw_rate) {
  /**
   * TODO: Add measurements to each particle and add random Gaussian noise.
   * NOTE: When adding noise you may find std::normal_distribution 
   *   and std::default_random_engine useful.
   *  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
   *  http://www.cplusplus.com/reference/random/default_random_engine/
   */
  std::default_random_engine gen;

  // Creates normal (Gaussian) distributions (for x, y and theta)
  normal_distribution<double> dist_x(0, std_pos[0]);
  normal_distribution<double> dist_y(0, std_pos[1]);
  normal_distribution<double> dist_theta(0, std_pos[2]);
    
  for (int i = 0; i < num_particles; i++) {     
    // Predict new state from given velocity and yaw_rate 
    if (fabs(yaw_rate) > 0) {
      particles[i].x += velocity / yaw_rate * (sin(particles[i].theta + yaw_rate * delta_t) - sin(particles[i].theta));
      particles[i].y += velocity / yaw_rate * (cos(particles[i].theta) - cos(particles[i].theta + yaw_rate * delta_t));
      particles[i].theta += yaw_rate * delta_t;
    } else {
      // In special case when yaw_rate = 0
      particles[i].x += velocity * delta_t * cos(particles[i].theta);
      particles[i].y += velocity * delta_t * sin(particles[i].theta);
    }

    // Add Gaussian noise
    particles[i].x += dist_x(gen);
    particles[i].y += dist_y(gen);
    particles[i].theta += dist_theta(gen);

    // Show predicted data of the first five particles
    if (debug == true) {
      if (i < 5) {
        cout << "Particle #" << i + 1 << endl;      
        cout << "particles[i].x = " << particles[i].x << ", particles[i].y = " << particles[i].y << ", particles[i].theta = " << particles[i].theta << endl;
      }
    }    
  }
}

void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted, 
                                     vector<LandmarkObs>& observations) {
  /**
   * TODO: Find the predicted measurement that is closest to each 
   *   observed measurement and assign the observed measurement to this 
   *   particular landmark.
   * NOTE: this method will NOT be called by the grading code. But you will 
   *   probably find it useful to implement this method and use it as a helper 
   *   during the updateWeights phase.
   */
  // Loop for all observed measurements
  for (unsigned int i = 0; i < observations.size(); i++) {
        // Initialize the minimum distance
        double min_dist = numeric_limits<double>::max();

        // Initialize the id of predicted measurement (to be associated with the observation)
        int associated_id = -1;   

        // Get current observation
        LandmarkObs obs = observations[i];  

        // Loop for all predicted measurements
        for (unsigned int j = 0; j < predicted.size(); j++) {
            LandmarkObs predict = predicted[j];
            double current_dist = dist(obs.x, obs.y, predict.x, predict.y);
            if (current_dist < min_dist) {
                min_dist = current_dist;
                associated_id = predict.id;
            }
        }

        // Show some information of the first five observations
        if (debug == true) {
          if (i < 5) {
            cout << "Observation #" << i + 1 << endl;
            cout << "obs.x = " << obs.x << ", obs.y = " << obs.y << endl;
            cout << "min_dist = " << min_dist << endl;      
            cout << "associated_id = " << associated_id << endl;
            cout << "predicted[id].x = " << predicted[associated_id].x << ", predicted[id].y = " << predicted[associated_id].y << endl;
          }
        }        

        // Select the id of nearest predicted measurements
        observations[i].id = associated_id;
    }   
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
                                   const vector<LandmarkObs> &observations, 
                                   const Map &map_landmarks) {
  /**
   * TODO: Update the weights of each particle using a mult-variate Gaussian 
   *   distribution. You can read more about this distribution here: 
   *   https://en.wikipedia.org/wiki/Multivariate_normal_distribution
   * NOTE: The observations are given in the VEHICLE'S coordinate system. 
   *   Your particles are located according to the MAP'S coordinate system. 
   *   You will need to transform between the two systems. Keep in mind that
   *   this transformation requires both rotation AND translation (but no scaling).
   *   The following is a good resource for the theory:
   *   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
   *   and the following is a good resource for the actual equation to implement
   *   (look at equation 3.33) http://planning.cs.uiuc.edu/node99.html
   */

  // Loop for each particle
  for (int i = 0; i < num_particles; i++) {        
    // Get x, y, theta of each particle
    double p_x = particles[i].x;
    double p_y = particles[i].y;
    double p_theta = particles[i].theta;
        
    // Vector for predicted landmarks (within sensor range of each particle)
    vector<LandmarkObs> predicted;
        
    // Loop for each landmark on the map
    for (unsigned int j = 0; j < map_landmarks.landmark_list.size(); j++) {
      // Get id and coordinates
      int landmark_id = map_landmarks.landmark_list[j].id_i;
      float landmark_x = map_landmarks.landmark_list[j].x_f;
      float landmark_y = map_landmarks.landmark_list[j].y_f;
            
      // Select the landmarks within sensor range of the particle
      if (fabs(p_x - landmark_x) <= sensor_range && fabs(p_y - landmark_y) <= sensor_range) {
        predicted.push_back(LandmarkObs{landmark_id, landmark_x, landmark_y});
      }
    }
    
    // Transform the observations: vehicle coordinates --> map coordinates
    vector<LandmarkObs> transformed_obs;    
    for (unsigned int j = 0; j < observations.size(); j++) {
      double transformed_x = cos(p_theta) * observations[j].x - sin(p_theta) * observations[j].y + p_x;
      double transformed_y = sin(p_theta) * observations[j].x + cos(p_theta) * observations[j].y + p_y;
      transformed_obs.push_back(LandmarkObs{observations[j].id, transformed_x, transformed_y});
    }
        
    // Call function to associate the transformed observation with predicted landmarks
    dataAssociation(predicted, transformed_obs);
        
    // Initialize the weight
    particles[i].weight = 1.0;
        
    // Loop for each transformed observation
    for (unsigned int j = 0; j < transformed_obs.size(); j++) {
      // Get cordinates of the observation
      double obs_x = transformed_obs[j].x;
      double obs_y = transformed_obs[j].y;
            
      // Get coordinates of the predicted measurement (associated with the observation)
      double predicted_x, predicted_y;
      for (unsigned int k = 0; k < predicted.size(); k++) {
        if (predicted[k].id == transformed_obs[j].id) {
          predicted_x = predicted[k].x;
          predicted_y = predicted[k].y;
        }
      }
            
      // Calculate the weight for this observation with multivariate Gaussian          
      double obs_weight = multiv_prob(std_landmark[0], std_landmark[1], obs_x, obs_y, predicted_x, predicted_y);
      
      // Multiply the calculated weight  
      particles[i].weight *= obs_weight;
    }

    // Show updated weight of the first five particles
    if (debug == true) {
      if (i < 5) {
        cout << "Particle #" << i + 1 << endl;
        cout << "Updated weight = " << particles[i].weight << endl;
      }
    }
  }
}

void ParticleFilter::resample() {
  /**
   * TODO: Resample particles with replacement with probability proportional 
   *   to their weight. 
   * NOTE: You may find std::discrete_distribution helpful here.
   *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   */
  std::default_random_engine gen;

  // Create a vector for particle weights
  vector<double> weights;
  for (int i = 0; i < num_particles; i++) {
    weights.push_back(particles[i].weight);
  }

  // Initial setting
  vector<Particle> new_particles;
  double beta = 0.0;
  uniform_int_distribution<int> uniintdist(0, num_particles-1);
  int index = uniintdist(gen);
  double mw = *max_element(weights.begin(), weights.end());
  uniform_real_distribution<double> unirealdist(0.0, 2.0 * mw);

  // Algorithm to sample the new particles
  for (int i = 0; i < num_particles; i++) {
    beta += unirealdist(gen);
    while (beta > weights[index]) {
      beta -= weights[index];
      index = (index + 1) % num_particles;
    }
    new_particles.push_back(particles[index]);

    // Show the index of some new samples
    if (debug == true) {
      if (i < 5) {
        cout << "Index of #" << i + 1 << " sample = " << index <<endl;
      }
    }
  }
  
  // Replace the particles with newly sampled ones
  particles = new_particles;
}

void ParticleFilter::SetAssociations(Particle& particle, 
                                     const vector<int>& associations, 
                                     const vector<double>& sense_x, 
                                     const vector<double>& sense_y) {
  // particle: the particle to which assign each listed association, 
  //   and association's (x,y) world coordinates mapping
  // associations: The landmark id that goes along with each listed association
  // sense_x: the associations x mapping already converted to world coordinates
  // sense_y: the associations y mapping already converted to world coordinates
  particle.associations= associations;
  particle.sense_x = sense_x;
  particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best) {
  vector<int> v = best.associations;
  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}

string ParticleFilter::getSenseCoord(Particle best, string coord) {
  vector<double> v;

  if (coord == "X") {
    v = best.sense_x;
  } else {
    v = best.sense_y;
  }

  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}