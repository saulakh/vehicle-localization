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
using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  /**
   * TODO: Set the number of particles. Initialize all particles to 
   *   first position (based on estimates of x, y, theta and their uncertainties
   *   from GPS) and all weights to 1. 
   * TODO: Add random Gaussian noise to each particle.
   * NOTE: Consult particle_filter.h for more information about this method 
   *   (and others in this file).
   */
  num_particles = 50;  // TODO: Set the number of particles
  
  // From Lesson 5
  std::default_random_engine gen;
  
  // Normal Gaussian distribution, Lesson 5
  normal_distribution<double> dist_x(x, std[0]);
  normal_distribution<double> dist_y(y, std[1]);
  normal_distribution<double> dist_theta(theta, std[2]);
  
  for (int i = 0; i < num_particles; ++i) {
    Particle particle; // Structure from particle_filter.h
    particle.id = i;
    particle.x = dist_x(gen);
    particle.y = dist_y(gen);
    particle.theta = dist_theta(gen);
    particle.weight = 1.0;
    
    particles.push_back(particle);
    
    weights.push_back(1.0);
  }
  
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
  
  // Normal Gaussian distribution, Lesson 5
  normal_distribution<double> dist_x(0, std_pos[0]);
  normal_distribution<double> dist_y(0, std_pos[1]);
  normal_distribution<double> dist_theta(0, std_pos[2]);
  
  for (int i = 0; i < num_particles; i++) {
    
    if (fabs(yaw_rate) < 0.0001) {
      
      // Equations from Lesson 8, if yaw rate is nonzero
      particles[i].x += velocity*delta_t*cos(particles[i].theta);
      particles[i].y += velocity*delta_t*sin(particles[i].theta);
      //particles[i].theta = particles[i].theta;
    }
    else {
      particles[i].x += (velocity/yaw_rate) * (sin(particles[i].theta + yaw_rate*delta_t) - sin(particles[i].theta));
      particles[i].y += (velocity/yaw_rate) * (cos(particles[i].theta) - cos(particles[i].theta + yaw_rate*delta_t));
      particles[i].theta += yaw_rate*delta_t;
    }
    
    // Add noise to particles
    particles[i].x += dist_x(gen);
    particles[i].y += dist_y(gen);
    particles[i].theta += dist_theta(gen);
    
  }
}

void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted_landmarks, 
                                     vector<LandmarkObs>& tr_observations) {
  /**
   * TODO: Find the predicted measurement that is closest to each 
   *   observed measurement and assign the observed measurement to this 
   *   particular landmark.
   * NOTE: this method will NOT be called by the grading code. But you will 
   *   probably find it useful to implement this method and use it as a helper 
   *   during the updateWeights phase.
   */
  
  /*
  
  double min_dist = std::numeric_limits<double>::max();
  double closest_landmark = -1;
  
  // Loop through transformed observations
  for (unsigned int i = 0; i < tr_observations.size(); i++) {
    LandmarkObs obs = tr_observations[i];
    
    // Loop through predicted landmarks
    for (unsigned int j = 0; j < predicted_landmarks.size(); j++) {
      LandmarkObs pred = predicted_landmarks[i];
      
      // Find distance between observation and prediction
      double current_dist = dist(obs.x, obs.y, pred.x, pred.y);
      
      // If closest so far, update min_dist and closest_landmark
      if (current_dist < min_dist) {
        min_dist = current_dist;
        closest_landmark = pred.id;
      }
    }
    
    // Assign observed measurement to this landmark
    tr_observations[i].id = closest_landmark;
    cout << "closest landmark from data association loop: " << closest_landmark << endl;
  }
  */
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
  
  // Outer loop for particles
  for (int p=0; p < num_particles; p++) {
    
    particles[p].weight = 1.0;
  
    // Create predictions vector for data association
    //vector<LandmarkObs> predictions;
  
    // Tranforming observations from vehicle's frame to particle's frame
    for (unsigned int i = 0; i < observations.size(); i++) {
      // LandmarkObs structure: (id, x, y) from helper_functions.h
      vector<LandmarkObs> transformed_obs;
      LandmarkObs tr_obs;
      //int association = 0;
    
      // Using equations from Lesson 16
      tr_obs.x = particles[p].x + (observations[i].x*cos(particles[p].theta)) - (observations[i].y*sin(particles[p].theta));
      tr_obs.y = particles[p].y + (observations[i].x*sin(particles[p].theta)) + (observations[i].y*cos(particles[p].theta));

      transformed_obs.push_back(tr_obs);
  
      //particles[p].weight = 1.0;
      
      double min_dist = sensor_range;
      
      // Landmark structure: (id, x, y) from map.h
      for (unsigned int j = 0; j < map_landmarks.landmark_list.size(); j++) {
        double landmark_x = map_landmarks.landmark_list[j].x_f;
        double landmark_y = map_landmarks.landmark_list[j].y_f;
        int landmark_id = map_landmarks.landmark_list[j].id_i;
        
        // Distance between landmark and transformed observations
        double current_dist = dist(transformed_obs[i].x, transformed_obs[i].y, landmark_x, landmark_y);
        
        // Find closest landmark
        //if (current_dist < min_dist) {
          //association = landmark_id;
          //min_dist = current_dist;
        //}
        
        // Find landmarks within sensor range
        if (current_dist < sensor_range) {    
          
          // Add landmarks to predictions
          //predictions.push_back(LandmarkObs{ landmark_id, landmark_x, landmark_y });
          
          // Find closest landmark
          //dataAssociation(predictions, transformed_obs);
      
          // Update particle weights, Lesson 19
        
          // x and y are the observations in map coordinates
          double obs_x = transformed_obs[i].x;
          double obs_y = transformed_obs[i].y;
          // mu_x and mu_y are coordinates of closest landmarks
          double mu_x = map_landmarks.landmark_list[landmark_id].x_f;
          double mu_y = map_landmarks.landmark_list[landmark_id].y_f;
      
          // Multivariate Gaussian
          const double pi = 3.14159265358979323846;
          double gauss_norm = 1 / (2*pi*std_landmark[0]*std_landmark[1]);
          double exponent = (pow(obs_x - mu_x, 2) / (2*pow(std_landmark[0], 2))) + (pow(obs_y - mu_y, 2) / (2*pow(std_landmark[1], 2)));
          double current_weight = gauss_norm*exp(-exponent);
      
          if (current_weight > 0) {
            particles[p].weight *= current_weight;
          }
        }
                
        weights[p] = particles[p].weight;
      
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
  
  // From project Q&A
  std::default_random_engine gen;
  discrete_distribution<int> dist(weights.begin(), weights.end());
  
  vector<Particle> resample_particles;
  
  for (int i = 0; i < num_particles; i++)
  {
    
    resample_particles.push_back(particles[dist(gen)]);
  }

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