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


void ParticleFilter::init(double x, double y, double theta, double std[]) {
  /**
   * TODO: Set the number of particles. Initialize all particles to 
   *   first position (based on estimates of x, y, theta and their uncertainties
   *   from GPS) and all weights to 1. 
   * TODO: Add random Gaussian noise to each particle.
   * NOTE: Consult particle_filter.h for more information about this method 
   *   (and others in this file).
   */
  //std::cout <<"init"<< std::flush;
  num_particles = 100;  // TODO: Set the number of particles
  std::default_random_engine gen;
  double std_x, std_y, std_theta;  // Standard deviations for x, y, and theta


  std_x = std[0];
  std_y = std[1];
  std_theta = std[2];


  // This line creates normal distributions for x, y and theta
  normal_distribution<double> dist_x(x, std_x);
  normal_distribution<double> dist_y(y, std_y);
  normal_distribution<double> dist_theta(theta, std_theta);

  Particle particle;
  // Creates the "num_particles" particules:
  for (int i = 0; i < num_particles; i++) {
    particle.id = i;
    particle.x = dist_x(gen);
    particle.y = dist_y(gen);
    particle.theta = dist_theta(gen);
    particle.weight = 1.;
    particles.push_back(particle);
  }

  // Initialization done
  is_initialized = true;
  //std::cout <<"isInit"<< std::flush;
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
  double eps = 1e-6;
  //std::cout <<"init"<< std::flush;

  // This lines creates normal distributions for x, y and theta position
  normal_distribution<double> noise_x(0, std_pos[0]);
  normal_distribution<double> noise_y(0, std_pos[1]);
  normal_distribution<double> noise_theta(0, std_pos[2]);

  //std::cout<<fabs(fmod(yaw_rate,2*M_PI)) <<" ; "<<std::endl<<std::flush;

  if (fabs(fmod(yaw_rate,2*M_PI)) > eps){// we are never too sure :)
    for (int i=0; i< num_particles; i++) {

      particles[i].x += velocity/yaw_rate*(sin(particles[i].theta + yaw_rate*delta_t) - sin(particles[i].theta));
      particles[i].x += noise_x(gen);
      particles[i].y += velocity/yaw_rate*(cos(particles[i].theta) - cos(particles[i].theta + yaw_rate*delta_t));
      particles[i].y += noise_y(gen);
      particles[i].theta += yaw_rate*delta_t;
      particles[i].theta += noise_theta(gen);
    }
  }else{
    for (int i=0; i< num_particles; i++){
      particles[i].x += velocity*cos(particles[i].theta)*delta_t;
      particles[i].x += noise_x(gen);
      particles[i].y += velocity*sin(particles[i].theta)*delta_t;
      particles[i].y += noise_y(gen);
      particles[i].theta += noise_theta(gen);
    }
  }
  //std::cout <<"end"<< std::flush;
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

  for (unsigned i = 0; i < observations.size(); i++) {
    double min_dist = 1e6;
    int min_dist_prediction_index = -1;
    for (unsigned j = 0; j < predicted.size(); j++) {
      double dist_ij = dist(observations[i].x, observations[i].y, predicted[j].x, predicted[j].y);
      if (dist_ij < min_dist) {
        min_dist = dist_ij;
        min_dist_prediction_index = j;
      }
    }
    //std::cout<<"min_dist"<<min_dist<<std::endl<<std::flush;
    //std::cout<<"min_dist_prediction_index"<<min_dist_prediction_index<<std::endl<<std::flush;
    observations[i].id = predicted[min_dist_prediction_index].id;
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
  
  
  

  for (int i = 0; i < num_particles; i++) {

    double p_x = particles[i].x;
    double p_y = particles[i].y;
    double p_theta = particles[i].theta;
    vector<LandmarkObs> map_observations;

    //1: transformation of the observation fro car to world referencial
    for (unsigned j=0; j< observations.size(); j++) {



      LandmarkObs current_obs;

      current_obs.id = observations[j].id;
      current_obs.x  = observations[j].x*cos(p_theta) - observations[j].y*sin(p_theta) + p_x;
      current_obs.y  = observations[j].x*sin(p_theta) + observations[j].y*cos(p_theta) + p_y;

      map_observations.push_back(current_obs);
    }

    // 2. find landmarks within the particle's sensor's range
    vector<LandmarkObs> nearby_landmarks;
    for (unsigned j=0; j<map_landmarks.landmark_list.size(); j++){

      LandmarkObs current_landmark;

      current_landmark.id = map_landmarks.landmark_list[j].id_i;
      current_landmark.x = map_landmarks.landmark_list[j].x_f;
      current_landmark.y = map_landmarks.landmark_list[j].y_f;
      
      if (dist(p_x, p_y, current_landmark.x, current_landmark.y) < sensor_range){
        nearby_landmarks.push_back(current_landmark);
      } 
    }

    //3: find the nearest neighbor
    dataAssociation(nearby_landmarks, map_observations);

    //4: Finally updating the weights of each particles
    particles[i].weight = 1.0;
    
    double std_x = std_landmark[0];
    double std_y = std_landmark[1];
    double na = 2.0 * std_x * std_x;
    double nb = 2.0 * std_y * std_y;
    double gauss_norm = 2.0 * M_PI * std_x * std_y;
    
    for (unsigned j=0; j<map_observations.size(); j++){
      
      double o_x = map_observations[j].x;
      double o_y = map_observations[j].y;
      
      double pr_x, pr_y;
      for (unsigned k = 0; k < nearby_landmarks.size(); k++) {
        if (nearby_landmarks[k].id == map_observations[j].id) {
            pr_x = nearby_landmarks[k].x;
            pr_y = nearby_landmarks[k].y;
            break;
        }
      }
      // product of all the observation weights
      particles[i].weight *= 1/gauss_norm * exp( - (pow(pr_x-o_x,2)/na + pow(pr_y-o_y,2)/nb) );
      //std::cout<<pr_x-o_x<<" et "<<pr_y-o_y<<" avec pi = "<<M_PI<<" ; "<<std::flush;  

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
  

  vector<double> parents_weights;


  bool genetic_sample = true;

  // get the current weight for the discrete distribution
  for (int i = 0; i < num_particles; i++) {
    parents_weights.push_back(particles[i].weight);
  } 

  std::default_random_engine gen;
  std::discrete_distribution<int> distribution(parents_weights.begin(), parents_weights.end());
  vector<Particle> child_particles;

  //push back the new set of particles
  for (int i = 0; i < num_particles; i++) {

    if(genetic_sample == false){
      child_particles.push_back(particles[distribution(gen)]);
      
    }else{

      // kinda genetic algorithm for finding the next generation of particle
      Particle parent_A = particles[distribution(gen)];
      Particle parent_B = particles[distribution(gen)];
      Particle child = parent_A;
    


      double sum_weights = (parent_A.weight + parent_B.weight);

      //std::cout<<"child.x"<<(parent_A.x*parent_A.weight + parent_B.x*parent_B.weight) / sum_weights<<std::endl<<std::flush;

      child.x = (parent_A.x*parent_A.weight + parent_B.x*parent_B.weight) / sum_weights;
      child.y = (parent_A.y*parent_A.weight + parent_B.y*parent_B.weight) / sum_weights;
      child.theta = (parent_A.theta*parent_A.weight + parent_B.theta*parent_B.weight) / sum_weights;
      child.weight = sum_weights / 2;

      //std::cout<<"child.x"<<child.x<<std::endl<<std::flush;
      child_particles.push_back(child);
    }
    
  }
  for (int i = 0; i < num_particles; i++) {
    particles[i]=child_particles[i];
    //replace particles with resampled particles
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