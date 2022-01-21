#pragma once

#include <fstream>
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <cstdlib>

class Config
{
public:
  static Config* is()
  {
    static Config* instance = new Config;
    return instance;
  }
  Config()
  {
    // set default value
    n_of_particles = 100;
    update = 100;
    max_lrf_range = 30.0;
    min_lrf_range = 0.1;
    target_area[0] = -10.0;
    target_area[1] = -10.0;
    target_area[2] = 10.0;
    target_area[3] = 10.0;
    m_sigma = 0.01*0.01;
    pos_noise = 0.05;
    vel_noise = 0.05;
    particle_area = 0.1;
    max_vel = 0.1;
    m_max_ID = 100;
    m_min_distance = 1.0;
    m_initial_dist = 0.5;

    std::string home = std::getenv("HOME");
    std::string filepath = home + "/catkin_ws/src/ros_tms/tms_ss/tms_ss_pot/config.yaml";
    std::cout << "filepath " << filepath << std::endl;

    try
    {
      YAML::Node config = YAML::LoadFile(filepath);

      if (config["n_of_particles"])
        n_of_particles = config["n_of_particles"].as< int >();
      if (config["update"])
        update = config["update"].as< int >();
      if (config["max_lrf_range"])
        max_lrf_range = config["max_lrf_range"].as< double >();
      if (config["min_lrf_range"])
        min_lrf_range = config["min_lrf_range"].as< double >();
      if (config["target_area"])
      {
        if (config["target_area"].IsSequence())
        {
          for (int i = 0; i < 4; i++)
            target_area[i] = config["target_area"][i].as< double >();
        }
        else
        {
          target_area[0] = -config["target_area"].as< double >();
          target_area[1] = -config["target_area"].as< double >();
          target_area[2] = config["target_area"].as< double >();
          target_area[3] = config["target_area"].as< double >();
        }
      }

      if (config["m_sigma"])
        m_sigma = config["m_sigma"].as< double >();
      if (config["pos_noise"])
        pos_noise = config["pos_noise"].as< double >();
      if (config["vel_noise"])
        vel_noise = config["vel_noise"].as< double >();
      if (config["particle_area"])
        particle_area = config["particle_area"].as< double >();
      if (config["max_vel"])
        max_vel = config["max_vel"].as< double >();
      if (config["m_max_ID"])
        m_max_ID = config["m_max_ID"].as< int >();
      if (config["m_min_distance"])
        m_min_distance = config["m_min_distance"].as< double >();
      if (config["m_initial_dist"])
        m_initial_dist = config["m_initial_dist"].as< double >();

      // Get LRF Pos
      if (config["lrf1_pos"]) {
          if (config["lrf1_pos"].IsSequence()) {
          for (int i = 0; i < 3; i++)
            lrf1_pos[i] = config["lrf1_pos"][i].as< double >();
        }
      }
      if (config["lrf2_pos"]) {
        if (config["lrf2_pos"].IsSequence()) {
          for (int i = 0; i < 3; i++)
            lrf2_pos[i] = config["lrf2_pos"][i].as< double >();
        }
      }
      if (config["lrf3_pos"]) {
        if (config["lrf3_pos"].IsSequence()) {
          for (int i = 0; i < 3; i++)
            lrf3_pos[i] = config["lrf3_pos"][i].as< double >();
        }
      }
      if (config["lrf4_pos"]) {
        if (config["lrf4_pos"].IsSequence()) {
          for (int i = 0; i < 3; i++)
            lrf4_pos[i] = config["lrf4_pos"][i].as< double >();
        }
      }
      if (config["lrf_active"]) {
          if (config["lrf_active"].IsSequence()) {
          for (int i = 0; i < 4; i++)
            lrf_active[i] = config["lrf_active"][i].as< bool >();
        }
      }

    }
    catch (YAML::BadFile& e)
    {
      std::cerr << e.what() << " or Bad file path: /current/" << filepath << std::endl;
    }
    catch (YAML::KeyNotFound& e)
    {
      std::cerr << e.what() << std::endl;
    }
  }

  int n_of_particles;
  int update;
  double max_lrf_range;
  double min_lrf_range;
  double target_area[4];
  double m_sigma;
  double pos_noise;
  double vel_noise;
  double particle_area;
  double max_vel;
  int m_max_ID;
  double m_min_distance;
  double m_initial_dist;
  double lrf1_pos[3];
  double lrf2_pos[3];
  double lrf3_pos[3];
  double lrf4_pos[3];
  bool lrf_active[4];
};
