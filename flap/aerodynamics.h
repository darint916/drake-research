#pragma once

//FIX REDUNDANCY AND CLEAN UP LATER
#include <drake/systems/framework/leaf_system.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/multibody/tree/rigid_body.h>
//Later above tree/body.h deprecated to rigid_body.h, https://github.com/RobotLocomotion/drake/blob/master/multibody/tree/rigid_body.cc
// this includes body index and such from header files that are linked
#include <drake/geometry/scene_graph.h>
#include <Eigen/Dense>
#include <vector>
#include <string>
#include "drake/geometry/scene_graph.h"
#include "drake/math/rigid_transform.h"
#include <drake/multibody/plant/externally_applied_spatial_force.h>
#include <drake/multibody/math/spatial_force.h>
// #include <drake/multibody/tree/body.h>
#include <thread>
#include <algorithm>
#include <future>
#include <mutex>
#include <condition_variable>
#include <iostream>
#include <drake/systems/framework/input_port.h>
#include <drake/math/roll_pitch_yaw.h>
#include <chrono>
#include <fstream>
#include <atomic>
#include <drake/multibody/tree/revolute_joint.h>
// #include <drake/common/yaml/yaml_io.h>
#include <yaml-cpp/yaml.h>
// #include <json/json.h>
namespace drake {
namespace flap {

using drake::systems::LeafSystem;
using drake::multibody::MultibodyPlant;
using drake::multibody::Body;
using drake::multibody::SpatialVelocity;
using drake::multibody::ExternallyAppliedSpatialForce;
using drake::math::RigidTransform;
// using drake::geometry::SceneGraph;
using Eigen::Vector3d;

class Aerodynamics: public LeafSystem<double> {
public:
    explicit Aerodynamics(const MultibodyPlant<double>& plant);
    virtual ~Aerodynamics() noexcept override;
 
protected:
    void CalcOutput(const systems::Context<double>& context,
    std::vector<drake::multibody::ExternallyAppliedSpatialForce<double>>* forces) const;
    void CalcOutputThreaded(const systems::Context<double>& context,
    std::vector<drake::multibody::ExternallyAppliedSpatialForce<double>>* output) const;
    void LoggerThread(std::vector<std::vector<double>>& buffer) const;
    void CalcForceBatch(size_t start, size_t end, Eigen::Vector3d up_vector_rw, drake::math::RigidTransformd pose_world_rw, drake::multibody::SpatialVelocity<double> velocity_world_rw,
        Eigen::Vector3d up_vector_lw, drake::math::RigidTransformd pose_world_lw, drake::multibody::SpatialVelocity<double> velocity_world_lw, std::vector<drake::multibody::ExternallyAppliedSpatialForce<double>>* output) const;
    Eigen::Matrix3d MakeSkewSymmetric(const Eigen::Vector3d& w);
private:
    const MultibodyPlant<double>& plant_;
    const drake::multibody::Body<double>* right_wing_body_;
    const drake::multibody::Body<double>* left_wing_body_;
    const drake::multibody::Body<double>* right_wing_joint_link_;
    const drake::multibody::RevoluteJoint<double>* right_wing_passive_joint_;
    const drake::multibody::Body<double>* main_body_;
    // const drake::multibody::Joint<double>* left_wing_passive_joint_;

    drake::multibody::BodyIndex main_body_index_;
    drake::multibody::BodyIndex right_wing_body_index_;
    drake::multibody::BodyIndex left_wing_body_index_;
    drake::multibody::BodyIndex right_wing_joint_link_index_;
    Vector3d orthogonal_vec_;
    std::vector<Vector3d> center_pressure_body_lw_;
    std::vector<Vector3d> center_pressure_body_rw_;
    double blade_area_;
    size_t cp_size_;
    std::vector<double> blade_area_list_lw_;
    std::vector<double> blade_area_list_rw_;
    double air_density_;
    double drag_coef_;
    size_t num_threads_;
    
    size_t batch_size_;
    mutable std::mutex output_mutex_; //mutable since const functions implicitly make these const
    mutable int counter; //ad hoc
    double rad_to_deg;
    mutable std::chrono::high_resolution_clock::time_point start_time_;
    mutable std::mutex buffer_mutex_;
    //terrible mutable design, drake requires this for leaf system const function declare abstract port T_T
    mutable std::vector<std::vector<double>> data_buffer_1_;  
    mutable std::vector<std::vector<double>> data_buffer_2_;
    mutable std::vector<std::vector<double>>* active_buffer_;
    mutable std::vector<std::vector<double>>* background_buffer_;
    
    // std::chrono::high_resolution_clock::time_point end_time_;
    mutable std::ofstream data_file_;
    mutable bool first_time_;
    mutable std::thread logger_thread_;
    mutable std::atomic<bool> buffer_thread_finished_;
    mutable int log_count_; 
    mutable int log_counter_;
    mutable int buffer_index_;
    const int buffer_size_ = 50000; //migrate to yaml later?
};

}  // namespace flap
}  // namespace drake
