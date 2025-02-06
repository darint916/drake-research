// Entrypoint for the four-bar linkage simulation demo.
#include <gflags/gflags.h>
#include <iostream>
#include <filesystem>
#include <chrono>
#include <yaml-cpp/yaml.h>

#include "drake/geometry/meshcat.h"
#include "drake/geometry/meshcat_visualizer.h"
#include "drake/geometry/meshcat_visualizer_params.h"
#include "drake/geometry/scene_graph.h"

#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/tree/linear_bushing_roll_pitch_yaw.h"
#include "drake/multibody/tree/revolute_joint.h"

#include "drake/systems/analysis/simulator.h"
#include "drake/systems/analysis/simulator_print_stats.h"
#include "drake/systems/analysis/simulator_gflags.h"

#include "drake/systems/framework/diagram_builder.h"
#include "drake/visualization/visualization_config_functions.h"

// Use the appropriate Drake namespaces.
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::multibody::LinearBushingRollPitchYaw;
using drake::multibody::RevoluteJoint;
using drake::geometry::SceneGraph;
using drake::systems::DiagramBuilder;
using drake::systems::Simulator;
using Eigen::Vector3d;

// Define gflags parameters (these could be overridden on the command-line).
DEFINE_double(simulation_time, 10.0, "Duration of the simulation in seconds.");
DEFINE_double(force_stiffness, 30000,
              "Translational stiffness (N/m) for the LinearBushingRollPitchYaw force element.");
DEFINE_double(force_damping, 1500,
              "Translational damping (N·s/m) for the LinearBushingRollPitchYaw force element.");
DEFINE_double(torque_stiffness, 30000,
              "Rotational stiffness (N·m/rad) for the LinearBushingRollPitchYaw force element.");
DEFINE_double(torque_damping, 1500,
              "Rotational damping (N·m·s/rad) for the LinearBushingRollPitchYaw force element.");
DEFINE_double(applied_torque, 0.0,
              "Constant torque applied at joint_WA.");
DEFINE_double(initial_velocity, 3.0,
              "Initial angular rate (radians per second) at joint_WA.");

// Wrap the simulation in a dedicated namespace.
namespace drake {
namespace multibody {
namespace four_bar {
namespace {

int DoMain() {
  // Optionally, load simulation parameters from a YAML config file.
  // For example:
  // YAML::Node config = YAML::LoadFile("path/to/four_bar_config.yaml");
  // (Then use config[...] to override gflags values if desired.)

  // Create the diagram builder.
  DiagramBuilder<double> builder;

  // Create a MultibodyPlant with zero time step and a SceneGraph.
  auto [four_bar, scene_graph] = drake::multibody::AddMultibodyPlantSceneGraph(
      &builder, std::make_unique<MultibodyPlant<double>>(0.0));

  // Load the four-bar model from its SDF file.
  const std::string sdf_url = "/home/darin/Github/drake/flapgood/models/four_bar.sdf";
  Parser parser(&four_bar);
  parser.AddModels(sdf_url);

  // Retrieve the two frames for the bushing.
  const auto& frame_Bc = four_bar.GetFrameByName("Bc_bushing");
  const auto& frame_Cb = four_bar.GetFrameByName("Cb_bushing");

  // Define stiffness and damping constants (using the same value for each axis).
  const double k_xyz = FLAGS_force_stiffness;
  const double d_xyz = FLAGS_force_damping;
  const double k_rpy = FLAGS_torque_stiffness;
  const double d_rpy = FLAGS_torque_damping;

  // For this demo we assume that only a revolute (z-axis) degree-of-freedom is active.
  // Thus, we choose nonzero stiffness/damping only for the first two rotational axes.
  const Vector3d force_stiffness_constants{k_xyz, k_xyz, k_xyz};  // N/m
  const Vector3d force_damping_constants{d_xyz, d_xyz, d_xyz};      // N·s/m
  const Vector3d torque_stiffness_constants{k_rpy, k_rpy, 0};       // N·m/rad
  const Vector3d torque_damping_constants{d_rpy, d_rpy, 0};         // N·m·s/rad

  // Add the linear bushing force element to model the kinematic loop.
  four_bar.AddForceElement<LinearBushingRollPitchYaw>(
      frame_Bc, frame_Cb,
      torque_stiffness_constants, torque_damping_constants,
      force_stiffness_constants, force_damping_constants);

  // Finalize the MultibodyPlant.
  four_bar.Finalize();

  // Add default visualization (which sets up Meshcat if available).
  drake::visualization::AddDefaultVisualization(&builder);

  // Build the complete system diagram.
  auto diagram = builder.Build();

  // Create a context for the diagram and extract the subcontext for the MultibodyPlant.
  std::unique_ptr<drake::systems::Context<double>> diagram_context = diagram->CreateDefaultContext();
  auto& plant_context = four_bar.GetMyMutableContextFromRoot(diagram_context.get());

  // Apply a constant torque at joint_WA.
  four_bar.get_actuation_input_port().FixValue(&plant_context, FLAGS_applied_torque);

  // Set initial conditions.
  // Retrieve joints by name.
  const RevoluteJoint<double>& joint_WA = four_bar.GetJointByName<RevoluteJoint>("joint_WA");
  const RevoluteJoint<double>& joint_AB = four_bar.GetJointByName<RevoluteJoint>("joint_AB");
  const RevoluteJoint<double>& joint_WC = four_bar.GetJointByName<RevoluteJoint>("joint_WC");

  // Initialize joint angles.
  // Here we choose the angles so that joint_WA ≈ 75.52°, joint_AB and joint_WC ≈ 104.48°.
  const double qA = std::atan2(std::sqrt(15.0), 1.0);
  const double qB = M_PI - qA;
  const double qC = qB;

  joint_WA.set_angle(&plant_context, qA);
  joint_AB.set_angle(&plant_context, qB);
  joint_WC.set_angle(&plant_context, qC);

  // Set the initial angular rate for joint_WA.
  joint_WA.set_angular_rate(&plant_context, FLAGS_initial_velocity);

  // Optionally, one might record the simulation start time.
  auto start_time = std::chrono::high_resolution_clock::now();
  std::cout << "Starting simulation at time: " << start_time.time_since_epoch().count() << std::endl;

  // Create and run the simulator.
  Simulator<double> simulator(*diagram, std::move(diagram_context));
  simulator.set_target_realtime_rate(1.0);
  simulator.AdvanceTo(FLAGS_simulation_time);

  // Print simulation statistics.
  drake::systems::PrintSimulatorStatistics(simulator);

  // Optionally, print the simulation end time.
  auto end_time = std::chrono::high_resolution_clock::now();
  std::cout << "Simulation ended at time: " << end_time.time_since_epoch().count() << std::endl;

  // (Optionally, keep the process alive so that the Meshcat visualization remains open.)
  while (true) {
  }

  return 0;
}

}  // namespace
}  // namespace four_bar
}  // namespace multibody
}  // namespace drake

// Main entrypoint: parse flags and call our simulation main function.
int main(int argc, char* argv[]) {
  gflags::SetUsageMessage("A four-bar linkage demo demonstrating the use of a linear bushing to model a kinematic loop.");
  FLAGS_simulator_target_realtime_rate = 1.0;  // Ensures visualization is realistic.
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::multibody::four_bar::DoMain();
}
