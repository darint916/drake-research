// Entrypoint for the four-bar linkage simulation demo.
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <gflags/gflags.h>
#include <yaml-cpp/yaml.h>

#include "drake/geometry/meshcat.h"
#include "drake/geometry/meshcat_visualizer.h"
#include "drake/geometry/meshcat_visualizer_params.h"
#include "drake/geometry/scene_graph.h"

#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/linear_bushing_roll_pitch_yaw.h"
#include "drake/multibody/tree/revolute_joint.h"

#include "drake/systems/analysis/simulator.h"
#include "drake/systems/analysis/simulator_gflags.h"
#include "drake/systems/analysis/simulator_print_stats.h"

#include <drake/systems/analysis/implicit_euler_integrator.h>
#include <drake/systems/analysis/runge_kutta2_integrator.h>
#include <drake/systems/primitives/vector_log_sink.h>
#include "drake/systems/framework/diagram_builder.h"
#include "drake/visualization/visualization_config_functions.h"

// Use the appropriate Drake namespaces.
using drake::geometry::SceneGraph;
using drake::multibody::LinearBushingRollPitchYaw;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::multibody::RevoluteJoint;
using drake::systems::DiagramBuilder;
using drake::systems::Simulator;
using Eigen::Vector3d;

// Define gflags parameters (these could be overridden on the command-line).
DEFINE_double(simulation_time, 15.0, "Duration of the simulation in seconds.");
DEFINE_double(force_stiffness, 30000000,
              "Translational stiffness (N/m) for the LinearBushingRollPitchYaw force element.");
DEFINE_double(force_damping, 15000, "Translational damping (N·s/m) for the LinearBushingRollPitchYaw force element.");
DEFINE_double(torque_stiffness, 30000000,
              "Rotational stiffness (N·m/rad) for the LinearBushingRollPitchYaw force element.");
DEFINE_double(torque_damping, 1500, "Rotational damping (N·m·s/rad) for the LinearBushingRollPitchYaw force element.");
DEFINE_double(applied_torque, -10000, "Constant torque applied at joint_WA.");
DEFINE_double(initial_velocity, 0, "Initial angular rate (radians per second) at joint_WA.");

// Wrap the simulation in a dedicated namespace.
namespace drake
{
namespace multibody
{
namespace four_bar
{
namespace
{

int DoMain()
{
    // Optionally, load simulation parameters from a YAML config file.
    // For example:
    // YAML::Node config = YAML::LoadFile("path/to/four_bar_config.yaml");
    // (Then use config[...] to override gflags values if desired.)

    // Create the diagram builder.
    DiagramBuilder<double> builder;

    // Create a MultibodyPlant with zero time step and a SceneGraph.
    auto [four_bar, scene_graph] =
        drake::multibody::AddMultibodyPlantSceneGraph(&builder, std::make_unique<MultibodyPlant<double>>(0));

    // Load the four-bar model from its SDF file.
    // const std::string sdf_url = "/home/darin/Github/drake/flapgood/models/wing_asm_simple.sdf";
    // const std::string sdf_url = "/home/darin/Github/drake/flapgood/models/wing_asm_flat.sdf";
    const std::string sdf_url = "/home/darin/Github/drake/flapgood/models/four_more.sdf";
    // const std::string sdf_url = "/home/darin/Github/drake/flapgood/models/four_bar.sdf";
    // const std::string sdf_url = "/home/darin/Github/drake/flapgood/models/four_bar_wield.sdf";
    Parser parser(&four_bar);
    parser.AddModels(sdf_url);

    auto meshcat = std::make_shared<drake::geometry::Meshcat>(7001);
    auto& meshcat_visualizer = drake::geometry::MeshcatVisualizer<double>::AddToBuilder(
        &builder, scene_graph, meshcat, drake::geometry::MeshcatVisualizerParams());

    // Retrieve the two frames for the bushing.
    // const auto& frame_Hr = four_bar.GetFrameByName("humerus_radial_bushing");
    // const auto& frame_Rh = four_bar.GetFrameByName("radial_humerus_bushing");
    const auto& frame_Hr = four_bar.GetFrameByName("Bc_bushing");
    const auto& frame_Rh = four_bar.GetFrameByName("Cb_bushing");

    const auto& frame_Bf = four_bar.GetFrameByName("Bf_bushing");
    const auto& frame_Fb = four_bar.GetFrameByName("Fb_bushing");

    // Define stiffness and damping constants (using the same value for each axis).
    const double k_xyz = FLAGS_force_stiffness;
    const double d_xyz = FLAGS_force_damping;
    const double k_rpy = FLAGS_torque_stiffness;
    const double d_rpy = FLAGS_torque_damping;

    // const double k_xyz = 1e8;
    // const double d_xyz = 1e6;
    // const double k_rpy = 1e5;
    // const double d_rpy = 1e5;

    // For this demo we assume that only a revolute (z-axis) degree-of-freedom is active.
    // Thus, we choose nonzero stiffness/damping only for the first two rotational axes.
    const Vector3d force_stiffness_constants{k_xyz, k_xyz, k_xyz}; // N/m
    const Vector3d force_damping_constants{d_xyz, d_xyz, d_xyz};   // N·s/m
    const Vector3d torque_stiffness_constants{k_rpy, k_rpy, 0};
    const Vector3d torque_damping_constants{d_rpy, d_rpy, 0}; // N·m·s/rad

    // Add the linear bushing force element to model the kinematic loop.
    four_bar.AddForceElement<LinearBushingRollPitchYaw>(frame_Hr, frame_Rh, torque_stiffness_constants,
                                                        torque_damping_constants, force_stiffness_constants,
                                                        force_damping_constants);

    four_bar.AddForceElement<LinearBushingRollPitchYaw>(frame_Fb, frame_Bf, torque_stiffness_constants,
                                                        torque_damping_constants, force_stiffness_constants,
                                                        force_damping_constants);

    // Finalize the MultibodyPlant.
    four_bar.Finalize();

    // requires plant finalization, we get data
    const auto& state_output = four_bar.get_state_output_port();
    auto state_logger = drake::systems::LogVectorOutput(state_output, &builder);
    state_logger->set_name("state_logger");

    // const auto& world_poses_output = four_bar.get_body_poses_output_port();
    // auto world_pos_logger = drake::systems::LogVectorOutput(world_poses_output, &builder);

    const auto& world_velocities_output = four_bar.get_body_spatial_velocities_output_port();
    const BodyIndex wing_body_index = four_bar.GetBodyByName("G").index();
    auto world_vel_logger = drake::systems::LogVectorOutput(world_velocities_output, &builder);

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
    // const RevoluteJoint<double>& driving_joint = four_bar.GetJointByName<RevoluteJoint>("driving_joint");
    // const RevoluteJoint<double>& joint_AB = four_bar.GetJointByName<RevoluteJoint>("humerus_joint");
    // const RevoluteJoint<double>& joint_WC = four_bar.GetJointByName<RevoluteJoint>("mid_radial_joint");

    // Initialize joint angles.
    // Here we choose the angles so that driving_joint ≈ 75.52°, joint_AB and joint_WC ≈ 104.48°.
    // const double qA = std::atan2(std::sqrt(15.0), 1.0);
    // const double qB = M_PI - qA;
    // const double qC = qB;
    // const double qC = 0.0383 * std::sin(0.261799);
    // driving_joint.set_angle(&plant_context, 0.261799);
    // joint_AB.set_angle(&plant_context, -0.261799);
    // joint_WC.set_angle(&plant_context, 0.261799);

    // driving_joint.set_angle(&plant_context, qA);
    // // Set the initial angular rate for driving_joint.
    // joint_WC.set_angular_rate(&plant_context, FLAGS_initial_velocity);
    // // Initialize joint angles.
    // // Here we choose the angles so that joint_WA ≈ 75.52°, joint_AB and joint_WC ≈ 104.48°.
    // const double qA = std::atan2(std::sqrt(15.0), 1.0);
    // const double qB = M_PI - qA;
    // const double qC = qB;

    // joint_WA.set_angle(&plant_context, qA);
    // joint_AB.set_angle(&plant_context, qB);
    // joint_WC.set_angle(&plant_context, qC);

    // // Set the initial angular rate for joint_WA.
    // joint_WA.set_angular_rate(&plant_context, FLAGS_initial_velocity);
    // // Set the initial angular rate for driving_joint.
    // joint_WC.set_angular_rate(&plant_context, FLAGS_initial_velocity);

    // fourbar
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
    simulator.set_target_realtime_rate(1);
    simulator.reset_integrator<drake::systems::RungeKutta2Integrator<double>>(1e-4);
    // simulator.reset_integrator<drake::systems::ImplicitEulerIntegrator<double>>(  *diagram,
    // &simulator.get_mutable_context(), 1e-4);
    meshcat_visualizer.StartRecording();
    simulator.Initialize();
    simulator.AdvanceTo(FLAGS_simulation_time);
    // meshcat

    // Print simulation statistics.
    drake::systems::PrintSimulatorStatistics(simulator);

    // Optionally, print the simulation end time.
    auto end_time = std::chrono::high_resolution_clock::now();
    std::cout << "Simulation ended at time: " << end_time.time_since_epoch().count() << std::endl;

    meshcat_visualizer.PublishRecording();

    // logger plot of states
    // const auto sim_context = simulator.get_context();
    // const auto& state_logs = state_logger->FindLog(sim_context);
    // std::ofstream file("/home/darin/Github/drake/flapgood/four_bar.csv");
    // // Check the number of states and samples
    // const int num_states = state_logs.data().rows();  // Number of state variables (e.g., joint positions,
    // velocities) const int num_samples = state_logs.num_samples(); // Number of logged timesteps

    // // Print the dimensions of the log
    // std::cout << "Logged " << num_samples << " samples with " << num_states << " state variables each." << std::endl;

    // // Write header
    // file << "time";
    // file << "\n";

    // // Loop through and write data
    // for (int i = 0; i < num_samples; ++i)
    // {
    //     file << state_logs.sample_times()(i); // Time at sample i
    //     for (int j = 0; j < num_states; ++j)
    //     {
    //         file << ", " << state_logs.data()(j, i); // State value at (row j, col i)
    //     }
    //     file << "\n";
    // }

    // // Close the file
    // file.close();
    // std::cout << "Log written to four_bar.csv" << std::endl;
    // std::cout << "Number of generalized positions: " << four_bar.num_positions() << std::endl;
    // std::cout << "Number of generalized velocities: " << four_bar.num_velocities() << std::endl;
    // std::cout << "Total state variables: " << four_bar.num_multibody_states() << std::endl;
    // std::vector<std::string> velocity_names = four_bar.GetVelocityNames();
    // std::vector<std::string> position_names = four_bar.GetPositionNames();
    // std::cout << "Velocity Names:\n";
    // for (const std::string& name : velocity_names)
    // {
    //     std::cout << name << std::endl;
    // }
    // std::cout << "Position Names:\n";
    // for (const std::string& name : position_names)
    // {
    //     std::cout << name << std::endl;
    // }

    // logger plot of world velocities
    const auto& world_vel_logs = world_vel_logger->FindLog(simulator.get_context());
    std::ofstream file("/home/darin/Github/drake/flapgood/four_bar_vel_end.csv");
    // Check the number of states and samples
    const int num_states = world_vel_logs.data().rows(); // Number of state variables (e.g., joint positions, velocities)
    const int num_samples = world_vel_logs.num_samples(); // Number of logged timesteps
    std::cout << "Wing body index: " << wing_body_index << std::endl;
    // Print the dimensions of the log
    std::cout << "Logged " << num_samples << " samples with " << num_states << " pose variables each." << std::endl;
    four_bar.GetVelocityNames();
    // Write header
    file << "time, vx, vy, vz, wx, wy, wz\n";

    int start_index = 6 * wing_body_index;
    for (int i = 0; i < world_vel_logs.num_samples(); i++)
    {
        double time = world_vel_logs.sample_times()(i);
        file << time;
        Eigen::Vector3d linear_velocity = world_vel_logs.data().col(i).segment<3>(start_index);
        Eigen::Vector3d angular_velocity = world_vel_logs.data().col(i).segment<3>(start_index + 3);

        file << ", " << linear_velocity.x() << ", " << linear_velocity.y() << ", " << linear_velocity.z();
        file << ", " << angular_velocity.x() << ", " << angular_velocity.y() << ", " << angular_velocity.z();
        file << "\n";
        
    }

    // Close the file
    file.close();
    std::cout << "Log written to four_bar_vel.csv" << std::endl;

    // (Optionally, keep the process alive so that the Meshcat visualization remains open.)
    while (true)
    {
    }

    return 0;
}

} // namespace
} // namespace four_bar
} // namespace multibody
} // namespace drake

// Main entrypoint: parse flags and call our simulation main function.
int main(int argc, char* argv[])
{
    gflags::SetUsageMessage(
        "A four-bar linkage demo demonstrating the use of a linear bushing to model a kinematic loop.");
    FLAGS_simulator_target_realtime_rate = 1.0; // Ensures visualization is realistic.
    gflags::ParseCommandLineFlags(&argc, &argv, true);
    return drake::multibody::four_bar::DoMain();
}
