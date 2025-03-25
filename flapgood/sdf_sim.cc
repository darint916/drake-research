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
#include "drake/systems/framework/leaf_system.h"
#include "drake/visualization/visualization_config_functions.h"

#include "drake/systems/controllers/pid_controller.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/primitives/matrix_gain.h"

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
DEFINE_double(simulation_time, 15, "Duration of the simulation in seconds.");
DEFINE_double(force_stiffness, 10000000,
              "Translational stiffness (N/m) for the LinearBushingRollPitchYaw force element.");
DEFINE_double(force_damping, 100000, "Translational damping (N·s/m) for the LinearBushingRollPitchYaw force element.");
DEFINE_double(torque_stiffness, 300000,
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

class ScalarTo2Vector : public drake::systems::LeafSystem<double>
{
  public:
    ScalarTo2Vector()
    {
        this->DeclareVectorInputPort("scalar", 1);
        this->DeclareVectorOutputPort("vector", 2, &ScalarTo2Vector::CalcOutput);
    }

  private:
    void CalcOutput(const drake::systems::Context<double>& context, drake::systems::BasicVector<double>* output) const
    {
        output->SetAtIndex(0, 0); // use pos 0
        const auto& input_vector = this->EvalVectorInput(context, 0)->get_value();
        output->SetAtIndex(1, input_vector(0));
    }
};

class BodySpatialVelocitiesToVector : public drake::systems::LeafSystem<double>
{
  public:
    // In a real implementation, pass the number of bodies as a parameter.
    BodySpatialVelocitiesToVector(int num_bodies) : num_bodies_(num_bodies)
    {
        this->DeclareAbstractInputPort("body_spatial_velocities",
                                       drake::Value<std::vector<drake::multibody::SpatialVelocity<double>>>());
        // Each body contributes 6 numbers (3 translational + 3 rotational).
        this->DeclareVectorOutputPort("body_spatial_velocities_to_vector", 6 * num_bodies,
                                      &BodySpatialVelocitiesToVector::OutputVector);
    }

  private:
    int num_bodies_;

    void OutputVector(const drake::systems::Context<double>& context, drake::systems::BasicVector<double>* output) const
    {
        const auto& abstract_val = this->EvalAbstractInput(context, 0);
        const auto& spatial_velocities =
            abstract_val->get_value<std::vector<drake::multibody::SpatialVelocity<double>>>();

        // For safety, you might want to check that spatial_velocities.size() == num_bodies_
        Eigen::VectorXd vec(6 * num_bodies_);
        for (int i = 0; i < num_bodies_; ++i)
        {
            // Extract the translational (first 3) and rotational (next 3) components.
            vec.segment<3>(6 * i) = spatial_velocities[i].translational();
            vec.segment<3>(6 * i + 3) = spatial_velocities[i].rotational();
        }
        output->set_value(vec);
    }

}; // namespace

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
    const std::string sdf_url = "/home/darin/Github/drake/flapgood/models/four_more_iterated.sdf";
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

    const auto& abstract_world_velocities_output = four_bar.get_body_spatial_velocities_output_port();
    int num_bodies = four_bar.num_bodies();
    auto converter = builder.AddSystem<BodySpatialVelocitiesToVector>(num_bodies);
    builder.Connect(abstract_world_velocities_output, converter->get_input_port());

    const auto& world_velocities_output = converter->get_output_port();
    const BodyIndex wing_body_index = four_bar.GetBodyByName("G").index();
    auto world_vel_logger = drake::systems::LogVectorOutput(world_velocities_output, &builder);

    // Add pid controller
    const RevoluteJoint<double>& joint_WA = four_bar.GetJointByName<RevoluteJoint>("joint_WA");
    int state_dim = four_bar.num_multibody_states();
    Eigen::RowVectorXd extract_vec = Eigen::RowVectorXd::Zero(state_dim); // all zeros to mult and extract
    int driving_index = joint_WA.velocity_start();
    std::cout << "driving index: " << driving_index << std::endl;
    std::cout << "straight idx: " << joint_WA.index() << std::endl;
    std::cout << "state dim: " << state_dim << std::endl;
    std::cout << "num_pos: " << four_bar.num_positions() << std::endl;
    int velocity_index = four_bar.num_positions() + joint_WA.velocity_start();
    extract_vec(velocity_index) = 1;
    auto velocity_extractor = builder.AddSystem<drake::systems::MatrixGain<double>>(extract_vec);
    builder.Connect(four_bar.get_state_output_port(), velocity_extractor->get_input_port());

    //need to remap to a 2d vector to enter to the PID controller
    auto state2d = builder.AddSystem<ScalarTo2Vector>();
    state2d->set_name("state_2d");
    builder.Connect(velocity_extractor->get_output_port(), state2d->get_input_port());
    
    //target
    const double desired_speed = -20.0; // 60 rad/s too fast, lowered donw
    Eigen::Vector2d desired_state;
    desired_state << 0, desired_speed;
    auto desired_speed_source =
        builder.AddSystem<drake::systems::ConstantVectorSource<double>>(desired_state);

    Eigen::VectorXd kp = Eigen::VectorXd::Constant(1, 1000000);
    Eigen::VectorXd ki = Eigen::VectorXd::Constant(1, 10000.0);
    Eigen::VectorXd kd = Eigen::VectorXd::Constant(1, 50000);
    //the actual controller
    auto pid_controller = builder.AddSystem<drake::systems::controllers::PidController<double>>(kp, ki, kd);
    pid_controller->set_name("PIDController");
    builder.Connect(state2d->get_output_port(), pid_controller->get_input_port_estimated_state());
    builder.Connect(pid_controller->get_output_port(), four_bar.get_actuation_input_port());
    builder.Connect(desired_speed_source->get_output_port(), pid_controller->get_input_port_desired_state());

    //log controller
    auto pid_controller_logger = drake::systems::LogVectorOutput(pid_controller->get_output_port(), &builder);
    pid_controller_logger->set_name("pid_controller_logger");


    // Add default visualization (which sets up Meshcat if available).
    drake::visualization::AddDefaultVisualization(&builder);

    // Build the complete system diagram.
    auto diagram = builder.Build();

    // Create a context for the diagram and extract the subcontext for the MultibodyPlant.
    std::unique_ptr<drake::systems::Context<double>> diagram_context = diagram->CreateDefaultContext();
    auto& plant_context = four_bar.GetMyMutableContextFromRoot(diagram_context.get());

    // Apply a constant torque at joint_WA.
    // four_bar.get_actuation_input_port().FixValue(&plant_context, FLAGS_applied_torque);

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
    // const RevoluteJoint<double>& joint_WC = four_bar.GetJointByName<RevoluteJoint>("joint_WC");
    // const RevoluteJoint<double>& joint_WC = four_bar.GetJointByName<RevoluteJoint>("joint_WC");

    // Initialize joint angles.
    // Here we choose the angles so that joint_WA ≈ 75.52°, joint_AB and joint_WC ≈ 104.48°.
    // const double qA = std::atan2(std::sqrt(15.0), 1.0);
    // const double qB = M_PI - qA;
    // const double qC = qB;
    // const double qA = 0;
    // const double qB = 0;
    // const double qC = 0;
    joint_WA.set_angle(&plant_context, M_PI);
    // joint_WC.set_angle(&plant_context, M_PI);
    // joint_WC.set_angle(&plant_context, qC);

    // Set the initial angular rate for joint_WA.
    joint_WA.set_angular_rate(&plant_context, 0);

    // Optionally, one might record the simulation start time.
    auto start_time = std::chrono::high_resolution_clock::now();
    std::cout << "Starting simulation at time: " << start_time.time_since_epoch().count() << std::endl;

    // Create and run the simulator.
    Simulator<double> simulator(*diagram, std::move(diagram_context));
    simulator.set_target_realtime_rate(1);
    simulator.reset_integrator<drake::systems::RungeKutta2Integrator<double>>(1e-4);
    // simulator.reset_integrator<drake::systems::ImplicitEulerIntegrator<double>>(  *diagram,
    // &simulator.get_mutable_context(), 1e-4);
    meshcat_visualizer.StartRecording(128);
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
    std::ofstream file("/home/darin/Github/drake/flapgood/opt_data/four_bar_vel_end.csv");
    // Check the number of states and samples
    const int num_states =
        world_vel_logs.data().rows(); // Number of state variables (e.g., joint positions, velocities)
    const int num_samples = world_vel_logs.num_samples(); // Number of logged timesteps
    std::cout << "Wing body index: " << wing_body_index << std::endl;
    // Print the dimensions of the log
    std::cout << "Logged " << num_samples << " samples with " << num_states << " pose variables each." << std::endl;
    four_bar.GetVelocityNames();
    // Write header
    file << "time,vx,vy,vz,wx,wy,wz\n";

    int start_index = 6 * wing_body_index;
    for (int i = 0; i < world_vel_logs.num_samples(); i++)
    {
        double time = world_vel_logs.sample_times()(i);
        file << time;
        Eigen::Vector3d linear_velocity = world_vel_logs.data().col(i).segment<3>(start_index);
        Eigen::Vector3d angular_velocity = world_vel_logs.data().col(i).segment<3>(start_index + 3);

        file << "," << linear_velocity.x() << "," << linear_velocity.y() << "," << linear_velocity.z();
        file << "," << angular_velocity.x() << "," << angular_velocity.y() << "," << angular_velocity.z();
        file << "\n";
    }

    // Close the file
    file.close();
    std::cout << "Log written to four_bar_vel.csv" << std::endl;
    //--------------------------------------------------------------------------------
    //--------------------------------------------------------------------------------
    // wingtip vel
    std::ofstream file_tip("/home/darin/Github/drake/flapgood/opt_data/four_bar_vel_tip.csv");
    // Write header
    file_tip << "time,vx,vy,vz,wx,wy,wz\n";
    const BodyIndex wing_tip_index = four_bar.GetBodyByName("H").index();
    std::cout << "Wing body index: " << wing_tip_index << std::endl;

    start_index = 6 * wing_tip_index;
    for (int i = 0; i < world_vel_logs.num_samples(); i++)
    {
        double time = world_vel_logs.sample_times()(i);
        file_tip << time;
        Eigen::Vector3d linear_velocity = world_vel_logs.data().col(i).segment<3>(start_index);
        Eigen::Vector3d angular_velocity = world_vel_logs.data().col(i).segment<3>(start_index + 3);

        file_tip << "," << linear_velocity.x() << "," << linear_velocity.y() << "," << linear_velocity.z();
        file_tip << "," << angular_velocity.x() << "," << angular_velocity.y() << "," << angular_velocity.z();
        file_tip << "\n";
    }
    file_tip.close();
    std::cout << "Log written to four_bar_vel_tip.csv" << std::endl;

    //PID controller log
    std::ofstream file_pid("/home/darin/Github/drake/flapgood/opt_data/four_bar_pid.csv");
    file_pid << "time,torque\n";
    const auto& pid_logs = pid_controller_logger->FindLog(simulator.get_context());
    for (int i = 0; i < pid_logs.num_samples(); i++)
    {
        file_pid << pid_logs.sample_times()(i) << "," << pid_logs.data().col(i)(0) << "\n";
    }
    file_pid.close();
    std::cout << "Log written to four_bar_pid.csv" << std::endl;


    // logger for wing tip angles off fixed
    //create 3d vector (1.5004972495905828, 0, 5.127977866954097)
    Eigen::Vector3d fixed_pos(1.5004972495905828, 0, 5.127977866954097);
    Eigen::Vector2d x_axis(1, 0);
    std::ofstream file_tip_angle("/home/darin/Github/drake/flapgood/opt_data/four_bar_tip_angle.csv");
    file_tip_angle << "time,angle\n";

    auto& angle_context = simulator.get_mutable_context();
    auto& plant_context_log = four_bar.GetMyMutableContextFromRoot(&angle_context);
    const auto& state_log = state_logger->FindLog(simulator.get_context());
    const Body<double>& wing_tip = four_bar.GetBodyByName("H");
    for (int i = 0; i < num_samples; i++)
    {
        double time = world_vel_logs.sample_times()(i);
        const Eigen::VectorXd& x = state_log.data().col(i);
        angle_context.SetTime(time);
        four_bar.SetPositionsAndVelocities(&plant_context_log, x);

        const math::RigidTransformd& tip_pose = four_bar.EvalBodyPoseInWorld(plant_context_log, wing_tip);
        const Eigen::Vector3d& tip_pos = tip_pose.translation();
        Eigen::Vector2d tip_to_fixed(tip_pos.x() - fixed_pos.x(), tip_pos.z() - fixed_pos.z());
        double angle_rad = std::atan2(tip_to_fixed.y(), tip_to_fixed.x());
        double angle_deg = angle_rad * 180 / M_PI;
        file_tip_angle << time << "," << angle_deg << "\n";
    }
    file_tip_angle.close();
    std::cout << "Log written to four_bar_tip_angle.csv" << std::endl;
    // // (Optionally, keep the process alive so that the Meshcat visualization remains open.)
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