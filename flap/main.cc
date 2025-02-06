//Entrypoint
#include "aerodynamics.h"
#include <gflags/gflags.h>
#include <iostream>
#include <filesystem>
#include <chrono>
#include <yaml-cpp/yaml.h>
#include <drake/math/rotation_matrix.h>
#include "drake/geometry/scene_graph.h"
#include <drake/geometry/meshcat.h>
#include "drake/geometry/meshcat_visualizer.h"
#include <drake/geometry/meshcat_visualizer_params.h>
#include <drake/multibody/plant/multibody_plant.h>
#include "drake/multibody/parsing/parser.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/leaf_system.h"
// #include <drake/systems/primitives/constant_value_source.h>
#include "drake/visualization/visualization_config_functions.h"
#include <drake/systems/primitives/multiplexer.h>
#include <drake/systems/primitives/sine.h>
#include "drake/math/rigid_transform.h"
// "multibody" namespace is ambiguous here without "drake::".
using drake::multibody::AddMultibodyPlantSceneGraph;
using drake::multibody::Parser;
using drake::math::RigidTransform;
// using drake::systems::Multiplexer;
namespace drake {
namespace flapping {

using Eigen::Vector3d;
//DRAKE issues: with builder at 0 seconds, continuous system, some joints fail and fly drops
//Can add define constants
int DoMain() {
    YAML::Node config = YAML::LoadFile("/home/darin/Github/drake/flap/aerodynamics_config.yaml");
    drake::systems::DiagramBuilder<double> builder;
    // std::unique_ptr<drake::geometry::SceneGraph<double>> scene_graph = nullptr;
    auto result = AddMultibodyPlantSceneGraph(&builder, config["time_step"].as<double>());
    // auto result = AddMultibodyPlantSceneGraph(&builder, 0.0);
    drake::multibody::MultibodyPlant<double>& plant = result.plant;
    drake::geometry::SceneGraph<double>& scene_graph = result.scene_graph;
    plant.set_name("plant");


    //new meshcat instance
    auto meshcat = std::make_shared<drake::geometry::Meshcat>(7001);

    auto& meshcat_visualizer = drake::geometry::MeshcatVisualizer<double>::AddToBuilder(&builder, scene_graph, meshcat, drake::geometry::MeshcatVisualizerParams());
    
    // auto& meshcat_visualizer = *builder.AddSystem<drake::geometry::MeshcatVisualizer<double>>(meshcat, drake::geometry::MeshcatVisualizerParams()); 

    // std::cout << "Current path is " << current_path << std::endl;
    const std::string sdf_url = "package://drake/flap/URDF_LargeWings/urdf/URDF_LargeWings.sdf";
    // std::filesystem::path sdf_package_path = current_path.parent_path() / "URDF_LargeWings";
    Parser parser(&plant, &scene_graph);

    std::filesystem::path source_file_path(__FILE__);
    std::cout << "current path: " << source_file_path << std::endl;
    parser.package_map().Add("URDF_LargeWings", source_file_path.parent_path() / "URDF_LargeWings");
    
    parser.AddModelsFromUrl(sdf_url);
    
    //check plant jonts
    std::cout << "plant joints: " << plant.num_joints() << std::endl;
    // std::cout << "Added models from " << sdf_url << std::endl;
    // for (const std::string& package_name : parser.package_map().GetPackageNames()) {
    //     std::cout << "Package name: " << package_name << std::endl;
    // }
    // std::cout << "package names: " << parser.package_map().GetPackageNames() << std::endl;
 

    //checking coordinate locations
    auto& left_wing_body = plant.GetBodyByName("RW_Pitch");
    auto center_pressure_body_rw = Vector3<double>(0.00833, -0.025, 0);
    // size_t i = 0;
    Vector3<double> midpoint = center_pressure_body_rw / 2.0;
    double length = center_pressure_body_rw.norm();
    RigidTransform<double> cylinder_pose = RigidTransform<double>::Identity();
    cylinder_pose.set_translation(midpoint);
    drake::geometry::Cylinder cylinder_shape(0.001, length);

    Vector3<double> z_axis(0, 0, 1);
    Vector3<double> direction = center_pressure_body_rw.normalized();
    drake::math::RotationMatrix<double> rotation = drake::math::RotationMatrix<double>::MakeFromOneVector(direction, 2);
    cylinder_pose.set_rotation(rotation);

    plant.RegisterVisualGeometry(left_wing_body, cylinder_pose, cylinder_shape, "wing_cylinder_1" , Vector4<double>(0, 1, 0, 0.5));
    
    center_pressure_body_rw = Vector3<double>(0.14161, -0.025, 0);
    midpoint = center_pressure_body_rw / 2.0;
    length = center_pressure_body_rw.norm();
    cylinder_pose = RigidTransform<double>::Identity();
    cylinder_pose.set_translation(midpoint);
    drake::geometry::Cylinder cylinder_shape2(0.001, length);

    direction = center_pressure_body_rw.normalized();
    rotation = drake::math::RotationMatrix<double>::MakeFromOneVector(direction, 2);
    cylinder_pose.set_rotation(rotation);

    plant.RegisterVisualGeometry(left_wing_body, cylinder_pose, cylinder_shape2, "wing_cylinder_2" , Vector4<double>(0, 1, 0, 0.5));
    
    auto& right_wing_joint_link = plant.GetBodyByName("RW_Flap");
    
    // auto& gravity_field = plant.mutable_gravity_field();
    // gravity_field.set_gravity_vector(Vector3<double>(0, 0, -.00000981));
    plant.Finalize();
    std::cout << "plant finalized" << std::endl;
    {
        auto& aerodynamics_system = *builder.AddSystem<drake::flap::Aerodynamics>(plant);
        builder.Connect(aerodynamics_system.get_output_port(), plant.get_applied_spatial_force_input_port());
        builder.Connect(plant.get_body_poses_output_port(), aerodynamics_system.get_input_port(0));
        builder.Connect(plant.get_body_spatial_velocities_output_port(), aerodynamics_system.get_input_port(1));
        std::cout << "added aerodynamics" << std::endl;
        //Add aerodynamic logger later
        
        //connect sine waves

        // auto sine1 = builder.AddSystem<drake::systems::Sine>(3.0, 30.0, 1.0, 1.0);
        // auto sine2 = builder.AddSystem<drake::systems::Sine>(-3.0, 30.0, 1.0, 1.0);

        auto sine1 = builder.AddSystem<drake::systems::Sine>(
            config["sine_lw"]["amplitude"].as<double>(),
            config["sine_lw"]["frequency"].as<double>() * 6.28,
            config["sine_lw"]["phase"].as<double>(),
            1.0); //ports
        auto sine2 = builder.AddSystem<drake::systems::Sine>(
            -config["sine_rw"]["amplitude"].as<double>(),
            config["sine_rw"]["frequency"].as<double>() * 6.28, // converts hertz to radians/sec
            config["sine_rw"]["phase"].as<double>(),
            1.0);
        // auto sine1 = builder.AddSystem<drake::systems::Sine>(10.0, 1.0, 1.0, 1.0);
        // auto sine2 = builder.AddSystem<drake::systems::Sine>(-10.0, 1.0, 1.0, 1.0);
        std::cout << "added sine waves" << std::endl;
        auto mux = builder.AddSystem<drake::systems::Multiplexer<double>>(std::vector<int>{1, 1});
        std::cout << "added systems" << std::endl;
        // Connect the Sine systems to the Multiplexer
        builder.Connect(sine1->get_output_port(0), mux->get_input_port(0));
        builder.Connect(sine2->get_output_port(0), mux->get_input_port(1));
        builder.Connect(mux->get_output_port(0), plant.get_actuation_input_port());
        std::cout << "connected systems" << std::endl;
        auto diagram = builder.Build();
        
        std::cout << "built diagram" << std::endl;
        drake::systems::Simulator<double> simulator(*diagram);
        simulator.set_target_realtime_rate(1.0);
        meshcat_visualizer.StartRecording();
        simulator.Initialize();
        simulator.AdvanceTo(config["simulation_length"].as<double>());
        // double sim_length = config["simulation_length"].as<double>();
        // double time_step = config["time_step"].as<double>();
        // int num_steps = static_cast<int>(sim_length / time_step);
        // std::vector<std::vector<double>> joint_angles;
        // for (int i = 0; i < num_steps; i++){
        //     const auto& sim_context = simulator.get_context();
        //     std::vector<double> curr_joint_angles;
        //     plant.Get
        // }
        std::chrono::high_resolution_clock::time_point end_time = std::chrono::high_resolution_clock::now();
        std::cout << "End time: " << end_time.time_since_epoch().count() << std::endl;
        meshcat_visualizer.PublishRecording();
    } //allows sim to run while enabling destruction of aerdynamics system to data dump
    while(1){
        
    }
    return 0;
}
int main(int argc, char* argv[]) {
    gflags::ParseCommandLineFlags(&argc, &argv, true);
    return drake::flapping::DoMain();
}
}
}
