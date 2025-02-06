#include "aerodynamics.h"

namespace drake {
namespace flap {

// struct ConfigData{
//     std::vector<Vector3d> center_pressure_body_lw_;

// };

Aerodynamics::Aerodynamics(const MultibodyPlant<double>& plant) 
    : LeafSystem<double>(), plant_(plant) {
    // this->DeclareAbstractInputPort("body_spatial_velocities", 
    //                                Value<std::vector<SpatialVelocity<double>>>());
    // this->DeclareAbstractInputPort("body_poses", 
    //                                Value<std::vector<RigidTransform<double>>>());
    // std::vector<drake::multibody::ExternallyAppliedSpatialForce<double>> model_value;
    // this->DeclareAbstractOutputPort("force", model_value, &Aerodynamics::CalcOutput);
    // this->DeclareAbstractInputPort

    this->DeclareAbstractInputPort("body_poses", Value<std::vector<RigidTransform<double>>>()); //port 0
    this->DeclareAbstractInputPort("body_spatial_velocities", Value<std::vector<SpatialVelocity<double>>>()); //port 1
    this->DeclareAbstractOutputPort("force", &Aerodynamics::CalcOutput);
    // const double periodic_sec = 0.01; // 10 ms, 100 Hz
    // const double offset_sec = 0.0;
    // this->DeclarePeriodicPublishEvent(periodic_sec, offset_sec, &Aerodynamics::CalcOutput);
    // std::cout << "start" << std::endl;
    right_wing_body_ = &plant_.GetBodyByName("RW_Pitch");
    left_wing_body_ = &plant_.GetBodyByName("LW_Pitch");
    right_wing_joint_link_ = &plant_.GetBodyByName("RW_Flap");
    
    main_body_ = &plant_.GetBodyByName("base_link");
    main_body_index_ = main_body_->index();

    right_wing_passive_joint_ = &plant_.GetJointByName<drake::multibody::RevoluteJoint>("joint_RW_J_Pitch");
    // right_wing_passive_joint_ = &(plant_.GetJointByName<drake::multibody::RevoluteJoint>("joint_RW_J_Pitch"));
    // left_wing_passive_joint_ = &plant_.GetJointByName("joint_LW_J_Pitch");
    right_wing_body_index_ = right_wing_body_->index();
    left_wing_body_index_ = left_wing_body_->index();
    right_wing_joint_link_index_ = right_wing_joint_link_->index();

    // left_wing_body_index_ = plant_.GetBodyIndices(left_wing_body_->index())[0];
    orthogonal_vec_ = Vector3<double>(0, 0, 1);

    YAML::Node config = YAML::LoadFile("/home/darin/Github/drake/flap/aerodynamics_config.yaml");
    for (const auto& cp_node : config["center_pressure_body_rw"]) {
        center_pressure_body_rw_.emplace_back(cp_node["x"].as<double>(), cp_node["y"].as<double>(), cp_node["z"].as<double>());
    }
    for (const auto& cp_node : config["center_pressure_body_lw"]) {
        center_pressure_body_lw_.emplace_back(cp_node["x"].as<double>(), cp_node["y"].as<double>(), cp_node["z"].as<double>());
    }

    // Initialize center_pressure_body_lw_ and center_pressure_body_rw_
    // center_pressure_body_lw_ = {
    //     Vector3<double>(-0.00833, -0.025, 0),
    //     Vector3<double>(-0.02499, -0.025, 0),
    //     Vector3<double>(-0.04165, -0.025, 0),
    //     Vector3<double>(-0.05831, -0.025, 0),
    //     Vector3<double>(-0.07497, -0.025, 0),
    //     Vector3<double>(-0.09163, -0.025, 0),
    //     Vector3<double>(-0.10829, -0.025, 0),
    //     Vector3<double>(-0.12495, -0.025, 0),
    //     Vector3<double>(-0.14161, -0.025, 0)
    // };
    // const ConfigData config_data = yaml::LoadYamlFile<ConfigData>  //sucks, need to serialize advanced types
    // center_pressure_body_rw_ = {
    //     Vector3<double>(0.00833, -0.025, 0),
    //     Vector3<double>(0.02499, -0.025, 0),
    //     Vector3<double>(0.04165, -0.025, 0),
    //     Vector3<double>(0.05831, -0.025, 0),
    //     Vector3<double>(0.07497, -0.025, 0),
    //     Vector3<double>(0.09163, -0.025, 0),
    //     Vector3<double>(0.10829, -0.025, 0),
    //     Vector3<double>(0.12495, -0.025, 0),
    //     Vector3<double>(0.14161, -0.025, 0)
    // };

    blade_area_ = 0.00765;
    cp_size_ = center_pressure_body_rw_.size();
    blade_area_list_lw_.assign(cp_size_, blade_area_ / cp_size_);
    blade_area_list_rw_.assign(cp_size_, blade_area_ / cp_size_);
    air_density_ = 1.293;
    drag_coef_ = 1.28;
    // std::cout << "thread" << std::endl;
    num_threads_ = std::min(static_cast<size_t>((std::thread::hardware_concurrency())), cp_size_ / 3); 
    // batch_size_ = center_pressure_body_lw_.size() / num_threads_;
    counter = 0;
    rad_to_deg = 180.0 / M_PI;
    active_buffer_ = &data_buffer_1_;
    // background_buffer_ = &data_buffer_2_; 
    // data_file_ = std::ofstream("package://drake/flap/data/aerodynamics_data.csv");
    data_file_ = std::ofstream("/home/darin/Github/drake/flap/data/aerodynamics_data.csv");
    if (!data_file_.is_open()) {
        throw std::runtime_error("file not open");
    }
    std::vector<std::string> headers = {
        "time",
        "rw_origin_x", "rw_origin_y", "rw_origin_z",
        "rw_rot_0_0", "rw_rot_1_0", "rw_rot_2_0",
        "rw_rot_0_1", "rw_rot_1_1", "rw_rot_2_1",
        "rw_rot_0_2", "rw_rot_1_2", "rw_rot_2_2",
        "rw_joint_link_origin_x", "rw_joint_link_origin_y", "rw_joint_link_origin_z",
        "rw_joint_link_rot_0_0", "rw_joint_link_rot_1_0", "rw_joint_link_rot_2_0",
        "rw_joint_link_rot_0_1", "rw_joint_link_rot_1_1", "rw_joint_link_rot_2_1",
        "rw_joint_link_rot_0_2", "rw_joint_link_rot_1_2", "rw_joint_link_rot_2_2",
        "rw_orthogonal_vector_x", "rw_orthogonal_vector_y", "rw_orthogonal_vector_z",
        "rw_origin_velocity_trans_x", "rw_origin_velocity_trans_y", "rw_origin_velocity_trans_z",
        "rw_origin_velocity_rot_x", "rw_origin_velocity_rot_y", "rw_origin_velocity_rot_z",
    };
    for(const auto& header : headers) {
        data_file_ << header << ",";
    }
    for (size_t i = 0; i < cp_size_; i++) {
        data_file_ << "rw_cp_velocity_x_" + std::to_string(i) << ",";
        data_file_ << "rw_cp_velocity_y_" + std::to_string(i) << ",";
        data_file_ << "rw_cp_velocity_z_" + std::to_string(i) << ",";
        data_file_ << "rw_cp_orthogonal_speed_" + std::to_string(i) << ",";
        data_file_ << "rw_cp_drag_scalar_" + std::to_string(i) << ",";
        data_file_ << "rw_cp_force_vector_x_" + std::to_string(i) << ",";
        data_file_ << "rw_cp_force_vector_y_" + std::to_string(i) << ",";
        data_file_ << "rw_cp_force_vector_z_" + std::to_string(i) << ",";
    }
    std::vector<std::string> lw_header = {
        "lw_origin_x", "lw_origin_y", "lw_origin_z",
        "lw_orthogonal_vector_x", "lw_orthogonal_vector_y", "lw_orthogonal_vector_z",
        "lw_origin_velocity_trans_x", "lw_origin_velocity_trans_y", "lw_origin_velocity_trans_z",
        "lw_origin_velocity_rot_x", "lw_origin_velocity_rot_y", "lw_origin_velocity_rot_z"
    };
    for (const auto& header : lw_header) {
        data_file_ << header << ",";
    }
    for (size_t i = 0; i < cp_size_; i++) {
        data_file_ << "lw_cp_velocity_x_" + std::to_string(i) << ",";
        data_file_ << "lw_cp_velocity_y_" + std::to_string(i) << ",";
        data_file_ << "lw_cp_velocity_z_" + std::to_string(i) << ",";
        data_file_ << "lw_cp_orthogonal_speed_" + std::to_string(i) << ",";
        data_file_ << "lw_cp_drag_scalar_" + std::to_string(i) << ",";
        data_file_ << "lw_cp_force_vector_x_" + std::to_string(i) << ",";
        data_file_ << "lw_cp_force_vector_y_" + std::to_string(i) << ",";
        data_file_ << "lw_cp_force_vector_z_" + std::to_string(i) << ",";
    }
    data_file_ << "\n";
    // buffer_size_ = 50000; //migrate to yaml config later?
    // data_buffer_1_.resize(buffer_size_);
    // data_buffer_2_.resize(buffer_size_);
    buffer_index_ = 0;
    first_time_ = true;
    buffer_thread_finished_ = true;
    // data_
    //MOVE DATA HEADERS TO YAML LATER
    if (config["log_period"].as<double>() < 0 || config["time_step"].as<double>() <= 0){
        log_count_ = 10000;
    } else {
        log_count_ = static_cast<int>(config["log_period"].as<double>() / config["time_step"].as<double>());
    }
    std::cout << "log_count_: " << log_count_ << std::endl;
    log_counter_ = 0;
} //end constructor

// Aerodynamics::~Aerodynamics() noexcept {}
void Aerodynamics::CalcOutput(const systems::Context<double>& context,
    std::vector<drake::multibody::ExternallyAppliedSpatialForce<double>>* output) const {
        // std::cout << "start" << std::endl;
    // if (context.get_time() < 0.2) return; //instability phase
    // if (context.get_time() < .3) return; //instability phase
    if (first_time_) {
        start_time_ = std::chrono::high_resolution_clock::now();
        std::cout << "start time (ns): " << start_time_.time_since_epoch().count() << std::endl;
        first_time_ = false;
    }
    
    std::vector<double> data_row_rw;
    std::vector<double> data_row_lw;
    const auto& poses = this->EvalAbstractInput(context, 0)->get_value<std::vector<RigidTransform<double>>>();
    const auto& spatial_velocities = this->EvalAbstractInput(context, 1)->get_value<std::vector<SpatialVelocity<double>>>();
    data_row_rw.push_back(context.get_time());
    drake::math::RigidTransformd pose_world_rw = poses[right_wing_body_index_];
    drake::math::RigidTransformd pose_world_lw = poses[left_wing_body_index_];
    drake::math::RigidTransformd pose_world_rw_joint_link = poses[right_wing_joint_link_index_];   
    drake::math::RigidTransformd pose_world_main_body = poses[main_body_index_];
    drake::multibody::SpatialVelocity<double> velocity_world_rw = spatial_velocities[right_wing_body_index_];
    drake::multibody::SpatialVelocity<double> velocity_world_lw = spatial_velocities[left_wing_body_index_];
    drake::multibody::SpatialVelocity<double> velocity_world_rw_joint_link = spatial_velocities[right_wing_joint_link_index_];
    Eigen::Vector3d up_vector_rw = pose_world_rw.rotation() * orthogonal_vec_; //orthgonal vec in world frame, inverse rotation for local frame
    Eigen::Vector3d up_vector_lw = pose_world_lw.rotation() * orthogonal_vec_;
    // std::cout << "initializing output" << std::endl;
    // auto trans_v_rw = velocity_world_rw.translational();
    // auto trans_v_lw = velocity_world_lw.translational();
    // std::unique_ptr<drake::systems::Context<double>> plant_context = plant_.CreateDefaultContext();
    // plant_context->SetTime(context.get_time());
    log_counter_++;
    if (log_counter_ >= log_count_){
        // std::cout << "time: " << context.get_time() / 1000 << std::endl;
        // log_counter_ = 0;
        // std::cout << "rw joint angle: " << right_wing_passive_joint_->get_angle(*plant_context) << std::endl;
        // data_row_rw.push_back(right_wing)
        //flatten rotation matrix to be 9 printable values

        data_row_rw.insert(data_row_rw.end(), pose_world_rw.translation().data(), pose_world_rw.translation().data() + 3); //world position 
        data_row_rw.push_back(pose_world_rw.rotation().matrix()(0,0)); //world position
        data_row_rw.push_back(pose_world_rw.rotation().matrix()(1,0)); //world position
        data_row_rw.push_back(pose_world_rw.rotation().matrix()(2,0)); //world position
        data_row_rw.push_back(pose_world_rw.rotation().matrix()(0,1)); //world position
        data_row_rw.push_back(pose_world_rw.rotation().matrix()(1,1)); //world position
        data_row_rw.push_back(pose_world_rw.rotation().matrix()(2,1)); //world position
        data_row_rw.push_back(pose_world_rw.rotation().matrix()(0,2)); //world position
        data_row_rw.push_back(pose_world_rw.rotation().matrix()(1,2)); //world position
        data_row_rw.push_back(pose_world_rw.rotation().matrix()(2,2)); //world position

        data_row_rw.insert(data_row_rw.end(), pose_world_rw_joint_link.translation().data(), pose_world_rw_joint_link.translation().data() + 3); //world position
        data_row_rw.push_back(pose_world_rw_joint_link.rotation().matrix()(0,0)); //world position
        data_row_rw.push_back(pose_world_rw_joint_link.rotation().matrix()(1,0)); //world position
        data_row_rw.push_back(pose_world_rw_joint_link.rotation().matrix()(2,0)); //world position
        data_row_rw.push_back(pose_world_rw_joint_link.rotation().matrix()(0,1)); //world position
        data_row_rw.push_back(pose_world_rw_joint_link.rotation().matrix()(1,1)); //world position
        data_row_rw.push_back(pose_world_rw_joint_link.rotation().matrix()(2,1)); //world position
        data_row_rw.push_back(pose_world_rw_joint_link.rotation().matrix()(0,2)); //world position
        data_row_rw.push_back(pose_world_rw_joint_link.rotation().matrix()(1,2)); //world position
        data_row_rw.push_back(pose_world_rw_joint_link.rotation().matrix()(2,2)); //world position

        data_row_rw.insert(data_row_rw.end(), up_vector_rw.data(), up_vector_rw.data() + 3);
        data_row_rw.insert(data_row_rw.end(), velocity_world_rw.translational().data(), velocity_world_rw.translational().data() + 3);
        data_row_rw.insert(data_row_rw.end(), velocity_world_rw.rotational().data(), velocity_world_rw.rotational().data() + 3);
        data_row_lw.insert(data_row_lw.end(), pose_world_lw.translation().data(), pose_world_lw.translation().data() + 3); //world position 
        data_row_lw.insert(data_row_lw.end(), up_vector_lw.data(), up_vector_lw.data() + 3);
        data_row_lw.insert(data_row_lw.end(), velocity_world_lw.translational().data(), velocity_world_lw.translational().data() + 3);
        data_row_lw.insert(data_row_lw.end(), velocity_world_lw.rotational().data(), velocity_world_lw.rotational().data() + 3);

        //finding rotation matrix relative R_{sp} = R_{ws}^T * R_{wp}
        auto Rsw = pose_world_rw.rotation().transpose();
        auto Rsp = Rsw * pose_world_rw.rotation();
        
        // Rbs = Rwb^T * Rwp
        auto Rbs = pose_world_main_body.rotation().transpose() * pose_world_rw_joint_link.rotation();
        
        auto Rwb = pose_world_main_body.rotation();

        auto w_p = velocity_world_rw.rotational();
        std::cout << "w_p: " << w_p(0) << ", " << w_p(1) << ", " << w_p(2) << std::endl;
        auto w_p_w = pose_world_rw.rotation().transpose() * w_p;
        std::cout << "w_p_w: " << w_p_w(0) << ", " << w_p_w(1) << ", " << w_p_w(2) << std::endl;
        
        std::cout << "Rsp: " << Rsp.matrix()(0,0) << ", " << Rsp.matrix()(0,1) << ", " << Rsp.matrix()(0,2) << std::endl;
        std::cout << "Rsp: " << Rsp.matrix()(1,0) << ", " << Rsp.matrix()(1,1) << ", " << Rsp.matrix()(1,2) << std::endl;
        std::cout << "Rsp: " << Rsp.matrix()(2,0) << ", " << Rsp.matrix()(2,1) << ", " << Rsp.matrix()(2,2) << std::endl;
        // auto Rbp = pose_world_main_body.rotation().transpose() * pose_world_rw.rotation();
        // std::cout << "Rbp: " << Rbp.matrix()(0,0) << ", " << Rbp.matrix()(0,1) << ", " << Rbp.matrix()(0,2) << std::endl;
        // std::cout << "Rbp: " << Rbp.matrix()(1,0) << ", " << Rbp.matrix()(1,1) << ", " << Rbp.matrix()(1,2) << std::endl;
        // std::cout << "Rbp: " << Rbp.matrix()(2,0) << ", " << Rbp.matrix()(2,1) << ", " << Rbp.matrix()(2,2) << std::endl;

        auto w_s = velocity_world_rw_joint_link.rotational();
        // std::cout << "w_s: " << w_s(0) << ", " << w_s(1) << ", " << w_s(2) << std::endl;
        auto w_s_w = pose_world_rw_joint_link.rotation().transpose() * w_s;
        // std::cout << "w_s_w: " << w_s_w(0) << ", " << w_s_w(1) << ", " << w_s_w(2) << std::endl;
        //print rbs
        // std::cout << "Rbs: " << Rbs.matrix()(0,0) << ", " << Rbs.matrix()(0,1) << ", " << Rbs.matrix()(0,2) << std::endl;
        // std::cout << "Rbs: " << Rbs.matrix()(1,0) << ", " << Rbs.matrix()(1,1) << ", " << Rbs.matrix()(1,2) << std::endl;
        // std::cout << "Rbs: " << Rbs.matrix()(2,0) << ", " << Rbs.matrix()(2,1) << ", " << Rbs.matrix()(2,2) << std::endl;
        // Eigen::Matrix3d W = MakeSkewSymmetric(w_p);
        // drake::math::RotationMatrix<double> w_p_hat(W);
        // auto Rpb_dot = w_p_hat() 

        // Rsb_dot = Rsp * ()
        // auto lin_vel = Rwb*(Rbs)
    }


    for(size_t i = 0; i < cp_size_; i++) {
        // size_t i = 0;
        // Eigen::Vector3d wing_cp_v_mapped_rw = velocity_world_rw.rotational().cross(pose_world_rw * center_pressure_body_rw_[i]) + trans_v_rw;
        drake::multibody::SpatialVelocity<double> v_mapped_rw = velocity_world_rw.Shift(center_pressure_body_rw_[i]);
        drake::multibody::SpatialVelocity<double> v_mapped_lw = velocity_world_lw.Shift(center_pressure_body_lw_[i]);
        Eigen::Vector3d wing_cp_v_mapped_rw = v_mapped_rw.translational();
        Eigen::Vector3d wing_cp_v_mapped_lw = v_mapped_lw.translational();
        // Eigen::Vector3d wing_cp_v_mapped_lw = velocity_world_lw.rotational().cross(pose_world_lw * center_pressure_body_lw_[i]) +  trans_v_lw;

        double vel_dot_up_rw = wing_cp_v_mapped_rw.dot(up_vector_rw);
        double vel_dot_up_lw = wing_cp_v_mapped_lw.dot(up_vector_lw);

        // double flap_drag_scalar_rw =  vel_dot_up_rw > 0 ? 1 : -1;
        // double flap_drag_scalar_lw =  vel_dot_up_lw > 0 ? 1 : -1;
        //time scaled by 10^-3 from s to s
        double flap_drag_scalar_rw = 0.5 * drag_coef_ * air_density_ * vel_dot_up_rw * vel_dot_up_rw * blade_area_list_rw_[i];
        double flap_drag_scalar_lw = 0.5 * drag_coef_ * air_density_ * vel_dot_up_lw * vel_dot_up_lw * blade_area_list_lw_[i];
        // if(flap_drag_scalar_rw > .5){
        //     flap_drag_scalar_rw = 0.5;
        // }if(flap_drag_scalar_rw < -.5){
        //     flap_drag_scalar_rw = -0.5;
        // }
        Eigen::Vector3d force_vector_rw;
        Eigen::Vector3d force_vector_lw;
        if(vel_dot_up_rw < 0){
            force_vector_rw = flap_drag_scalar_rw * up_vector_rw;
        } else {
            force_vector_rw = flap_drag_scalar_rw * (-up_vector_rw);
        }
        if(vel_dot_up_lw < 0){
            force_vector_lw = flap_drag_scalar_lw * up_vector_lw;
        } else {
            force_vector_lw = flap_drag_scalar_lw * (-up_vector_lw);
        }
        // Eigen::Vector3d force_vector_rw = up_vector_rw * flap_drag_scalar_rw;
        // Eigen::Vector3d force_vector_lw = up_vector_lw * flap_drag_scalar_lw; //there was a * 2???

        
        // drake::multibody::SpatialForce<double> force_rw(center_pressure_body_rw_[i].cross(force_vector_rw), Eigen::Vector3d::Zero());
        drake::multibody::SpatialForce<double> force_rw(Eigen::Vector3d::Zero(), force_vector_rw);
        // drake::multibody::SpatialForce<double> force_rw(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
        drake::multibody::SpatialForce<double> force_lw(Eigen::Vector3d::Zero(), force_vector_lw);

        drake::multibody::ExternallyAppliedSpatialForce<double> force_rw_applied;
        drake::multibody::ExternallyAppliedSpatialForce<double> force_lw_applied;
        force_rw_applied.body_index = right_wing_body_index_;
        // force_rw_applied.p_BoBq_B = Vector3<double>(0, 0, 0);
        force_rw_applied.p_BoBq_B = center_pressure_body_rw_[i];
        force_rw_applied.F_Bq_W = force_rw;
        force_lw_applied.body_index = left_wing_body_index_;
        force_lw_applied.p_BoBq_B = center_pressure_body_lw_[i];
        force_lw_applied.F_Bq_W = force_lw;

        if (log_counter_ >= log_count_){
            data_row_rw.insert(data_row_rw.end(), wing_cp_v_mapped_rw.data(), wing_cp_v_mapped_rw.data() + 3);
            data_row_rw.push_back(vel_dot_up_rw);
            data_row_rw.push_back(flap_drag_scalar_rw);
            data_row_rw.insert(data_row_rw.end(), force_vector_rw.data(), force_vector_rw.data() + 3);
            data_row_lw.insert(data_row_lw.end(), wing_cp_v_mapped_lw.data(), wing_cp_v_mapped_lw.data() + 3);
            data_row_lw.push_back(vel_dot_up_lw);
            data_row_lw.push_back(flap_drag_scalar_lw);
            data_row_lw.insert(data_row_lw.end(), force_vector_lw.data(), force_vector_lw.data() + 3);
        }
        // if (0) {
        // // if((counter-- <= 0)){
        //     std::cout<<"time: " << context.get_time() << std::endl;
        //     std::cout<<"Output applied spatial force: "<<force_rw<<std::endl;
        //     std::cout<<"Rotated orthogonal cp vector: "<< up_vector_rw(0) << ", " 
        //   << up_vector_rw(1) << ", " 
        //   << up_vector_rw(2) << std::endl;
        //     std::cout<<"current mapped velocity: "<<wing_cp_v_mapped_rw(0) << ", " 
        //     << wing_cp_v_mapped_rw(1) << ", "
        //     << wing_cp_v_mapped_rw(2) << std::endl;
        //     std::cout<<"mapped force vector: "<<force_vector_rw(0) << ", "
        //     << force_vector_rw(1) << ", "
        //     << force_vector_rw(2) << std::endl;
        //     counter = 100;
        //     std::cout <<"origin linear speed: " << velocity_world_rw.translational()(0) << ", " <<
        //     velocity_world_rw.translational()(1) << ", " <<
        //     velocity_world_rw.translational()(2) << std::endl;
        //     std::cout <<"origin angular speed: " << velocity_world_rw.rotational()(0) << ", " <<
        //     velocity_world_rw.rotational()(1) << ", " <<
        //     velocity_world_rw.rotational()(2) << std::endl;
        //     if (abs(vel_dot_up_rw) > 1.5) {
        //         drake::math::RollPitchYaw<double> rpy(pose_world_rw.rotation());
        //         std::cout << "\033[1;31mOrientation (RPY): "
        //                 << "Roll: " << rpy.roll_angle() * rad_to_deg << ", "
        //                 << "Pitch: " << rpy.pitch_angle() * rad_to_deg<< ", "
        //                 << "Yaw: " << rpy.yaw_angle()* rad_to_deg
        //                 << "\033[0m\n";
        //         const auto& translation = pose_world_rw.translation();
        //         std::cout << "\033[1;31mTranslation: "
        //                 << "X: " << translation(0) << ", "
        //                 << "Y: " << translation(1) << ", "
        //                 << "Z: " << translation(2)
        //                 << "\033[0m\n";
        //         auto transformed_point = pose_world_rw * center_pressure_body_rw_[i];
        //         std::cout << "\033[1;31mTransformed Point: "
        //                 << "X: " << transformed_point(0) << ", "
        //                 << "Y: " << transformed_point(1) << ", "
        //                 << "Z: " << transformed_point(2)
        //                 << "\033[0m\n";
        //         const auto& translational = velocity_world_rw.translational();
        //         const auto& rotational = velocity_world_rw.rotational();
        //         std::cout << "Mapped velocity on orthogonal: " << vel_dot_up_rw << "\n";
        //         std::cout << "\033[1;31m vel: "
        //             << "translational: "
        //             << translational(0) << ", "
        //             << translational(1) << ", "
        //             << translational(2)
        //             << " rotational: "
        //             << rotational(0) << ", "
        //             << rotational(1) << ", "
        //             << rotational(2)
        //             << "\033[0m\n";
                //print position of wing at cp

                // std::cout << "pose_world_rw: " << pose_world_rw.translation() << std::endl;
            // }
            
        // }
        output->push_back(force_rw_applied);
        output->push_back(force_lw_applied);
    }
    if (log_counter_ >= log_count_){
        log_counter_ = 0;
        // std::cout << "writing to buffer" << std::endl;
        data_row_rw.insert(data_row_rw.end(), data_row_lw.begin(), data_row_lw.end());
        active_buffer_->push_back(data_row_rw);
        // (*active_buffer_)[buffer_index_] = data_row_rw;
        buffer_index_++;
        // std::cout << "active_buffer_ size: " << active_buffer_->size() << std::endl;
        // std::cout << "active_buffer_ size: " << active_buffer_->size() << std::endl;
        // std::cout << "active_buffer_ capacity: " << active_buffer_->capacity() << std::endl;
        // if (active_buffer_->size() == active_buffer_->capacity()) {
        if (buffer_index_ >= buffer_size_) {
            buffer_index_ = 0;
            if (!buffer_thread_finished_) {
                std::cout << "buffer overflowed, waiting for logger thread to finish" << std::endl;            
            } 
            if (logger_thread_.joinable()) {
                logger_thread_.join();
            }
            if (active_buffer_ == &data_buffer_1_) {
                logger_thread_ = std::thread(&Aerodynamics::LoggerThread, this, std::ref(data_buffer_1_));
                active_buffer_ = &data_buffer_2_;
            } else {
                logger_thread_ = std::thread(&Aerodynamics::LoggerThread, this, std::ref(data_buffer_2_));
                active_buffer_ = &data_buffer_1_;
            }
        }
    }
} //end CalcOutput

Eigen::Matrix3d MakeSkewSymmetric(const Eigen::Vector3d& w) {
  Eigen::Matrix3d W;
  W <<  0,     -w.z(),  w.y(),
        w.z(),  0,     -w.x(),
       -w.y(),  w.x(),  0;
  return W;
}


void Aerodynamics::LoggerThread(std::vector<std::vector<double>>& buffer) const {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    std::cout << "buffer dump" << std::endl;
    // if (buffer.empty()) return;
    for (auto& row : buffer) {
        for (size_t i = 0; i < row.size(); i++) {
            data_file_ << row[i];
            if (i < row.size() - 1) {
                data_file_ << ",";
            }
        }
        data_file_ << std::endl;
    }
    std::cout << "buffer dump complete, clearing" << std::endl;
    buffer.clear();
}

Aerodynamics::~Aerodynamics() noexcept {
    std::cout << "aero destructor" << std::endl;
    if (logger_thread_.joinable()) {
            logger_thread_.join();
        }
    if (!active_buffer_->empty()) {
        LoggerThread(*active_buffer_);
    }
    data_file_.close(); // just good practice, though garbage collection should handle this
}

void Aerodynamics::CalcOutputThreaded(const systems::Context<double>& context,
    std::vector<drake::multibody::ExternallyAppliedSpatialForce<double>>* output) const {
    
    // const auto& plant_context = this->GetSubsystemContext(plant_, context);

    // drake::math::RigidTransformd pose_world_rw = plant_.EvalBodyPoseInWorld(plant_context, *right_wing_body_); //const math::RigidTransform< T > & 
    // drake::math::RigidTransformd pose_world_lw = plant_.EvalBodyPoseInWorld(plant_context, *left_wing_body_);
    // drake::multibody::SpatialVelocity<double> velocity_world_rw = plant_.EvalBodySpatialVelocityInWorld(plant_context, *right_wing_body_);
    // drake::multibody::SpatialVelocity<double> velocity_world_lw = plant_.EvalBodySpatialVelocityInWorld(plant_context, *left_wing_body_);
    const auto& poses = this->EvalAbstractInput(context, 0)->get_value<std::vector<RigidTransform<double>>>();
    const auto& spatial_velocities = this->EvalAbstractInput(context, 1)->get_value<std::vector<SpatialVelocity<double>>>();
    
    drake::math::RigidTransformd pose_world_rw = poses[right_wing_body_index_];
    drake::math::RigidTransformd pose_world_lw = poses[left_wing_body_index_];
    drake::multibody::SpatialVelocity<double> velocity_world_rw = spatial_velocities[right_wing_body_index_];
    drake::multibody::SpatialVelocity<double> velocity_world_lw = spatial_velocities[left_wing_body_index_];

    Eigen::Vector3d up_vector_rw = pose_world_rw.rotation() * orthogonal_vec_; //orthgonal vec in world frame, inverse rotation for local frame
    Eigen::Vector3d up_vector_lw = pose_world_lw.rotation() * orthogonal_vec_;
    
    //spin up threads
    std::vector<std::thread> threads;

    for (size_t i = 0; i < num_threads_; i++) {
        size_t start = i * batch_size_;
        size_t end = (i == num_threads_ - 1) ? cp_size_ : (i + 1) * batch_size_;
        threads.emplace_back(&Aerodynamics::CalcForceBatch, this, start, end, up_vector_rw, pose_world_rw, velocity_world_rw, up_vector_lw, pose_world_lw, velocity_world_lw, output);
    }

    //wait for threads to all finish
    for (auto& thread : threads) {
        thread.join();
    }
    
    // const auto& spatial_velocities = this->EvalAbstractInput(context, "body_spatial_velocities")->GetValue<std::vector<SpatialVelocity<double>>>();
    // const auto& poses = this->EvalAbstractInput(context, "body_poses")->GetValue<std::vector<RigidTransform<double>>>();
}

void Aerodynamics::CalcForceBatch(size_t start, size_t end, Eigen::Vector3d up_vector_rw, drake::math::RigidTransformd pose_world_rw, drake::multibody::SpatialVelocity<double> velocity_world_rw,
    Eigen::Vector3d up_vector_lw, drake::math::RigidTransformd pose_world_lw, drake::multibody::SpatialVelocity<double> velocity_world_lw, std::vector<drake::multibody::ExternallyAppliedSpatialForce<double>>* output) const {
    // std::cout << "start: " << start << " end: " << end << std::endl;
    
    std::vector<drake::multibody::ExternallyAppliedSpatialForce<double>> local_force_list;

    for (size_t i = start; i < end; i++) {
        // std::cout << "i: " << i << std::endl;
        //mapped wing rotation
        Eigen::Vector3d wing_cp_v_mapped_rw = velocity_world_rw.rotational().cross(center_pressure_body_rw_[i]) + velocity_world_rw.translational();
        // SpatialVelocity<double> v_mapped_rw = velocity_world_rw.Shift(center_pressure_body_rw_[i]);
        
        Eigen::Vector3d wing_cp_v_mapped_lw = velocity_world_lw.rotational().cross(center_pressure_body_lw_[i]) + velocity_world_lw.translational();
        double vel_dot_up_rw = wing_cp_v_mapped_rw.dot(up_vector_rw);
        double vel_dot_up_lw = wing_cp_v_mapped_lw.dot(up_vector_lw); //up vector points in the -y direction (green) on drake

        double flap_drag_scalar_rw =  vel_dot_up_rw > 0 ? 1 : -1;
        double flap_drag_scalar_lw =  vel_dot_up_lw > 0 ? 1 : -1;
        flap_drag_scalar_rw *= 0.5 * drag_coef_ * air_density_ * vel_dot_up_rw * vel_dot_up_rw * blade_area_list_rw_[i];
        flap_drag_scalar_lw *= 0.5 * drag_coef_ * air_density_ * vel_dot_up_lw * vel_dot_up_lw * blade_area_list_lw_[i];

        Eigen::Vector3d force_vector_rw = up_vector_rw * flap_drag_scalar_rw;
        Eigen::Vector3d force_vector_lw = up_vector_lw * flap_drag_scalar_lw;
        
        
        // Eigen::Vector3d force_rw = up_vector_rw * blade_area_list_rw_[i] * air_density_ * velocity_world_rw.rotational()[2] * velocity_world_rw.rotational()[2] * drag_coef_ / 2;
        // Eigen::Vector3d force_lw = up_vector_rw * blade_area_list_lw_[i] * air_density_ * velocity_world_rw.rotational()[2] * velocity_world_rw.rotational()[2] * drag_coef_ / 2;
        
        drake::multibody::SpatialForce<double> force_rw(Eigen::Vector3d::Zero(), force_vector_rw);
        drake::multibody::SpatialForce<double> force_lw(Eigen::Vector3d::Zero(), force_vector_lw);

        // std::cout << "force_rw: " << force_rw << std::endl;
        // std::cout << "force_lw: " << force_lw << std::endl;
        // std::cout << "center_pressure_body_rw_[i]: " << center_pressure_body_rw_[i] << std::endl;
        // std::cout << "center_pressure_body_lw_[i]: " << center_pressure_body_lw_[i] << std::endl;
        // drake::multibody::ExternallyAppliedSpatialForce<double> force_rw_applied;
        // drake::multibody::ExternallyAppliedSpatialForce<double> force_lw_applied;
        
        // force_rw_applied.body_index = right_wing_body_index_;
        // force_rw_applied.p_BoBq_B = center_pressure_body_rw_[i];
        // force_rw_applied.F_Bq_W = force_rw;
        // force_lw_applied.body_index = left_wing_body_index_;
        // force_lw_applied.p_BoBq_B = center_pressure_body_lw_[i];
        // force_lw_applied.F_Bq_W = force_lw;

        local_force_list.emplace_back(right_wing_body_index_, center_pressure_body_rw_[i], force_rw); //constructs inplace
        local_force_list.emplace_back(left_wing_body_index_, center_pressure_body_lw_[i], force_lw);

        
        // std::cout << "force_rw_applied: " << force_rw_applied << std::endl;
        // std::cout << "force_lw_applied: " << force_lw_applied << std::endl;
        
    }
    {
        std::lock_guard<std::mutex> lock(output_mutex_);
        output->insert(output->end(), local_force_list.begin(), local_force_list.end());
    }
} 
}  // namespace flap
}  // namespace drake