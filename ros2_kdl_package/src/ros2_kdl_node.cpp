// Copyright  (C)  2007  Francois Cauwe <francois at cauwe dot org>
 
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.
 
#include <stdio.h>
#include <iostream>
#include <chrono>
#include <cstdlib>
#include <memory>

#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/wait_for_message.hpp"

#include "kdl_robot.h"
#include "kdl_control.h"
#include "kdl_planner.h"
#include "kdl_parser/kdl_parser.hpp"
#include <rclcpp_action/rclcpp_action.hpp>
#include "ros2_kdl_interfaces/action/execute_linear_traj.hpp"
#include <thread>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_kdl/tf2_kdl.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "ros_gz_interfaces/srv/set_entity_pose.hpp"     
#include "ros2_kdl_interfaces/srv/set_aruco_pose.hpp"     


 
using namespace KDL;
using FloatArray = std_msgs::msg::Float64MultiArray;
using namespace std::chrono_literals;
using ExecuteLinearTraj = ros2_kdl_interfaces::action::ExecuteLinearTraj;
using GoalHandleExec = rclcpp_action::ServerGoalHandle<ExecuteLinearTraj>;
static constexpr const char* kActionName = "execute_linear_traj";




class Iiwa_pub_sub : public rclcpp::Node
{
    public:
        Iiwa_pub_sub()
        : Node("ros2_kdl_node"), 
        node_handle_(std::shared_ptr<Iiwa_pub_sub>(this))
        {
            // declare cmd_interface parameter (position, velocity)
            declare_parameter("cmd_interface", "velocity"); // default to "position"
            get_parameter("cmd_interface", cmd_interface_);
            RCLCPP_INFO(get_logger(),"Current cmd interface is: '%s'", cmd_interface_.c_str());

            if (!(cmd_interface_ == "position" || cmd_interface_ == "velocity" || cmd_interface_ == "effort" ))
            {
                RCLCPP_ERROR(get_logger(),"Selected cmd interface is not valid! Use 'position', 'velocity' or 'effort' instead..."); return;
            }

            // declare traj_type parameter (linear, circular)
            declare_parameter("traj_type", "linear");
            get_parameter("traj_type", traj_type_);
            RCLCPP_INFO(get_logger(),"Current trajectory type is: '%s'", traj_type_.c_str());
            if (!(traj_type_ == "linear" || traj_type_ == "circular"))
            {
                RCLCPP_INFO(get_logger(),"Selected traj type is not valid!"); return;
            }

            // declare s_type parameter (trapezoidal, cubic)
            declare_parameter("s_type", "trapezoidal");
            get_parameter("s_type", s_type_);
            RCLCPP_INFO(get_logger(),"Current s type is: '%s'", s_type_.c_str());
            if (!(s_type_ == "trapezoidal" || s_type_ == "cubic"))
            {
                RCLCPP_INFO(get_logger(),"Selected s type is not valid!"); return;
            }

            //point 1.a

            declare_parameter("traj_duration", 1.5);
            get_parameter("traj_duration", traj_duration_);
            RCLCPP_INFO(get_logger(),"Current traj duration is: '%.3f'", traj_duration_);

            declare_parameter("acc_duration", 0.5);
            get_parameter("acc_duration", acc_duration_);
            RCLCPP_INFO(get_logger(),"Current acc duration is: '%.3f'", acc_duration_);

            declare_parameter("total_time", 1.5);
            get_parameter("total_time", total_time_);
            RCLCPP_INFO(get_logger(),"Current total time is: '%.3f'", total_time_);

            declare_parameter("trajectory_len", 150);
            get_parameter("trajectory_len", trajectory_len_);
            RCLCPP_INFO(get_logger(),"Current trajectory len is: '%d'", trajectory_len_);


            declare_parameter("traj_radius", 0.15);
            get_parameter("traj_radius", traj_radius_);
            RCLCPP_INFO(get_logger(),"Current traj radius is: '%.3f'", traj_radius_);
            
            declare_parameter("Kp", 5.0);
            get_parameter("Kp", Kp_);
            RCLCPP_INFO(get_logger(),"Current Kp is: '%.3f'", Kp_);
            
            // 3 components end_position
            auto end_pos_vec = this->declare_parameter<std::vector<double>>(
            "end_position", std::vector<double>{0.4, 0.1, 0.25});
            if (end_pos_vec.size() != 3) {
            RCLCPP_ERROR(get_logger(), "end_position must have 3 elements [x,y,z]. Using defaults.");
            } else {
            end_position_ = Eigen::Vector3d(end_pos_vec[0], end_pos_vec[1], end_pos_vec[2]);
            }

            // point 1.b
            // ctrl_mode: velocity_ctrl | velocity_ctrl_nul

            declare_parameter("ctrl_mode", "vision_ctrl");
            get_parameter("ctrl_mode", ctrl_mode_);
            RCLCPP_INFO(get_logger(), "Current ctrl_mode is: '%s'", ctrl_mode_.c_str());

            if (!(ctrl_mode_ == "velocity_ctrl" ||
                ctrl_mode_ == "velocity_ctrl_null" ||
                ctrl_mode_ == "vision_ctrl"))
            {
            RCLCPP_ERROR(get_logger(),
                "Selected ctrl_mode is not valid! Use 'velocity_ctrl', 'velocity_ctrl_null' or 'vision_ctrl'.");
            return;
            }

            // point 1.c
            declare_parameter("action_mode", false);
            get_parameter("action_mode", action_mode_);
            RCLCPP_INFO(get_logger(), "Current action_mode is: %s", action_mode_ ? "true" : "false");
            
            declare_parameter("lambda", 0.5);
            get_parameter("lambda", lambda_);
            
            //point 2.b
            declare_parameter("Kv", 0.5);
            get_parameter("Kv", Kv_);

            //point 2.c
            world_name_ = declare_parameter<std::string>("world_name", "default");
            // 
            client_node_ = std::make_shared<rclcpp::Node>(
                "set_pose_client_node",
                rclcpp::NodeOptions().context(this->get_node_base_interface()->get_context())
            );

            gz_set_pose_client_ = client_node_->create_client<ros_gz_interfaces::srv::SetEntityPose>(
                "/world/" + world_name_ + "/set_pose"
            );

            set_marker_pose_srv_ = this->create_service<ros2_kdl_interfaces::srv::SetArucoPose>(
                "set_aruco_pose",
                std::bind(&Iiwa_pub_sub::setArucoPoseCb, this, std::placeholders::_1, std::placeholders::_2));





            iteration_ = 0; t_ = 0;
            joint_state_available_ = false; 

            // retrieve robot_description param
            auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(node_handle_, "robot_state_publisher");
            while (!parameters_client->wait_for_service(1s)) {
                if (!rclcpp::ok()) {
                    RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                    rclcpp::shutdown();
                }
                RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
            }
            auto parameter = parameters_client->get_parameters({"robot_description"});

            // create KDLrobot structure
            KDL::Tree robot_tree;
            if (!kdl_parser::treeFromString(parameter[0].value_to_string(), robot_tree)){
                std::cout << "Failed to retrieve robot_description param!";
            }
            robot_ = std::make_shared<KDLRobot>(robot_tree);  
            
            // Create joint array
            unsigned int nj = robot_->getNrJnts();
            KDL::JntArray q_min(nj), q_max(nj);
            q_min.data << -2.96,-2.09,-2.96,-2.09,-2.96,-2.09,-2.96; //-2*M_PI,-2*M_PI; // TODO: read from urdf file
            q_max.data <<  2.96,2.09,2.96,2.09,2.96,2.09,2.96; //2*M_PI, 2*M_PI; // TODO: read from urdf file          
            robot_->setJntLimits(q_min,q_max);     
            
            q_min_ = q_min;
            q_max_ = q_max;


            joint_positions_.resize(nj); 
            joint_velocities_.resize(nj); 
            joint_positions_cmd_.resize(nj); 
            joint_velocities_cmd_.resize(nj); 
            joint_efforts_cmd_.resize(nj); joint_efforts_cmd_.data.setZero();

            // Subscriber to jnt states
            jointSubscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
                "/joint_states", 10, std::bind(&Iiwa_pub_sub::joint_state_subscriber, this, std::placeholders::_1));

            // Wait for the joint_state topic
            while(!joint_state_available_){
                RCLCPP_INFO(this->get_logger(), "No data received yet! ...");
                rclcpp::spin_some(node_handle_);
            }

            
            if(ctrl_mode_ == "vision_ctrl"){
                // Subscriber to aruco marker pose
                aruco_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                    "/aruco_single/pose", 10, std::bind(&Iiwa_pub_sub::aruco_pose_cb_, this, std::placeholders::_1));

                while(!have_marker_){
                    RCLCPP_INFO(this->get_logger(), "No marker received yet! ...");
                    rclcpp::spin_some(node_handle_);
                }


            }
            

            // Update KDLrobot object
            robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));
            KDL::Frame f_T_ee = KDL::Frame::Identity();
            robot_->addEE(f_T_ee);
            robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));

            // Compute EE frame
            init_cart_pose_ = robot_->getEEFrame();
            // std::cout << "The initial EE pose is: " << std::endl;  
            // std::cout << init_cart_pose_ <<std::endl;


            if(ctrl_mode_ == "vision_ctrl"){
                    
                tf_buffer_   = std::make_unique<tf2_ros::Buffer>(this->get_clock());
                tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
                ee_frame_  = declare_parameter<std::string>("ee_frame",  "tool0");
                cam_frame_ = declare_parameter<std::string>("cam_frame", "camera_optical_frame");
                // prova subito; se non c'è ancora, riprova per qualche secondo
                for (int i=0; i<20 && rclcpp::ok(); ++i) {
                try {
                    auto T_ec = tf_buffer_->lookupTransform(ee_frame_, cam_frame_, tf2::TimePointZero);
                    F_ec_ = tf2::transformToKDL(T_ec);        // ^eT_c
                    have_F_ec_ = true;
                    RCLCPP_INFO(get_logger(),"Cached F_ec: '%s' <- '%s'", ee_frame_.c_str(), cam_frame_.c_str());
                    break;
                } catch (const tf2::TransformException&) {
                    rclcpp::sleep_for(100ms);
                }
                }
                if (!have_F_ec_) {
                RCLCPP_WARN(get_logger(), "F_ec not ready yet; will hold zeros until available.");
                }
            }

            // Compute IK
            KDL::JntArray q(nj);
            robot_->getInverseKinematics(init_cart_pose_, q);
            // std::cout << "The inverse kinematics returned: " <<std::endl; 
            // std::cout << q.data <<std::endl;

            // Initialize controller
            //KDLController controller_(*robot_);
            controller_ = std::make_unique<KDLController>(*robot_);

            // EE's trajectory initial position (just an offset)
            Eigen::Vector3d init_position(Eigen::Vector3d(init_cart_pose_.p.data) - Eigen::Vector3d(0,0,0.0));

            // EE's trajectory end position (just opposite y)
            //Eigen::Vector3d end_position; end_position << init_position[0], -init_position[1], init_position[2];
            Eigen::Vector3d end_position = end_position_; 

            // Plan trajectory
            double traj_duration = traj_duration_, acc_duration = acc_duration_;
            double traj_radius = traj_radius_;; // 



            // Retrieve the first trajectory point
            if ((!action_mode_) && (ctrl_mode_ != "vision_ctrl")) {

                if(traj_type_ == "linear"){
                    planner_ = KDLPlanner(traj_duration, acc_duration, init_position, end_position); // currently using trapezoidal velocity profile
                    if(s_type_ == "trapezoidal")
                    {
                        p_ = planner_.linear_traj_trapezoidal(t_);
                    }else if(s_type_ == "cubic")
                    {
                        p_ = planner_.linear_traj_cubic(t_);
                    }
                } 
                else if(traj_type_ == "circular")
                {
                    planner_ = KDLPlanner(traj_duration, init_position, traj_radius, acc_duration);
                    if(s_type_ == "trapezoidal")
                    {
                        p_ = planner_.circular_traj_trapezoidal(t_);
                    }else if(s_type_ == "cubic")
                    {
                        p_ = planner_.circular_traj_cubic(t_);
                    }
                }
            }
            // // Retrieve the first trajectory point
            // trajectory_point p = planner_.compute_trajectory(t);

            // compute errors
            // Eigen::Vector3d error = computeLinearError(p_.pos, Eigen::Vector3d(init_cart_pose_.p.data));
            //std::cout << "The initial error is : " << error << std::endl;
            
            if(cmd_interface_ == "position"){
                // Create cmd publisher
                cmdPublisher_ = this->create_publisher<FloatArray>("/iiwa_arm_controller/commands", 10);
                if (!action_mode_) {
                    timer_ = this->create_wall_timer(std::chrono::milliseconds(100), 
                                               std::bind(&Iiwa_pub_sub::cmd_publisher, this));
                }
                // Send joint position commands
                for (long int i = 0; i < joint_positions_.data.size(); ++i) {
                    desired_commands_[i] = joint_positions_(i);
                }
            }
            else if(cmd_interface_ == "velocity"){
                // Create cmd publisher
                cmdPublisher_ = this->create_publisher<FloatArray>("/velocity_controller/commands", 10);
                if (!action_mode_) {
                    if (ctrl_mode_ == "vision_ctrl"){
                        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), 
                                                std::bind(&Iiwa_pub_sub::cmd_vision_publisher, this));
                    }
                    else {
                        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), 
                                                std::bind(&Iiwa_pub_sub::cmd_publisher, this));
                    }
                    
                }

                if (ctrl_mode_ != "vision_ctrl"){
                // Set joint velocity commands
                    for (long int i = 0; i < joint_velocities_.data.size(); ++i) {
                        desired_commands_[i] = joint_velocities_(i);
                    }
                }
            }
            else if(cmd_interface_ == "effort"){
                // Create cmd publisher
                cmdPublisher_ = this->create_publisher<FloatArray>("/effort_controller/commands", 10);
                if (!action_mode_) {
                    timer_ = this->create_wall_timer(std::chrono::milliseconds(100), 
                                                std::bind(&Iiwa_pub_sub::cmd_publisher, this));
                }
                // Set joint effort commands
                for (long int i = 0; i < joint_efforts_cmd_.data.size(); ++i) {
                    desired_commands_[i] = joint_efforts_cmd_(i);
                }
            }


            if (!action_mode_ ) {
                if (ctrl_mode_ != "vision_ctrl"){
                    std_msgs::msg::Float64MultiArray cmd_msg;
                    cmd_msg.data = desired_commands_;
                    cmdPublisher_->publish(cmd_msg);
                    RCLCPP_INFO(this->get_logger(), "Starting trajectory execution ...");
                } else {
                    RCLCPP_INFO(this->get_logger(), "Starting vision control ...");
                }
                
            }
            else {
                using std::placeholders::_1;
                using std::placeholders::_2;

                action_server_ = rclcpp_action::create_server<ExecuteLinearTraj>(
                    this,
                    kActionName, // "execute_linear_traj"
                    std::bind(&Iiwa_pub_sub::handle_goal, this, _1, _2),
                    std::bind(&Iiwa_pub_sub::handle_cancel, this, _1),
                    std::bind(&Iiwa_pub_sub::handle_accepted, this, _1)
                );

                RCLCPP_INFO(this->get_logger(), "Action mode ON: %s waiting for goals ...", kActionName);

            }
            

        }

    private:

        void cmd_vision_publisher()
        {
                  
            const Eigen::Vector3d s_des(0,0,1);
            joint_velocities_cmd_.data = controller_->vision_ctrl(p_cO_, F_ec_, s_des, Kv_, q_min_, q_max_, lambda_);

    
            //  Publish
            for (long i = 0; i < joint_velocities_.data.size(); ++i)
                desired_commands_[i] = joint_velocities_cmd_(i);

            std_msgs::msg::Float64MultiArray msg;
            msg.data = desired_commands_;
            cmdPublisher_->publish(msg);
            robot_->update(toStdVector(joint_positions_.data), toStdVector(joint_velocities_.data));

            RCLCPP_DEBUG(get_logger(), "[VisionCtrl] z=%.3f", p_cO_.z());
        }




        void cmd_publisher(){

            iteration_ = iteration_ + 1;

            // define trajectory
            //double total_time = 1.5; // 
            //int trajectory_len = 150; // 
            //int loop_rate = trajectory_len / total_time;
            //double dt = 1.0 / loop_rate;
            //int Kp = 5;
            //t_+=dt;

            double total_time = total_time_;
            int trajectory_len = trajectory_len_;
            double loop_rate = static_cast<double>(trajectory_len) / total_time;
            double dt = 1.0 / loop_rate;
            double Kp = Kp_;
            t_+=dt;

            if (t_ < total_time){

                // Set endpoint twist
                // double t = iteration_;
                // joint_velocities_.data[2] = 2 * 0.3 * cos(2 * M_PI * t / trajectory_len);
                // joint_velocities_.data[3] = -0.3 * sin(2 * M_PI * t / trajectory_len);

                // Integrate joint velocities
                // joint_positions_.data += joint_velocities_.data * dt;

                // Retrieve the trajectory point based on the trajectory type
                if(traj_type_ == "linear"){
                    if(s_type_ == "trapezoidal")
                    {
                        p_ = planner_.linear_traj_trapezoidal(t_);
                    }else if(s_type_ == "cubic")
                    {
                        p_ = planner_.linear_traj_cubic(t_);
                    }
                } 
                else if(traj_type_ == "circular")
                {
                    if(s_type_ == "trapezoidal")
                    {
                        p_ = planner_.circular_traj_trapezoidal(t_);
                    }else if(s_type_ == "cubic")
                    {
                        p_ = planner_.circular_traj_cubic(t_);
                    }
                }

                // Compute EE frame
                KDL::Frame cartpos = robot_->getEEFrame();           

                // Compute desired Frame
                KDL::Frame desFrame; desFrame.M = cartpos.M; desFrame.p = toKDL(p_.pos); 

                // compute errors
                Eigen::Vector3d error = computeLinearError(p_.pos, Eigen::Vector3d(cartpos.p.data));
                Eigen::Vector3d o_error = computeOrientationError(toEigen(init_cart_pose_.M), toEigen(cartpos.M));
                std::cout << "The error norm is : " << error.norm() << std::endl;

                if(cmd_interface_ == "position"){
                    // Next Frame
                    KDL::Frame nextFrame; nextFrame.M = cartpos.M; nextFrame.p = cartpos.p + (toKDL(p_.vel) + toKDL(Kp*error))*dt; 

                    // Compute IK
                    joint_positions_cmd_ = joint_positions_;
                    robot_->getInverseKinematics(nextFrame, joint_positions_cmd_);
                }
                else if(cmd_interface_ == "velocity"){
                    // Compute differential IK
                    Vector6d cartvel; cartvel << p_.vel + Kp*error, o_error;                   

                    if (ctrl_mode_ == "velocity_ctrl") {
                        joint_velocities_cmd_.data = pseudoinverse(robot_->getEEJacobian().data)*cartvel;
                        std::cout << "VELOCITY CONTROL " << error.norm() << std::endl;

                    } 
                    else if (ctrl_mode_ == "velocity_ctrl_null") {
                        joint_velocities_cmd_.data = controller_->velocity_ctrl_null(error, o_error, p_.vel, Kp, q_min_, q_max_, lambda_);
                        std::cout << "VELOCITY CONTROL NULL" << error.norm() << std::endl;
                    }
                }
                else if(cmd_interface_ == "effort"){
                    joint_efforts_cmd_.data[0] = 0.1*std::sin(2*M_PI*t_/total_time);
                }

                // Update KDLrobot structure
                robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));

                if(cmd_interface_ == "position"){
                    // Set joint position commands
                    for (long int i = 0; i < joint_positions_.data.size(); ++i) {
                        desired_commands_[i] = joint_positions_cmd_(i);
                    }
                }
                else if(cmd_interface_ == "velocity"){
                    // Set joint velocity commands
                    for (long int i = 0; i < joint_velocities_.data.size(); ++i) {
                        desired_commands_[i] = joint_velocities_cmd_(i);
                    }
                }
                else if(cmd_interface_ == "effort"){
                    // Set joint effort commands
                    for (long int i = 0; i < joint_efforts_cmd_.data.size(); ++i) {
                        desired_commands_[i] = joint_efforts_cmd_(i);
                    }
                } 

                // Create msg and publish
                std_msgs::msg::Float64MultiArray cmd_msg;
                cmd_msg.data = desired_commands_;
                cmdPublisher_->publish(cmd_msg);

                // std::cout << "/////////////////////////////////////////////////" <<std::endl <<std::endl;
                // std::cout << "EE pose is: " << robot_->getEEFrame() <<std::endl;  
                // std::cout << "Jacobian: " << robot_->getEEJacobian().data <<std::endl;
                // std::cout << "joint_positions_: " << joint_positions_.data <<std::endl;
                // std::cout << "joint_velocities_: " << joint_velocities_.data <<std::endl;
                // std::cout << "iteration_: " << iteration_ <<std::endl <<std::endl;
                // std::cout << "/////////////////////////////////////////////////" <<std::endl <<std::endl;
            }
            else{
                RCLCPP_INFO_ONCE(this->get_logger(), "Trajectory executed successfully ...");
                
                // Send joint velocity commands
                if(cmd_interface_ == "position"){
                    // Set joint position commands
                    for (long int i = 0; i < joint_positions_.data.size(); ++i) {
                        desired_commands_[i] = joint_positions_cmd_(i);
                    }
                }
                else if(cmd_interface_ == "velocity"){
                    // Set joint velocity commands
                    for (long int i = 0; i < joint_velocities_.data.size(); ++i) {
                        desired_commands_[i] = 0.0;
                    }
                }
                else if(cmd_interface_ == "effort"){
                    // Set joint effort commands
                    for (long int i = 0; i < joint_efforts_cmd_.data.size(); ++i) {
                        desired_commands_[i] = joint_efforts_cmd_(i);
                    }
                }
                
                // Create msg and publish
                std_msgs::msg::Float64MultiArray cmd_msg;
                cmd_msg.data = desired_commands_;
                cmdPublisher_->publish(cmd_msg);
            }
        }

        void joint_state_subscriber(const sensor_msgs::msg::JointState& sensor_msg){

            joint_state_available_ = true;
            for (unsigned int i  = 0; i < sensor_msg.position.size(); i++){
                joint_positions_.data[i] = sensor_msg.position[i];
                joint_velocities_.data[i] = sensor_msg.velocity[i];
            }
        }

        void aruco_pose_cb_(const geometry_msgs::msg::PoseStamped::SharedPtr msg){

            geometry_msgs::msg::PoseStamped cam_pose;
            cam_pose = *msg;
            // p_cO in camera optical frame
            p_cO_ = Eigen::Vector3d(cam_pose.pose.position.x,
                                    cam_pose.pose.position.y,
                                    cam_pose.pose.position.z);
            have_marker_ = true;
        }



        void setArucoPoseCb(const std::shared_ptr<ros2_kdl_interfaces::srv::SetArucoPose::Request> req,
        std::shared_ptr<ros2_kdl_interfaces::srv::SetArucoPose::Response> res)
        {
            if (!gz_set_pose_client_->service_is_ready()) {
                res->success = false;
                res->message = "Gazebo set_pose not ready";
                return;
            }

            auto gz_req = std::make_shared<ros_gz_interfaces::srv::SetEntityPose::Request>();
            gz_req->entity.name = req->name;
            
            gz_req->pose = req->pose;  

            auto fut = gz_set_pose_client_->async_send_request(gz_req);

            rclcpp::executors::SingleThreadedExecutor exec;
            exec.add_node(client_node_);
            auto rc = exec.spin_until_future_complete(fut, 1s);
            exec.remove_node(client_node_);

            if (rc != rclcpp::FutureReturnCode::SUCCESS) {
            res->success = false;
            res->message = (rc == rclcpp::FutureReturnCode::TIMEOUT) ? "timeout" : "error";
            return;
            }
            res->success = fut.get()->success;    
            res->message = res->success ? "OK" : "FAILED";

        }



        //////////////////////////////////////////////////////////
        rclcpp_action::GoalResponse handle_goal(
            const rclcpp_action::GoalUUID&,
            std::shared_ptr<const ExecuteLinearTraj::Goal> goal)
        {
        
        if (goal->traj_duration <= 0.0 || goal->trajectory_len <= 0) {
            RCLCPP_WARN(this->get_logger(), "Rejecting goal: traj_duration>0, trajectory_len>0 required.");
            return rclcpp_action::GoalResponse::REJECT;
        }
        if (goal->acc_duration < 0.0 || goal->acc_duration > goal->traj_duration) {
            RCLCPP_WARN(this->get_logger(), "Rejecting goal: 0 <= acc_duration <= traj_duration.");
            return rclcpp_action::GoalResponse::REJECT;
        }
        if (!(goal->s_type == "trapezoidal" || goal->s_type == "cubic")) {
            RCLCPP_WARN(this->get_logger(), "Rejecting goal: s_type must be 'trapezoidal' o 'cubic'.");
            return rclcpp_action::GoalResponse::REJECT;
        }
        if (has_active_goal_.load()) {
            RCLCPP_INFO(this->get_logger(), "Rejecting goal: another keeps running.");
            return rclcpp_action::GoalResponse::REJECT;
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }

        rclcpp_action::CancelResponse handle_cancel(
            const std::shared_ptr<GoalHandleExec> )
        {
        RCLCPP_INFO(this->get_logger(), "Cancel request.");
        return rclcpp_action::CancelResponse::ACCEPT;
        }

        void handle_accepted(
            const std::shared_ptr<GoalHandleExec> goal_handle)
        {
            {
                std::lock_guard<std::mutex> lk(active_goal_mtx_);
                has_active_goal_.store(true);
                active_goal_ = goal_handle;
            }
        
            std::thread([this, goal_handle](){
                this->execute_goal(goal_handle);
                {
                    std::lock_guard<std::mutex> lk(active_goal_mtx_);
                    has_active_goal_.store(false);
                    active_goal_.reset();
                }
            }).detach();
        }

        void execute_goal(
            const std::shared_ptr<GoalHandleExec> goal_handle)
        {
            auto goal = goal_handle->get_goal();

            // goal parameters
            const double T  = goal->traj_duration;
            const double Ta = goal->acc_duration;
            const int    N  = goal->trajectory_len;
            const bool   trapezoid = (goal->s_type == "trapezoidal");

            const double rate_hz = static_cast<double>(N) / T;
            rclcpp::Rate rate(rate_hz);
            double t = 0.0;

            // Start/goal 
            robot_->update(toStdVector(joint_positions_.data), toStdVector(joint_velocities_.data));

            KDL::Frame start_cart = robot_->getEEFrame();
            Eigen::Vector3d start_pos(Eigen::Vector3d(start_cart.p.data));
            Eigen::Vector3d goal_pos(goal->end_position.x, goal->end_position.y, goal->end_position.z);

            // Planner 
            KDLPlanner planner(T, Ta, start_pos, goal_pos);

            // Controller gains and ctrl_mode
            const double Kp = Kp_;
            const std::string ctrl_mode = ctrl_mode_;
            const std::string cmd_if = cmd_interface_;

    
            double last_err_norm = 0.0;

            RCLCPP_INFO(this->get_logger(),
                        "Executing linear traj to [%.3f %.3f %.3f], T=%.3f, acc=%.3f, N=%d, s=%s",
                        goal_pos[0], goal_pos[1], goal_pos[2], T, Ta, N, goal->s_type.c_str());

            while (rclcpp::ok() && t <= T) {
                if (goal_handle->is_canceling()) {
                // stop the robot
                std_msgs::msg::Float64MultiArray stop;
                stop.data = desired_commands_;
                for (auto &v : stop.data) v = 0.0;
                cmdPublisher_->publish(stop);

                auto res = std::make_shared<ExecuteLinearTraj::Result>();
                res->success = false;
                res->final_error_norm = last_err_norm;
                goal_handle->canceled(res);
                RCLCPP_INFO(this->get_logger(), "Goal CANCELED.");
                return;
                }

            
                trajectory_point p;
                if (trapezoid) p = planner.linear_traj_trapezoidal(t);
                else           p = planner.linear_traj_cubic(t);

                KDL::Frame cart = robot_->getEEFrame();

                Eigen::Vector3d e = computeLinearError(p.pos, Eigen::Vector3d(cart.p.data));
                Eigen::Vector3d o_error = computeOrientationError(toEigen(start_cart.M), toEigen(cart.M));
                last_err_norm = e.norm();

                // Control
                if (cmd_if == "position") {
                KDL::Frame next = cart;
                next.p = cart.p + (toKDL(p.vel) + toKDL(Kp*e))*(1.0/rate_hz);
                joint_positions_cmd_ = joint_positions_;
                robot_->getInverseKinematics(next, joint_positions_cmd_);
                for (long int i = 0; i < joint_positions_.data.size(); ++i)
                    desired_commands_[i] = joint_positions_cmd_(i);
                } else if (cmd_if == "velocity") {
                Vector6d cartvel; cartvel << p.vel + Kp*e, o_error;
                if (ctrl_mode == "velocity_ctrl") {
                    joint_velocities_cmd_.data = pseudoinverse(robot_->getEEJacobian().data)*cartvel;
                } else { // "velocity_ctrl_null"
                    joint_velocities_cmd_.data = controller_->velocity_ctrl_null(
                    e, o_error, p.vel, Kp, q_min_, q_max_, lambda_);
                }
                for (long int i = 0; i < joint_velocities_.data.size(); ++i)
                    desired_commands_[i] = joint_velocities_cmd_(i);
                } else if (cmd_if == "effort") {
                
                for (long int i = 0; i < joint_efforts_cmd_.data.size(); ++i)
                    desired_commands_[i] = joint_efforts_cmd_(i);
                }

                
                robot_->update(toStdVector(joint_positions_.data), toStdVector(joint_velocities_.data));

                // Publish commands
                std_msgs::msg::Float64MultiArray cmd;
                cmd.data = desired_commands_;
                cmdPublisher_->publish(cmd);

                // Feedback
                auto fb = std::make_shared<ExecuteLinearTraj::Feedback>();
                fb->elapsed_time = t;
                fb->error_xyz = { e[0], e[1], e[2] };
                goal_handle->publish_feedback(fb);

                // step
                rate.sleep();
                t += 1.0/rate_hz;
            }

            if (cmd_if == "velocity") {
                std_msgs::msg::Float64MultiArray stop;
                stop.data = desired_commands_;
                for (auto &v : stop.data) v = 0.0;
                cmdPublisher_->publish(stop);
            }
            auto res = std::make_shared<ExecuteLinearTraj::Result>();
            res->success = true;
            res->final_error_norm = last_err_norm;
            goal_handle->succeed(res);
            RCLCPP_INFO(this->get_logger(), "Goal SUCCEEDED. Final error=%.6f", last_err_norm);
        }


        void publish_zeros_()
        {
        std_msgs::msg::Float64MultiArray stop;
        stop.data.assign(desired_commands_.size(), 0.0);
        cmdPublisher_->publish(stop);
        }

        /////////////////////////////////////////////////////////////////////////////////////////////////
        
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointSubscriber_;
        rclcpp::Publisher<FloatArray>::SharedPtr cmdPublisher_;
        rclcpp::TimerBase::SharedPtr timer_; 
        rclcpp::TimerBase::SharedPtr subTimer_;
        rclcpp::Node::SharedPtr node_handle_;
        

        std::vector<double> desired_commands_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        KDL::JntArray joint_positions_;
        KDL::JntArray joint_velocities_;

        KDL::JntArray joint_positions_cmd_;
        KDL::JntArray joint_velocities_cmd_;
        KDL::JntArray joint_efforts_cmd_;

        std::shared_ptr<KDLRobot> robot_;
        KDLPlanner planner_;

        trajectory_point p_;

        int iteration_;
        bool joint_state_available_;
        double t_;
        std::string cmd_interface_;
        std::string traj_type_;
        std::string s_type_;
        // point 1.a
        double traj_duration_;
        double traj_radius_;
        double acc_duration_;
        double total_time_;
        int    trajectory_len_;
        double Kp_;
        Eigen::Vector3d end_position_; // x,y,z in base frame
        std::string ctrl_mode_;
        KDL::JntArray q_min_, q_max_;
        double lambda_;
        std::unique_ptr<KDLController> controller_;

        //point 1.c
        bool action_mode_{true};
        // ---- ACTION SERVER ----
        rclcpp_action::Server<ExecuteLinearTraj>::SharedPtr action_server_;
        std::atomic<bool> has_active_goal_{false};
        std::mutex active_goal_mtx_;
        std::weak_ptr<GoalHandleExec> active_goal_;
        
        //point 2.b
        double Kv_{0.7};
        bool have_marker_{false};
        Eigen::Vector3d p_cO_;                
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
        std::string ee_frame_, cam_frame_;
        KDL::Frame F_ec_;
        bool have_F_ec_{false};
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr aruco_pose_sub_;
        
        //point 2.c
        rclcpp::Service<ros2_kdl_interfaces::srv::SetArucoPose>::SharedPtr set_marker_pose_srv_;
        std::string world_name_{"default"};

        rclcpp::Node::SharedPtr client_node_;
        rclcpp::Client<ros_gz_interfaces::srv::SetEntityPose>::SharedPtr gz_set_pose_client_;


        KDL::Frame init_cart_pose_;
};

 
int main( int argc, char** argv )
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Iiwa_pub_sub>());
    rclcpp::shutdown();
    return 1;
}