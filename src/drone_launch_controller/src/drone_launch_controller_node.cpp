#include <ros/ros.h>
#include <drone_launch_controller/takeoff_controller.h>
#include <drone_launch_controller/flight_status_publisher.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/ParamSet.h>
#include <std_srvs/Empty.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/PoseStamped.h>

namespace drone_launch_controller {

class DroneLaunchControllerNode {
public:
    DroneLaunchControllerNode()
        : nh_("~")
        , takeoff_controller_(nh_)
        , status_publisher_(nh_, &takeoff_controller_)
    {
        state_sub_ = nh_.subscribe("/mavros/state", 10, &DroneLaunchControllerNode::stateCallback, this);
        local_pos_sub_ = nh_.subscribe("/mavros/local_position/pose", 10, &DroneLaunchControllerNode::localPosCallback, this);
        takeoff_service_ = nh_.advertiseService("/drone_launch_controller/takeoff", &DroneLaunchControllerNode::takeoffCallback, this);
        health_check_service_ = nh_.advertiseService("/drone_launch_controller/drone_health_check", &DroneLaunchControllerNode::healthCheckCallback, this);
        arm_service_ = nh_.advertiseService("/drone_launch_controller/arm_motors", &DroneLaunchControllerNode::armMotorsCallback, this);
        connection_service_ = nh_.advertiseService("/drone_launch_controller/connection_status", &DroneLaunchControllerNode::connectionStatusCallback, this);

        set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
        arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
        command_client_ = nh_.serviceClient<mavros_msgs::CommandLong>("/mavros/cmd/command");
        param_set_client_ = nh_.serviceClient<mavros_msgs::ParamSet>("/mavros/param/set");
        local_pos_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);

        ROS_INFO("DroneLaunchControllerNode initialized");
    }

    void run() {
        status_publisher_.startPublishing(10.0);
        ros::spin();
    }

private:
    void stateCallback(const mavros_msgs::State::ConstPtr& state) {
        current_state_ = *state;
        state_received_ = true;
    }

    void localPosCallback(const geometry_msgs::PoseStamped::ConstPtr& pose) {
        current_pose_ = *pose;
        pose_received_ = true;
    }

    void publishSetpoint(const geometry_msgs::PoseStamped& pose) {
        geometry_msgs::PoseStamped setpoint = pose;
        setpoint.header.stamp = ros::Time::now();
        setpoint.header.frame_id = "map";
        local_pos_pub_.publish(setpoint);
    }

    bool sendSetpointsBeforeOffboard(double duration_seconds, double rate_hz) {
        ROS_INFO("[Setpoint Pre-send] Starting to send setpoints for %.1f seconds at %.1f Hz...", duration_seconds, rate_hz);
        
        if (!pose_received_) {
            ROS_ERROR("[Setpoint Pre-send] No local position received yet! Cannot send setpoints.");
            return false;
        }

        ros::Rate rate(rate_hz);
        int num_setpoints = static_cast<int>(duration_seconds * rate_hz);
        
        for (int i = 0; i < num_setpoints && ros::ok(); ++i) {
            publishSetpoint(current_pose_);
            ros::spinOnce();
            rate.sleep();
        }
        
        ROS_INFO("[Setpoint Pre-send] Sent %d setpoints successfully", num_setpoints);
        return true;
    }

    bool disableRTLAndFailsafe() {
        ROS_INFO("[PX4 Config] Disabling RTL and RC failsafe for SITL...");
        
        // Set NAV_RCL_ACT to 0 (disabled) - RC loss action
        mavros_msgs::ParamSet param_set;
        param_set.request.param_id = "NAV_RCL_ACT";
        param_set.request.value.integer = 0;
        param_set.request.value.real = 0.0;
        
        if (param_set_client_.call(param_set)) {
            if (param_set.response.success) {
                ROS_INFO("[PX4 Config] NAV_RCL_ACT set to 0 (disabled)");
            } else {
                ROS_WARN("[PX4 Config] Failed to set NAV_RCL_ACT");
            }
        } else {
            ROS_WARN("[PX4 Config] Could not call param/set service");
        }
        
        // Set NAV_DLL_ACT to 0 (disabled) - Data link loss action
        param_set.request.param_id = "NAV_DLL_ACT";
        param_set.request.value.integer = 0;
        param_set.request.value.real = 0.0;
        
        if (param_set_client_.call(param_set)) {
            if (param_set.response.success) {
                ROS_INFO("[PX4 Config] NAV_DLL_ACT set to 0 (disabled)");
            } else {
                ROS_WARN("[PX4 Config] Failed to set NAV_DLL_ACT");
            }
        }
        
        // Set COM_RCL_EXCEPT to 4 (allow offboard without RC)
        param_set.request.param_id = "COM_RCL_EXCEPT";
        param_set.request.value.integer = 4;
        param_set.request.value.real = 4.0;
        
        if (param_set_client_.call(param_set)) {
            if (param_set.response.success) {
                ROS_INFO("[PX4 Config] COM_RCL_EXCEPT set to 4 (allow offboard without RC)");
            } else {
                ROS_WARN("[PX4 Config] Failed to set COM_RCL_EXCEPT");
            }
        }
        
        // Give PX4 time to process parameter changes
        ros::Duration(0.5).sleep();
        return true;
    }

    bool waitForOffboardMode(double timeout_seconds) {
        ROS_INFO("[Offboard Mode] Waiting for OFFBOARD mode confirmation...");
        ros::Time start = ros::Time::now();
        ros::Rate rate(10);
        
        while (ros::ok()) {
            ros::spinOnce();
            
            if (current_state_.mode == "OFFBOARD") {
                ROS_INFO("[Offboard Mode] Confirmed: now in OFFBOARD mode");
                return true;
            }
            
            ros::Duration elapsed = ros::Time::now() - start;
            if (elapsed.toSec() > timeout_seconds) {
                ROS_ERROR("[Offboard Mode] Timeout waiting for OFFBOARD mode. Current mode: %s", 
                          current_state_.mode.c_str());
                return false;
            }
            
            rate.sleep();
        }
        return false;
    }

    bool setOffboardMode(int max_retries = 3) {
        ROS_INFO("[Offboard Mode] Attempting to set OFFBOARD mode...");
        
        for (int attempt = 1; attempt <= max_retries; ++attempt) {
            ROS_INFO("[Offboard Mode] Attempt %d/%d", attempt, max_retries);
            
            mavros_msgs::SetMode set_mode_srv;
            set_mode_srv.request.custom_mode = "OFFBOARD";
            
            if (set_mode_client_.call(set_mode_srv)) {
                if (set_mode_srv.response.mode_sent) {
                    ROS_INFO("[Offboard Mode] Mode switch request sent, waiting for confirmation...");
                    // Wait for the mode to actually change
                    if (waitForOffboardMode(3.0)) {
                        ros::Duration(0.5).sleep();
                        return true;
                    }
                    ROS_WARN("[Offboard Mode] Mode request sent but mode did not change");
                } else {
                    ROS_WARN("[Offboard Mode] Mode switch request rejected (attempt %d)", attempt);
                }
            } else {
                ROS_ERROR("[Offboard Mode] Failed to call set_mode service (attempt %d)", attempt);
            }
            
            if (attempt < max_retries) {
                ROS_INFO("[Offboard Mode] Retrying in 1 second...");
                ros::Duration(1.0).sleep();
            }
        }
        
        ROS_ERROR("[Offboard Mode] Failed to set OFFBOARD mode after %d attempts", max_retries);
        return false;
    }

    bool armMotorsWithCheck() {
        ROS_INFO("[Arming] Checking preconditions for arming...");
        
        if (!state_received_ || !current_state_.connected) {
            ROS_ERROR("[Arming] MAVROS not connected, cannot arm");
            return false;
        }
        
        if (current_state_.mode != "OFFBOARD") {
            ROS_ERROR("[Arming] Not in OFFBOARD mode (current: %s), cannot arm", current_state_.mode.c_str());
            return false;
        }
        
        if (!pose_received_) {
            ROS_ERROR("[Arming] No local position received, cannot arm");
            return false;
        }
        
        ROS_INFO("[Arming] All preconditions met, sending arm command...");
        mavros_msgs::CommandBool arm_cmd;
        arm_cmd.request.value = true;
        
        if (arming_client_.call(arm_cmd)) {
            if (arm_cmd.response.success) {
                ROS_INFO("[Arming] Motors armed successfully");
                ros::Duration(0.5).sleep();
                return true;
            } else {
                ROS_ERROR("[Arming] Arming failed, result code: %d", arm_cmd.response.result);
                return false;
            }
        } else {
            ROS_ERROR("[Arming] Failed to call arming service");
            return false;
        }
    }

    bool takeoffCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
        ROS_INFO("========================================");
        ROS_INFO("[Takeoff] Takeoff service called");
        ROS_INFO("========================================");

        if (!state_received_ || !current_state_.connected) {
            ROS_ERROR("[Takeoff] MAVROS not connected, cannot takeoff");
            ROS_ERROR("[Takeoff] state_received: %s, connected: %s", 
                      state_received_ ? "true" : "false",
                      current_state_.connected ? "true" : "false");
            return true;
        }
        
        ROS_INFO("[Takeoff] MAVROS connected: Yes");
        ROS_INFO("[Takeoff] Current mode: %s", current_state_.mode.c_str());
        ROS_INFO("[Takeoff] Armed: %s", current_state_.armed ? "Yes" : "No");
        ROS_INFO("[Takeoff] Pose received: %s", pose_received_ ? "Yes" : "No");

        // Step 0: Disable RTL and failsafe for SITL
        disableRTLAndFailsafe();

        // Step 1: Send setpoints before switching to OFFBOARD (PX4 requirement)
        if (!sendSetpointsBeforeOffboard(2.0, 20.0)) {
            ROS_ERROR("[Takeoff] Failed to pre-send setpoints");
            return true;
        }

        // Step 2: Switch to OFFBOARD mode
        if (current_state_.mode != "OFFBOARD") {
            if (!setOffboardMode(3)) {
                ROS_ERROR("[Takeoff] Failed to set OFFBOARD mode, aborting takeoff");
                return true;
            }
        } else {
            ROS_INFO("[Takeoff] Already in OFFBOARD mode");
        }

        // Step 3: Arm motors
        if (!current_state_.armed) {
            if (!armMotorsWithCheck()) {
                ROS_ERROR("[Takeoff] Failed to arm motors, aborting takeoff");
                return true;
            }
        } else {
            ROS_INFO("[Takeoff] Motors already armed");
        }

        // Step 4: Execute takeoff
        float target_altitude = 5.0f;
        float timeout = 60.0f;

        ROS_INFO("[Takeoff] Starting takeoff to %.2f meters with timeout %.1f seconds", target_altitude, timeout);
        bool takeoff_success = takeoff_controller_.takeoff(target_altitude, timeout);

        if (takeoff_success) {
            ROS_INFO("[Takeoff] Takeoff initiated successfully, monitoring progress...");
            ros::Rate rate(10);
            int timeout_count = 0;
            int max_timeout = static_cast<int>(timeout * 10);

            while (ros::ok() && takeoff_controller_.isInProgress() && timeout_count < max_timeout) {
                ros::spinOnce();
                rate.sleep();
                timeout_count++;
            }

            FlightState state = takeoff_controller_.getCurrentState();
            if (state == HOVER) {
                ROS_INFO("[Takeoff] SUCCESS! Hovering at %.2f meters", takeoff_controller_.getCurrentAltitude());
            } else if (state == EMERGENCY) {
                ROS_ERROR("[Takeoff] FAILED! Emergency state detected");
            } else {
                ROS_WARN("[Takeoff] Ended with state: %d (expected HOVER=%d)", state, HOVER);
            }
        } else {
            ROS_ERROR("[Takeoff] Failed to initiate takeoff");
        }

        ROS_INFO("========================================");
        return true;
    }

    bool healthCheckCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
        ROS_INFO("========================================");
        ROS_INFO("[Health Check] Health check service called");
        ROS_INFO("[Health Check] Connected: %s", current_state_.connected ? "Yes" : "No");
        ROS_INFO("[Health Check] Armed: %s", current_state_.armed ? "Yes" : "No");
        ROS_INFO("[Health Check] Mode: %s", current_state_.mode.c_str());
        ROS_INFO("[Health Check] Pose received: %s", pose_received_ ? "Yes" : "No");
        if (pose_received_) {
            ROS_INFO("[Health Check] Current altitude: %.2f m", current_pose_.pose.position.z);
        }
        ROS_INFO("========================================");
        return true;
    }

    bool armMotorsCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
        ROS_INFO("[Arm Service] Arm motors service called");

        if (!state_received_ || !current_state_.connected) {
            ROS_ERROR("[Arm Service] MAVROS not connected");
            return true;
        }

        // Disable RTL and failsafe
        disableRTLAndFailsafe();

        // Pre-send setpoints before attempting to switch to OFFBOARD
        if (!sendSetpointsBeforeOffboard(2.0, 20.0)) {
            ROS_ERROR("[Arm Service] Failed to pre-send setpoints");
            return true;
        }

        if (current_state_.mode != "OFFBOARD") {
            if (!setOffboardMode(3)) {
                ROS_ERROR("[Arm Service] Failed to set OFFBOARD mode");
                return true;
            }
        }

        if (armMotorsWithCheck()) {
            ROS_INFO("[Arm Service] Arm motors completed successfully");
        } else {
            ROS_ERROR("[Arm Service] Arm motors failed");
        }

        return true;
    }

    bool connectionStatusCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
        ROS_INFO("[Connection] Connection status: %s", current_state_.connected ? "Connected" : "Disconnected");
        if (current_state_.connected) {
            ROS_INFO("[Connection] Mode: %s, Armed: %s", 
                     current_state_.mode.c_str(), 
                     current_state_.armed ? "Yes" : "No");
        }
        return true;
    }

    ros::NodeHandle nh_;
    ros::Subscriber state_sub_;
    ros::Subscriber local_pos_sub_;
    ros::ServiceServer takeoff_service_;
    ros::ServiceServer health_check_service_;
    ros::ServiceServer arm_service_;
    ros::ServiceServer connection_service_;

    ros::ServiceClient set_mode_client_;
    ros::ServiceClient arming_client_;
    ros::ServiceClient command_client_;
    ros::ServiceClient param_set_client_;
    ros::Publisher local_pos_pub_;

    TakeoffController takeoff_controller_;
    FlightStatusPublisher status_publisher_;

    mavros_msgs::State current_state_;
    geometry_msgs::PoseStamped current_pose_;
    bool state_received_ = false;
    bool pose_received_ = false;
};

}

int main(int argc, char** argv) {
    ros::init(argc, argv, "drone_launch_controller");
    ROS_INFO("========================================");
    ROS_INFO("Drone Launch Controller Node Started");
    ROS_INFO("========================================");

    drone_launch_controller::DroneLaunchControllerNode node;
    node.run();

    return 0;
}
