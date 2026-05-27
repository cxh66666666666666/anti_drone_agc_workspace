#include <drone_launch_controller/takeoff_controller.h>
#include <ros/ros.h>
#include <mavros_msgs/SetMode.h>
#include <geometry_msgs/PoseStamped.h>
#include <cmath>

namespace drone_launch_controller {

TakeoffController::TakeoffController(ros::NodeHandle& nh)
    : nh_(nh)
    , target_altitude_(0.0f)
    , takeoff_timeout_(60.0f)
    , takeoff_in_progress_(false)
    , current_state_(INIT)
    , paused_(false)
    , hover_maintain_running_(false)
{
    set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    local_pos_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
    local_pos_sub_ = nh_.subscribe("/mavros/local_position/pose", 10, &TakeoffController::localPosCallback, this);

    ROS_INFO("[TakeoffController] Initialized");
}

TakeoffController::~TakeoffController() {
    stopHoverMaintainThread();
    stopTakeoffThread();
}

void TakeoffController::localPosCallback(const geometry_msgs::PoseStamped::ConstPtr& pose) {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    current_pose_ = *pose;
}

float TakeoffController::getCurrentAltitude() const {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    return current_pose_.pose.position.z;
}

bool TakeoffController::isInProgress() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return takeoff_in_progress_;
}

FlightState TakeoffController::getCurrentState() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return current_state_;
}

void TakeoffController::setState(FlightState state) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (current_state_ != state) {
        ROS_INFO("[TakeoffController] Flight state changed: %d -> %d", current_state_, state);
        current_state_ = state;
    }
}

bool TakeoffController::takeoff(float target_altitude, float timeout) {
    if (takeoff_in_progress_) {
        ROS_WARN("[TakeoffController] Takeoff already in progress");
        return false;
    }

    // Stop any existing hover maintain thread
    stopHoverMaintainThread();

    target_altitude_ = target_altitude;
    takeoff_timeout_ = timeout;
    takeoff_start_time_ = ros::Time::now();
    takeoff_in_progress_ = true;
    paused_ = false;

    setState(TAKEOFF);

    ROS_INFO("[TakeoffController] Starting takeoff thread: target=%.2fm, timeout=%.1fs", 
             target_altitude, timeout);
    takeoff_thread_ = std::thread(&TakeoffController::takeoffLoop, this);

    return true;
}

void TakeoffController::takeoffLoop() {
    ROS_INFO("[TakeoffController] Takeoff loop started");
    ROS_INFO("[TakeoffController] Target altitude: %.2f m", target_altitude_);

    ros::Rate rate(20);
    float step_size = 0.5f;
    float current_target = 0.0f;
    int stable_count = 0;
    const int stable_threshold = 20;

    // Wait a moment for OFFBOARD mode to fully activate
    ROS_INFO("[TakeoffController] Waiting for OFFBOARD mode to stabilize...");
    ros::Duration(0.5).sleep();

    // Get initial altitude
    float initial_alt = getCurrentAltitude();
    ROS_INFO("[TakeoffController] Initial altitude: %.2f m", initial_alt);

    while (ros::ok() && takeoff_in_progress_) {
        if (paused_) {
            rate.sleep();
            continue;
        }

        // Check for GPS loss
        if (checkGPSLoss()) {
            ROS_ERROR("[TakeoffController] GPS loss detected during takeoff!");
            setState(EMERGENCY);
            takeoff_in_progress_ = false;
            break;
        }

        // Check for timeout
        ros::Duration elapsed = ros::Time::now() - takeoff_start_time_;
        if (elapsed.toSec() > takeoff_timeout_) {
            ROS_ERROR("[TakeoffController] Takeoff timeout! Elapsed: %.1f seconds", elapsed.toSec());
            setState(EMERGENCY);
            takeoff_in_progress_ = false;
            break;
        }

        float current_alt = getCurrentAltitude();
        
        // Gradual altitude increase
        if (current_target < target_altitude_) {
            current_target = std::min(current_target + step_size / 20.0f, target_altitude_);
        }

        publishSetpoint(current_target);

        // Log progress every 1 second (every 20 iterations at 20Hz)
        static int log_counter = 0;
        if (++log_counter >= 20) {
            ROS_INFO("[TakeoffController] Ascending: current=%.2f m, target=%.2f m, setpoint=%.2f m", 
                     current_alt, target_altitude_, current_target);
            log_counter = 0;
        }

        // Check if target altitude is reached (with hysteresis)
        if (current_alt >= target_altitude_ - 0.2f) {
            stable_count++;
            if (stable_count >= stable_threshold) {
                ROS_INFO("[TakeoffController] Target altitude reached and stable: %.2f m", current_alt);
                setState(HOVER);
                takeoff_in_progress_ = false;
                
                // Start hover maintain thread to continuously send setpoints
                startHoverMaintainThread();
                break;
            }
        } else {
            stable_count = 0;
        }

        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("[TakeoffController] Takeoff loop finished, final state: %d", getCurrentState());
}

void TakeoffController::startHoverMaintainThread() {
    if (hover_maintain_running_) {
        return;
    }
    
    hover_maintain_running_ = true;
    ROS_INFO("[TakeoffController] Starting hover maintain thread to keep sending setpoints");
    hover_maintain_thread_ = std::thread(&TakeoffController::hoverMaintainLoop, this);
}

void TakeoffController::stopHoverMaintainThread() {
    hover_maintain_running_ = false;
    if (hover_maintain_thread_.joinable()) {
        hover_maintain_thread_.join();
        ROS_INFO("[TakeoffController] Hover maintain thread stopped");
    }
}

void TakeoffController::hoverMaintainLoop() {
    ROS_INFO("[TakeoffController] Hover maintain loop started - continuously sending setpoints at target altitude");
    
    ros::Rate rate(20);  // 20Hz to maintain OFFBOARD mode
    int log_counter = 0;
    
    while (ros::ok() && hover_maintain_running_) {
        publishSetpoint(target_altitude_);
        
        // Log every 5 seconds (100 iterations at 20Hz)
        if (++log_counter >= 100) {
            ROS_INFO("[TakeoffController] Hovering at %.2f m - maintaining position", target_altitude_);
            log_counter = 0;
        }
        
        ros::spinOnce();
        rate.sleep();
    }
    
    ROS_INFO("[TakeoffController] Hover maintain loop ended");
}

void TakeoffController::publishSetpoint(float altitude) {
    geometry_msgs::PoseStamped setpoint;
    setpoint.header.stamp = ros::Time::now();
    setpoint.header.frame_id = "map";
    
    {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        setpoint.pose.position.x = current_pose_.pose.position.x;
        setpoint.pose.position.y = current_pose_.pose.position.y;
        setpoint.pose.orientation = current_pose_.pose.orientation;
    }
    
    setpoint.pose.position.z = altitude;
    local_pos_pub_.publish(setpoint);
}

bool TakeoffController::cancelTakeoff() {
    if (!takeoff_in_progress_) {
        ROS_WARN("[TakeoffController] No takeoff in progress to cancel");
        return false;
    }

    ROS_INFO("[TakeoffController] Cancelling takeoff");
    takeoff_in_progress_ = false;
    paused_ = false;

    stopTakeoffThread();
    setState(HOVER);

    // Start hover maintain at current altitude
    target_altitude_ = getCurrentAltitude();
    startHoverMaintainThread();

    return true;
}

bool TakeoffController::pauseTakeoff() {
    if (!takeoff_in_progress_) {
        ROS_WARN("[TakeoffController] No takeoff in progress to pause");
        return false;
    }

    ROS_INFO("[TakeoffController] Pausing takeoff");
    paused_ = true;
    return true;
}

bool TakeoffController::resumeTakeoff() {
    if (!takeoff_in_progress_) {
        ROS_WARN("[TakeoffController] No takeoff in progress to resume");
        return false;
    }

    if (!paused_) {
        ROS_WARN("[TakeoffController] Takeoff is not paused");
        return false;
    }

    ROS_INFO("[TakeoffController] Resuming takeoff");
    paused_ = false;
    return true;
}

bool TakeoffController::checkObstacle() {
    return false;
}

bool TakeoffController::checkGPSLoss() {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    if (current_pose_.header.stamp.isZero()) {
        ROS_WARN("[TakeoffController] GPS check: No pose data received yet");
        return true;
    }

    ros::Time now = ros::Time::now();
    ros::Time pose_time = current_pose_.header.stamp;
    double time_diff = (now - pose_time).toSec();
    
    if (time_diff > 2.0) {
        ROS_WARN("[TakeoffController] GPS data too old, time diff: %.2f seconds", time_diff);
        return true;
    }

    return false;
}

bool TakeoffController::checkControlTimeout() {
    ros::Time now = ros::Time::now();
    ros::Duration elapsed = now - takeoff_start_time_;
    return (elapsed.toSec() > takeoff_timeout_);
}

void TakeoffController::stopTakeoffThread() {
    if (takeoff_thread_.joinable()) {
        takeoff_thread_.join();
    }
}

}
