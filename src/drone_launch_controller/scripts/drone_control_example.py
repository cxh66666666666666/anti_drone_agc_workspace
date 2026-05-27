#!/usr/bin/env python3

import rospy
import sys
import argparse
from drone_launch_controller.srv import (
    ConnectionStatus, ConnectionStatusRequest,
    DroneHealthCheck, DroneHealthCheckRequest,
    ArmMotors, ArmMotorsRequest,
    Takeoff, TakeoffRequest
)
from drone_launch_controller.msg import FlightStatus

FLIGHT_STATE_NAMES = {
    0: "INIT",
    1: "CONNECTED",
    2: "HEALTH_CHECK",
    3: "ARMED",
    4: "TAKEOFF",
    5: "HOVER",
    6: "LAND",
    7: "EMERGENCY"
}


class DroneController:
    def __init__(self):
        rospy.init_node('drone_control_example', anonymous=True)

        self._wait_for_services()
        self._setup_subscribers()

        self.latest_status = None
        self.connection_established = False

    def _wait_for_services(self):
        rospy.loginfo("Waiting for services...")
        try:
            rospy.wait_for_service('/drone_launch_controller/connection_status', timeout=10.0)
            rospy.wait_for_service('/drone_launch_controller/drone_health_check', timeout=10.0)
            rospy.wait_for_service('/drone_launch_controller/arm_motors', timeout=10.0)
            rospy.wait_for_service('/drone_launch_controller/takeoff', timeout=10.0)
            rospy.loginfo("All services available")
        except rospy.ROSException as e:
            rospy.logerr(f"Service timeout: {e}")
            sys.exit(1)

        self.connection_srv = rospy.ServiceProxy(
            '/drone_launch_controller/connection_status', ConnectionStatus)
        self.health_srv = rospy.ServiceProxy(
            '/drone_launch_controller/drone_health_check', DroneHealthCheck)
        self.arm_srv = rospy.ServiceProxy(
            '/drone_launch_controller/arm_motors', ArmMotors)
        self.takeoff_srv = rospy.ServiceProxy(
            '/drone_launch_controller/takeoff', Takeoff)

    def _setup_subscribers(self):
        self.status_sub = rospy.Subscriber(
            '/drone_launch_controller/flight_status', FlightStatus,
            self._status_callback, queue_size=10)

    def _status_callback(self, msg):
        self.latest_status = msg

    def check_connection(self):
        try:
            resp = self.connection_srv()
            status = "Connected" if resp.connected else "Disconnected"
            rospy.loginfo(f"[Connection] Status: {status}, System ID: {resp.system_id}")
            rospy.loginfo(f"[Connection] Message: {resp.message}")
            self.connection_established = resp.connected
            return resp.connected
        except rospy.ServiceException as e:
            rospy.logerr(f"[Connection] Service call failed: {e}")
            return False

    def perform_health_check(self):
        rospy.loginfo("[HealthCheck] Performing system health check...")
        try:
            resp = self.health_srv()
            result_str = "PASS" if resp.success else "FAIL"
            rospy.loginfo(f"[HealthCheck] Result: {result_str}")

            hs = resp.health_status
            rospy.loginfo(f"[HealthCheck] Drone: {hs.drone_name}")
            rospy.loginfo(f"[HealthCheck] Battery: {hs.battery_voltage:.2f}V ({hs.battery_percentage*100:.0f}%)")
            rospy.loginfo(f"[HealthCheck] GPS: {'OK' if hs.gps_status else 'FAIL'}")
            rospy.loginfo(f"[HealthCheck] IMU: {'OK' if hs.imu_status else 'FAIL'}")
            rospy.loginfo(f"[HealthCheck] Barometer: {'OK' if hs.barometer_status else 'FAIL'}")
            rospy.loginfo(f"[HealthCheck] Magnetometer: {'OK' if hs.magnetometer_status else 'FAIL'}")
            rospy.loginfo(f"[HealthCheck] ESC: {'OK' if hs.esc_status else 'FAIL'}")
            rospy.loginfo(f"[HealthCheck] Error Code: 0x{hs.error_code:04X}")
            rospy.loginfo(f"[HealthCheck] Message: {hs.status_message}")
            return resp.success
        except rospy.ServiceException as e:
            rospy.logerr(f"[HealthCheck] Service call failed: {e}")
            return False

    def arm_motors(self, force=False):
        rospy.loginfo(f"[ArmMotors] Attempting to arm motors (force={force})...")
        try:
            req = ArmMotorsRequest()
            req.force = force
            resp = self.arm_srv(req)
            status = "SUCCESS" if resp.success else "FAIL"
            rospy.loginfo(f"[ArmMotors] Result: {status}")
            rospy.loginfo(f"[ArmMotors] Response Code: {resp.result}")
            rospy.loginfo(f"[ArmMotors] Message: {resp.message}")
            return resp.success
        except rospy.ServiceException as e:
            rospy.logerr(f"[ArmMotors] Service call failed: {e}")
            return False

    def takeoff(self, altitude, timeout=30.0):
        rospy.loginfo(f"[Takeoff] Commanding takeoff to {altitude}m (timeout={timeout}s)...")
        try:
            req = TakeoffRequest()
            req.altitude = altitude
            req.timeout = timeout
            resp = self.takeoff_srv(req)
            status = "SUCCESS" if resp.success else "FAIL"
            rospy.loginfo(f"[Takeoff] Result: {status}")
            rospy.loginfo(f"[Takeoff] Response Code: {resp.result}")
            rospy.loginfo(f"[Takeoff] Achieved Altitude: {resp.achieved_altitude:.2f}m")
            rospy.loginfo(f"[Takeoff] Message: {resp.message}")
            return resp.success
        except rospy.ServiceException as e:
            rospy.logerr(f"[Takeoff] Service call failed: {e}")
            return False

    def monitor_flight_status(self, duration=None):
        if duration:
            rospy.loginfo(f"[Monitor] Monitoring flight status for {duration} seconds...")
            end_time = rospy.Time.now() + rospy.Duration(duration)
            while not rospy.is_shutdown() and rospy.Time.now() < end_time:
                self._publish_current_status()
                rospy.sleep(0.5)
        else:
            rospy.loginfo("[Monitor] Monitoring flight status (Ctrl+C to stop)...")
            rate = rospy.Rate(1)
            try:
                while not rospy.is_shutdown():
                    self._publish_current_status()
                    rate.sleep()
            except rospy.ROSInterruptException:
                pass

    def _publish_current_status(self):
        if self.latest_status:
            state_name = FLIGHT_STATE_NAMES.get(
                self.latest_status.state, f"UNKNOWN({self.latest_status.state})")
            armed_str = "Armed" if self.latest_status.armed else "Disarmed"
            rospy.loginfo(
                f"[Status] Mode: {self.latest_status.flight_mode}, "
                f"State: {state_name}, "
                f"Armed: {armed_str}, "
                f"Alt: {self.latest_status.current_altitude:.2f}m / "
                f"{self.latest_status.target_altitude:.2f}m"
            )
        else:
            rospy.loginfo_throttle(5.0, "[Monitor] Waiting for flight status data...")


def print_separator(title):
    rospy.loginfo("=" * 50)
    rospy.loginfo(f" {title}")
    rospy.loginfo("=" * 50)


def main():
    parser = argparse.ArgumentParser(description='Drone Control Example')
    parser.add_argument('--altitude', type=float, default=5.0,
                       help='Target takeoff altitude in meters (default: 5.0)')
    parser.add_argument('--timeout', type=float, default=30.0,
                       help='Takeoff timeout in seconds (default: 30.0)')
    parser.add_argument('--force-arm', action='store_true',
                       help='Force arm motors without safety checks')
    parser.add_argument('--monitor-only', action='store_true',
                       help='Only monitor flight status, do not execute commands')
    parser.add_argument('--monitor-duration', type=float, default=None,
                       help='Monitor for specified duration then exit')
    args = parser.parse_args(rospy.myargv()[1:])

    drone = DroneController()

    if args.monitor_only:
        print_separator("MONITOR MODE")
        drone.monitor_flight_status(args.monitor_duration)
        return

    print_separator("STEP 1: CHECK CONNECTION")
    if not drone.check_connection():
        rospy.logerr("[Error] Drone not connected! Exiting.")
        sys.exit(1)

    print_separator("STEP 2: SYSTEM HEALTH CHECK")
    if not drone.perform_health_check():
        rospy.logwarn("[Warning] Health check reported issues!")
        user_input = input("Continue anyway? (y/n): ").strip().lower()
        if user_input != 'y':
            rospy.loginfo("Aborted by user.")
            sys.exit(0)

    print_separator("STEP 3: ARM MOTORS")
    if not drone.arm_motors(force=args.force_arm):
        rospy.logerr("[Error] Failed to arm motors! Exiting.")
        sys.exit(1)

    rospy.sleep(1.0)

    print_separator("STEP 4: TAKEOFF")
    if not drone.takeoff(args.altitude, args.timeout):
        rospy.logerr("[Error] Takeoff failed! Exiting.")
        sys.exit(1)

    print_separator("STEP 5: MONITOR FLIGHT")
    drone.monitor_flight_status()

    print_separator("SHUTDOWN COMPLETE")


if __name__ == '__main__':
    main()
