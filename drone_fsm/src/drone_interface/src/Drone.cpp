#include "drone/Drone.hpp"

#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <string>
#include <thread>
#include <unistd.h>

#include "px4_msgs/msg/vehicle_local_position_setpoint.hpp"
#include "tf2/utils.h"

#include <vision_msgs/msg/detection2_d_array.hpp>
#include <custom_msgs/msg/gesture.hpp>
#include <custom_msgs/msg/hand_location.hpp>
#include <custom_msgs/msg/bar_code.hpp>
#include <custom_msgs/msg/multi_bar_code.hpp>
#include "std_msgs/msg/string.hpp"


Drone::Drone() {

	//if (argc != 0 && argv != nullptr) {
	//	rclcpp::init(argc, argv);
	//}
	
	this->exec_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
	this->px4_node_ = std::make_shared<rclcpp::Node>("Drone");
	this->exec_->add_node(px4_node_);
	this->spin_thread_ = std::thread(
		[this]() {
			this->exec_->spin();
		}
	);

	rclcpp::QoS qos_profile(10);
	qos_profile.best_effort();

	std::string vehicle_id_prefix = "";

	// Essa parte será útil para múltiplos drones
	//if (this->vehicle_id_ != 0) {
	//	vehicle_id_prefix = "/px4_" + std::to_string(this->vehicle_id_);
	//	this->target_system_ = this->vehicle_id_ + 1;
	//}

	this->vehicle_status_sub_ = this->px4_node_->create_subscription<px4_msgs::msg::VehicleStatus>(
		vehicle_id_prefix + "/fmu/out/vehicle_status",
		qos_profile,
		[this](px4_msgs::msg::VehicleStatus::ConstSharedPtr msg) {
		auto set_arm_disarm_reason = [](uint8_t reason)
		{
			DronePX4::ARM_DISARM_REASON value;
			switch (reason) {
			case px4_msgs::msg::VehicleStatus::ARM_DISARM_REASON_TRANSITION_TO_STANDBY:
				value = DronePX4::ARM_DISARM_REASON::TRANSITION_TO_STANDBY;
				break;
			case px4_msgs::msg::VehicleStatus::ARM_DISARM_REASON_RC_STICK:
				value = DronePX4::ARM_DISARM_REASON::RC_STICK;
				break;
			case px4_msgs::msg::VehicleStatus::ARM_DISARM_REASON_RC_SWITCH:
				value = DronePX4::ARM_DISARM_REASON::RC_SWITCH;
				break;
			case px4_msgs::msg::VehicleStatus::ARM_DISARM_REASON_COMMAND_INTERNAL:
				value = DronePX4::ARM_DISARM_REASON::COMMAND_INTERNAL;
				break;
			case px4_msgs::msg::VehicleStatus::ARM_DISARM_REASON_COMMAND_EXTERNAL:
				value = DronePX4::ARM_DISARM_REASON::COMMAND_EXTERNAL;
				break;
			case px4_msgs::msg::VehicleStatus::ARM_DISARM_REASON_MISSION_START:
				value = DronePX4::ARM_DISARM_REASON::MISSION_START;
				break;
			case px4_msgs::msg::VehicleStatus::ARM_DISARM_REASON_SAFETY_BUTTON:
				value = DronePX4::ARM_DISARM_REASON::SAFETY_BUTTON;
				break;
			case px4_msgs::msg::VehicleStatus::ARM_DISARM_REASON_AUTO_DISARM_LAND:
				value = DronePX4::ARM_DISARM_REASON::AUTO_DISARM_LAND;
				break;
			case px4_msgs::msg::VehicleStatus::ARM_DISARM_REASON_AUTO_DISARM_PREFLIGHT:
				value = DronePX4::ARM_DISARM_REASON::AUTO_DISARM_PREFLIGHT;
				break;
			case px4_msgs::msg::VehicleStatus::ARM_DISARM_REASON_KILL_SWITCH:
				value = DronePX4::ARM_DISARM_REASON::KILL_SWITCH;
				break;
			case px4_msgs::msg::VehicleStatus::ARM_DISARM_REASON_LOCKDOWN:
				value = DronePX4::ARM_DISARM_REASON::LOCKDOWN;
				break;
			case px4_msgs::msg::VehicleStatus::ARM_DISARM_REASON_FAILURE_DETECTOR:
				value = DronePX4::ARM_DISARM_REASON::FAILURE_DETECTOR;
				break;
			case px4_msgs::msg::VehicleStatus::ARM_DISARM_REASON_SHUTDOWN:
				value = DronePX4::ARM_DISARM_REASON::SHUTDOWN;
				break;
			default:
				value = DronePX4::ARM_DISARM_REASON::ARM_DISARM_REASON_NONE;
			}
			return value;
		};

		this->arm_reason_ = set_arm_disarm_reason(msg->latest_arming_reason);
		this->disarm_reason_ = set_arm_disarm_reason(msg->latest_disarming_reason);

		switch (msg->failure_detector_status) {
			case px4_msgs::msg::VehicleStatus::FAILURE_NONE:
			this->failure_ = DronePX4::FAILURE::NONE;
			break;
			case px4_msgs::msg::VehicleStatus::FAILURE_ROLL:
			this->failure_ = DronePX4::FAILURE::ROLL;
			break;
			case px4_msgs::msg::VehicleStatus::FAILURE_PITCH:
			this->failure_ = DronePX4::FAILURE::PITCH;
			break;
			case px4_msgs::msg::VehicleStatus::FAILURE_ALT:
			this->failure_ = DronePX4::FAILURE::ALT;
			break;
			case px4_msgs::msg::VehicleStatus::FAILURE_ARM_ESC:
			this->failure_ = DronePX4::FAILURE::ARM_ESC;
			break;
			case px4_msgs::msg::VehicleStatus::FAILURE_BATTERY:
			this->failure_ = DronePX4::FAILURE::BATTERY;
			break;
			case px4_msgs::msg::VehicleStatus::FAILURE_IMBALANCED_PROP:
			this->failure_ = DronePX4::FAILURE::IMBALANCED_PROP;
			break;
			case px4_msgs::msg::VehicleStatus::FAILURE_MOTOR:
			this->failure_ = DronePX4::FAILURE::MOTOR;
			break;
			case px4_msgs::msg::VehicleStatus::FAILURE_EXT:
			this->failure_ = DronePX4::FAILURE::EXT;
			break;
			default:
			this->failure_ = DronePX4::FAILURE::NONE;
		}

		switch (msg->arming_state) {
			case px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED:
			this->arming_state_ = DronePX4::ARMING_STATE::ARMED;
			break;
			case px4_msgs::msg::VehicleStatus::ARMING_STATE_STANDBY:
			this->arming_state_ = DronePX4::ARMING_STATE::DISARMED;
			break;
			default:
			this->arming_state_ = DronePX4::ARMING_STATE::DISARMED;
		}

		switch (msg->nav_state) {
			case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_MANUAL:
			this->flight_mode_ = DronePX4::FLIGHT_MODE::MANUAL;
			break;
			case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_ALTCTL:
			this->flight_mode_ = DronePX4::FLIGHT_MODE::ALTCTL;
			break;
			case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_POSCTL:
			this->flight_mode_ = DronePX4::FLIGHT_MODE::POSCTL;
			break;
			case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_MISSION:
			this->flight_mode_ = DronePX4::FLIGHT_MODE::AUTO_MISSION;
			break;
			case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_LOITER:
			this->flight_mode_ = DronePX4::FLIGHT_MODE::AUTO_LOITER;
			break;
			case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_RTL:
			this->flight_mode_ = DronePX4::FLIGHT_MODE::AUTO_RTL;
			break;
			case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_ACRO:
			this->flight_mode_ = DronePX4::FLIGHT_MODE::ACRO;
			break;
			case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_TERMINATION:
			this->flight_mode_ = DronePX4::FLIGHT_MODE::TERMINATION;
			break;
			case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD:
			this->flight_mode_ = DronePX4::FLIGHT_MODE::OFFBOARD;
			break;
			case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_STAB:
			this->flight_mode_ = DronePX4::FLIGHT_MODE::STAB;
			break;
			case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_TAKEOFF:
			this->flight_mode_ = DronePX4::FLIGHT_MODE::AUTO_TAKEOFF;
			break;
			case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_LAND:
			this->flight_mode_ = DronePX4::FLIGHT_MODE::AUTO_LAND;
			break;
			case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_FOLLOW_TARGET:
			this->flight_mode_ = DronePX4::FLIGHT_MODE::AUTO_FOLLOW_TARGET;
			break;
			case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_PRECLAND:
			this->flight_mode_ = DronePX4::FLIGHT_MODE::AUTO_PRECLAND;
			break;
			case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_ORBIT:
			this->flight_mode_ = DronePX4::FLIGHT_MODE::ORBIT;
			break;
			case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_VTOL_TAKEOFF:
			this->flight_mode_ = DronePX4::FLIGHT_MODE::AUTO_VTOL_TAKEOFF;
			break;
			case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_DESCEND:
			this->flight_mode_ = DronePX4::FLIGHT_MODE::DESCEND;
			break;
			default:
			this->flight_mode_ = DronePX4::FLIGHT_MODE::UNKNOWN_MODE;
		}
		}
	);

	this->vehicle_timesync_sub_ = this->px4_node_->create_subscription<px4_msgs::msg::TimesyncStatus>(
		vehicle_id_prefix + "/fmu/out/timesync_status",
		qos_profile,
		[this](px4_msgs::msg::TimesyncStatus::ConstSharedPtr msg) {
		this->timestamp_ = std::chrono::time_point<std::chrono::high_resolution_clock>(
			std::chrono::nanoseconds(msg->timestamp));
		});

	this->vehicle_odometry_sub_ = this->px4_node_->create_subscription<px4_msgs::msg::VehicleOdometry>(
		vehicle_id_prefix + "/fmu/out/vehicle_odometry",
		qos_profile,
		[this](px4_msgs::msg::VehicleOdometry::ConstSharedPtr msg) {
			this->odom_timestamp_ = std::chrono::time_point<std::chrono::high_resolution_clock>(
				std::chrono::nanoseconds(msg->timestamp));
			this->ground_speed_ = std::sqrt(
				std::pow(msg->velocity[0], 2) + std::pow(msg->velocity[1], 2));

			this->current_pos_x_ = msg->position[0];
			this->current_pos_y_ = msg->position[1];
			this->current_pos_z_ = msg->position[2];
			this->current_vel_x_ = msg->velocity[0];
			this->current_vel_y_ = msg->velocity[1];
			this->current_vel_z_ = msg->velocity[2];

			// if the quaternion is valid, extract the euler angles for convenience
			if (msg->q[0] != NAN) {
				double y = 0, p = 0, r = 0;
				// the ordering is different: PX4 does wxyz, TF2/Bullet does xyzw
				tf2::getEulerYPR(
					tf2::Quaternion(msg->q[1], msg->q[2], msg->q[3], msg->q[0]),
					y, p, r
				);
				this->yaw_ = static_cast<float>(y);
				this->pitch_ = static_cast<float>(p);
				this->roll_ = static_cast<float>(r);
			}
		}
	);

	this->vehicle_airspeed_sub_ = this->px4_node_->create_subscription<px4_msgs::msg::Airspeed>(
		vehicle_id_prefix + "/fmu/out/airspeed",
		qos_profile,
		[this](px4_msgs::msg::Airspeed::ConstSharedPtr msg) {
			this->airspeed_ = msg->true_airspeed_m_s;
		}
	);

	this->vehicle_rates_setpoint_pub_ = this->px4_node_->create_publisher<px4_msgs::msg::VehicleRatesSetpoint>(
		vehicle_id_prefix + "/fmu/in/vehicle_rates_setpoint", qos_profile);

	this->vehicle_command_pub_ = this->px4_node_->create_publisher<px4_msgs::msg::VehicleCommand>(
		vehicle_id_prefix + "/fmu/in/vehicle_command", qos_profile);

	this->vehicle_trajectory_setpoint_pub_ = this->px4_node_->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
		vehicle_id_prefix + "/fmu/in/trajectory_setpoint", qos_profile);

	this->vehicle_offboard_control_mode_pub_ = this->px4_node_->create_publisher<px4_msgs::msg::OffboardControlMode>(
		vehicle_id_prefix + "/fmu/in/offboard_control_mode", qos_profile);

	this->vertical_camera_sub_ = this->px4_node_->create_subscription<sensor_msgs::msg::Image>(
		"/vertical_camera",
		qos_profile,
		[this](sensor_msgs::msg::Image::SharedPtr msg) {
			vertical_cv_ptr_ = cv_bridge::toCvCopy(msg, msg->encoding);
		}
	);

	this->horizontal_camera_sub_ = this->px4_node_->create_subscription<sensor_msgs::msg::Image>(
		"/horizontal_camera",
		qos_profile,
		[this](sensor_msgs::msg::Image::SharedPtr msg) {
			horizontal_cv_ptr_ = cv_bridge::toCvCopy(msg, msg->encoding);
		}
	);

	this->classification_sub_ = this->px4_node_->create_subscription<vision_msgs::msg::Detection2DArray>(
		"/vertical_classification",
		qos_profile,
		[this](vision_msgs::msg::Detection2DArray::SharedPtr msg){
			detections_.clear();
			for (const auto &detection : msg->detections) {
				bbox_center_x_ = detection.bbox.center.position.x;
				bbox_center_y_ = detection.bbox.center.position.y;
				bbox_size_x_ = detection.bbox.size_x;
				bbox_size_y_ = detection.bbox.size_y;
				detections_.push_back({bbox_center_x_, bbox_center_y_, bbox_size_x_, bbox_size_y_});
			}
		}
	);

	this->gesture_sub_ = this->px4_node_->create_subscription<custom_msgs::msg::Gesture>(
		"/gesture/classification",
		qos_profile,
		[this](custom_msgs::msg::Gesture::SharedPtr msg) {
			this->gestures_ = msg->gestures;
		}
	);

	this->hand_location_sub_ = this->px4_node_->create_subscription<custom_msgs::msg::HandLocation>(
		"/gesture/hand_location",
		qos_profile,
		[this](custom_msgs::msg::HandLocation::SharedPtr msg) {
			this->hand_location_x_ = msg->hand_x;
			this->hand_location_y_ = msg->hand_y;
		}
	);


	this->bar_code_sub_ = this->px4_node_->create_subscription<custom_msgs::msg::MultiBarCode>(
		"/barcode/bounding_boxes",
		qos_profile,
		[this](const custom_msgs::msg::MultiBarCode::SharedPtr msg) {
			barcode_detections_.clear();
			
			for (const auto &detection : msg->barcodes) {
				// Access the bounding box information in each BarCode message
				float bbox_center_x_ = detection.center_x;
				float bbox_center_y_ = detection.center_y;
				float bbox_size_x_ = detection.width;
				float bbox_size_y_ = detection.height;

				// Store the values in detections_ or process as needed
				barcode_detections_.push_back(Eigen::Vector4d({bbox_center_x_, bbox_center_y_, bbox_size_x_, bbox_size_y_}));
			}
		}
	);

	this->qr_code_sub_ = this->px4_node_->create_subscription<std_msgs::msg::String>(
		"/qr_code_string",
		qos_profile,
		[this](std_msgs::msg::String::SharedPtr msg) {
			this->qr_code_data_ = msg->data;
	});

}

Drone::~Drone() {
	this->destroy();
}

void Drone::destroy()
{
  if (this->exec_) {
    // this is the type of code you write just before the velociraptors attack
    for (int i = 0; i < 42; i++) {
      this->exec_->cancel();
      usleep(100);  // sleep a bit to force a thread context switch
    }
    // we've asked it to shut down many times, so clearly now we're good lol
    this->exec_ = nullptr;
    rclcpp::shutdown();
    this->spin_thread_.join();
  }
}

DronePX4::ARMING_STATE Drone::getArmingState() {
	return this->arming_state_;
}

DronePX4::FLIGHT_MODE Drone::getFlightMode() {
  	return this->flight_mode_;
}

DronePX4::ARM_DISARM_REASON Drone::getArmReason() {
  	return this->arm_reason_;
}

DronePX4::ARM_DISARM_REASON Drone::getDisarmReason() {
	return this->disarm_reason_;
}

DronePX4::FAILURE Drone::getFailure() {
	return this->failure_;
}


Eigen::Vector3d Drone::getLocalPosition() {
	const Eigen::Vector3d posNED(this->current_pos_x_, this->current_pos_y_, this->current_pos_z_);
	const Eigen::Vector3d posFRD = this->convertPositionNEDtoFRD(posNED); 
	return posFRD;
}


float Drone::getAltitude() {
	return this->current_pos_z_;
}


float Drone::getGroundSpeed() {
	return this->ground_speed_;
}

float Drone::getAirSpeed() {
	return this->airspeed_;
}

Eigen::Vector3d Drone::getOrientation() {
	return Eigen::Vector3d({
		this->roll_,
		this->pitch_,
		this->yaw_
	});
}

void Drone::arm() {
    this->sendCommand(
		px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
		this->target_system_,
		this->target_component_,
		this->source_system_,
		this->source_component_,
		this->confirmation_,
		this->from_external_,
		1.0f
	);
}
void Drone::armSync() {
	while (getArmingState() != DronePX4::ARMING_STATE::ARMED && rclcpp::ok()) {
    	this->arm();
    	usleep(1e5);  // 100 ms
  	}
}

void Drone::disarm() {
  	this->sendCommand(
		px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
		this->target_system_,
		this->target_component_,
		this->source_system_,
		this->source_component_,
		this->confirmation_,
		this->from_external_,
		0.0f
	);
}

void Drone::disarmSync() {
  	while (getArmingState() != DronePX4::ARMING_STATE::DISARMED && rclcpp::ok()) {
		this->disarm();
		usleep(1e5);  // 100 ms
  	}
}

/*
void Drone::takeoff() {
  	this->sendCommand(
   		// https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_TAKEOFF
		px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_TAKEOFF,
		this->target_system_,
		this->target_component_,
		this->source_system_,
		this->source_component_,
		this->confirmation_,
		this->from_external_,
		0.1f,  // Minimum pitch (if airspeed sensor present), desired pitch without sensor (degrees)
		0,     // Empty
		0,     // Empty
		1.57,  // Yaw angle (degrees)
		this->lat_,  // Latitude
		this->lon_,  // Longitude
		this->alt_ + 5.0f // Altitude (meters)
	);  
}
*/

void Drone::land()
{
  this->sendCommand(
    px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND,
    this->target_system_,
    this->target_component_,
    this->source_system_,
    this->source_component_,
    this->confirmation_,
    this->from_external_,
    0.1f,
    0,
    0,
    this->yaw_, // orientation
	0.0f,
	0.0f  
	);
}

void Drone::setLocalPosition(float x, float y, float z, float yaw) {

	this->setOffboardControlMode(DronePX4::CONTROLLER_TYPE::POSITION);

	px4_msgs::msg::TrajectorySetpoint msg;

	msg.timestamp = this->px4_node_->get_clock()->now().nanoseconds() / 1000;

	const Eigen::Vector3d positionFRD(x, y, z);
	const Eigen::Vector3d positionNED = this->convertPositionFRDtoNED(positionFRD);

	msg.position[0] = positionNED.x();
	msg.position[1] = positionNED.y();
	msg.position[2] = positionNED.z();
	msg.yaw = yaw;

	// non-NaN velocity and acceleration fields are used as feedforward terms.
	// We will just set them all to NaN, to keep this API simple.

	msg.velocity[0] = std::numeric_limits<float>::quiet_NaN();
	msg.velocity[1] = std::numeric_limits<float>::quiet_NaN();
	msg.velocity[2] = std::numeric_limits<float>::quiet_NaN();
	msg.yawspeed = std::numeric_limits<float>::quiet_NaN();

	msg.acceleration[0] = std::numeric_limits<float>::quiet_NaN();
	msg.acceleration[1] = std::numeric_limits<float>::quiet_NaN();
	msg.acceleration[2] = std::numeric_limits<float>::quiet_NaN();

	this->vehicle_trajectory_setpoint_pub_->publish(msg);
}

void Drone::setLocalPositionSync(
	double x,
	double y,
	double z,
	double yaw,
	double airspeeed,
	double distance_threshold,
	DronePX4::CONTROLLER_TYPE controller_type)
{
	while (rclcpp::ok()) {
		this->setOffboardControlMode(controller_type);
		this->setLocalPosition(x, y, z, yaw);
		this->setAirSpeed(airspeeed);

		const Eigen::Vector3d currentPosition = getLocalPosition();
		const Eigen::Vector3d goal = Eigen::Vector3d({x,y,z});

		const auto distance = (currentPosition - goal).norm();

		if (distance < distance_threshold) {
			break;
		}

		usleep(1e5);  // 100 ms
	}
}

void Drone::setLocalVelocity(float vx, float vy, float vz, float yaw_rate) {
	
	this->setOffboardControlMode(DronePX4::CONTROLLER_TYPE::VELOCITY);
	
	px4_msgs::msg::TrajectorySetpoint msg;

	msg.timestamp = this->px4_node_->get_clock()->now().nanoseconds() / 1000;

	msg.position[0] = std::numeric_limits<float>::quiet_NaN();
	msg.position[1] = std::numeric_limits<float>::quiet_NaN();
	msg.position[2] = std::numeric_limits<float>::quiet_NaN();
	msg.yaw = std::numeric_limits<float>::quiet_NaN();

	const Eigen::Vector3d velocityFRD(vx, vy, vz);
	const Eigen::Vector3d velocityNED = this->convertVelocityFRDtoNED(velocityFRD);
	msg.velocity[0] = velocityNED.x();
	msg.velocity[1] = velocityNED.y();
	msg.velocity[2] = velocityNED.z();
	msg.yawspeed = yaw_rate;

	msg.acceleration[0] = std::numeric_limits<float>::quiet_NaN();
	msg.acceleration[1] = std::numeric_limits<float>::quiet_NaN();
	msg.acceleration[2] = std::numeric_limits<float>::quiet_NaN();

	this->vehicle_trajectory_setpoint_pub_->publish(msg);
}

void Drone::setGroundSpeed(float speed) {
	this->setSpeed(speed, true);
}

void Drone::setAirSpeed(float speed) {
  	this->setSpeed(speed, false);
}

void Drone::setOffboardControlMode(DronePX4::CONTROLLER_TYPE type) {
	px4_msgs::msg::OffboardControlMode msg;
	msg.timestamp = this->px4_node_->get_clock()->now().nanoseconds() / 1000;

	msg.position = false;
	msg.velocity = false;
	msg.acceleration = false;
	msg.attitude = false;
	msg.body_rate = false;
	msg.actuator = false;

	if (type == DronePX4::CONTROLLER_TYPE::POSITION) {
		msg.position = true;
	} else if (type == DronePX4::CONTROLLER_TYPE::VELOCITY) {
		msg.velocity = true;
	} else if (type == DronePX4::CONTROLLER_TYPE::BODY_RATES) {
		msg.body_rate = true;
	} else {
		RCLCPP_WARN(this->px4_node_->get_logger(), "No controller is defined");
	}

	this->vehicle_offboard_control_mode_pub_->publish(msg);
}

void Drone::toOffboardSync() {
	for (int i = 0; i < 20; i++) {
		setLocalPosition(
			current_pos_x_,
			current_pos_y_,
			current_pos_z_,
			std::numeric_limits<float>::quiet_NaN());
		setOffboardControlMode(DronePX4::CONTROLLER_TYPE::POSITION);
		usleep(1e5);  // 100 ms
	}
	setOffboardMode();
}

void Drone::setOffboardMode() {
	this->sendCommand(
		px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE,
		this->target_system_,
		this->target_component_,
		this->source_system_,
		this->source_component_,
		this->confirmation_,
		this->from_external_,
		1.0f,
		6.0f
	);
}

void Drone::toPositionSync() {
	for (int i = 0; i < 10; i++){
		this->sendCommand(
			px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE,
			this->target_system_,
			this->target_component_,
			this->source_system_,
			this->source_component_,
			this->confirmation_,
			this->from_external_,
			1.0f,
			7.0f
		);
		usleep(1e5);
	}
}

void Drone::setHomePosition(const Eigen::Vector3d& fictual_home) {
	// Variables for Coordinate Systems transformations
	this->frd_home_position_ = fictual_home;
	this->ned_home_position_ = Eigen::Vector3d({current_pos_x_, current_pos_y_, current_pos_z_});
	this->initial_yaw_ = yaw_;

	this->sendCommand(
		px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_HOME,
		this->target_system_,
		this->target_component_,
		this->source_system_,
		this->source_component_,
		this->confirmation_,
		this->from_external_,
		1.0f
	);
}

void Drone::sendCommand(
	uint32_t command, uint8_t target_system, uint8_t target_component, uint8_t source_system,
	uint8_t source_component, uint8_t confirmation, bool from_external,
	float param1, float param2, float param3,
	float param4, float param5, float param6,
	float param7)
{
	px4_msgs::msg::VehicleCommand msg;
	msg.timestamp = this->px4_node_->get_clock()->now().nanoseconds() / 1000.0;
	msg.command = command;

	msg.param1 = param1;
	msg.param2 = param2;
	msg.param3 = param3;
	msg.param4 = param4;
	msg.param5 = param5;
	msg.param6 = param6;
	msg.param7 = param7;
	msg.confirmation = confirmation;
	msg.source_system = source_system;
	msg.target_system = target_system;
	msg.target_component = target_component;
	msg.from_external = from_external;
	msg.source_component = source_component;

	this->vehicle_command_pub_->publish(msg);
}

void Drone::setSpeed(float speed, bool is_ground_speed)
{
  float speed_type = is_ground_speed ? 1.0f : 0.0f;  // true = ground speed, false = air speed
  this->sendCommand(
    // https://mavlink.io/en/messages/common.html#MAV_CMD_DO_CHANGE_SPEED
    px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_CHANGE_SPEED,
    this->target_system_,
    this->target_component_,
    this->source_system_,
    this->source_component_,
    this->confirmation_,
    this->from_external_,
    speed_type,  // Speed type (0=Airspeed, 1=Ground Speed, 2=Climb Speed, 3=Descent Speed)
    speed,       // Speed (-1 indicates no change, -2 indicates return to default vehicle speed)
    -1.0f);      // Throttle (-1 indicates no change, -2 indicates return to default throttle value)
}

double Drone::getTime() {
	return this->px4_node_->get_clock()->now().seconds();
}

void Drone::log(const std::string &info) {
	RCLCPP_INFO(this->px4_node_->get_logger(), info.c_str());
}

/*
	Image functions
*/

cv_bridge::CvImagePtr& Drone::getHorizontalImage() {
	return horizontal_cv_ptr_;
}

cv_bridge::CvImagePtr& Drone::getVerticalImage() {
	return vertical_cv_ptr_;

}

void Drone::create_image_publisher(const std::string& topic_name) {
	rclcpp::QoS qos_profile(10);
	qos_profile.best_effort();

	image_publishers_.emplace(
		topic_name,
		this->px4_node_->create_publisher<sensor_msgs::msg::Image>(
			topic_name, qos_profile)
	);

}

void Drone::publish_image(const std::string& topic_name, const cv_bridge::CvImagePtr& cv_ptr) {
	sensor_msgs::msg::Image::SharedPtr msg = cv_ptr->toImageMsg();
	this->image_publishers_.at(topic_name)->publish(*msg.get());
}

void Drone::publish_image(const std::string& topic_name, const cv::Mat& image) {

    std::string encoding = encoding_map_[cv::typeToString(image.type())];

	sensor_msgs::msg::Image::SharedPtr msg =
            cv_bridge::CvImage(std_msgs::msg::Header(), encoding, image)
                .toImageMsg();
	this->image_publishers_.at(topic_name)->publish(*msg.get());
}

std::unordered_map<std::string, std::string> Drone::encoding_map_ = {
	{"CV_8UC1", "mono8"},
	{"CV_8UC3", "bgr8"},
	{"CV_16UC1", "mono16"},
	{"CV_16UC3", "bgr16"},
	{"CV_32FC1", "32FC1"},
	{"CV_32FC3", "32FC3"}	
};

std::vector<DronePX4::BoundingBox> Drone::getBoundingBox(){
	return detections_;
}

std::vector<Eigen::Vector4d> Drone::getBarCodeLocation() {
	return barcode_detections_;
}



std::vector<std::string> Drone::getHandGestures() { 
	return gestures_;
}

std::array<float, 2> Drone::getHandLocation() {
	return {hand_location_x_, hand_location_y_};
}

void Drone::resetHands() {
	hand_location_x_ = 0.5;
	hand_location_y_ = 0.5;
	gestures_ = {"", ""};
}

std::string Drone::readQRCode(){
	return qr_code_data_;
}

//Coordinate System transformations (private functions)

Eigen::Vector3d Drone::convertPositionNEDtoFRD(const Eigen::Vector3d& position_ned) const
{
    // Translate the NED coordinates to be relative to the NED home position
    Eigen::Vector3d translated_position = position_ned - this->ned_home_position_;

    // Apply rotation to account for the initial yaw
    Eigen::Matrix3d rotation;
    rotation << cos(-this->initial_yaw_), -sin(-this->initial_yaw_), 0,
                sin(-this->initial_yaw_), cos(-this->initial_yaw_), 0,
                0, 0, 1;

    // Translate to be relative to the FRD home position
    return rotation * translated_position + this->frd_home_position_;
}

Eigen::Vector3d Drone::convertPositionFRDtoNED(const Eigen::Vector3d& position_frd) const
{
    // Translate the FRD coordinates to be relative to the FRD home position
    Eigen::Vector3d translated_position = position_frd - this->frd_home_position_;

    // Apply inverse rotation to go from FRD to NED coordinates
    Eigen::Matrix3d rotation;
    rotation << cos(this->initial_yaw_), -sin(this->initial_yaw_), 0,
                sin(this->initial_yaw_), cos(this->initial_yaw_), 0,
                0, 0, 1;

    return rotation * translated_position + this->ned_home_position_;
}

Eigen::Vector3d Drone::convertVelocityNEDtoFRD(const Eigen::Vector3d& velocity_ned) const
{
    // Apply rotation to account for the initial yaw
    Eigen::Matrix3d rotation;
    rotation << cos(-this->initial_yaw_), -sin(-this->initial_yaw_), 0,
                sin(-this->initial_yaw_), cos(-this->initial_yaw_), 0,
                0, 0, 1;

    return rotation * velocity_ned;
}

Eigen::Vector3d Drone::convertVelocityFRDtoNED(const Eigen::Vector3d& velocity_frd) const
{
    // Apply inverse rotation to go from FRD to NED coordinates
    Eigen::Matrix3d rotation;
    rotation << cos(this->initial_yaw_), -sin(this->initial_yaw_), 0,
                sin(this->initial_yaw_), cos(this->initial_yaw_), 0,
                0, 0, 1;

    return rotation * velocity_frd;
}