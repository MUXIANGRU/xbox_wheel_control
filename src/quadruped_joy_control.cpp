#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <free_gait_msgs/ExecuteStepsGoal.h>
#include <free_gait_msgs/BaseTarget.h>
#include <free_gait_msgs/BaseAuto.h>
#include <free_gait_msgs/Footstep.h>
#include <free_gait_msgs/EndEffectorTarget.h>
#include <free_gait_msgs/ExecuteStepsAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/Bool.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <kindr_ros/kindr_ros.hpp>
#include <tf/transform_listener.h>
#include <kindr/Core>
#include <free_gait_ros/gait_generate_client.hpp>
#include <free_gait_msgs/RobotState.h>

class QuadrupedTeleOp
{
public:
    QuadrupedTeleOp(ros::NodeHandle& nodehandle);
    void publishCommand();
    void publishDirectJointCommand();
    int directJointFlag_;
private:
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
    void commandUpdate();
    void RobotStateCallback(const free_gait_msgs::RobotState& rs);

    ros::NodeHandle nh_;

    boost::thread commandUpdateThread_;

    int eStopFlag_, setInitialStateFlag_, legMoveFlag_, trotFlag_, paceFlag_, standFlag_, count_;
    geometry_msgs::Pose target_pose_;
    geometry_msgs::Point target_position_, target_euler_, lf_pos_, rf_pos_, rh_pos_, lh_pos_;
    double l_scale_, a_scale_;
    std::string actionServerTopic_;
    geometry_msgs::Twist twist, legTwist;
    free_gait_msgs::ExecuteStepsGoal step_goal;
    geometry_msgs::Quaternion target_q;
    float leftStickLR ,leftStickUD,LT,rightStickLR ,rightStickUD, RT, crossKeyLR, crossKeyUD;//1/-1
    int keyA, keyB, keyX, keyY, LB, RB, keyBack, keyStart, keyPower;//0/1

    ros::Publisher vel_pub_, eStopPublisher_, legMovePub_, cartesianDiffPub_,jointCommandPub_, homingCommandPub_, emergencyStopPub_, resetCommandPub_;
    ros::Subscriber joy_sub_;
    ros::Publisher robot_state_pub_;
    ros::Subscriber robot_state_sub_;
    free_gait_msgs::RobotState robot_state_, initial_state_;

    ros::ServiceClient trotswitchClient_, paceswitchClient_;

    //! Step action client.
    std::unique_ptr<actionlib::SimpleActionClient<free_gait_msgs::ExecuteStepsAction>> stepActionClient_;

    tf::TransformListener baseTFListener_;

    tf::StampedTransform baseToOdomTransform_, footprintToOdomTransform_;
    kindr::HomTransformQuatD base_to_odom_, footprint_to_odom_;
    kindr::RotationQuaternionD initial_base_state_;
};


QuadrupedTeleOp::QuadrupedTeleOp(ros::NodeHandle& nodehandle):
    directJointFlag_(0),
    nh_(nodehandle),
    eStopFlag_(0),
    legMoveFlag_(0),
    setInitialStateFlag_(0)
{

    trotFlag_ = 0;
    paceFlag_ = 0;
    standFlag_ = 0;

    lh_pos_.x = -0.42;
    lh_pos_.y = 0.25;
    lh_pos_.z = 0.1;

    rh_pos_.x = -0.42;
    rh_pos_.y = -0.25;
    rh_pos_.z = 0.1;

    lf_pos_.x = 0.42;
    lf_pos_.y = 0.25;
    lf_pos_.z = 0.1;

    rf_pos_.x = 0.42;
    rf_pos_.y = -0.25;
    rf_pos_.z = 0.1;


    nh_.param("scale_angular", a_scale_, 1.2);
    ROS_INFO_STREAM("the value of a_scale is " << a_scale_ << std::endl);
    nh_.param("scale_linear", l_scale_, 1.2);
    nh_.param("action_server_topic", actionServerTopic_,
              std::string("/free_gait/action_server"));
    ROS_INFO("test,augular scale is %f, linear scale is %f",a_scale_,l_scale_);

    stepActionClient_ = std::unique_ptr<
        actionlib::SimpleActionClient<free_gait_msgs::ExecuteStepsAction>>(
        new actionlib::SimpleActionClient<free_gait_msgs::ExecuteStepsAction>(
            actionServerTopic_, true));
    eStopPublisher_ = nh_.advertise<std_msgs::Bool>("/e_stop", 1);
    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("/joy", 10, &QuadrupedTeleOp::joyCallback, this);

    trotswitchClient_ = nh_.serviceClient<std_srvs::SetBool>("/gait_generate_switch", false);
    paceswitchClient_ = nh_.serviceClient<std_srvs::SetBool>("/pace_switch", false);

    commandUpdateThread_ = boost::thread(boost::bind(&QuadrupedTeleOp::commandUpdate, this));

    robot_state_pub_ = nh_.advertise<free_gait_msgs::RobotState>("/desired_robot_state", 1);
    robot_state_.base_pose.header.frame_id = "/odom";
    robot_state_.base_pose.child_frame_id = "/odom";

    robot_state_.lf_leg_joints.position.resize(3);
    robot_state_.rf_leg_joints.position.resize(3);
    robot_state_.rh_leg_joints.position.resize(3);
    robot_state_.lh_leg_joints.position.resize(3);

    robot_state_.lf_target.target_position.resize(1);
    robot_state_.lf_target.target_velocity.resize(1);
    robot_state_.lf_target.target_acceleration.resize(1);

    robot_state_.rf_target.target_position.resize(1);
    robot_state_.rf_target.target_velocity.resize(1);
    robot_state_.rf_target.target_acceleration.resize(1);

    robot_state_.rh_target.target_position.resize(1);
    robot_state_.rh_target.target_velocity.resize(1);
    robot_state_.rh_target.target_acceleration.resize(1);

    robot_state_.lh_target.target_position.resize(1);
    robot_state_.lh_target.target_velocity.resize(1);
    robot_state_.lh_target.target_acceleration.resize(1);

    initial_state_.base_pose.header.frame_id = "/odom";
    initial_state_.base_pose.child_frame_id = "/odom";

    initial_state_.lf_leg_joints.position.resize(3);
    initial_state_.rf_leg_joints.position.resize(3);
    initial_state_.rh_leg_joints.position.resize(3);
    initial_state_.lh_leg_joints.position.resize(3);

    initial_state_.lf_target.target_position.resize(1);
    initial_state_.lf_target.target_velocity.resize(1);
    initial_state_.lf_target.target_acceleration.resize(1);

    initial_state_.rf_target.target_position.resize(1);
    initial_state_.rf_target.target_velocity.resize(1);
    initial_state_.rf_target.target_acceleration.resize(1);

    initial_state_.rh_target.target_position.resize(1);
    initial_state_.rh_target.target_velocity.resize(1);
    initial_state_.rh_target.target_acceleration.resize(1);

    initial_state_.lh_target.target_position.resize(1);
    initial_state_.lh_target.target_velocity.resize(1);
    initial_state_.lh_target.target_acceleration.resize(1);

    robot_state_sub_ = nh_.subscribe("/desired_robot_state", 1, &QuadrupedTeleOp::RobotStateCallback, this);

}

void QuadrupedTeleOp::RobotStateCallback(const free_gait_msgs::RobotState &rs)
{
    robot_state_ = rs;
    ROS_INFO_STREAM("update the robot_state" << std::endl);
    ROS_INFO_STREAM("the height of the robot is " << robot_state_.base_pose.pose.pose.position.z <<std::endl);
}
void QuadrupedTeleOp::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{

    //sudo jstest /dev/input/jsX chenk the axis and button
    free_gait_msgs::ExecuteStepsGoal step_goal;
    leftStickLR = joy->axes[0];//contiunus 1~-1
    leftStickUD = joy->axes[1];//contiunus 1~-1
    LT = joy->axes[2];//contiunus 0~2
    rightStickLR = joy->axes[3];//contiunus 1~-1
    rightStickUD = joy->axes[4];//contiunus 1~-1
    RT = joy->axes[5];//contiunus 0~2
    crossKeyLR = joy->axes[6];//1/-1
    crossKeyUD = joy->axes[7];//1/-1
    keyA = joy->buttons[0];//0/1
    keyB = joy->buttons[1];//0/1
    keyX = joy->buttons[2];//0/1
    keyY = joy->buttons[3];//0/1
    LB = joy->buttons[4];//0/1
    RB = joy->buttons[5];//0/1
    keyBack = joy->buttons[6];//0/1
    keyStart = joy->buttons[7];//0/1
    keyPower = joy->buttons[8];//0/1

//    ROS_INFO_STREAM("the status of the crosskey ud is"<< crossKeyUD << std::endl);

    float velocityScale = l_scale_*RT +1;
    float angularScale = a_scale_*LT +1;

    if(keyPower == 1)
    {
        ROS_INFO("Emergency Stop");
        std_msgs::Bool e_stop_msg;
        e_stop_msg.data = true;

        eStopPublisher_.publish(e_stop_msg);
        eStopFlag_ = 1;
        return;
    }

}
void QuadrupedTeleOp::commandUpdate()
{
    ros::Rate rate(30);
    count_ = 0;
    while (!eStopFlag_ && ros::ok()) {

        try {
            baseTFListener_.waitForTransform("/odom", "/base_link", ros::Time(0), ros::Duration(0.5));
            baseTFListener_.lookupTransform("/odom", "/base_link", ros::Time(0), baseToOdomTransform_);
            baseTFListener_.waitForTransform("/odom", "/foot_print", ros::Time(0), ros::Duration(0.5));
            baseTFListener_.lookupTransform("/odom", "/foot_print", ros::Time(0), footprintToOdomTransform_);
            base_to_odom_ = kindr::HomTransformQuatD(kindr::Position3D(baseToOdomTransform_.getOrigin().getX(),
                                                                       baseToOdomTransform_.getOrigin().getY(),
                                                                       baseToOdomTransform_.getOrigin().getZ()),
                                                     kindr::RotationQuaternionD(baseToOdomTransform_.getRotation().getW(),
                                                                                baseToOdomTransform_.getRotation().getX(),
                                                                                baseToOdomTransform_.getRotation().getY(),
                                                                                baseToOdomTransform_.getRotation().getZ()));

            footprint_to_odom_ = kindr::HomTransformQuatD(kindr::Position3D(footprintToOdomTransform_.getOrigin().getX(),
                                                                            footprintToOdomTransform_.getOrigin().getY(),
                                                                            footprintToOdomTransform_.getOrigin().getZ()),
                                                          kindr::RotationQuaternionD(footprintToOdomTransform_.getRotation().getW(),
                                                                                     footprintToOdomTransform_.getRotation().getX(),
                                                                                     footprintToOdomTransform_.getRotation().getY(),
                                                                                     footprintToOdomTransform_.getRotation().getZ()));
        } catch (tf::TransformException &ex) {
            ROS_ERROR("%s", ex.what());
        }

        if(keyBack == 1)//sit down
        {
            ROS_INFO("Reset Initial Quadruped State");
            free_gait_msgs::Step initial_steps;
            free_gait_msgs::BaseAuto base_auto_msg;
            base_auto_msg.height = 0.25;
            base_auto_msg.average_linear_velocity = 0.2;
            base_auto_msg.average_angular_velocity = 0.1;
            initial_steps.base_auto.push_back(base_auto_msg);
            step_goal.steps.push_back(initial_steps);
            stepActionClient_->sendGoal(step_goal);
            stepActionClient_->waitForResult();
            setInitialStateFlag_ = 1;
        }

        if(keyStart == 1)//stand up
        {
            ROS_INFO("The robot is standing up~");
            free_gait_msgs::Step initial_steps;
            free_gait_msgs::BaseAuto base_auto_msg;
            base_auto_msg.height = 0.5;
            base_auto_msg.average_linear_velocity = 0.2;
            base_auto_msg.average_angular_velocity = 0.1;
            initial_steps.base_auto.push_back(base_auto_msg);
            step_goal.steps.push_back(initial_steps);

            stepActionClient_->sendGoal(step_goal);
            stepActionClient_->waitForResult();
            setInitialStateFlag_ = 1;
        }

        //    linearX_ = 0;
        //    linearY_ = 0;
        //    angular_ = 0;

        //! WSHY: controller switch
        if(setInitialStateFlag_)
        {

        }

        //! WSHY: set gait
        if(crossKeyLR == 1)
        {
            // Trot
            ROS_INFO("Set Trot Flag");
            trotFlag_ = 1;
            paceFlag_ = 0;
            standFlag_ = 0;
            std_srvs::SetBool trot_switch;
            trot_switch.request.data = true;
            trotswitchClient_.call(trot_switch.request, trot_switch.response);
            if(trot_switch.response.success)
            {
                ROS_INFO_STREAM("get the trot response success" << std::endl);
            }
        }else if (crossKeyLR == -1) {
            // Pace
            ROS_INFO("Set Pace Flag");
            trotFlag_ = 0;
            paceFlag_ = 1;
            standFlag_ = 0;
            std_srvs::SetBool pace_switch;
            pace_switch.request.data = true;
            paceswitchClient_.call(pace_switch.request, pace_switch.response);
            if (pace_switch.response.success)
            {
                ROS_INFO_STREAM("Get the pace response success" << std::endl);
            }
        }else if (crossKeyUD == 1) {
            // Stand and stop gait
            ROS_INFO("Set Stand Flag");
            trotFlag_ = 0;
            paceFlag_ = 0;
            standFlag_ = 1;
        }
        else if (crossKeyUD == -1) {
            trotFlag_ = 0;
            paceFlag_ = 0;
            standFlag_ = 0;
            std_srvs::SetBool gait_stop_switch;
            gait_stop_switch.request.data = false;
            trotswitchClient_.call(gait_stop_switch.request, gait_stop_switch.response);
            paceswitchClient_.call(gait_stop_switch.request, gait_stop_switch.response);
        }
        //! WSHY: Set velocity
        if(keyA == 1)
        {
            initial_state_ = robot_state_;
            initial_state_.base_pose.pose.pose.position.x = base_to_odom_.getPosition().x();
            initial_state_.base_pose.pose.pose.position.y = base_to_odom_.getPosition().y();
            initial_state_.base_pose.pose.pose.position.z = base_to_odom_.getPosition().z();
            initial_state_.base_pose.pose.pose.orientation.w = base_to_odom_.getRotation().w();
            initial_state_.base_pose.pose.pose.orientation.x = base_to_odom_.getRotation().x();
            initial_state_.base_pose.pose.pose.orientation.y = base_to_odom_.getRotation().y();
            initial_state_.base_pose.pose.pose.orientation.z = base_to_odom_.getRotation().z();

            initial_base_state_ = base_to_odom_.getRotation();
            ROS_INFO("store the initial robot_state");
        }
        if(trotFlag_)
        {
            geometry_msgs::Twist base_vel;
            base_vel.linear.x = leftStickUD;
            base_vel.linear.y = leftStickLR;
            base_vel.linear.z = rightStickUD;// react to value near the limits to avoiding unexpected trigger
            base_vel.angular.z = rightStickLR;
            vel_pub_.publish(base_vel);
            ROS_INFO("set Trot Velocity : vx = %f, vy = %f, vz = %f, wz = %f", base_vel.linear.x,
                     base_vel.linear.y, base_vel.linear.z, base_vel.angular.z);
        }
        if(paceFlag_)
        {
            geometry_msgs::Twist base_vel;
            base_vel.linear.x = leftStickUD/2;
            base_vel.linear.y = leftStickLR/2;
            base_vel.linear.z = rightStickUD;// react to value near the limits to avoiding unexpected trigger
            base_vel.angular.z = rightStickLR/2;
            vel_pub_.publish(base_vel);
            ROS_INFO("set Pace Velocity : vx = %f, vy = %f, vz = %f, wz = %f", base_vel.linear.x,
                     base_vel.linear.y, base_vel.linear.z, base_vel.angular.z);
        }
        if(standFlag_)
        {
            if(LB == 1)
            {
                target_position_.x = initial_state_.base_pose.pose.pose.position.x + leftStickUD * 0.15;
                target_position_.y = initial_state_.base_pose.pose.pose.position.y + leftStickLR * 0.15;
                target_position_.z = initial_state_.base_pose.pose.pose.position.z + rightStickUD * 0.1;

                robot_state_.base_pose.pose.pose.position.x = target_position_.x;
                robot_state_.base_pose.pose.pose.position.y = target_position_.y;
                robot_state_.base_pose.pose.pose.position.z = target_position_.z;

                robot_state_pub_.publish(robot_state_);

            }
            if (RB == 1)
            {
                kindr::RotationQuaternionD q_base, q_target;

                target_euler_.x = leftStickLR * 0.15;
                target_euler_.y = leftStickUD * 0.15;
                target_euler_.z = rightStickLR * 0.15;

                q_base = kindr::EulerAnglesXyzD(target_euler_.x, target_euler_.y, target_euler_.z);
//                q_target = q_base * base_to_odom_.getRotation();

                q_target = q_base * initial_base_state_;

                ROS_INFO_STREAM("q_target is " << q_target << std::endl);
                ROS_INFO_STREAM("base_to_odom.getRotation is " << base_to_odom_.getRotation() << std::endl);

//                q_target = q_base * initial_state_.base_pose.pose.pose.orientation;
                target_q.w = q_target.w();
                target_q.x = q_target.x();
                target_q.y = q_target.y();
                target_q.z = q_target.z();

                robot_state_.base_pose.pose.pose.orientation.w = target_q.w;
                robot_state_.base_pose.pose.pose.orientation.x = target_q.x;
                robot_state_.base_pose.pose.pose.orientation.y = target_q.y;
                robot_state_.base_pose.pose.pose.orientation.z = target_q.z;

                robot_state_pub_.publish(robot_state_);
            }
            if (keyB == 1)
            {
                robot_state_pub_.publish(initial_state_);
            }

        }
        rate.sleep();
    }
}

void QuadrupedTeleOp::publishCommand()
{
    vel_pub_.publish(twist);
}

void QuadrupedTeleOp::publishDirectJointCommand()
{

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "joy_to_twist_node");
    ros::NodeHandle nh("~");
    QuadrupedTeleOp teleop_turtle(nh);
    ros::Rate rate(50);
    while (ros::ok()) {
        rate.sleep();
        ros::spinOnce();
    }

}
