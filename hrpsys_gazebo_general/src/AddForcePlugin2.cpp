#include <iostream>
#include <stdio.h>

#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Time.hh>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>
#include <ros/subscribe_options.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/String.h>

#include "PubQueue.h"


namespace gazebo
{
  class AddForce : public ModelPlugin
  {

  public:
    // Initialize
    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
      // Store the pointer to the model
      this->model = _parent;

      // read option args in sdf tags
      this->obj_name = "";
      if (_sdf->HasElement("objname")) {
	this->obj_name = _sdf->Get<std::string>("objname");
      }
      this->link_name1 = "root";
      this->link_name2 = "root";
      if (_sdf->HasElement("linkname1")) {
	this->link_name1 = _sdf->Get<std::string>("linkname1");
      }
      if (_sdf->HasElement("linkname2")) {
	this->link_name2 = _sdf->Get<std::string>("linkname2");
      }
      this->force1 = math::Vector3(0, 0, 0);
      if (_sdf->HasElement("force1")) {
	this->force1 = _sdf->Get<math::Vector3>("force1");
      }
      this->force2 = math::Vector3(0, 0, 0);
      if (_sdf->HasElement("force2")) {
	this->force2 = _sdf->Get<math::Vector3>("force2");
      }
      this->torque1 = math::Vector3(0, 0, 0);
      if (_sdf->HasElement("torque1")) {
	this->torque1 = _sdf->Get<math::Vector3>("torque1");
      }
      this->torque2 = math::Vector3(0, 0, 0);
      if (_sdf->HasElement("torque2")) {
	this->torque2 = _sdf->Get<math::Vector3>("torque2");
      }
      this->position1 = math::Vector3(0, 0, 0);
      if(_sdf->HasElement("position1")) {
	this->position1 = _sdf->Get<math::Vector3>("position1");
      }
      this->position2 = math::Vector3(0, 0, 0);
      if(_sdf->HasElement("position2")) {
	this->position2 = _sdf->Get<math::Vector3>("position2");
      }

      // for debug
      // for(int i = 0; i < this->model->GetLinks().size(); i++) {
      // 	gzerr << "Link" << i << " " << this->model->GetLinks()[i]->GetName() << std::endl;
      // }

      // find root link
      this->link1 = this->model->GetLink(this->link_name1);
      if(!this->link1) {
	gzerr << "Root link are not found. (link_name is "<< this->link_name1 << ")" << std::endl;
	return;
      }
      this->link2 = this->model->GetLink(this->link_name2);
      if(!this->link2) {
	gzerr << "Root link are not found. (link_name is "<< this->link_name2 << ")" << std::endl;
	return;
      }
      world = this->model->GetWorld();      

      // Make sure the ROS node for Gazebo has already been initialized                                                                                    
      if (!ros::isInitialized()) {
	gzerr << "A ROS node for Gazebo has not been initialized, unable to load plugin. "
	      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)";
	return;
      }
      // ros node
      this->rosNode = new ros::NodeHandle("");
      // ros callback queue for processing subscription
      this->deferredLoadThread = boost::thread(boost::bind(&AddForce::DeferredLoad, this));

      gzmsg << "AddForcePlugin2 was loaded ! (" << " linkname1: " << this->link_name1 << " linkname2: " << this->link_name2 << " force: " << this->force1 << "  torque: " << this->torque1 << "  position: " << this->position1 << ")" << std::endl;
    }

    void DeferredLoad() {
      // ros topic subscribtions
      ros::SubscribeOptions ForceCommandSo1 =
	ros::SubscribeOptions::create<geometry_msgs::Wrench>("/" + this->obj_name + "/AddForcePlugin/ForceCommand1", 100,
						    boost::bind(&AddForce::SetForceCommand1, this, _1),
						    ros::VoidPtr(), &this->rosQueue);
      ros::SubscribeOptions ForceCommandSo2 =
	ros::SubscribeOptions::create<geometry_msgs::Wrench>("/" + this->obj_name + "/AddForcePlugin/ForceCommand2", 100,
						    boost::bind(&AddForce::SetForceCommand2, this, _1),
						    ros::VoidPtr(), &this->rosQueue);

      // Enable TCP_NODELAY because TCP causes bursty communication with high jitter,
      ForceCommandSo1.transport_hints = ros::TransportHints().reliable().tcpNoDelay(true);
      ForceCommandSo2.transport_hints = ros::TransportHints().reliable().tcpNoDelay(true);
      this->subForceCommand1 = this->rosNode->subscribe(ForceCommandSo1);
      this->subForceCommand2 = this->rosNode->subscribe(ForceCommandSo2);

      // ros callback queue for processing subscription
      this->callbackQueeuThread = boost::thread(boost::bind(&AddForce::RosQueueThread, this));

      // Listen to the update event.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&AddForce::OnUpdate, this, _1));
    }

    void SetForceCommand1(const geometry_msgs::Wrench::ConstPtr &_msg)
    {
      this->force1.x = _msg->force.x;
      this->force1.y = _msg->force.y;
      this->force1.z = _msg->force.z;
      this->torque1.x = _msg->torque.x;
      this->torque1.y = _msg->torque.y;
      this->torque1.z = _msg->torque.z;
      gzmsg << "subscribed AddForceCommand. ( force: " << this->force1 << "  torque: " << this->torque1 << " )" << std::endl;
    }

    void SetForceCommand2(const geometry_msgs::Wrench::ConstPtr &_msg)
    {
      this->force2.x = _msg->force.x;
      this->force2.y = _msg->force.y;
      this->force2.z = _msg->force.z;
      this->torque2.x = _msg->torque.x;
      this->torque2.y = _msg->torque.y;
      this->torque2.z = _msg->torque.z;
      gzmsg << "subscribed AddForceCommand. ( force: " << this->force2 << "  torque: " << this->torque2 << " )" << std::endl;
    }

    // Called by the world update start event
    void OnUpdate(const common::UpdateInfo & /*_info*/)
    {
      common::Time curTime = this->world->GetSimTime();

      //this->link1->AddForce(this->force1);
      //this->link2->AddForce(this->force2);
      this->link1->AddRelativeForce(this->force1);
      this->link2->AddRelativeForce(this->force2);
      // this->link->AddForceAtRelativePosition(this->force, this->position);
      this->link1->AddTorque(this->torque1);
      this->link2->AddTorque(this->torque2);
    }

    // Ros loop thread function
    void RosQueueThread() {
      static const double timeout = 0.01;

      while (this->rosNode->ok()) {
	this->rosQueue.callAvailable(ros::WallDuration(timeout));
      }
    }

  private:
    physics::ModelPtr model;
    physics::WorldPtr world;
    std::string link_name1;
    std::string link_name2;
    std::string obj_name;
    math::Vector3 force1;
    math::Vector3 force2;
    math::Vector3 torque1;
    math::Vector3 torque2;
    math::Vector3 position1;
    math::Vector3 position2;
    physics::LinkPtr link1;
    physics::LinkPtr link2;
    event::ConnectionPtr updateConnection;

    ros::NodeHandle* rosNode;
    ros::CallbackQueue rosQueue;
    ros::Subscriber subForceCommand1;
    ros::Subscriber subForceCommand2;
    boost::thread callbackQueeuThread;
    boost::thread deferredLoadThread;
  };

  GZ_REGISTER_MODEL_PLUGIN(AddForce)
}
