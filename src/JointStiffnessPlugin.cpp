#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ros/ros.h>
#include <softactuator/SetStiffness.h>
#include <softactuator/SetDamping.h>
#include <softactuator/SetFriction.h>
#include <fullurdf2/SetStiffness.h>
#include <fullurdf2/SetDamping.h>
#include <fullurdf2/SetFriction.h>
#include <fullurdf3/SetStiffness.h>
#include <fullurdf3/SetDamping.h>
#include <fullurdf3/SetFriction.h>
#include <iostream>

namespace gazebo
{
  class JointStiffnessPlugin : public ModelPlugin
  {
  public:
    JointStiffnessPlugin() : ModelPlugin(), nh("~")
    {
      std::cout << "JointStiffnessPlugin Loaded" << std::endl;
    }

    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
      // Initialize ROS node if not already initialized
      if (!ros::isInitialized())
      {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "gazebo_client", ros::init_options::NoSigintHandler);
      }

      // Store the pointer to the model
      this->model = _model;

      // ROS node handle
      this->nh = ros::NodeHandle("~");

      // Advertise the service for softactuator
      this->serviceStiffnessSoftactuator = nh.advertiseService("set_joint_stiffness_softactuator", &JointStiffnessPlugin::SetJointStiffnessSoftactuator, this);
     
      this->serviceDampingSoftactuator = nh.advertiseService("set_joint_damping_softactuator", &JointStiffnessPlugin::SetJointDampingSoftactuator, this);

      this->serviceFrictionSoftactuator = nh.advertiseService("set_joint_friction_softactuator", &JointStiffnessPlugin::SetJointFrictionSoftactuator, this);

      // Advertise the service for fullurdf2
      this->serviceStiffnessFullurdf2 = nh.advertiseService("set_joint_stiffness_fullurdf2", &JointStiffnessPlugin::SetJointStiffnessFullurdf2, this);

      this->serviceDampingFullurdf2 = nh.advertiseService("set_joint_damping_fullurdf2", &JointStiffnessPlugin::SetJointDampingFullurdf2, this);

      this->serviceFrictionFullurdf2 = nh.advertiseService("set_joint_friction_fullurdf2", &JointStiffnessPlugin::SetJointFrictionFullurdf2, this);

      // Advertise the service for fullurdf3
      this->serviceStiffnessFullurdf3 = nh.advertiseService("set_joint_stiffness_fullurdf3", &JointStiffnessPlugin::SetJointStiffnessFullurdf3, this);

      this->serviceDampingFullurdf3 = nh.advertiseService("set_joint_damping_fullurdf3", &JointStiffnessPlugin::SetJointDampingFullurdf3, this);

      this->serviceFrictionFullurdf3 = nh.advertiseService("set_joint_friction_fullurdf3", &JointStiffnessPlugin::SetJointFrictionFullurdf3, this);    
      
      
      // Log that the plugin is loaded
      ROS_INFO("JointStiffnessPlugin loaded and running!");

      // Loop through all <joint> elements
      if (_sdf->HasElement("joint"))
      {
        sdf::ElementPtr jointElem = _sdf->GetElement("joint");
        while (jointElem)
        {
          // Get the joint name
          std::string jointName = jointElem->Get<std::string>("name");
          physics::JointPtr joint = this->model->GetJoint(jointName);

          // Get stiffness and damping parameters
          double stiffness = jointElem->Get<double>("stiffness");
          double damping = jointElem->Get<double>("damping");
          double friction = jointElem->Get<double>("friction");

          if (joint)
          {
            // Store the joint parameters
            JointParams params;
            params.joint = joint;
            params.stiffness = stiffness;
            params.damping = damping;
            params.friction = friction;
            params.restAngle = 0.0;
            // params.previousAngle = 0.0;
            // params.restVelocity = 0.0;

            this->joints.push_back(params);

            std::cout << "Configured joint: " << jointName << " with stiffness: " << stiffness << " , damping: " << damping << " and friction: " << friction << std::endl;
          }
          else
          {
            gzerr << "Joint " << jointName << " not found!" << std::endl;
          }

          jointElem = jointElem->GetNextElement("joint");
        }
      }
      else
      {
        gzerr << "No joints specified in plugin configuration." << std::endl;
        return;
      }

      // Connect to the update event, which is broadcast every simulation iteration
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&JointStiffnessPlugin::OnUpdate, this));
    }

    void OnUpdate()
    {
      // double timeStep = 0.001;
      for (auto& params : this->joints)
      {
        // Get the current joint angle and velocity
        double jointAngle = params.joint->Position(0);
        double jointVelocity = params.joint->GetVelocity(0);

        // double jointVelocity = (params.joint->Position(0) - params.previousAngle) / timeStep;


        // Calculate the restoring force (spring force)
        double springForce = -params.stiffness * (jointAngle - params.restAngle);

        // Calculate the damping force
        // double dampingForce = -params.damping * (jointVelocity - params.restVelocity);

        //*****double dampingForce = -params.damping * jointVelocity;
        double dampingForce = params.damping; //* (jointAngle - params.restAngle);

        //////////////////////////////////////////////////
        // Ensure the damping force resists the motion
      // if (jointVelocity < 0)
      // {
      //   dampingForce = -fabs(dampingForce);
      // }
      // else
      // {
      //   dampingForce = fabs(dampingForce);
      // }

      /////////////////////////////////////////////////
        // Apply the forces to the joint
        double totalForce = springForce * dampingForce;
        /////////
        // double totalVelocity = dampingForce;
        
        params.joint->SetForce(0, totalForce);

        // params.joint->SetVelocity(0, totalVelocity);
        // params.previousAngle = params.joint->Position(0);

        if (params.friction > 0.0)
        {
          params.joint->SetParam("friction", 0, params.friction);
        }

       }
     }

    // ROSService for softactuator
    bool SetJointStiffnessSoftactuator(softactuator::SetStiffness::Request &req, softactuator::SetStiffness::Response &res)
    {
      for (auto& params : this->joints)
      {
        if (params.joint->GetName() == req.joint_name)
        {
          params.stiffness = req.stiffness;
          ROS_INFO("Set stiffness of joint %s to %f", req.joint_name.c_str(), req.stiffness);
          res.success = true;
          res.message = "Stiffness updated";
          return true;
        }
      }
      res.success = false;
      res.message = "Joint not found";
      return false;
    }

    bool SetJointDampingSoftactuator(softactuator::SetDamping::Request &req, softactuator::SetDamping::Response &res)
    {
      for (auto& params : this->joints)
      {
        if (params.joint->GetName() == req.joint_name)
        {
          params.damping = req.damping;
          ROS_INFO("Set damping of joint %s to %f", req.joint_name.c_str(), req.damping);
          res.success = true;
          res.message = "Damping updated";
          return true;
        }
      }
      res.success = false;
      res.message = "Joint not found";
      return false;
    }

    bool SetJointFrictionSoftactuator(softactuator::SetFriction::Request &req, softactuator::SetFriction::Response &res)
    {
      for (auto& params : this->joints)
      {
        if (params.joint->GetName() == req.joint_name)
        {
          params.friction = req.friction;
          ROS_INFO("Set friction of joint %s to %f", req.joint_name.c_str(), req.friction);
          res.success = true;
          res.message = "Friction updated";
          return true;
        }
      }
      res.success = false;
      res.message = "Joint not found";
      return false;
    }

    ///////////////////////////////////////////////////////////////////////
    // ROSService for quadruped
    bool SetJointStiffnessFullurdf2(fullurdf2::SetStiffness::Request &req, fullurdf2::SetStiffness::Response &res)
    {
      for (auto& params : this->joints)
      {
        if (params.joint->GetName() == req.joint_name)
        {
          params.stiffness = req.stiffness;
          ROS_INFO("Set stiffness of joint %s to %f", req.joint_name.c_str(), req.stiffness);
          res.success = true;
          res.message = "Stiffness updated";
          return true;
        }
      }
      res.success = false;
      res.message = "Joint not found";
      return false;
    }

    bool SetJointDampingFullurdf2(fullurdf2::SetDamping::Request &req, fullurdf2::SetDamping::Response &res)
    {
      for (auto& params : this->joints)
      {
        if (params.joint->GetName() == req.joint_name)
        {
          params.damping = req.damping;
          ROS_INFO("Set damping of joint %s to %f", req.joint_name.c_str(), req.damping);
          res.success = true;
          res.message = "Damping updated";
          return true;
        }
      }
      res.success = false;
      res.message = "Joint not found";
      return false;
    }

    bool SetJointFrictionFullurdf2(fullurdf2::SetFriction::Request &req, fullurdf2::SetFriction::Response &res)
    {
      for (auto& params : this->joints)
      {
        if (params.joint->GetName() == req.joint_name)
        {
          params.friction = req.friction;
          ROS_INFO("Set friction of joint %s to %f", req.joint_name.c_str(), req.friction);
          res.success = true;
          res.message = "Friction updated";
          return true;
        }
      }
      res.success = false;
      res.message = "Joint not found";
      return false;
    }

    //for fullurdf3
    ///////////////////////
    bool SetJointStiffnessFullurdf3(fullurdf3::SetStiffness::Request &req, fullurdf3::SetStiffness::Response &res)
    {
      for (auto& params : this->joints)
      {
        if (params.joint->GetName() == req.joint_name)
        {
          params.stiffness = req.stiffness;
          ROS_INFO("Set stiffness of joint %s to %f", req.joint_name.c_str(), req.stiffness);
          res.success = true;
          res.message = "Stiffness updated";
          return true;
        }
      }
      res.success = false;
      res.message = "Joint not found";
      return false;
    }

    bool SetJointDampingFullurdf3(fullurdf3::SetDamping::Request &req, fullurdf3::SetDamping::Response &res)
    {
      for (auto& params : this->joints)
      {
        if (params.joint->GetName() == req.joint_name)
        {
          params.damping = req.damping;
          ROS_INFO("Set damping of joint %s to %f", req.joint_name.c_str(), req.damping);
          res.success = true;
          res.message = "Damping updated";
          return true;
        }
      }
      res.success = false;
      res.message = "Joint not found";
      return false;
    }

    bool SetJointFrictionFullurdf3(fullurdf3::SetFriction::Request &req, fullurdf3::SetFriction::Response &res)
    {
      for (auto& params : this->joints)
      {
        if (params.joint->GetName() == req.joint_name)
        {
          params.friction = req.friction;
          ROS_INFO("Set friction of joint %s to %f", req.joint_name.c_str(), req.friction);
          res.success = true;
          res.message = "Friction updated";
          return true;
        }
      }
      res.success = false;
      res.message = "Joint not found";
      return false;
    }
    /////////////////////////////////////////////////////////////////////
  private:
    struct JointParams
    {
      physics::JointPtr joint;
      double stiffness;
      double damping;
      double restAngle;
      double friction;
      ////
      // double restVelocity;
      // double previousAngle;
    };

    // List of joint parameters
    std::vector<JointParams> joints;

    // Pointer to the model
    physics::ModelPtr model;

    // Connection to the update event
    event::ConnectionPtr updateConnection;

    // ROS node handle
    ros::NodeHandle nh;

    // ROS service servers for softactuator
    ros::ServiceServer serviceStiffnessSoftactuator;
    ros::ServiceServer serviceDampingSoftactuator;
    ros::ServiceServer serviceFrictionSoftactuator;

    // ROS service servers for fullurdf2
    ros::ServiceServer serviceStiffnessFullurdf2;
    ros::ServiceServer serviceDampingFullurdf2;
    ros::ServiceServer serviceFrictionFullurdf2;

    // ROS service servers for fullurdf3
    ros::ServiceServer serviceStiffnessFullurdf3;
    ros::ServiceServer serviceDampingFullurdf3;
    ros::ServiceServer serviceFrictionFullurdf3;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(JointStiffnessPlugin)
}