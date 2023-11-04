#include <ignition/math/Pose3.hh>
#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"
#include <boost/format.hpp>
#include "gazebo_msgs/DeleteModel.h"

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"

#include "utrafman_main/insert_model.h"
#include "utrafman_main/remove_model.h"
#include "utrafman_main/teletransport.h"


namespace gazebo
{
    class UTRAFMAN_God : public WorldPlugin
    {
        private:
            //Number of UAVs in the simulation
            int num_uavs = 0;

            //World pointer
            physics::WorldPtr parent;
            //ROS node handle
            ros::NodeHandle *rosNode;
            //ROS services
            ros::ServiceServer insert_service;
            ros::ServiceServer transport_service;
            ros::ServiceServer remove_service;
            //ROS callback queue
            //ros::CallbackQueue rosQueue;
            //Spinners
            //ros::AsyncSpinner rosSpinners = ros::AsyncSpinner(1, &this->rosQueue);

        public:
            bool insert_callback(utrafman_main::insert_model::Request &req, utrafman_main::insert_model::Response &res) {
                //SDF object, model pointer and model string from the request
                sdf::SDF sdf_object;
                sdf::ElementPtr model_ptr;
                std::string model = req.modelSDF;

                //Convert from model string to SDF format
                sdf_object.SetFromString(model);

                //Get the model
                model_ptr = sdf_object.Root()->GetElement("model");

                //Insert the model in the world
                this->parent->InsertModelSDF(sdf_object);

                //Increase the number of UAVs
                this->num_uavs++;

                ROS_INFO("A new UAV has been inserted. Total UAVs: %i", this->num_uavs);
                return true;
            }

            bool remove_callback(utrafman_main::remove_model::Request &req, utrafman_main::remove_model::Response &res) {
                res.success.data = false;

                //Sent message to a topic to kill the drone
                std::string topic_name = "/drone/" + std::to_string(req.uavId) + "/kill";
                ros::Publisher topic_pub = this->rosNode->advertise<std_msgs::Bool>(topic_name, 10);
                topic_pub.publish(std_msgs::Bool());

                //Wait a few seconds (to be sure that the drone has destroyed all its resources)
                ros::Duration(1.0).sleep();

                //Call Gazebo deleteModel service
                ros::ServiceClient gazebo_remove_service = this->rosNode->serviceClient<gazebo_msgs::DeleteModel>("/gazebo/delete_model");
                gazebo_msgs::DeleteModel srv;
                srv.request.model_name = "drone_" + std::to_string(req.uavId);
                gazebo_remove_service.call(srv);

                //Check if the model has been removed
                if (srv.response.success) {
                    res.success.data = true;
                    //Decrease the number of UAVs
                    this->num_uavs--;
                }

                ROS_INFO("A UAV has been removed. Total UAVs: %i", this->num_uavs);
                return true;
            }

            bool transport_callback(utrafman_main::teletransport::Request &req, utrafman_main::teletransport::Response &res) {
                //Get the model
                physics::ModelPtr drone = this->parent->ModelByName("drone_" + std::to_string(req.uavId));
                res.success.data = false;

                //Check if the model exists
                if (drone != NULL) {
                    //Set the new pose
                    ignition::math::Pose3d pose(req.pose.position.x, req.pose.position.y, req.pose.position.z, req.pose.orientation.x, req.pose.orientation.y, req.pose.orientation.z);
                    drone->SetWorldPose(pose, true);
                    res.success.data = true;
                }
                return true;
            }

            void removeModel(const std_msgs::String::ConstPtr& msg)
            {
                ROS_INFO("Elimando drone_%s", msg->data.c_str());
                //Eliminamos el modelo con el nombre indicado
                //this->parent->RemoveModel("drone_" + std::string(msg->data.c_str())); //Esta implementacion no funciona (hay que liberar los recursos antes)
                this->parent->ModelByName("drone_" + std::string(msg->data.c_str()))->Fini();
            }


            void Load(physics::WorldPtr _parent, sdf::ElementPtr /*_sdf*/)
            {
                //Store world pointer
                this->parent = _parent;

                //Check if ROS is up
                if (ros::isInitialized()){
                    int argc = 0;
                    char **argv = NULL;
                    ros::init(argc, argv, "airspaceGod");
                }

                //Create ROS node
                this->rosNode = new ros::NodeHandle("god");

                this->insert_service = this->rosNode->advertiseService("/godservice/insert_model", &UTRAFMAN_God::insert_callback, this);
                this->remove_service = this->rosNode->advertiseService("/godservice/remove_model", &UTRAFMAN_God::remove_callback, this);
                this->transport_service = this->rosNode->advertiseService("/godservice/transport_model", &UTRAFMAN_God::transport_callback, this);

            }
    };

    // Register this plugin with the simulator
    GZ_REGISTER_WORLD_PLUGIN(UTRAFMAN_God)

}
