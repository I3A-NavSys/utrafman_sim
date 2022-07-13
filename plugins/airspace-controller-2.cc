#include <ignition/math/Pose3.hh>
#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"
#include "libs/boost/format.hpp"

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"

#define N_SPHERES 2

namespace gazebo
{
    class AirspaceController2 : public WorldPlugin
    {
        private:
            physics::WorldPtr parent;
            std::unique_ptr<ros::NodeHandle> rosNode;
            ros::Subscriber rosSub;
            ros::CallbackQueue rosQueue;
            std::thread rosQueueThread;

        public:
            void Load(physics::WorldPtr _parent, sdf::ElementPtr /*_sdf*/)
            {
                // Option 1: Insert model from file via function call.
                // The filename must be in the GAZEBO_MODEL_PATH environment variable.
                //_parent->InsertModelFile("model://box");

                this->parent = _parent;

                sdf::SDF box[N_SPHERES];
                sdf::ElementPtr model[N_SPHERES];

                std::ostringstream fichero;
                fichero << std::ifstream("/opt/ros/noetic/share/airspace-controller/models/sphere/sphere_template.txt").rdbuf();
                std::string plantillaModelo = fichero.str();

                std::ostringstream ss;
                std::string sdfString;


                for (int i=0; i<N_SPHERES; i++)
                {
                    ss << boost::format(plantillaModelo) % i % i % i % (i*10);

                    sdfString = ss.str();
                    box[i].SetFromString(sdfString);
                    model[i] = box[i].Root()->GetElement("model");
                    model[i]->GetAttribute("name")->SetFromString("unique-box"+std::to_string(i));
                    _parent->InsertModelSDF(box[i]);
                    ss.str("");
                }

                std::cout << "Finalizada la carga de los modelos" << std::endl;
                ROS_INFO("Finalizado");

                if (ros::isInitialized()){
                    int argc = 0;
                    char **argv = NULL;
                    ros::init(argc, argv, "airspaceGod",
                              ros::init_options::NoSigintHandler);
                }

                this->rosNode.reset(new ros::NodeHandle("airspaceGod"));
                ros::SubscribeOptions so = ros::SubscribeOptions::create<std_msgs::String>(
                    "/godCommands",
                    1000,
                    boost::bind(&AirspaceController2::OnRosMsg, this, _1),
                    ros::VoidPtr(),
                    &this->rosQueue
                );
                this->rosSub = this->rosNode->subscribe(so);

                this->rosQueueThread = std::thread(std::bind(&AirspaceController2::QueueThread, this));
            }

            void OnRosMsg(const std_msgs::String::ConstPtr &msg){
                ROS_INFO("I heard: [%s]", msg->data.c_str());

                sdf::SDF sdffile;
                sdf::ElementPtr modelPtr;

                sdffile.SetFromString(msg->data.c_str());
                modelPtr = sdffile.Root()->GetElement("model");
                this->parent->InsertModelSDF(sdffile);
            }

        private:
            void QueueThread(){
                static const double timeout = 1;
                while(this->rosNode->ok()){
                    this->rosQueue.callAvailable(ros::WallDuration(timeout));
                }
            }
    };

    // Register this plugin with the simulator
    GZ_REGISTER_WORLD_PLUGIN(AirspaceController2)

}