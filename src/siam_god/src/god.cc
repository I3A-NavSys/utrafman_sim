#include <ignition/math/Pose3.hh>
#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"
#include "libs/boost/format.hpp"

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"

#define N_SPHERES 10

namespace gazebo
{
    class SIAM_God : public WorldPlugin
    {
        private:
            int drones = 0;

            //Puntero al mundo
            physics::WorldPtr parent;

            //Manejadores de ROS/////////////////////
            //Nodo de ROS
//            std::unique_ptr<ros::NodeHandle> rosNode;
            ros::NodeHandle *rosNode;

            //Suscriptores
            ros::Subscriber rosSub_insert;
            ros::Subscriber rosSub_remove;

            ros::CallbackQueue rosQueue;

            //Spinners
            ros::AsyncSpinner rosSpinners = ros::AsyncSpinner(1, &this->rosQueue);

            //Cola de mensajes
//            ros::CallbackQueue rosQueue;
            //Thread que ejecuta el procesamiento
//            std::thread rosQueueThread;

        public:
            void insertModel(const std_msgs::String::ConstPtr& msg)
            {
                ROS_INFO("Añadiendo drone con SDF [%s]", msg->data.c_str());

                sdf::SDF sdf_object;
                sdf::ElementPtr model_ptr;

                //Establecemos el SDF del modelo con el contenido del topic
                sdf_object.SetFromString(msg->data.c_str());
                //Obtenemos el modelo
                model_ptr = sdf_object.Root()->GetElement("model");
                model_ptr->GetAttribute("name")->SetFromString("drone-" + std::to_string(this->drones));
                this->drones++;
                this->parent->InsertModelSDF(sdf_object);
            }

            void removeModel(const std_msgs::String::ConstPtr& msg)
            {
                ROS_INFO("Elimando drone-%s", msg->data.c_str());
                this->parent->RemoveModel("drone-" + std::string(msg->data.c_str()));
            }

            void Load(physics::WorldPtr _parent, sdf::ElementPtr /*_sdf*/)
            {
                // Option 1: Insert model from file via function call.
                // The filename must be in the GAZEBO_MODEL_PATH environment variable.
                //_parent->InsertModelFile("model://box");

                //Almacenamos el puntero al elemento padre
                this->parent = _parent;

                //Reservamos espacio para los modelos
                sdf::SDF box[N_SPHERES];
                sdf::ElementPtr model[N_SPHERES];

                //Leemos el fichero con la plantilla del SDF del modelo
                std::ostringstream fichero;
                fichero << std::ifstream("/opt/ros/noetic/share/airspace-controller/models/sphere/sphere_template.txt").rdbuf();
                std::string plantillaModelo = fichero.str();

                std::ostringstream ss;
                std::string sdfString;

                for (int i=0; i<N_SPHERES; i++)
                {
                    //Damos formato al modelo
                    ss << boost::format(plantillaModelo) % i % i % i % (i*10);
                    sdfString = ss.str();
                    std::cout << ss.str() << std::endl;

                    //Asignamos el SDF al objeto
                    box[i].SetFromString(sdfString);
                    model[i] = box[i].Root()->GetElement("model");
                    //Modificamos si atributo name por un valor unico
                    model[i]->GetAttribute("name")->SetFromString("drone-" + std::to_string(i));
                    //Añadimos el modelo al escenario
                    _parent->InsertModelSDF(box[i]);
                    this->drones++;
                    //Reseteamos el contenido de ss
                    ss.str("");
                }

                //Informamos de que se ha terminado de cargar los modelos
                ROS_INFO("Finalizado");

                //Comprobamos si ROS ha sido inicializado
                if (ros::isInitialized()){
                    int argc = 0;
                    char **argv = NULL;
                    ros::init(argc, argv, "airspaceGod");
                }
                //Creamos el nodo
                this->rosNode = new ros::NodeHandle();

                ros::SubscribeOptions so_insert = ros::SubscribeOptions::create<std_msgs::String>(
                        "god/insert",
                        1000,
                        boost::bind(&SIAM_God::insertModel, this, _1),
                        ros::VoidPtr(),
                        &this->rosQueue
                );

                ros::SubscribeOptions so_remove = ros::SubscribeOptions::create<std_msgs::String>(
                        "god/remove",
                        1000,
                        boost::bind(&SIAM_God::removeModel, this, _1),
                        ros::VoidPtr(),
                        &this->rosQueue
                );

                //Creamos la suscripcion al nodo
                this->rosSub_insert = this->rosNode->subscribe(so_insert);
                this->rosSub_remove = this->rosNode->subscribe(so_remove);
//
//
//                this->rosSub = this->rosNode->subscribe("/god", 1000, boost::bind(&SIAM_God::insertModel, this, _1), ros::VoidConstPtr(), ros::TransportHints());

                //Ejecutamos el spiner de manera asincrona y no bloqueante
                this->rosSpinners.start();

                //ros::waitForShutdown();
            }

//            void OnRosMsg(const std_msgs::String::ConstPtr &msg){
//                ROS_INFO("I heard: [%s]", msg->data.c_str());
//
//                sdf::SDF sdffile;
//                sdf::ElementPtr modelPtr;
//
//                sdffile.SetFromString(msg->data.c_str());
//                modelPtr = sdffile.Root()->GetElement("model");
//                this->parent->InsertModelSDF(sdffile);
//            }
//
//        private:
//            void QueueThread(){
//                static const double timeout = 1;
//                while(this->rosNode->ok()){
//                    this->rosQueue.callAvailable(ros::WallDuration(timeout));
//                }
//            }
    };

    // Register this plugin with the simulator
    GZ_REGISTER_WORLD_PLUGIN(SIAM_God)

}