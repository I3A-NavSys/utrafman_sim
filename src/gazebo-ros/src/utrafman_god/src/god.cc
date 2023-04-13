#include <ignition/math/Pose3.hh>
#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"
#include <boost/format.hpp>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"

#include "utrafman_main/insert_model.h"

namespace gazebo
{
    class UTRAFMAN_God : public WorldPlugin
    {
        private:
            int drones = 0;

            //Puntero al mundo
            physics::WorldPtr parent;

            //ROS
            //Nodo de ROS
            ros::NodeHandle *rosNode;

            //Suscriptores
            ros::Subscriber rosSub_insert;
            ros::Subscriber rosSub_remove;
            ros::ServiceServer insert_service;

            //Cola de mensajes
            ros::CallbackQueue rosQueue;

            //Spinners
            ros::AsyncSpinner rosSpinners = ros::AsyncSpinner(1, &this->rosQueue);

        public:
            //Handle para la insercion de modelos por cada uno de los mensajes en el topico
            void insertModel(const std_msgs::String::ConstPtr& msg)
            {
                //Modelo SDF y puntero al modelo
                sdf::SDF sdf_object;
                sdf::ElementPtr model_ptr;

                //Establecemos el SDF del modelo con el contenido del mensaje del topico
                sdf_object.SetFromString(msg->data.c_str());

                //Obtenemos el modelo insertado y cambiamos el nombre
                model_ptr = sdf_object.Root()->GetElement("model");

                //Aumentamos el contador de drones
                this->drones++;

                //Insertamos el modelo en el mundo
                this->parent->InsertModelSDF(sdf_object);

                ROS_INFO("Nuevo drone anadido. Total: %i", this->drones);
            }

            bool insert_callback(utrafman_main::insert_model::Request &req, utrafman_main::insert_model::Response &res) {
                //Modelo SDF y puntero al modelo
                sdf::SDF sdf_object;
                sdf::ElementPtr model_ptr;
                std::string model = req.modelSDF;

                //Establecemos el SDF del modelo con el contenido del mensaje del topico
                sdf_object.SetFromString(model);

                //Obtenemos el modelo insertado y cambiamos el nombre
                model_ptr = sdf_object.Root()->GetElement("model");

                //Aumentamos el contador de drones
                this->drones++;

                //Insertamos el modelo en el mundo
                this->parent->InsertModelSDF(sdf_object);
                //std::string  name = model_ptr->GetName();

                /*physics::ModelPtr drone = this->parent->ModelByName("drone_" + std::to_string(this->drones-1));
                if (drone != NULL) {
                    drone->SetScale(ignition::math::Vector3d(7, 7, 7), true);
                    std::cout << "Drone " << this->drones << " modificado: " << drone << std::endl;
                }*/

                //auto models = this->parent->Models();
                //for (auto model : models) {
                //    std::cout << "Modelo: " << model->GetName() << std::endl;
                //}

                ROS_INFO("Nuevo drone anadido. Total: %i", this->drones);
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
                //Almacenamos el puntero al elemento padre del mundo
                this->parent = _parent;

                    //Para cargar un elemento de forma dinamica cuando se inicia la simulacion
                    //Reservamos espacio para los modelos
                    /*sdf::SDF box[N_SPHERES];
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
                        model[i]->GetAttribute("name")->SetFromString("drone_" + std::to_string(i));
                        //Añadimos el modelo al escenario
                        _parent->InsertModelSDF(box[i]);
                        this->drones++;
                        //Reseteamos el contenido de ss
                        ss.str("");
                    }

                    //Informamos de que se ha terminado de cargar los modelos
                    ROS_INFO("Finalizado");
                //-----------------------------------------------------------*/

                //Comprobamos si ROS ha sido inicializado, y lo iniciamos si no es asi
                if (ros::isInitialized()){
                    int argc = 0;
                    char **argv = NULL;
                    ros::init(argc, argv, "airspaceGod");
                }

                //Creamos el nodo
                this->rosNode = new ros::NodeHandle("god");

                //Creamos el subscriptor handle para insertar modelos
                ros::SubscribeOptions so_insert = ros::SubscribeOptions::create<std_msgs::String>(
                        "insert",
                        1000,
                        boost::bind(&UTRAFMAN_God::insertModel, this, _1),
                        ros::VoidPtr(),
                        &this->rosQueue
                );

                this->insert_service = this->rosNode->advertiseService("/godservice/insert_model", &UTRAFMAN_God::insert_callback, this);
                //Creamos el subscriptor handle para eliminar modelos
                /*ros::SubscribeOptions so_remove = ros::SubscribeOptions::create<std_msgs::String>(
                        "remove",
                        1000,
                        boost::bind(&UTRAFMAN_God::removeModel, this, _1),
                        ros::VoidPtr(),
                        &this->rosQueue
                );*/

                //Creamos la suscripcion del nodo a los topicos
                this->rosSub_insert = this->rosNode->subscribe(so_insert);
                //this->rosSub_remove = this->rosNode->subscribe(so_remove);

                //Ejecutamos el spiner de manera asincrona y no bloqueante
                this->rosSpinners.start();
            }
    };

    // Register this plugin with the simulator
    GZ_REGISTER_WORLD_PLUGIN(UTRAFMAN_God)

}
