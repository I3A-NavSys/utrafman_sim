#include <ignition/math/Pose3.hh>
#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"
#include "libs/boost/format.hpp"

#define N_SPHERES 20

namespace gazebo
{
    class AirspaceController : public WorldPlugin
    {
        public: void Load(physics::WorldPtr _parent, sdf::ElementPtr /*_sdf*/)
        {
            // Option 1: Insert model from file via function call.
            // The filename must be in the GAZEBO_MODEL_PATH environment variable.
            //_parent->InsertModelFile("model://box");

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
        }
    };

    // Register this plugin with the simulator
    GZ_REGISTER_WORLD_PLUGIN(AirspaceController)
}