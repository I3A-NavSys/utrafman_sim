#include <boost/bind.hpp>

#include "ros/ros.h"
#include "ros/subscribe_options.h"
#include "ros/callback_queue.h"

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

#include <gazebo_msgs/ModelState.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Bool.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <stdio.h>
#include <iostream>
#include <fstream>

#include <cmath>

//Custom messages
#include "siam_main/Telemetry.h"
#include "siam_main/Waypoint.h"
#include "siam_main/Uplan.h"

namespace gazebo
{
    class DroneControl : public ModelPlugin
    {
    private:
        //Maximum log mode
        // 1 -> Normal logging
        // 2 -> Extended logging
        // 3 -> All data loggging
        int log_mode = 1;

        // Pointers to the model and the link
        physics::ModelPtr model;
        physics::LinkPtr link;

        // Pointer to the update event connection
        event::ConnectionPtr updateConnection;

        //Needed to control telemetry publish
        common::Time last_odom_publish_time;
        double odom_publish_rate = 2; // updates per second

        //ROS structures
        ros::NodeHandle *ros_node;
        ros::Subscriber ros_sub_uplans;
        ros::Subscriber ros_sub_kill;
        ros::Publisher ros_pub_telemetry;
        ros::CallbackQueue ros_queue;
        ros::AsyncSpinner ros_spinner = ros::AsyncSpinner(1,&this->ros_queue);

        //Topic structures
        std::string uplans_topic = "uplan";
        std::string telemetry_topic = "telemetry";
        std::string kill_topic = "kill";

        //Drone and U-plan execution
        std::string id;
        bool uplan_inprogress = false;
        siam_main::Uplan::ConstPtr uplan_local;
        std::vector<siam_main::Waypoint> route_local;

        //To control navigation
        int actual_route_point;

        //Log file
        std::ofstream control_out_file;


        //-------------------------------------------------------------------------------------------------
        // Parametros del drone

        // Posicion de los rotores
        // Distancia: 25cms desde el centro de masas, inclinacion: 45º desde los ejes
    private:
        ignition::math::Vector3<double> pos_CM = ignition::math::Vector3<double>(0, 0, 0);              // centro de masas
        ignition::math::Vector3<double> pos_NE = ignition::math::Vector3<double>(0.1768, -0.1768, 0);   // cosd(45º) * 0.25m
        ignition::math::Vector3<double> pos_NW = ignition::math::Vector3<double>(0.1768, 0.1768, 0);
        ignition::math::Vector3<double> pos_SE = ignition::math::Vector3<double>(-0.1768, -0.1768, 0);
        ignition::math::Vector3<double> pos_SW = ignition::math::Vector3<double>(-0.1768, 0.1768, 0);

        // Margen de velocidad angular de los motores
        const double w_max = 1.5708e+03; // rad/s = 15000rpm
        const double w_min = 0; // rad/s =     0rpm

        /* Fuerza de empuje aerodinamico
            La fuerza principal generada por los rotores
            FT = kFT * w²
            Asumimos que
               FT_max = 1kg = 9.8N
            por tanto, queda que... */
        const double kFT = 3.9718e-06;

        /* Momento de arrastre de los rotores
            Momento que experimenta el rotor en sentido contrario a su velocidad
            MDR = kMDR * w²
            Asumimos que
               ...
            por tanto, queda que... */
        const double kMDR = 1.3581e-07;

        /* Fuerza de arrastre aerodinamico.
            Fuerza de rozamiento con el aire, contraria a la velocidad.
            FD = -kFD * r_dot*|r_dot|
            Depende de la forma del objeto en cada eje.  */

        /* Ejes horizontales:
            Asumimos
               rozamiento similar en ambos ejes (aunque el fuselaje no sea igual)
               Vh_max = 20km/h = 5.5556m/s  (velocidad horizontal maxima)
               roll_max = 30º = 0.5236rad   (inclinacion maxima)
            operando
               FTh_max = 4*FT_max * sin(roll_max)
               FTh_max = FDh_max = 19.6
            por tanto queda que...  */
        const double kFDx = 0.6350;
        const double kFDy = 0.6350;

        /* Eje vertical:
            Debe verificarse a velocidad limite ascendente que
               FTmax * 4 = Fg + FD_max
            Asumimos que
               Vz_max = 3m/s  (maxima velocidad de ascenso)
            que nos dará una velocidad limite de descenso de
               Vz_lim = 2.7689m/s
            operando
               FT_max = 9.8N
               Fg = 1.840gr * 9.8m/s
               FD_max = 21.1681N
            por tanto queda que...  */
        const double kFDz = 2.3520;

        /* Momento de arrastre aerodinamico.
            Momento de rozamiento con el aire que sufre el drone al girar.
            Es contrario a la velocidad angular.
            MD = -kMD * rpy_dot * |rpy_dot|
            Depende de la forma del objeto en cada eje.  */

        /* Ejes horizontales:
            Asumimos
               rozamiento similar en ambos ejes (aunque el fuselaje no sea igual)
               escenario sin gravedad
               el drone es propulsado por dos rotores del mismo lado a maxima velocidad
               la velocidad angular maxima que alcanza es  Vrp_max = 2 * 2*pi;
            operando
               kMDxy =  2 * FT_max * sin(deg2rad(45))^2 / Vrp_max^2
            por tanto queda que...  */
        const double kMDx = 0.0621;
        const double kMDy = 0.0621;

        /* Eje vertical:
            Debe verificarse a velocidad limite de rotación sobre el eje Z que
               ...
            Asumimos que
               Vyaw_max = 4*pi rad/s  (maxima velocidad de rotacion en Z de 2rev/s)
               w_hov2                 (velocidad del rotor para que dos rotores mantengan la sustentacion)
            Ya teniamos que
               MDR = kMDR * w²
               MDz = kMDz * Vyaw²
            operando
               MDz  = MDR             (el rozamiento con el aire compensa el efecto de los rotores)
               kMDz = kMDR* (2 * w_hov2²) / Vyaw_max²
            por tanto queda que...  */
        const double kMDz = 0.0039;

//------------------------------------------------------------------------------------------------------

        // Control commands
        double cmd_on = 0;
        double cmd_velX = 0.0;
        double cmd_velY = 0.0;
        double cmd_velZ = 0.0;
        double cmd_rotZ = 0.0;

        // Control matrices
        Eigen::Matrix<double, 8, 1> x; // model state
        Eigen::Matrix<double, 4, 1> y; // model output
        Eigen::Matrix<double, 4, 8> Kx; // state control matrix
        Eigen::Matrix<double, 4, 4> Ky; // error control matrix
        Eigen::Matrix<double, 4, 1> Hs; // hovering speed
        Eigen::Matrix<double, 4, 1> Wr; // rotors speeds
        Eigen::Matrix<double, 4, 1> r; // model reference
        Eigen::Matrix<double, 4, 1> e; // model error
        Eigen::Matrix<double, 4, 1> E; // model acumulated error
        common::Time prev_iteration_time; // Time to integrate the acumulated error


    public:
        void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
        {
            // Store pointers to the model
            this->model = _parent;
            this->link = model->GetLink("dronelink");

            // Listen to the update event. This event is broadcast every iteration.
            this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                    boost::bind(&DroneControl::OnUpdate, this, _1));

            // Ensure that ROS has been initialized
            if (!ros::isInitialized())
            {
                ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized.");
                return;
            }

            // Getting Drone ID
            if(!_sdf->HasElement("id")){
                std::cout << "Missing parameter <id> in PluginCall, default to 0" << std::endl;
                this->id = "0";
            } else {
                this->id = _sdf->GetElement("id")->GetValue()->GetAsString();
            }

            //Creating a ROS Node for the drone
            this->ros_node = new ros::NodeHandle("drone/"+this->id);
            //Assigning the queue
            this->ros_node->setCallbackQueue(&this->ros_queue);

            // Initiates the publication topic
            this->ros_pub_telemetry = this->ros_node->advertise<siam_main::Telemetry>(this->telemetry_topic, 100);

            //Init last_odom_publish_time
            last_odom_publish_time = model->GetWorld()->SimTime();

            //Subscription options to Uplans topic
            ros::SubscribeOptions so = ros::SubscribeOptions::create<siam_main::Uplan>(
                    this->uplans_topic,
                    1000,
                    boost::bind(&DroneControl::UplansTopicCallback, this, _1),
                    ros::VoidPtr(),
                    &this->ros_queue);

            // Subscription
            this->ros_sub_uplans = this->ros_node->subscribe(so);

            //Subscription options for kill topic
            ros::SubscribeOptions so2 = ros::SubscribeOptions::create<std_msgs::Bool>(
                    this->kill_topic,
                    10,
                    boost::bind(&DroneControl::KillTopicCallback, this, _1),
                    ros::VoidPtr(),
                    &this->ros_queue);

            // Subscription
            this->ros_sub_kill = this->ros_node->subscribe(so2);

            //Asynchronous spinning start
            this->ros_spinner.start();

            Kx << -334.1327, -334.1327, -29.9223, -29.9223, 72.7456, -167.9315, 167.9315, 373.1147,
                    334.1327, -334.1327, 29.9223, -29.9223, -72.7456, -167.9315, -167.9315, 373.1147,
                    -334.1327, 334.1327, -29.9223, 29.9223, -72.7456, 167.9315, 167.9315, 373.1147,
                    334.1327, 334.1327, 29.9223, 29.9223, 72.7456, 167.9315, -167.9315, 373.1147;

            Ky << -0.3078, 0.3078, 1.5803, 0.3081,
                    -0.3078, -0.3078, 1.5803, -0.3081,
                    0.3078, 0.3078, 1.5803, -0.3081,
                    0.3078, -0.3078, 1.5803, 0.3081;
            Ky = Ky * 1.0e+03;

            Hs << sqrt(0.300 * 9.8 / 4 / kFT), sqrt(0.300 * 9.8 / 4 / kFT), sqrt(0.300 * 9.8 / 4 / kFT), sqrt(0.300 * 9.8 / 4 / kFT);
            //    std::cout  << Hs << " \n\n";

            Wr << 0, 0, 0, 0;
            //    std::cout  << Wr << " \n\n";

            r << 0, 0, 0, 0;
            //    std::cout  << r  << " \n\n";

            E << 0, 0, 0, 0;
            //    std::cout  << E  << " \n\n";

        }

        // Called by the world update start event
        void OnUpdate(const common::UpdateInfo &evento /*_info*/)
        {
            // Check if the simulation was reset
            common::Time current_time = model->GetWorld()->SimTime();
            if (current_time < prev_iteration_time)
                prev_iteration_time = current_time; // The simulation was reset

            //printf("current  iteration time: %.3f \n", current_time.Double());
            //printf("previous iteration time: %.3f \n", prev_iteration_time.Double());

            //Seconds since the last iteration
            double seconds_since_last_iteration = (current_time - prev_iteration_time).Double();
            prev_iteration_time = current_time;
            //printf("iteration time (seconds): %.6f \n", seconds_since_last_iteration);


            //Getting position, rotations and velocities
            // Getting model status
            ignition::math::Pose3<double> pose = model->WorldPose();
            //printf("drone xyz = %.2f,%.2f,%.2f \n", pose.pos.X(), pose.pos.Y(), pose.pos.Z());
            //printf("drone quaternion WXYZ = %.2f,%.2f,%.2f,%.2f \n", pose.rot.w, pose.rot.X(), pose.rot.Y(), pose.rot.Z());
            ignition::math::Vector3<double> pose_rot = pose.Rot().Euler();
            //printf("drone euler YPR = %.2f,%.2f,%.2f \n", pose_rot.Z(), pose_rot.Y(), pose_rot.X());
            ignition::math::Vector3<double> linear_vel = model->RelativeLinearVel();
            //printf("drone vel xyz = %.2f,%.2f,%.2f \n", linear_vel.X(), linear_vel.Y(), linear_vel.Z());
            ignition::math::Vector3<double> angular_vel = model->RelativeAngularVel();
            //printf("drone angular vel xyz = %.2f,%.2f,%.2f \n", angular_vel.X(), angular_vel.Y(), angular_vel.Z());

            //Variables used by the high level control
            double ttrw; // Time to reach the waypoint
            siam_main::Waypoint target_waypoint; //Next waypoint to reach
            double vx, vy, vz, vrotz, dx, dy, dz; //Distances and velocities in each axis

            //High level control execution -----------------------------------
            // If have a Uplan and the desired time to take off has been passed
            if (uplan_inprogress && (current_time.Double() >= uplan_local->dtto)){
                //Rotors on
                cmd_on = 1.0;

                //Compute target velocity
                ignition::math::Vector3d uplan_pos = this->UplanAbstractionLayer(current_time.Double());

                //Get target waypoint and positions vector
                target_waypoint = route_local[actual_route_point];
                ignition::math::Vector3<double> target_way_vector3d = ignition::math::Vector3d(target_waypoint.x, target_waypoint.y, target_waypoint.z);
                ignition::math::Vector2<double> target_way_vector2d = ignition::math::Vector2d(target_waypoint.x, target_waypoint.y);

                //Get distance from drone position to target waypoint
                double d3d = target_way_vector3d.Distance(pose.Pos());
                double d2d = target_way_vector2d.Distance(ignition::math::Vector2d(pose.Pos().X(),pose.Pos().Y()));

                //Get time remaining to the waypoint (to fullfill the Uplan)
                ttrw = target_waypoint.t.sec - current_time.Double();

                //Angle between drone heading and target
                double bearing = atan2(target_way_vector3d.Y()-pose.Pos().Y(),target_way_vector3d.X()-pose.Pos().X()) - pose_rot.Z();

                //Target distance descomposed in body axes
                dx = d2d * cos(bearing);
                dy = d2d * sin(bearing);
                dz = target_way_vector3d.Z() - pose.Pos().Z();

                //UAL
                //ignition::math::Vector3d posDiff = uplan_pos - pose.Pos();

                //If the time is negative, return to 0.1
                /*if(ttrw <= 0){
                    ttrw = 0.1;
                }*/

                //Velocities depending on ttrw
                vx = (dx/ttrw);
                vy = (dy/ttrw);
                vz = (dz/ttrw);

                //Velocities using UAL
                /*vx = posDiff.X();
                vy = posDiff.Y();
                vz = posDiff.Z();*/

                /*if (bearing > 3.1418){
                    bearing = -bearing;
                }

                vrotz = (bearing/2);*/
                vrotz = 0;

                //Getting the norm of the velocities
                ignition::math::Vector2<double> vel_h_vector = ignition::math::Vector2d(vx,vy);
                double norm_vel_h = vel_h_vector.Distance(ignition::math::Vector2d(0,0));

                //Normalize horizontal velocity
                /*if (norm_vel_h > 1.0){
                    vx = vx / norm_vel_h;
                    vy = vy / norm_vel_h;
                }*/

                //Normalize vertical velocity
                /*if (vz > 1.0){
                    vz = 1.0;
                }*/

                //Normalize Z rotational velocity
                /*if(vrotz > 1.0){
                    vrotz = 1.0;
                }*/

                ignition::math::Vector4<double> vel = ComputeVelocity(2,current_time.Double(),pose,pose_rot);

                //Giving velocities to the low leven control
                cmd_velX = vel.X();
                cmd_velY = vel.Y();
                cmd_velZ = vel.Z();
                cmd_rotZ = vel.W();

                //Detect if drone reached and change to the next waypoint
                //if (d3d < target_waypoint.r){
                    //Waypoint is changed only when waypoint time has passed
                    if (current_time.Double() >= target_waypoint.t.sec) {
                        //Check if exist more waypoint
                        if (actual_route_point < (route_local.size() - 1)) {
                            actual_route_point++;
                            std::cout << "Drone " << id << " has reached waypoint " << (actual_route_point - 1)
                                      << " and change to the waypoiny "
                                      << actual_route_point << std::endl;
                        } else {
                            //Stop
                            cmd_on = 0;
                            cmd_velX = 0.0;
                            cmd_velY = 0.0;
                            cmd_velZ = -1.0;
                            uplan_inprogress = false;
                            actual_route_point = 0;
                            std::cout << "Drone " << id << " has finished it U-plan" << std::endl;

                            //Save Uplan high control logbook
                            control_out_file.close();
                        }
                    }
                //}
            }

            //Low level control
            if (cmd_on){
                // Asignamos el estado del modelo
                x(0, 0) = pose_rot.X();    // ePhi
                x(1, 0) = pose_rot.Y();    // eTheta
                x(2, 0) = angular_vel.X(); // bWx
                x(3, 0) = angular_vel.Y(); // bWy
                x(4, 0) = angular_vel.Z(); // bWz
                x(5, 0) = linear_vel.X();  // bXdot
                x(6, 0) = linear_vel.Y();  // bYdot
                x(7, 0) = linear_vel.Z();  // bZdot

                // Asignamos la salida del modelo
                y(0, 0) = linear_vel.X();  // bXdot
                y(1, 0) = linear_vel.Y();  // bYdot
                y(2, 0) = linear_vel.Z();  // bZdot
                y(3, 0) = angular_vel.Z(); // bWz

                //Velocidad comandada en ejes del mundo
                Eigen::Matrix<double, 3, 1> h_cmd;
                h_cmd(0, 0) = cmd_velX; // eXdot
                h_cmd(1, 0) = cmd_velY; // eYdot
                h_cmd(2, 0) = cmd_velZ; // eZdot

                //Matriz de transformación de ejes de mundo al cuerpo del drone
                Eigen::Matrix<double, 3, 3> horizon2body;
                horizon2body = Eigen::AngleAxisd(-x(0, 0), Eigen::Vector3d::UnitX())    // roll
                               * Eigen::AngleAxisd(-x(1, 0), Eigen::Vector3d::UnitY()); // pitch

                // Transformamos comando de horizonte a body
                Eigen::Matrix<double, 3, 1> b_cmd;
                b_cmd = horizon2body * h_cmd;

                // Asignamos la referencia a seguir
                r(0, 0) = b_cmd(0, 0); // bXdot
                r(1, 0) = b_cmd(1, 0); // bYdot
                r(2, 0) = b_cmd(2, 0); // bZdot
                r(3, 0) = cmd_rotZ;    // hZdot

                // Error entre la salida y la referencia (entre la velocidad comandada y la del drone)
                e = y - r;

                // Error acumulado
                E = E + (e * seconds_since_last_iteration);

                // Truncamiento del error acumulado
                /*if (E(0, 0) > 1)
                    E(0, 0) = 1;
                if (E(0, 0) < -1)
                    E(0, 0) = -1;
                if (E(1, 0) > 1)
                    E(1, 0) = 1;
                if (E(1, 0) < -1)
                    E(1, 0) = -1;
                if (E(2, 0) > 1)
                    E(2, 0) = 1;
                if (E(2, 0) < -1)
                    E(2, 0) = -1;
                if (E(3, 0) > 1)
                    E(3, 0) = 1;
                if (E(3, 0) < -1)
                    E(3, 0) = -1;
                //std::cout  << "E:  " << E.transpose()  << " \n\n"; */

                // Velocidad de los rotores
                Wr = Hs - Kx * x - Ky * E;

                //Saturamos la velocidad de los motores en caso de superar la maxima o la minima
                if (Wr(0, 0) > w_max) Wr(0, 0) = w_max;
                if (Wr(0, 0) < w_min) Wr(0, 0) = w_min;

                if (Wr(1, 0) > w_max) Wr(1, 0) = w_max;
                if (Wr(1, 0) < w_min) Wr(1, 0) = w_min;

                if (Wr(2, 0) > w_max) Wr(2, 0) = w_max;
                if (Wr(2, 0) < w_min) Wr(2, 0) = w_min;

                if (Wr(3, 0) > w_max) Wr(3, 0) = w_max;
                if (Wr(3, 0) < w_min) Wr(3, 0) = w_min;
            } else {
                // Eliminamos el error acumulado y la rotacion de los motores
                E << 0, 0, 0, 0;
                Wr << 0, 0, 0, 0;
            }


            // Asignamos la rotación de los motores
            double w_rotor_NE = Wr(0, 0);
            double w_rotor_NW = Wr(1, 0);
            double w_rotor_SE = Wr(2, 0);
            double w_rotor_SW = Wr(3, 0);

            /* // con esto simulamos rotacion de sustentacion
            w_rotor_NE = sqrt(0.300*9.8/4 / kFT);  // = 430.18 con 4 rotores
            w_rotor_NW = w_rotor_NE;
            w_rotor_SE = w_rotor_NE;
            w_rotor_SW = w_rotor_NE;
            */

            // Aplicamos fuerzas/momentos por empuje de rotores
            ignition::math::Vector3<double> FT_NE = ignition::math::Vector3<double>(0, 0, kFT * pow(w_rotor_NE, 2));
            link->AddLinkForce(FT_NE, pos_NE);
            ignition::math::Vector3<double> FT_NW = ignition::math::Vector3<double>(0, 0, kFT * pow(w_rotor_NW, 2));
            link->AddLinkForce(FT_NW, pos_NW);
            ignition::math::Vector3<double> FT_SE = ignition::math::Vector3<double>(0, 0, kFT * pow(w_rotor_SE, 2));
            link->AddLinkForce(FT_SE, pos_SE);
            ignition::math::Vector3<double> FT_SW = ignition::math::Vector3<double>(0, 0, kFT * pow(w_rotor_SW, 2));
            link->AddLinkForce(FT_SW, pos_SW);

            // Aplicamos momentos por arrastre de rotores
            ignition::math::Vector3<double> MDR_NE = ignition::math::Vector3<double>(0, 0, kMDR * pow(w_rotor_NE, 2));
            ignition::math::Vector3<double> MDR_NW = ignition::math::Vector3<double>(0, 0, kMDR * pow(w_rotor_NW, 2));
            ignition::math::Vector3<double> MDR_SE = ignition::math::Vector3<double>(0, 0, kMDR * pow(w_rotor_SE, 2));
            ignition::math::Vector3<double> MDR_SW = ignition::math::Vector3<double>(0, 0, kMDR * pow(w_rotor_SW, 2));
            //    printf("MDR  = %.15f\n",MDR_NE.Z() - MDR_NW.Z() - MDR_SE.Z() + MDR_SW.Z());
            link->AddRelativeTorque(MDR_NE - MDR_NW - MDR_SE + MDR_SW);

            // Aplicamos fuerza de rozamiento con el aire
            ignition::math::Vector3<double> FD = ignition::math::Vector3<double>(
                    -kFDx * linear_vel.X() * fabs(linear_vel.X()),
                    -kFDy * linear_vel.Y() * fabs(linear_vel.Y()),
                    -kFDz * linear_vel.Z() * fabs(linear_vel.Z()));
            //    printf("drone relative vel \nbZ  %.2f\n|Z| %.2f\nFDz %.2f \n\n",
            //      linear_vel.Z(), fabs(linear_vel.Z()), -kFDz * linear_vel.Z() * fabs(linear_vel.Z()) );
            link->AddLinkForce(FD, pos_CM);

            // aplicamos momento de rozamiento con el aire
            ignition::math::Vector3<double> MD = ignition::math::Vector3<double>(
                    -kMDx * angular_vel.X() * fabs(angular_vel.X()),
                    -kMDy * angular_vel.Y() * fabs(angular_vel.Y()),
                    -kMDz * angular_vel.Z() * fabs(angular_vel.Z()));
            link->AddRelativeTorque(MD);

            // Comprobamos si es momento de publicar la telemetria
            if (current_time < last_odom_publish_time)
                last_odom_publish_time = current_time; // The simulation was reset
            //    printf("current time:  %.3f \n", current_time.Double());
            //    printf("last time:     %.3f \n", last_odom_publish_time.Double());

            double seconds_since_last_update = (current_time - last_odom_publish_time).Double();
            if (seconds_since_last_update > (1.0 / odom_publish_rate))
            {
                siam_main::Telemetry msg;
                //Position in the space (ejes del mundo)
                msg.pose.position.x = pose.Pos().X(); // eX
                msg.pose.position.y = pose.Pos().Y(); // eY
                msg.pose.position.z = pose.Pos().Z(); // eZ
                //Rotation in the space (ejes del mundo)
                msg.pose.orientation.w = 0;
                msg.pose.orientation.x = pose_rot.X(); // ePhi
                msg.pose.orientation.y = pose_rot.Y(); // eTheta
                msg.pose.orientation.z = pose_rot.Z(); // ePsi
                //Lineal velocity (ejes del drone)
                msg.velocity.linear.x = linear_vel.X(); // bXdot
                msg.velocity.linear.y = linear_vel.Y(); // bYdot
                msg.velocity.linear.z = linear_vel.Z(); // bZdot
                //Angular velocity (ejes del drone)
                msg.velocity.angular.x = angular_vel.X(); // bWx
                msg.velocity.angular.y = angular_vel.Y(); // bWy
                msg.velocity.angular.z = angular_vel.Z(); // bWz

                //Waypoint al que se dirige , Uplan en progreso
                msg.wip = actual_route_point;
                msg.fpip = uplan_inprogress;
                //Tiempo de telemetria
                ros::Time actual_time = ros::Time(current_time.Double());
                msg.time = actual_time;

                //Publicamos el mensaje en el topico
                ros_pub_telemetry.publish(msg);

                //Almacenamos la ultima vez que la telemetria fue enviada
                last_odom_publish_time = current_time;

                //Log
                PrintToFile(1, "OnUpdate", "TTRW: " + std::to_string(ttrw));
                PrintToFile(1, "OnUpdate", "Drone pos: X: " + std::to_string(pose.Pos().X()) + " Y: " + std::to_string(pose.Pos().Y()) + " Z: " + std::to_string(pose.Pos().Z()));
                PrintToFile(1,"OnUpdate", "Waypoint X: " + std::to_string(target_waypoint.x) + " Y: " + std::to_string(target_waypoint.y) + " Z: " + std::to_string(target_waypoint.z));
                PrintToFile(1, "OnUpdate", "Distance to W  X: " + std::to_string(dx) + " Y: " + std::to_string(dy) + " Z: " + std::to_string(dz));
                PrintToFile(1, "OnUpdate", "Current vel X: " + std::to_string(linear_vel.X()) + " Y: " + std::to_string(linear_vel.Y()) + " Z: " + std::to_string(linear_vel.Z()));
                PrintToFile(1, "OnUpdate", "CMD HL vel X: " + std::to_string(vx) + " Y: " + std::to_string(vy) + " Z: " + std::to_string(vz));
                PrintDelimToFile();
            }
        }

        //Member to receive an Uplan  using the topic
        void UplansTopicCallback(const siam_main::Uplan::ConstPtr &msg){
            //Open the file used to save logs
            if (!control_out_file.is_open()) {
                control_out_file.open("/tmp/drone-" + id + "_fp-" + std::to_string(msg->flightPlanId) + ".txt");
            }
            //If the drone is following a U-plan, not interrupt it (at this moment). If not, assign the Uplan
            if (this->uplan_inprogress){
                this->PrintToFile(1, "UPlanTopic", "New U-plan received for drone " + this->id + " but the drone is following its local U-plan");
                return;
            } else {
                this->PrintToFile(1, "UPlanTopic","Received U-plan for drone " + this->id);
                this->uplan_local = msg;
                this->actual_route_point = 0;
                this->route_local = msg->route;
                this->uplan_inprogress = true;
            }
        }

        //Member to abstract drone control from the definition of an Uplan
        ignition::math::Vector3<double> UplanAbstractionLayer(double t){
            //Function variables used in the function
            siam_main::UplanConstPtr uplan = this->uplan_local;     //Uplan
            std::vector<siam_main::Waypoint> route = uplan->route;  //Uplan route
            int route_s = route.size();                             //Number of waypoints in the route
            siam_main::Waypoint wp;                                 //Waypoint
            double timeDiffBetWPs, timeDiffBetWPt;                  //Time difference between waypoints and between waypooint and actual time
            ignition::math::Vector3d vectorDiff, position;          //Position and vector diff

            //In case Uplan starts after t
            if (t < route[0].t.sec){
                PrintToFile(2, "UplaAbsLay", "UPlan " + std::to_string(uplan_local->flightPlanId) + " not started yet");
                return ignition::math::Vector3d(route[0].x, route[0].y, route[0].z);
            }

            //In case Uplan already finish, return last waypoint
            if (t > route[route_s-1].t.sec){
                PrintToFile(2, "UplaAbsLay", "UPlan " + std::to_string(uplan_local->flightPlanId) + " already finished");
                return ignition::math::Vector3d(route[route_s-1].x, route[route_s-1].y, route[route_s-1].z);
            }

            for(int i = 1; i < route_s; i++){
                //Next waypoint
                wp = route[i];
                //If waypoint time is less than t, change to the next waypoint
                if (t > wp.t.sec){
                    continue;
                }
                //Time differences between waypoints and t and the previous waypoint
                timeDiffBetWPs = wp.t.sec - route[i-1].t.sec;
                timeDiffBetWPt = t - route[i-1].t.sec;

                //Position differences between waypoints (vector between those WPs)
                vectorDiff = ignition::math::Vector3d(wp.x - route[i-1].x, wp.y - route[i-1].y, wp.z - route[i-1].z);

                //Position where the drone must be in time t
                position = vectorDiff * (timeDiffBetWPt/timeDiffBetWPs) + ignition::math::Vector3d(route[i-1].x, route[i-1].y, route[i-1].z);
                PrintToFile(2, "UplaAbsLay", "Position to reach: X:" + std::to_string(position.X()) + " Y: " + std::to_string(position.Y()) + " Z: " + std::to_string(position.Z()) + " t: " + std::to_string(t));

                break;
            }
            //Return target position
            return position; //Target position at time t
        }

        //Member that compute the desired velocity depending the mode in use
        ignition::math::Vector4<double> ComputeVelocity(int mode, double t, ignition::math::Pose3<double> pose, ignition::math::Vector3<double> pose_rot){
            //Variables used by the high level control
            ignition::math::Vector4<double> final_vel;

            //Get last publish time to write logs in file
            double seconds_since_last_update = t - last_odom_publish_time.Double();

            //Navigation mode selector
            // 1 -> Navigation based in actual reference
            // 2 -> Navigation based in actual al future reference (2 secs future)
            if (mode == 1) {
                //Get target position in time t
                ignition::math::Vector3d uplan_pos = this->UplanAbstractionLayer(t);

                //Compute difference between actual position and target position
                ignition::math::Vector3d posDiff = uplan_pos - pose.Pos();

                //Assign final velocity
                final_vel = ignition::math::Vector4<double>(posDiff.X(), posDiff.Y(), posDiff.Z(), 0);

                //Log
                if (seconds_since_last_update > (1.0 / odom_publish_rate)){
                    PrintToFile(1, "ComputVeloci", "Target pos at t X: " + std::to_string(uplan_pos.X()) + " Y: " + std::to_string(uplan_pos.Y()) + " Z: " + std::to_string(uplan_pos.Z()));
                }
            } else if (mode == 2){
                //Get target position in time t
                ignition::math::Vector3d uplan_pos = this->UplanAbstractionLayer(t);

                //Get target position in time t+2
                ignition::math::Vector3d uplan_pos_future = this->UplanAbstractionLayer(t+2);

                //Compute difference between actual position and target position
                ignition::math::Vector3d pos_diff = uplan_pos - pose.Pos();
                ignition::math::Vector3d pos_diff_future = uplan_pos_future - pose.Pos();

                //Mix both velocities
                ignition::math::Vector3d vel = pos_diff*0.7 + pos_diff_future*0.3;

                //Assign final velocity
                final_vel = ignition::math::Vector4<double>(vel.X(), vel.Y(), vel.Z(), 0);

                //Log data
                if (seconds_since_last_update > (1.0 / odom_publish_rate)){
                    PrintToFile(1, "ComputVeloci", "Target pos at t X: " + std::to_string(uplan_pos.X()) + " Y: " + std::to_string(uplan_pos.Y()) + " Z: " + std::to_string(uplan_pos.Z()));
                    PrintToFile(1, "ComputVeloci", "Target pos at t+2 X: " + std::to_string(uplan_pos_future.X()) + " Y: " + std::to_string(uplan_pos_future.Y()) + " Z: " + std::to_string(uplan_pos_future.Z()));
                }
            }

            return final_vel;
        }

        //Member to disconnect the drone from the ROS network to remove the model
        void KillTopicCallback(const std_msgs::BoolConstPtr &value){
            //Disconnection from the Update events
            event::Events::worldUpdateBegin.Disconnect(this->updateConnection->Id());
            //Shutdown the topics
            this->ros_sub_uplans.shutdown();
            this->ros_pub_telemetry.shutdown();
            this->ros_sub_kill.shutdown();
            //Disable the queue
            this->ros_queue.clear();
            this->ros_queue.disable();
            //Shutdown the nodes
            this->ros_node->shutdown();
            //Now, Gazebo service /gazebo/remove-model with model name must be called from outside the plugin
        }

        //Member to Print message into file
        void PrintToFile(int mode, std::string module, std::string message){
            //Filter log messages
            if (log_mode < mode) return;
            //Get actual simulation time
            common::Time current_time = model->GetWorld()->SimTime();
            if (control_out_file.is_open()){
                control_out_file << "[" << std::fixed << std::setw(8) << std::setprecision(3) << current_time.Double() << "] (" << std::fixed << std::setw(12) << module << ") " << message << std::endl;
            }
        }

        //Print a delimiter into the file
        void PrintDelimToFile(){
            if (control_out_file.is_open()){
                control_out_file << "-----------------------------------------------------------------" << std::endl;
            }
        }

        void PrintToScreen(){
        }
    };


// Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(DroneControl)
} // namespace gazebo
