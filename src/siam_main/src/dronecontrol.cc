#include <boost/bind.hpp>
#include "ros/ros.h"
#include "ros/subscribe_options.h"
#include "ros/callback_queue.h"

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

#include <gazebo_msgs/ModelState.h>
#include <geometry_msgs/Pose.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <stdio.h>

namespace gazebo
{
class DroneControl : public ModelPlugin
{
   // quadcopter parameters

   // Posicion de los rotores
   // distancia: 25cms, inclinacion: 45º
   ignition::math::Vector3<double> pos_CM = ignition::math::Vector3<double>(0, 0, 0); // centro de masas
   ignition::math::Vector3<double> pos_NE = ignition::math::Vector3<double>(0.1768, -0.1768, 0);
   ignition::math::Vector3<double> pos_NW = ignition::math::Vector3<double>(0.1768, 0.1768, 0);
   ignition::math::Vector3<double> pos_SE = ignition::math::Vector3<double>(-0.1768, -0.1768, 0);
   ignition::math::Vector3<double> pos_SW = ignition::math::Vector3<double>(-0.1768, 0.1768, 0);

   // Margen de velocidad de los motores
private:
   const double w_max = 1.5708e+03; // rad/s = 15000rpm
private:
   const double w_min = 0; // rad/s =     0rpm

   /* Fuerza de empuje aerodinamico
       La fuerza principal generada por los rotores
       FT = kFT * w²  
       Asumimos que 
          FT_max = 1kg = 9.8N
       por tanto, queda que... */
private:
   const double kFT = 3.9718e-06;

   /* Momento de arrastre de los rotores
       Momento que experimenta el rotor en sentido contrario a su velocidad
       MDR = kMDR * w²
       Asumimos que 
          ...
       por tanto, queda que... */
private:
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
private:
   const double kFDx = 0.6350;

private:
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
private:
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
private:
   const double kMDx = 0.0621;

private:
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
private:
   const double kMDz = 0.0039;

   // Pointers to the model
private:
   physics::ModelPtr model;

private:
   physics::LinkPtr link;

   // Pointer to the update event connection
private:
   event::ConnectionPtr updateConnection;

   // ROS management
private:
   ros::NodeHandle *rosnode_;

   // Odometry publisher
private:
   ros::Publisher pub_;

private:
   common::Time last_odom_publish_time;

private:
   double odom_publish_rate = 10; // updates per second

   // Control subscriber
private:
   ros::Subscriber sub_;

private:
   std::string topic_subscripted_ = "bus_command";

private:
   ros::CallbackQueue queue_;

private:
   boost::thread callback_queue_thread_;

private:
   double cmd_on = 0;

private:
   double cmd_velX = 0.0;

private:
   double cmd_velY = 0.0;

private:
   double cmd_velZ = 0.0;

private:
   double cmd_rotZ = 0.0;

   // Control matrices
private:
   Eigen::Matrix<double, 8, 1> x; // model state
private:
   Eigen::Matrix<double, 4, 1> y; // model output
private:
   Eigen::Matrix<double, 4, 8> Kx; // state control matrix
private:
   Eigen::Matrix<double, 4, 4> Ky; // error control matrix
private:
   Eigen::Matrix<double, 4, 1> Hs; // hovering speed
private:
   Eigen::Matrix<double, 4, 1> Wr; // rotors speeds
private:
   Eigen::Matrix<double, 4, 1> r; // model reference
private:
   Eigen::Matrix<double, 4, 1> e; // model error
private:
   Eigen::Matrix<double, 4, 1> E; // model acumulated error
private:
   common::Time prev_iteration_time; // Time to integrate the acumulated error

   ////////////////////////////////////////////////////////
public:
   void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
   {
      //    _sdf->PrintValues("\n\n\n");

       /*_sdf->GetParent()->FindElement("link")->FindElement("sensor")->FindElement("plugin")->FindElement("robotNamespace")->GetValue()->SetFromString("quadcopter1");

       _sdf->GetParent()->FindElement("link")->FindElement("sensor")->FindElement("plugin")->Update();

       std::cout << _sdf->GetParent()->FindElement("link")->FindElement("sensor")->FindElement("plugin")->FindElement("robotNamespace")->ToString("") << std::endl;

       _parent->UpdateParameters(_sdf->GetParent()->FindElement("link")->FindElement("sensor")->FindElement("plugin"));*/



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

      // Configure a ROS node
       std::string id;

       if(!_sdf->HasElement("id")){
          std::cout << "Missing parameter <id> in PluginCall, default to 1" << std::endl;
          id = "1";
      } else {
          id = _sdf->GetElement("id")->GetValue()->GetAsString();
          std::cout << "ID del drone introducido" << std::endl;
      }

      this->rosnode_ = new ros::NodeHandle("quadcopter"+id);

      // Initiates the publication topic
      this->pub_ = this->rosnode_->advertise<gazebo_msgs::ModelState>("odometry", 10);
      last_odom_publish_time = model->GetWorld()->SimTime();

      // Initiates the subcripted topic

      ros::SubscribeOptions so = ros::SubscribeOptions::create<geometry_msgs::Pose>(
          topic_subscripted_,
          1000,
          boost::bind(&DroneControl::busCommandCallBack, this, _1),
          ros::VoidPtr(), &this->queue_);
      this->sub_ = this->rosnode_->subscribe(so);
      this->callback_queue_thread_ =
          boost::thread(boost::bind(&DroneControl::QueueThread, this));

      /*
      // Initiates control matrices
      Kx << -178.5366, -178.5366, -21.9430, -21.9430,  55.6290, -64.9335,  64.9335,  285.3230,
             178.5366, -178.5366,  21.9430, -21.9430, -55.6290, -64.9335, -64.9335,  285.3230,
            -178.5366,  178.5366, -21.9430,  21.9430, -55.6290,  64.9335,  64.9335,  285.3230,
             178.5366,  178.5366,  21.9430,  21.9430,  55.6290,  64.9335, -64.9335,  285.3230;
//    std::cout  << Kx << " \n\n";              
              
      Ky << -85.4924,  85.4924,  921.8127,  179.7244,
            -85.4924, -85.4924,  921.8127, -179.7244,
             85.4924,  85.4924,  921.8127, -179.7244,
             85.4924, -85.4924,  921.8127,  179.7244;
//    std::cout  << Ky << " \n\n";
*/

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

   ////////////////////////////////////////////////////////
public:
   void busCommandCallBack(const geometry_msgs::Pose::ConstPtr &msg)
   {

      /* Ejemplos de scripts para monitorizar los topic
           $ rostopic pub -v -1 /quadcopter/bus_command geometry_msgs/Pose '{position: {x: 1}, orientation: {x: 1, y: 2, z: 3, w: 4} }'
           $ rostopic pub -v -1 /quadcopter/odometry gazebo_msgs/ModelState '{ pose: { position: {x: 11, y: 2, z: 3}, orientation: { x: 1, y: 2, z: 3, w: 4} }, twist: { linear: { x: '21', y: '2', z: '3' }, angular: { x: 1, y:  22, z:  3} } }'
        */

      // Capturamos el comando del topic
      cmd_on = msg->position.x;
      cmd_velX = msg->orientation.x;
      cmd_velY = msg->orientation.y;
      cmd_velZ = msg->orientation.z;
      cmd_rotZ = msg->orientation.w;

      // Filtramos comandos fuera de rango
      if ((cmd_on != 1) && (cmd_on != 0))
         cmd_on = 0;
      if (cmd_velX > 1)
         cmd_velX = 1;
      if (cmd_velX < -1)
         cmd_velX = -1;
      if (cmd_velY > 1)
         cmd_velY = 1;
      if (cmd_velY < -1)
         cmd_velY = -1;
      if (cmd_velZ > 1)
         cmd_velZ = 1;
      if (cmd_velZ < -1)
         cmd_velZ = -1;
      if (cmd_rotZ > 1)
         cmd_rotZ = 1;
      if (cmd_rotZ < -1)
         cmd_rotZ = -1;

 //     printf("bus_command_on:   %f \nbus_command_velX: %.2f \nbus_command_velY: %.2f \nbus_command_velZ: %.2f \nbus_command_rotZ: %.2f \n---\n",
 //            cmd_on, cmd_velX, cmd_velY, cmd_velZ, cmd_rotZ);
   }

   ////////////////////////////////////////////////////////
   // Called by the world update start event
public:
   void OnUpdate(const common::UpdateInfo & /*_info*/)
   {

      // Tiempo desde la ultima iteracion
      common::Time current_time = model->GetWorld()->SimTime();
      if (current_time < prev_iteration_time)
         prev_iteration_time = current_time; // The simulation was reset

   //   printf("current  iteration time: %.3f \n", current_time.Double());
   //   printf("previous iteration time: %.3f \n", prev_iteration_time.Double());

      double seconds_since_last_iteration = (current_time - prev_iteration_time).Double();
      //printf("iteration time (seconds): %.6f \n", seconds_since_last_iteration);
      prev_iteration_time = current_time;

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

      if (cmd_on)
      {

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

         // Transformamos comando de horizonte a body
         Eigen::Matrix<double, 3, 1> h_cmd;
         h_cmd(0, 0) = cmd_velX; // eXdot
         h_cmd(1, 0) = cmd_velY; // eYdot
         h_cmd(2, 0) = cmd_velZ; // eZdot
                                 //        h_cmd = h_cmd * 2.0;        // cmd_calibration
                                 //        std::cout  << "\nh_cmd\n" << h_cmd << "\n\n";

         Eigen::Matrix<double, 3, 3> horizon2body;
         horizon2body = Eigen::AngleAxisd(-x(0, 0), Eigen::Vector3d::UnitX())    // roll
                        * Eigen::AngleAxisd(-x(1, 0), Eigen::Vector3d::UnitY()); // pitch
                                                                                 //        std::cout  << "\nhorizon2body\n" << horizon2body << "\n\n";

         Eigen::Matrix<double, 3, 1> b_cmd;
         b_cmd = horizon2body * h_cmd;
         //        std::cout  << "\nb_cmd\n" << b_cmd << "\n\n\n\n";

         // Asignamos la referencia a seguir
         r(0, 0) = b_cmd(0, 0); // bXdot
         r(1, 0) = b_cmd(1, 0); // bYdot
         r(2, 0) = b_cmd(2, 0); // bZdot
         r(3, 0) = cmd_rotZ;    // hZdot

         // Error entre la salida y la referencia
         e = y - r;

         // Error acumulado
         E = E + (e * seconds_since_last_iteration);

         if (E(0, 0) > 1)
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
         //        std::cout  << "E:  " << E.transpose()  << " \n\n";

         // Velocidad de los rotores
         Wr = Hs - Kx * x - Ky * E;
         if (Wr(0, 0) > 1650)
            Wr(0, 0) = 1650;
         if (Wr(0, 0) < 0)
            Wr(0, 0) = 0;
         if (Wr(1, 0) > 1650)
            Wr(1, 0) = 1650;
         if (Wr(1, 0) < 0)
            Wr(1, 0) = 0;
         if (Wr(2, 0) > 1650)
            Wr(2, 0) = 1650;
         if (Wr(2, 0) < 0)
            Wr(2, 0) = 0;
         if (Wr(3, 0) > 1650)
            Wr(3, 0) = 1650;
         if (Wr(3, 0) < 0)
            Wr(3, 0) = 0;
      }
      else
      {
         E << 0, 0, 0, 0;
         Wr << 0, 0, 0, 0;
      }

      //==========================================================
      /*

      // Asignamos la rotación de los motores
      double w_rotor_NE;  
      double w_rotor_NW;
      double w_rotor_SE;
      double w_rotor_SW;
*/

      // Asignamos la rotación de los motores
      double w_rotor_NE = Wr(0, 0);
      double w_rotor_NW = Wr(1, 0);
      double w_rotor_SE = Wr(2, 0);
      double w_rotor_SW = Wr(3, 0);
      /*
      // con esto simulamos rotacion de sustentacion
      w_rotor_NE = sqrt(0.300*9.8/4 / kFT);  // = 430.18 con 4 rotores
      w_rotor_NW = w_rotor_NE;
      w_rotor_SE = w_rotor_NE;
      w_rotor_SW = w_rotor_NE;
*/

      // aplicamos fuerzas/momentos por empuje de rotores
      ignition::math::Vector3<double> FT_NE = ignition::math::Vector3<double>(0, 0, kFT * pow(w_rotor_NE, 2));
      link->AddLinkForce(FT_NE, pos_NE);
      ignition::math::Vector3<double> FT_NW = ignition::math::Vector3<double>(0, 0, kFT * pow(w_rotor_NW, 2));
      link->AddLinkForce(FT_NW, pos_NW);
      ignition::math::Vector3<double> FT_SE = ignition::math::Vector3<double>(0, 0, kFT * pow(w_rotor_SE, 2));
      link->AddLinkForce(FT_SE, pos_SE);
      ignition::math::Vector3<double> FT_SW = ignition::math::Vector3<double>(0, 0, kFT * pow(w_rotor_SW, 2));
      link->AddLinkForce(FT_SW, pos_SW);

      // aplicamos momentos por arrastre de rotores
      ignition::math::Vector3<double> MDR_NE = ignition::math::Vector3<double>(0, 0, kMDR * pow(w_rotor_NE, 2));
      ignition::math::Vector3<double> MDR_NW = ignition::math::Vector3<double>(0, 0, kMDR * pow(w_rotor_NW, 2));
      ignition::math::Vector3<double> MDR_SE = ignition::math::Vector3<double>(0, 0, kMDR * pow(w_rotor_SE, 2));
      ignition::math::Vector3<double> MDR_SW = ignition::math::Vector3<double>(0, 0, kMDR * pow(w_rotor_SW, 2));
      //    printf("MDR  = %.15f\n",MDR_NE.Z() - MDR_NW.Z() - MDR_SE.Z() + MDR_SW.Z());
      link->AddRelativeTorque(MDR_NE - MDR_NW - MDR_SE + MDR_SW);

      // aplicamos fuerza de rozamiento con el aire
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

      // Is it time to publish odometry to topic?
      if (current_time < last_odom_publish_time)
         last_odom_publish_time = current_time; // The simulation was reset
                                                //    printf("current time:  %.3f \n", current_time.Double());
                                                //    printf("last time:     %.3f \n", last_odom_publish_time.Double());
      double seconds_since_last_update = (current_time - last_odom_publish_time).Double();
      if (seconds_since_last_update > (1.0 / odom_publish_rate))
      {
          //Publish odometry to topic
         //      printf("Publishing odometry--------------\n");

         gazebo_msgs::ModelState msg;
         msg.model_name = "quadcopter";

         msg.pose.position.x = pose.Pos().X(); // eX
         msg.pose.position.y = pose.Pos().Y(); // eY
         msg.pose.position.z = pose.Pos().Z(); // eZ

         msg.pose.orientation.w = 0;
         msg.pose.orientation.x = pose_rot.X(); // ePhi
         msg.pose.orientation.y = pose_rot.Y(); // eTheta
         msg.pose.orientation.z = pose_rot.Z(); // ePsi

         msg.twist.linear.x = linear_vel.X(); // bXdot
         msg.twist.linear.y = linear_vel.Y(); // bYdot
         msg.twist.linear.z = linear_vel.Z(); // bZdot

         msg.twist.angular.x = angular_vel.X(); // bWx
         msg.twist.angular.y = angular_vel.Y(); // bWy
         msg.twist.angular.z = angular_vel.Z(); // bWz

         pub_.publish(msg);

         last_odom_publish_time = current_time;
      }
   }

   // Custom Callback Queue
   ////////////////////////////////////////////////////////////////////////////////
   // custom callback queue thread
   void QueueThread()
   {
      static const double timeout = 0.01;

      while (this->rosnode_->ok())
      {
         this->queue_.callAvailable(ros::WallDuration(timeout));
      }
   }
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(DroneControl)
} // namespace gazebo
