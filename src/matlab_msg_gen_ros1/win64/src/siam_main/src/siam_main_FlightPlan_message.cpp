// Copyright 2019-2021 The MathWorks, Inc.
// Common copy functions for siam_main/FlightPlan
#include "boost/date_time.hpp"
#include "boost/shared_array.hpp"
#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable : 4244)
#pragma warning(disable : 4265)
#pragma warning(disable : 4458)
#pragma warning(disable : 4100)
#pragma warning(disable : 4127)
#pragma warning(disable : 4267)
#pragma warning(disable : 4068)
#pragma warning(disable : 4245)
#else
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wunused-local-typedefs"
#pragma GCC diagnostic ignored "-Wredundant-decls"
#pragma GCC diagnostic ignored "-Wnon-virtual-dtor"
#pragma GCC diagnostic ignored "-Wdelete-non-virtual-dtor"
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wunused-variable"
#pragma GCC diagnostic ignored "-Wshadow"
#endif //_MSC_VER
#include "ros/ros.h"
#include "siam_main/FlightPlan.h"
#include "visibility_control.h"
#include "MATLABROSMsgInterface.hpp"
#include "ROSPubSubTemplates.hpp"
class SIAM_MAIN_EXPORT siam_main_msg_FlightPlan_common : public MATLABROSMsgInterface<siam_main::FlightPlan> {
  public:
    virtual ~siam_main_msg_FlightPlan_common(){}
    virtual void copy_from_struct(siam_main::FlightPlan* msg, const matlab::data::Struct& arr, MultiLibLoader loader); 
    //----------------------------------------------------------------------------
    virtual MDArray_T get_arr(MDFactory_T& factory, const siam_main::FlightPlan* msg, MultiLibLoader loader, size_t size = 1);
};
  void siam_main_msg_FlightPlan_common::copy_from_struct(siam_main::FlightPlan* msg, const matlab::data::Struct& arr,
               MultiLibLoader loader) {
    try {
        //flightPlanId
        const matlab::data::TypedArray<uint16_t> flightPlanId_arr = arr["FlightPlanId"];
        msg->flightPlanId = flightPlanId_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'FlightPlanId' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'FlightPlanId' is wrong type; expected a uint16.");
    }
    try {
        //status
        const matlab::data::TypedArray<uint8_t> status_arr = arr["Status"];
        msg->status = status_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'Status' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'Status' is wrong type; expected a uint8.");
    }
    try {
        //priority
        const matlab::data::TypedArray<int8_t> priority_arr = arr["Priority"];
        msg->priority = priority_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'Priority' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'Priority' is wrong type; expected a int8.");
    }
    try {
        //operatorId
        const matlab::data::TypedArray<uint16_t> operatorId_arr = arr["OperatorId"];
        msg->operatorId = operatorId_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'OperatorId' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'OperatorId' is wrong type; expected a uint16.");
    }
    try {
        //droneId
        const matlab::data::TypedArray<uint16_t> droneId_arr = arr["DroneId"];
        msg->droneId = droneId_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'DroneId' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'DroneId' is wrong type; expected a uint16.");
    }
    try {
        //orig
        const matlab::data::StructArray orig_arr = arr["Orig"];
        auto msgClassPtr_orig = getCommonObject<geometry_msgs::Point>("geometry_msgs_msg_Point_common",loader);
        msgClassPtr_orig->copy_from_struct(&msg->orig,orig_arr[0],loader);
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'Orig' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'Orig' is wrong type; expected a struct.");
    }
    try {
        //dest
        const matlab::data::StructArray dest_arr = arr["Dest"];
        auto msgClassPtr_dest = getCommonObject<geometry_msgs::Point>("geometry_msgs_msg_Point_common",loader);
        msgClassPtr_dest->copy_from_struct(&msg->dest,dest_arr[0],loader);
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'Dest' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'Dest' is wrong type; expected a struct.");
    }
    try {
        //dtto
        const matlab::data::TypedArray<uint32_t> dtto_arr = arr["Dtto"];
        msg->dtto = dtto_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'Dtto' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'Dtto' is wrong type; expected a uint32.");
    }
    try {
        //route
        const matlab::data::StructArray route_arr = arr["Route"];
        for (auto _routearr : route_arr) {
        	geometry_msgs::Point _val;
        auto msgClassPtr_route = getCommonObject<geometry_msgs::Point>("geometry_msgs_msg_Point_common",loader);
        msgClassPtr_route->copy_from_struct(&_val,_routearr,loader);
        	msg->route.push_back(_val);
        }
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'Route' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'Route' is wrong type; expected a struct.");
    }
  }
  //----------------------------------------------------------------------------
  MDArray_T siam_main_msg_FlightPlan_common::get_arr(MDFactory_T& factory, const siam_main::FlightPlan* msg,
       MultiLibLoader loader, size_t size) {
    auto outArray = factory.createStructArray({size,1},{"MessageType","FlightPlanId","Status","Priority","OperatorId","DroneId","Orig","Dest","Dtto","Route"});
    for(size_t ctr = 0; ctr < size; ctr++){
    outArray[ctr]["MessageType"] = factory.createCharArray("siam_main/FlightPlan");
    // flightPlanId
    auto currentElement_flightPlanId = (msg + ctr)->flightPlanId;
    outArray[ctr]["FlightPlanId"] = factory.createScalar(currentElement_flightPlanId);
    // status
    auto currentElement_status = (msg + ctr)->status;
    outArray[ctr]["Status"] = factory.createScalar(currentElement_status);
    // priority
    auto currentElement_priority = (msg + ctr)->priority;
    outArray[ctr]["Priority"] = factory.createScalar(currentElement_priority);
    // operatorId
    auto currentElement_operatorId = (msg + ctr)->operatorId;
    outArray[ctr]["OperatorId"] = factory.createScalar(currentElement_operatorId);
    // droneId
    auto currentElement_droneId = (msg + ctr)->droneId;
    outArray[ctr]["DroneId"] = factory.createScalar(currentElement_droneId);
    // orig
    auto currentElement_orig = (msg + ctr)->orig;
    auto msgClassPtr_orig = getCommonObject<geometry_msgs::Point>("geometry_msgs_msg_Point_common",loader);
    outArray[ctr]["Orig"] = msgClassPtr_orig->get_arr(factory, &currentElement_orig, loader);
    // dest
    auto currentElement_dest = (msg + ctr)->dest;
    auto msgClassPtr_dest = getCommonObject<geometry_msgs::Point>("geometry_msgs_msg_Point_common",loader);
    outArray[ctr]["Dest"] = msgClassPtr_dest->get_arr(factory, &currentElement_dest, loader);
    // dtto
    auto currentElement_dtto = (msg + ctr)->dtto;
    outArray[ctr]["Dtto"] = factory.createScalar(currentElement_dtto);
    // route
    auto currentElement_route = (msg + ctr)->route;
    auto msgClassPtr_route = getCommonObject<geometry_msgs::Point>("geometry_msgs_msg_Point_common",loader);
    outArray[ctr]["Route"] = msgClassPtr_route->get_arr(factory,&currentElement_route[0],loader,currentElement_route.size());
    }
    return std::move(outArray);
  } 
class SIAM_MAIN_EXPORT siam_main_FlightPlan_message : public ROSMsgElementInterfaceFactory {
  public:
    virtual ~siam_main_FlightPlan_message(){}
    virtual std::shared_ptr<MATLABPublisherInterface> generatePublisherInterface(ElementType type);
    virtual std::shared_ptr<MATLABSubscriberInterface> generateSubscriberInterface(ElementType type);
    virtual std::shared_ptr<MATLABRosbagWriterInterface> generateRosbagWriterInterface(ElementType type);
};  
  std::shared_ptr<MATLABPublisherInterface> 
          siam_main_FlightPlan_message::generatePublisherInterface(ElementType type){
    if(type != eMessage){
        throw std::invalid_argument("Wrong input, Expected eMessage");
    }
    return std::make_shared<ROSPublisherImpl<siam_main::FlightPlan,siam_main_msg_FlightPlan_common>>();
  }
  std::shared_ptr<MATLABSubscriberInterface> 
         siam_main_FlightPlan_message::generateSubscriberInterface(ElementType type){
    if(type != eMessage){
        throw std::invalid_argument("Wrong input, Expected eMessage");
    }
    return std::make_shared<ROSSubscriberImpl<siam_main::FlightPlan,siam_main::FlightPlan::ConstPtr,siam_main_msg_FlightPlan_common>>();
  }
#include "ROSbagTemplates.hpp" 
  std::shared_ptr<MATLABRosbagWriterInterface>
         siam_main_FlightPlan_message::generateRosbagWriterInterface(ElementType type){
    if(type != eMessage){
        throw std::invalid_argument("Wrong input, Expected eMessage");
    }
    return std::make_shared<ROSBagWriterImpl<siam_main::FlightPlan,siam_main_msg_FlightPlan_common>>();
  }
#include "register_macro.hpp"
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
CLASS_LOADER_REGISTER_CLASS(siam_main_msg_FlightPlan_common, MATLABROSMsgInterface<siam_main::FlightPlan>)
CLASS_LOADER_REGISTER_CLASS(siam_main_FlightPlan_message, ROSMsgElementInterfaceFactory)
#ifdef _MSC_VER
#pragma warning(pop)
#else
#pragma GCC diagnostic pop
#endif //_MSC_VER
//gen-1