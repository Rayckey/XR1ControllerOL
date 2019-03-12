//
// Created by rocky on 19-3-9.
//

#include "xr1controllerol.h"




bool XR1ControllerOL::serviceIKPlanner(xr1controllerol::IKLinearServiceRequest & req ,
                                       xr1controllerol::IKLinearServiceResponse & res){

    temp_geo_trans = req.TargetTransform;

    tf::transformMsgToEigen(temp_geo_trans , itsafine);

    uint8_t control_group = req.ControlGroup;

    uint8_t base_group = XR1::Back_Y;

    if (req.BaseGroup == XR1::Knee_X)
        base_group = req.BaseGroup;



    // The default response
    res.inProgress = true;
    res.isReachable = false;
    res.isAccepted = false;

    if (XR1_ptr->isIKPlannerActive(control_group))
    {
        res.inProgress = true;
    }

    else {
        if (req.NewTarget){

            setControlMode(control_group , XR1::IKMode);
            res.inProgress = false;
            if (XR1_ptr->setEndEffectorPosition(control_group , itsafine , req.TargetElbowAngle , req.Period , base_group)){
                res.isReachable = true;
                res.isAccepted = true;

                XR1_ptr->setGrippingSwitch( control_group , req.Grip);
            }
        }
        res.inProgress = false;

    }

    return true;

}




bool XR1ControllerOL::serviceIKTracking(xr1controllerol::IKTrackingServiceRequest & req ,
                                       xr1controllerol::IKTrackingServiceResponse & res){


    // The default response
    res.inProgress = true;
    res.isReachable = false;
    res.isAccepted = false;

    uint8_t control_group = req.ControlGroup;

    if (control_group == XR1::HeadBody){

        try {
            EFF_Listener.lookupTransform( "/Back_Y", "/TrackingTarget",
                                      ros::Time(0), transform);
        }
        catch (tf::TransformException &ex) {
            return true;
        }

        transformTFToEigen(transform, itsafine);



        if (XR1_ptr->isIKPlannerActive(control_group))
        {
            res.inProgress = true;
        }

        else {
            if (req.NewTarget){

                setControlMode(control_group , XR1::IKMode);
                res.inProgress = false;
                if (XR1_ptr->setTrackingPosition(XR1::HeadBody, itsafine)){
                    res.isReachable = true;
                    res.isAccepted = true;
                }
            }
            res.inProgress = false;

        }


    }


    else if (control_group == XR1::BackBody){

        try {
            EFF_Listener.lookupTransform( "/Base", "/TrackingTarget",
                                      ros::Time(0), transform);
        }
        catch (tf::TransformException &ex) {
            return false;
        }

        transformTFToEigen(transform, itsafine);


        if (XR1_ptr->isIKPlannerActive(control_group))
        {
            res.inProgress = true;
        }

        else {
            if (req.NewTarget){

                setControlMode(control_group , XR1::IKMode);
                res.inProgress = false;
                if (XR1_ptr->setTrackingPosition(XR1::BackBody, itsafine)){
                    res.isReachable = true;
                    res.isAccepted = true;
                }
            }
            res.inProgress = false;

        }

    }



    return true;

}



//
//void XR1ControllerOL::lookupBackEFFTarget(tf::StampedTransform & transform,   Eigen::Affine3d & itsafine) {
//    try {
//        EFF_Listener.lookupTransform( "/Back_Y", "/TrackingTarget",
//                                      ros::Time(0), transform);
//    }
//    catch (tf::TransformException &ex) {
//        return;
//    }
//
//
//
////    if (ros::Time::now().toSec() - transform.stamp_.toSec() > 0.1 ) {
//////        ROS_INFO("Got overdue IK target, aborting movement");
////        return;
////    }
//
//    transformTFToEigen(transform, itsafine);
////    XR1_ptr->setEndEffectorPosition(XR1::LeftArm, itsafine , LeftElbowAngle);
//
//    XR1_ptr->setTrackingPosition(XR1::HeadBody, itsafine);
//
//
//
//
//    try {
//        EFF_Listener.lookupTransform( "/Base", "/TrackingTarget",
//                                      ros::Time(0), transform);
//    }
//    catch (tf::TransformException &ex) {
//        return;
//    }
//
//    transformTFToEigen(transform, itsafine);
//
//    XR1_ptr->setTrackingPosition(XR1::BackBody, itsafine);
//
//}


double XR1ControllerOL::getElbowAngle(uint8_t control_gourp) {
    return XR1_ptr->getElbowAngle(control_gourp);
}