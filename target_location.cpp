#include "GMM.cpp"



class SUB_PUB{
public:
    ros::NodeHandle nodeHandler;
    int varycameraid,varypointid,varylineid;
    int boolState;//1,record;2,compute;3,flight;
    std::string recordlocation=("/home/kobosp/SLAM_YOLO/target_record.txt");
    ros::Publisher marker_pub_camera,marker_pub_point,marker_pub_line;
    SUB_PUB(){
        ROS_INFO("SUB_PUB init");
        varycameraid=varypointid=varylineid=0;
        nodeHandler.param<int>("boolState", boolState,1);
    }
    void displayCameraOnRVIZ(Eigen::Vector3d point,Eigen::Quaterniond pose,Eigen::Vector3d color,double scale);
    void displayPointOnRVIZ(std::vector<Eigen::Vector3d> points,Eigen::Vector3d color,double scale);
    void displayLineOnRVIZ(std::vector<Eigen::Vector3d> A,std::vector<Eigen::Vector3d> B,Eigen::Vector3d color,double scale);

};



void SUB_PUB::displayPointOnRVIZ(std::vector<Eigen::Vector3d> points,Eigen::Vector3d color,double scale){
    marker_pub_point=nodeHandler.advertise<visualization_msgs::Marker>("visualization_point", 10);
    visualization_msgs::Marker point;
    {
        point.header.frame_id = "/basic_frame";
        point.header.stamp = ros::Time::now();
        point.ns =  "visualization_point";
        point.id = this->varypointid++;
        point.type = visualization_msgs::Marker::POINTS;
        point.action = visualization_msgs::Marker::ADD;
        point.scale.x = scale;
        point.scale.y = scale;
        point.scale.z = scale;
        point.color.r = color.x();
        point.color.g = color.y();
        point.color.b = color.z();
        point.color.a = 1.0;
        point.lifetime = ros::Duration();

        geometry_msgs::Point tmpp;
        for(int i=0;i<points.size();i++){
            tmpp.x = points[i].x();
            tmpp.y = points[i].y();
            tmpp.z = points[i].z();
            point.points.push_back(tmpp);
        }
        ROS_INFO("point.id:%d",point.id);
    }
    marker_pub_point.publish(point);
}

void SUB_PUB::displayLineOnRVIZ(std::vector<Eigen::Vector3d> A,std::vector<Eigen::Vector3d> B,Eigen::Vector3d color,double scale){
    marker_pub_line=nodeHandler.advertise<visualization_msgs::Marker>("visualization_line", 10);
    visualization_msgs::Marker line;
    {
        line.header.frame_id  = "/basic_frame";
        line.header.stamp  =ros::Time::now();
        line.ns  = "visualization_line";
        line.action  = visualization_msgs::Marker::ADD;
        line.id = this->varylineid++;
        line.type = visualization_msgs::Marker::LINE_LIST;
        line.scale.x = scale;
        line.color.r = color.x();
        line.color.g = color.y();
        line.color.b = color.z();
        line.color.a = 1.0;
        line.lifetime = ros::Duration();
        ROS_INFO("varylineid:%d",line.id);
    }
    geometry_msgs::Point tmpp;
    for(int i=0;i<A.size();i++){
        tmpp.x=A[i].x();
        tmpp.y=A[i].y();
        tmpp.z=A[i].z();
        line.points.push_back(tmpp);

        tmpp.x=B[i].x();
        tmpp.y=B[i].y();
        tmpp.z=B[i].z();
        line.points.push_back(tmpp);
    }
    marker_pub_line.publish(line);
}


void SUB_PUB::displayCameraOnRVIZ(Eigen::Vector3d point,Eigen::Quaterniond pose,Eigen::Vector3d color,double scale){
    marker_pub_camera=nodeHandler.advertise<visualization_msgs::Marker>("visualization_camera", 10);
    visualization_msgs::Marker camera;
    {
        camera.header.frame_id = "/basic_frame";
        camera.header.stamp = ros::Time::now();
        camera.ns = "visualization_camera";
        camera.id = this->varycameraid++;
        camera.type = visualization_msgs::Marker::CUBE;
        camera.action = visualization_msgs::Marker::ADD;
        camera.pose.position.x = point.x();
        camera.pose.position.y = point.y();
        camera.pose.position.z = point.z();
        camera.pose.orientation.x = pose.x();
        camera.pose.orientation.y = pose.y();
        camera.pose.orientation.z = pose.z();
        camera.pose.orientation.w = pose.w();
        camera.scale.x = 0.05*scale;//red,right
        camera.scale.y = 0.03*scale;//green,down
        camera.scale.z = 0.001*scale;//blue,front
        camera.color.r = color.x();
        camera.color.g = color.y();
        camera.color.b = color.z();
        camera.color.a = 1.0;
        camera.lifetime = ros::Duration();
        ROS_INFO("camera.id:%d",camera.id);
    }
    marker_pub_camera.publish(camera);
}

class RayLine {
public:
    Eigen::Vector3d point;
    Eigen::Vector3d pose;

    RayLine(double pointx, double pointy, double pointz, double posex, double posey, double posez) {
        this->point = Eigen::Vector3d(pointx, pointy, pointz);
        this->pose = Eigen::Vector3d(posex, posey, posez);
    }
    double getRayLineDistance(RayLine RL1,RayLine RL2);
    void getTargetLocation(std::vector<RayLine> allrayline,std::vector<Eigen::Vector3d>& centerpoints,std::vector<Eigen::Vector3d>& crosspoints);
};

double RayLine::getRayLineDistance(RayLine RL1,RayLine RL2){
    Eigen::Vector3d N = RL1.pose.cross(RL2.pose);//normal
    Eigen::Vector3d AB = (RL2.point - RL1.point);//start line
    double distance;
    if(N.norm()<FLOATEPS){//parallel
        distance=sqrt(pow(AB.x()*RL1.pose.y()-AB.y()*RL1.pose.x(),2)+
                      pow(AB.x()*RL1.pose.z()-AB.z()*RL1.pose.x(),2)+
                      pow(AB.y()*RL1.pose.z()-AB.z()*RL1.pose.y(),2))/RL1.pose.norm();
    }
    else {
        distance = N.dot(AB) / N.norm();
        Eigen::Vector3d cross1, cross2;
        double step1, step2;
        step1 = (AB.cross(RL2.pose)).dot(RL1.pose.cross(RL2.pose));
        step2 = (AB.cross(RL1.pose)).dot(RL1.pose.cross(RL2.pose));
        double dd = RL1.pose.cross(RL2.pose).norm();
        step1 /= dd * dd;
        step2 /= dd * dd;
        if(step1<FLOATEPS||step2<FLOATEPS||step1>0.5+FLOATEPS||step2>0.5+FLOATEPS){//reverse
            return -1.0;
        }
        cross1 = (RL1.point + RL1.pose * step1);
        cross2 = (RL2.point + RL2.pose * step2);
        this->point=(cross1+cross2)/2;
        this->pose=cross1-cross2;
    }
    return fabs(distance);
}


//rosparam set boolState 2
void RayLine::getTargetLocation(std::vector<RayLine> allrayline,std::vector<Eigen::Vector3d>& targetpoints,std::vector<Eigen::Vector3d>& crosspoints){
    GMM gmm(13,100,0.01);
    RayLine tmpRayLine(0,0,0,0,0,0);
    for(int i=0;i<allrayline.size();i++){
        for(int j=i+1;j<allrayline.size();j++){
            if(tmpRayLine.getRayLineDistance(allrayline[i],allrayline[j])<0){
                continue;
            }
            crosspoints.push_back(tmpRayLine.point);
        }
    }
    gmm.ClusterGMM(crosspoints);
    for(int i=0;i<gmm.m_clusterNum;i++){
        printf("gmm.m_means[%d]:%lf,%lf,%lf\n",i,gmm.m_means[i][0],gmm.m_means[i][1],gmm.m_means[i][2]);
    }
    targetpoints.swap(gmm.m_means);
    {
//    std::vector<TargetLine> alltarget;
//    double distancethreshold=0.1;
//    while(allRayLine.size()!=0){
//        std::vector<double> alldistance;
//        for(int i=1;i<allRayLine.size();i++){
//            double tmpdis=tmpRayLine.distanceRayLine(allRayLine[0],allRayLine[i]);
////            printf("distanceRayLine(allRayLine[0],allRayLine[%d]):%lf\n",i,tmpdis);
//            alldistance.push_back(tmpdis);
//        }
//        TargetLine tmptarget;
//        for(int i=alldistance.size()-1;i>=0;i--){
//            if(alldistance[i]<distancethreshold&&alldistance[i]>=-FLOATEPS){
////                printf("%d belong to 0\n",i+1);
//                tmptarget.ownerline.push_back(allRayLine[i+1]);
//                allRayLine.erase(allRayLine.begin()+i+1);
//            }
//        }
//        tmptarget.ownerline.push_back(allRayLine[0]);
//        allRayLine.erase(allRayLine.begin());
//        alltarget.push_back(tmptarget);
//        printf("loop %zu end-----size:%zu\n",alltarget.size()-1,tmptarget.ownerline.size());
//    }
//    ROS_INFO("alltarget.size():%zu",alltarget.size());
//    for(int i=0;i<alltarget.size();i++){
//        if(alltarget[i].ownerline.size()<10){
//            continue;
//        }
//        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> leftMat (3*alltarget[i].ownerline.size(),3+alltarget[i].ownerline.size());
//        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> rightMat (3*alltarget[i].ownerline.size(),1);
//        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> answerMat (3+alltarget[i].ownerline.size(),1);
//
//        for(int j=0;j<alltarget[i].ownerline.size();j++){//fill matrix
//            leftMat(j*3,0)=1.0,leftMat(j*3,1)=0.0,leftMat(j*3,2)=0.0;
//            leftMat(j*3+1,0)=0.0,leftMat(j*3+1,1)=1.0,leftMat(j*3+1,2)=0.0;
//            leftMat(j*3+2,0)=0.0,leftMat(j*3+2,1)=0.0,leftMat(j*3+2,2)=1.0;
//
//            for(int k=0;k<alltarget[i].ownerline.size();k++){
//                leftMat(j*3,3+k)=0;
//                leftMat(j*3+1,3+k)=0;
//                leftMat(j*3+2,3+k)=0;
//            }
//            leftMat(j*3,3+j)=-alltarget[i].ownerline[j].pose[0];
//            leftMat(j*3+1,3+j)=-alltarget[i].ownerline[j].pose[1];
//            leftMat(j*3+2,3+j)=-alltarget[i].ownerline[j].pose[2];
//
//            rightMat(j*3,0)=alltarget[i].ownerline[j].point[0];
//            rightMat(j*3+1,0)=alltarget[i].ownerline[j].point[1];
//            rightMat(j*3+2,0)=alltarget[i].ownerline[j].point[2];
//        }
//        answerMat=leftMat.colPivHouseholderQr().solve(rightMat);
////        std::cout<<"leftMat"<<std::endl<<leftMat<<std::endl;
////        std::cout<<"rightMat"<<std::endl<<rightMat<<std::endl;
//        std::cout<<"answerMat"<<std::endl<<answerMat<<std::endl;
//        alltarget[i].location[0]=answerMat(0,0);
//        alltarget[i].location[1]=answerMat(1,0);
//        alltarget[i].location[2]=answerMat(2,0);
//        ROS_INFO("target[%d]:%lf,%lf,%lf",i,alltarget[i].location[0],alltarget[i].location[1],alltarget[i].location[2]);
//        displayTargetOnRVIZ(alltarget[i].location);
//
//        std::vector<Eigen::Vector3d> startcameralist,endcameralist,starttargetlist,endtargetlist;
//        for(int j=0;j<alltarget[i].ownerline.size();j++) {
//            Eigen::Vector3d tmppoint(alltarget[i].ownerline[j].point+alltarget[i].ownerline[j].pose*answerMat(3+j,0));
//            endcameralist.push_back(tmppoint);
//            starttargetlist.push_back(tmppoint);
//            startcameralist.push_back(alltarget[i].ownerline[j].point);
//            endtargetlist.push_back(alltarget[i].location);
//        }
//        displayLinePTPOnRVIZ(startcameralist,endcameralist,Eigen::Vector3d(1.0,1.0,0.0),0.0009);
//        displayLinePTPOnRVIZ(starttargetlist,endtargetlist,Eigen::Vector3d(0.5,1.0,0.0),0.0009);
//    }
    }
}


class Targets{
public:
    std::vector<RayLine> allrayline;
    std::vector<Eigen::Vector3d> targetpoints;
    std::vector<Eigen::Vector3d> crosspoints;
};
Targets targets;

void callback(SUB_PUB &sub_pub, const geometry_msgs::PoseStamped::ConstPtr& pose_sub, const darknet_ros_msgs::BoundingBoxes::ConstPtr& boxes_sub){
    RayLine camerarayline(pose_sub->pose.position.x, pose_sub->pose.position.y, pose_sub->pose.position.z,0,0,3);
    //64.1 42.7 67.1 37.74 photo 3:2 video 16:9
    double cameraWidth = 63.32;
    double cameraHeight = 36.86;
    int frameWidth = boxes_sub->frameWidth;
    int frameHeight = boxes_sub->frameHeight;
    double focusofWidth = frameWidth / tan(cameraWidth * 0.5);
    double focusofHeight = frameHeight / tan(cameraHeight * 0.5);
    Eigen::Quaterniond cameraRotate(pose_sub->pose.orientation.w, pose_sub->pose.orientation.x,pose_sub->pose.orientation.y, pose_sub->pose.orientation.z);
    sub_pub.displayCameraOnRVIZ(camerarayline.point,cameraRotate,Eigen::Vector3d(0,0,1),1);
    for(int i=0;i<boxes_sub->bounding_boxes.size();i++){
        if(boxes_sub->bounding_boxes[i].probability>0.9&&boxes_sub->bounding_boxes[i].Class=="target") {//boxes_sub->bounding_boxes[i].Class.c_str()
            int targetWidth = boxes_sub->bounding_boxes[i].xmax - boxes_sub->bounding_boxes[i].xmin;
            int offsetWidth = (boxes_sub->bounding_boxes[i].xmax + boxes_sub->bounding_boxes[i].xmin - frameWidth);
            //int offsetHeight=(boxes_sub->bounding_boxes[i].ymax + boxes_sub->bounding_boxes[i].ymin-frameHeight);
            int offsetHeight = (boxes_sub->bounding_boxes[i].ymax - targetWidth * 0.5 - frameHeight * 0.5);
            Eigen::Vector3d photoTarget(offsetWidth / focusofWidth, offsetHeight / focusofHeight, 1);
            Eigen::Quaterniond photoRotate = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d(0,0,1), photoTarget.normalized());
            Eigen::Vector3d targetpose = cameraRotate * photoRotate * camerarayline.pose;
            targets.allrayline.push_back(RayLine(camerarayline.point.x(),camerarayline.point.y(),camerarayline.point.z(),
                                            targetpose.x(),targetpose.y(),targetpose.z()));
        }
    }
    {
//    std::vector<Eigen::Vector3d> starttargetlist,endtargetlist;
//    starttargetlist.push_back(camerarayline.point);
//    endtargetlist.push_back(camerarayline.point+targetpose);
//    displayLineOnRVIZ(starttargetlist,endtargetlist,Eigen::Vector3d(1.0,0.0,0.0),0.0004);
//
//    ROS_INFO("pose_sub");
//    ROS_INFO("position:%lf_%lf_%lf",pose_sub->pose.position.x, pose_sub->pose.position.y, pose_sub->pose.position.z);
//    ROS_INFO("boxes_sub");
//    ROS_INFO("frameWidth:%ld_frameHeight:%ld",boxes_sub->frameWidth,boxes_sub->frameHeight);
//    for(int i=0;i<boxes_sub->bounding_boxes.size();i++){
//        ROS_INFO("%ld_%ld_%ld_%ld_%lf_%s",
//                 boxes_sub->bounding_boxes[i].xmin,
//                 boxes_sub->bounding_boxes[i].ymin,
//                 boxes_sub->bounding_boxes[i].xmax,
//                 boxes_sub->bounding_boxes[i].ymax,
//                 boxes_sub->bounding_boxes[i].probability,
//                 boxes_sub->bounding_boxes[i].Class.c_str());
//    }
//    FILE *fp = fopen(this->recordlocation.c_str(),"a+");
//    fprintf(fp, "%lf,%lf,%lf,%lf,%lf,%lf\n", pose_sub->pose.position.x, pose_sub->pose.position.y,
//            pose_sub->pose.position.z, nowPose.x(), nowPose.y(), nowPose.z());
//    fclose(fp);
    }
}



int main(int argc, char **argv){
    ros::init(argc, argv, "target_location");
    SUB_PUB sub_pub;

    message_filters::Subscriber<geometry_msgs::PoseStamped> pose_sub(sub_pub.nodeHandler, "/ORBSLAM3_pose", 1, ros::TransportHints().tcpNoDelay());
    message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes> boxes_sub(sub_pub.nodeHandler, "/darknet_ros/bounding_boxes", 1, ros::TransportHints().tcpNoDelay());

    typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseStamped,darknet_ros_msgs::BoundingBoxes> syncPolicy;
    message_filters::Synchronizer<syncPolicy> sync(syncPolicy(1), pose_sub, boxes_sub);
    sync.registerCallback(boost::bind(&callback, sub_pub, _1, _2));

    ros::Rate loop_rate(10);
    bool havecompute=0;
    while(ros::ok()) {
        sub_pub.nodeHandler.getParam("boolState", sub_pub.boolState);
        if (sub_pub.boolState == 2 && havecompute==0) {
            std::string s="pkill -9 -f Mono_D435";
            system(s.c_str());
            s="pkill -9 -f nodelet";
            system(s.c_str());
            s="pkill -9 -f videototopic";
            system(s.c_str());
            printf("come to compute,allrayline.size():%zu\n",targets.allrayline.size());
            targets.allrayline[0].getTargetLocation(targets.allrayline,targets.targetpoints,targets.crosspoints);
            havecompute=!havecompute;
            loop_rate=0.1;
        }
        else if(sub_pub.boolState == 2 && havecompute==1){
            sub_pub.displayPointOnRVIZ(targets.crosspoints,Eigen::Vector3d(1.0,0.0,0.0),0.001);
            sub_pub.displayPointOnRVIZ(targets.targetpoints,Eigen::Vector3d(1.0,0.0,1.0),0.02);
        }
        else if(sub_pub.boolState == 3){
            break;
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

