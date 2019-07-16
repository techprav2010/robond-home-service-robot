
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <cmath>
#include <nav_msgs/Odometry.h>

float threshold=0.8;

std::string goal_name;
int iLoc=0;
bool completed = false;
const int total_num_goals = 12;
const int loop_num_goals = 12;

std::string goal_names[total_num_goals] = {
         "pick-up","drop-off"
        ,"second-pick-up","second-drop-off"
        ,"third-pick-up","third-drop-off"
        ,"fourth-pick-up","fourth-drop-off"
        ,"fifth-pick-up","fifth-drop-off"
        ,"sixth-pick-up","sixth-drop-off"
};
//loop the same  path twice... we can even loop endlessly
float goal_locations[total_num_goals][3] = {
        {3.4, 7.0, 0.6},
        {-4.5, 6.5, 0.8},

        {3.4, 7.0, 0.6},
        {-4.5, 6.5, 0.8},

        {3.4, 7.0, 0.6},
        {-4.5, 6.5, 0.8},

        {3.4, 7.0, 0.6},
        {-4.5, 6.5, 0.8},

        {3.4, 7.0, 0.6},
        {-4.5, 6.5, 0.8},

        {3.4, 7.0, 0.6},
        {-4.5, 6.5, 0.8}

};
bool goal_marked[total_num_goals]  = {
        false
        , false

        , false
        , false

        , false
        , false

        , false
        , false

        , false
        , false

        , false
        , false


};


ros::Publisher marker_pub;
visualization_msgs::Marker marker;
bool set_next_marker = false;
void setNextMarkerPosition() {
    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    if(!completed){
        marker.pose.position.x = goal_locations[iLoc][0];
        marker.pose.position.y =  goal_locations[iLoc][1];
        goal_name = goal_names[iLoc];
        ROS_INFO("setting goal %d %s (%f, %f)", iLoc , goal_name.c_str(), goal_locations[iLoc][0], goal_locations[iLoc][1]);
        iLoc = iLoc + 1; //next
    }
}

void publishMarker() {
    // Publish the marker
    while (marker_pub.getNumSubscribers() < 1) {
        ROS_WARN_ONCE("Please create a subscriber to the marker");
        sleep(1);
    }
    marker_pub.publish(marker);
}


void  addFirstMarker() {

    // initialize marker and show marker at pickup zone
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "add_markers";
    marker.id = 0;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = visualization_msgs::Marker::CUBE;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.4;
    marker.scale.y = 0.4;
    marker.scale.z = 0.4;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 1.0f;
    marker.color.g = 1.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0;

    // St marker's initial coordinates
    setNextMarkerPosition();

    marker.lifetime = ros::Duration();


}

float get_distance(float pos_x, float pos_y, int diff){
    int iLoc_cur = iLoc + diff;
    if(iLoc_cur <  loop_num_goals){
        float goal_x = goal_locations[iLoc_cur][0];
        float goal_y =  goal_locations[iLoc_cur][1];
        float dx = pos_x - goal_x;
        float dy = pos_y - goal_y;
        float distance = std::sqrt(dx*dx+dy*dy);
        return distance;
    }
    return 100; //bigger than threshold
}
void odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {

    if(set_next_marker  && iLoc < loop_num_goals && !goal_marked[iLoc ]){
        set_next_marker=false;
        goal_marked[iLoc]=true;
        // hide marker and set it to new coordinates
        setNextMarkerPosition();

        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;
        marker_pub.publish(marker);
//        marker.color.a = 0.0;
        ROS_INFO("location %s ... iLoc = %d", goal_name.c_str(), iLoc);
        //set we are at end of the loops
        completed = iLoc >= loop_num_goals;
        // publish marker
        publishMarker();
        ros::Duration(5).sleep();

    }
    else if (!completed) {

        float pos_x = msg->pose.pose.position.x;
        float pos_y = msg->pose.pose.position.y;

        float distance_prev = get_distance(pos_x, pos_y, -1);
        float distance_cur = get_distance(pos_x, pos_y, 0);
        float distance_next = get_distance(pos_x, pos_y, 1);

//        ROS_INFO("%s ... prev=%f cur=%f next=%f  %d  ",goal_name.c_str(),
//                distance_prev, distance_cur, distance_next, iLoc );

        if ( distance_prev <= threshold){
//            if (distance_prev  <= threshold  || distance_cur <= threshold || distance_next <= threshold){
            ROS_INFO("%s ... prev=%f cur=%f next=%f  %d  ",goal_name.c_str(),
                     distance_prev, distance_cur, distance_next, iLoc );

            // Delete marker
            marker.action = visualization_msgs::Marker::DELETE;
            marker_pub.publish(marker);

            // Wait 5 seconds before trying to show other markers
            ros::Duration(5).sleep();
            set_next_marker=true;

        }

    }else{
//        ROS_INFO("comleted already!");
    }
}


int main( int argc, char** argv )
{

    ROS_INFO("start add_markers ...");
    ros::init(argc, argv, "add_markers");
    ros::NodeHandle n;
    marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 2);

    addFirstMarker();
    // never auto-delete marker
    marker.lifetime = ros::Duration();

    while (marker_pub.getNumSubscribers() < 1) {
        ros::Duration(1.0).sleep();
        ROS_INFO("No visualization_marker %d ...", marker_pub.getNumSubscribers());
    }

    ROS_INFO("Adding marker ...");
    //marker_pub.publish(marker);
    marker.action = visualization_msgs::Marker::ADD;
    marker_pub.publish(marker);

    // subscribe to odometry topic
    ros::Subscriber odom_sub = n.subscribe("/odom", 1, odomCallback);


    ros::Rate r(10.0); // 10 Hz
    while (ros::ok()) {
        ros::spinOnce();
        r.sleep();
    }

    return 0;

}
