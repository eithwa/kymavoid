/**
 * @file BaseNode.h
 *
 * @brief Ros communication central!
 *
 * @date February 2014
 **/
/*****************************************************************************
 ** Ifdefs
 *****************************************************************************/

#ifndef TeamStrategy_nodeHandle_HPP_
#define TeamStrategy_nodeHandle_HPP_




/*****************************************************************************
 ** Includes
 *****************************************************************************/
#include <ros/ros.h>
#include <string>
#include <vector>
#include "../common/Env.h"
#include "../common/BaseNode.h"

#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "fira_status_plugin/RobotSpeedMsg.h"
#include "fira_status_plugin/ModelMsg.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "gazebo_msgs/ModelStates.h"
/*****************************************************************************
 ** Define
 *****************************************************************************/
#define Ball_Topic_Name "/FIRA/Strategy/WorldMap/soccer"
#define GameState_Topic "/FIRA/GameState"

//RobotNumber
#define RobotNumber_Topic "/FIRA/RobotNumber"
#define TeamColor_Topic "/FIRA/TeamColor"
//robot prefix
#define Robot_Topic_Prefix "/FIRA/R"
#define RobotOpt_Topic_Prefix "/FIRA_Opt/R"

//robot suffix
#define Robot_Position_Topic_Suffix "/Strategy/WorldMap/RobotPos"
#define Robot_Role_Topic_Suffix "/Strategy/Coach/role"

#define Node_Name "TeamStrategy"

#define ModelState_Topic_Name  "/gazebo/model_states"
/*****************************************************************************
 ** Class
 *****************************************************************************/

class TeamStrategy_nodeHandle :public BaseNode {
public:
    TeamStrategy_nodeHandle(int argc, char** argv);
    
    virtual ~TeamStrategy_nodeHandle(){}
    void setEnv(Environment *inEnv){global_env = inEnv;}
    void setOpponent(bool inBool){opponent = inBool;}
    
    void pubRole(int *roleAry){
        std_msgs::Int32 robot_2_role;
        robot_2_role.data = roleAry[1]; //roleAry[2-1]
        robot_2_role_pub.publish(robot_2_role);
        
        
        std_msgs::Int32 robot_3_role;
        robot_3_role.data = roleAry[2];  //roleAry[3-1]
        robot_3_role_pub.publish(robot_3_role);
    }
    
    
    ros::NodeHandle* getNodeHandle(){return n;}
    
protected:
    void ros_comms_init();
    
private:
    
    Environment *global_env;
    bool opponent;
    bool run_one = false;
    
    void Transfer(int);
    
    std::vector<std::string> model_array;   // was a fixed [11] array: overflowed on worlds with >11 models
    ros::NodeHandle *n;
    
    ros::Subscriber GameState;
    long gamestate;

    //RobotNumber
    ros::Subscriber RobotNumber;
    ros::Subscriber TeamColor;
    long robotnumber;

    //gazebo_ModelStates subscriber
    ros::Subscriber Gazebo_Model_Name_sub;
    //ball subscriber
    ros::Subscriber ball_sub;
    
    //robot subscriber
    ros::Subscriber robot_1_pos_sub  ;
    ros::Subscriber robot_2_pos_sub  ;
    ros::Subscriber robot_3_pos_sub  ;
    ros::Subscriber robotOpt_1_pos_sub  ;
    ros::Subscriber robotOpt_2_pos_sub  ;
    ros::Subscriber robotOpt_3_pos_sub  ;
    
    //robot publisher
    //no robot_1_role_pub, because robot_1 is always goal keeper
    ros::Publisher robot_2_role_pub;
    ros::Publisher robot_3_role_pub;
    
    
    void find_gazebo_model_name_fun(const gazebo_msgs::ModelStates::ConstPtr &msg){//----------------------------------printf here is ok, but printf next row will crash if i open over one robot map
        //        printf("testttttttttttttt\n");
        
        if(run_one) return;
        
        int model_length;
        model_length = msg->name.size();
        /*
         for(int i =0;i<11;i++){
         if(msg->name[i].c_str()==NULL){
         model_length = i;
         }
         if(model_length!= 0) i = 11;
         //            printf("%d\n",model_length);
         }*/
        printf("model_length= %d",model_length);
        model_array = msg->name;
        run_one = true;
        //        printf("%s\n",model_array[6].c_str());
    }
    // Returns -1 when the model is not in the world (e.g. no "soccer"/"R1" in the avoid challenge);
    // the old version fell off the end without a return value, which is undefined behaviour.
    int get_model_num(const std::string ModelName){
        for(size_t i = 0; i < model_array.size(); i++){
            if(model_array[i] == ModelName) return (int)i;
        }
        return -1;
    }
    
    void subGameState(const std_msgs::Int32::ConstPtr &msg){
        global_env->gameState=msg->data;
        //std::cout<<"receive"<<std::endl;
    }

    void subTeamColor(const std_msgs::String::ConstPtr &msg){
        global_env->teamcolor= msg->data;
        //std::cout<<"receive"<<std::endl;
    }

    void  ball_sub_fun(const gazebo_msgs::ModelStates::ConstPtr &msg){
//                global_env->currentBall.pos.x = msg->x;
        //        global_env->currentBall.pos.y = msg->y;
        int model_num;
        model_num = get_model_num("soccer");
        if(model_num < 0 || model_num >= (int)msg->pose.size()) return;
        //        printf("model_num=%d\n",model_num);
        //        ROS_INFO("name=%s",msg->name[1].c_str());
        geometry_msgs::Pose tPose = msg->pose[model_num];
        //        ROS_INFO("x=%lf",tPose.position.x);
        global_env->currentBall.pos.x = tPose.position.x;
        global_env->currentBall.pos.y = tPose.position.y;
        
        
        //std::cout << "[ball] x=" << msg->x <<",y=" << msg->y << std::endl;
    }
    
    
    void  robot_1_pos_fun(const gazebo_msgs::ModelStates::ConstPtr &msg){
        //        global_env->home[0].pos.x = msg->pose.pose.position.x;
        //        global_env->home[0].pos.y = msg->pose.pose.position.y;
        
        
        //        double w = msg->pose.pose.orientation.w;
        //        double x = msg->pose.pose.orientation.x;
        //        double y = msg->pose.pose.orientation.y;
        //        double z = msg->pose.pose.orientation.z;
        //        double yaw = atan2(  2*(w*z+x*y),  1-2*(y*y+z*z)  )*rad2deg;
        //        global_env->home[0].rotation = yaw;
        int model_num;
        model_num = get_model_num("R1");
        if(model_num < 0 || model_num >= (int)msg->pose.size()) return;
        //        printf("model_num=%d\n",model_num);
        geometry_msgs::Pose tPose = msg->pose[model_num];
        global_env->home[0].pos.x = tPose.position.x;
        global_env->home[0].pos.y = tPose.position.y;
        Transfer(0);
        
        double w = tPose.orientation.w;
        double x = tPose.orientation.x;
        double y = tPose.orientation.y;
        double z = tPose.orientation.z;
        double yaw = atan2(  2*(w*z+x*y),  1-2*(y*y+z*z)  )*rad2deg;
        global_env->home[0].rotation = yaw;
        
        //  std::cout << "x=" << global_env->home[0].pos.x <<",y=" << global_env->home[0].pos.y << std::endl;
    }
    
    void  robot_2_pos_fun(const gazebo_msgs::ModelStates::ConstPtr &msg){
        //        global_env->home[1].pos.x = msg->pose.pose.position.x;
        //        global_env->home[1].pos.y = msg->pose.pose.position.y;
        
        //        double w = msg->pose.pose.orientation.w;
        //        double x = msg->pose.pose.orientation.x;
        //        double y = msg->pose.pose.orientation.y;
        //        double z = msg->pose.pose.orientation.z;
        //        double yaw = atan2(  2*(w*z+x*y),  1-2*(y*y+z*z)  )*rad2deg;
        //        global_env->home[1].rotation = yaw;
        int model_num;
        model_num = get_model_num("R2");
        if(model_num < 0 || model_num >= (int)msg->pose.size()) return;
        geometry_msgs::Pose tPose = msg->pose[model_num];
        global_env->home[1].pos.x = tPose.position.x;
        global_env->home[1].pos.y = tPose.position.y;
        Transfer(1);
        
        double w = tPose.orientation.w;
        double x = tPose.orientation.x;
        double y = tPose.orientation.y;
        double z = tPose.orientation.z;
        double yaw = atan2(  2*(w*z+x*y),  1-2*(y*y+z*z)  )*rad2deg;
        global_env->home[1].rotation = yaw;
        //std::cout << "x=" << global_env->home[1].pos.x <<",y=" << global_env->home[1].pos.y << std::endl;
        
        
    }
    void  robot_3_pos_fun(const gazebo_msgs::ModelStates::ConstPtr &msg){
        //        global_env->home[2].pos.x = msg->pose.pose.position.x;
        //        global_env->home[2].pos.y = msg->pose.pose.position.y;
        
        //        double w = msg->pose.pose.orientation.w;
        //        double x = msg->pose.pose.orientation.x;
        //        double y = msg->pose.pose.orientation.y;
        //        double z = msg->pose.pose.orientation.z;
        //        double yaw = atan2(  2*(w*z+x*y),  1-2*(y*y+z*z)  )*rad2deg;
        //        global_env->home[2].rotation = yaw;
        
        int model_num;
        model_num = get_model_num("R3");
        if(model_num < 0 || model_num >= (int)msg->pose.size()) return;
        geometry_msgs::Pose tPose = msg->pose[model_num];
        global_env->home[2].pos.x = tPose.position.x;
        global_env->home[2].pos.y = tPose.position.y;
        Transfer(2);
        
        double w = tPose.orientation.w;
        double x = tPose.orientation.x;
        double y = tPose.orientation.y;
        double z = tPose.orientation.z;
        double yaw = atan2(  2*(w*z+x*y),  1-2*(y*y+z*z)  )*rad2deg;
        global_env->home[2].rotation = yaw;
        // std::cout << "x=" << global_env->home[1].pos.x <<",y=" << global_env->home[2].pos.y << std::endl;
        
    }
    
    
    void  robotOpt_1_pos_fun(const nav_msgs::Odometry::ConstPtr &msg){
        global_env->opponent[0].pos.x = msg->pose.pose.position.x;
        global_env->opponent[0].pos.y = msg->pose.pose.position.y;
        
        double w = msg->pose.pose.orientation.w;
        double x = msg->pose.pose.orientation.x;
        double y = msg->pose.pose.orientation.y;
        double z = msg->pose.pose.orientation.z;
        double yaw = atan2(  2*(w*z+x*y),  1-2*(y*y+z*z)  )*rad2deg;
        global_env->opponent[0].rotation = yaw;
    }
    
    
    void  robotOpt_2_pos_fun(const nav_msgs::Odometry::ConstPtr &msg){
        global_env->opponent[1].pos.x = msg->pose.pose.position.x;
        global_env->opponent[1].pos.y = msg->pose.pose.position.y;
        double w = msg->pose.pose.orientation.w;
        double x = msg->pose.pose.orientation.x;
        double y = msg->pose.pose.orientation.y;
        double z = msg->pose.pose.orientation.z;
        double yaw = atan2(  2*(w*z+x*y),  1-2*(y*y+z*z)  )*rad2deg;
        global_env->opponent[1].rotation = yaw;
    }
    
    
    void  robotOpt_3_pos_fun(const nav_msgs::Odometry::ConstPtr &msg){
        global_env->opponent[2].pos.x = msg->pose.pose.position.x;
        global_env->opponent[2].pos.y = msg->pose.pose.position.y;
        double w = msg->pose.pose.orientation.w;
        double x = msg->pose.pose.orientation.x;
        double y = msg->pose.pose.orientation.y;
        double z = msg->pose.pose.orientation.z;
        double yaw = atan2(  2*(w*z+x*y),  1-2*(y*y+z*z)  )*rad2deg;
        global_env->opponent[2].rotation = yaw;
    }
    
};

#endif /* NODE_HPP_ */
