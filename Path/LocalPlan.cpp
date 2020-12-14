#pragma once
#include "ros/ros.h"
#include "PathMaker.cpp"
#include "Control_pkg/path.h"
#include <geometry_msgs/Point.h>
class PathMaker;
class LocalPlan{
    static PathMaker* pm;
public:
    bool localMode=false;
    int mode=0;
    /*
    0 HDMAP
    1 LOCAL
    2 LINECATCH
    */

    // 제어에 주는 값 - 다음 위치의 상대좌표. 단위 = 1M
    float coordX=0;
    float coordY=0;
    float globalCoordX = 0;
    float globalCoordY = 0;
    LocalPlan(){
        ROS_INFO("[Constructor] LocalPlan_ON");
    }
    void getLocalPath(){
        //토픽으로 뭔가 보내~~(리턴값으로 보낼예정)
    }
    void square(){
        /*
            교차로에서 어떻게 할건지~~
        */
    }
    //레퍼런스형으로 변경 요함
    void getObstacleAvoidPath(std::vector<geometry_msgs::Point>  obsMin, std::vector<geometry_msgs::Point>  obsMax, std::vector<geometry_msgs::Point>  obsCen){
  
    }
    void parkingPath(){

    }
};

/*
    void sick_callback(serial_example::msgsick_vector input){

        std::vector<geometry_msgs::Point>  obsMin;
        std::vector<geometry_msgs::Point>  obsMax;
        std::vector<geometry_msgs::Point>  obsCen;

        obsMin.clear();
        obsMax.clear();
        obsCen.clear();

        minY=maxY=coordX=coordY=0.0;

        for(int i = 0; i < input.msg_ctr.size();i++){
            obsMin.push_back(input.msg_min[i]);
            obsMax.push_back(input.msg_max[i]);
            obsCen.push_back(input.msg_ctr[i]);

        }
        
        //정적장애물 콜백함수

        std_msgs::Float32 sick_msg;
        
        //std::cerr<<"[static_before]"<<std::endl;
        for(int i=0;i<obsMin.size();i++){
            float obstacle_width=abs(obsMax.at(i).y-obsMin.at(i).y);
           // std::cerr<<"[static_for]"<<std::endl;
            if((line_width-obstacle_width)>1.2) // 차선변경을 하지 않아도 되는 경우, 1.2는 ERP폭
            {   
                //std::cerr<<"[static_if1]"<<std::endl;
                if(!straight){
                    if(obsCen.at(i).y>=0)
                    { 
                        right_Y = obsCen.at(i).y - 2.0;
                        steer_t = -right_Y / 1.5 * 2000 * 0.8;
                        std::cerr<<" [static]right_Y = "<<right_Y<<std::endl;
                        //speed_t = 40;
                        std::cerr<<" [static] prev_steer_t = "<< prev_steer_t <<std::endl;
                        std::cerr<<" [static] prev_steer_flag_t = "<< prev_steer_flag <<std::endl;
                        sick_msg.data = right_Y;
                        steer_flag = 1;
                    }
                    else if(obsCen.at(i).y < 0)
                    {
                    //left_Y = obsMax.at(i).y + obstacle_width/2.0;
                        left_Y = obsCen.at(i).y + 2.0;
                        steer_t = -left_Y / 1.5 * 2000 * 0.8;
                        //speed_t = 40;
                        std::cerr<<" [static]left_Y = "<<left_Y<<std::endl;
                        sick_msg.data = left_Y;
                        steer_flag = 2;                    
                    }
                        prev_steer_flag = steer_flag;
                        prev_steer_t = steer_t;
                }
                else {
                    //steer_t = 800;
                    steer_flag = 0;
                    std::cerr<<" [static] prev_steer_t = "<< prev_steer_t <<std::endl;
                    std::cerr<<" [static] prev_steer_flag_t = "<< prev_steer_flag <<std::endl;                    

                }
                
                if(steer_t > 2000)
                    {
                        steer_t = 2000;
                    }  
               if(steer_t < -2000)
                    {
                        steer_t = -2000;
                    }

                //std::cerr<<" [static] prev_steer_t = "<< prev_steer_t <<std::endl;


            }
            // else // 차선변경을 해야하는 경우
            narrow_y_pub.publish(sick_msg);
            left_Y=0;
            right_Y=0;
        }

        obsCen.clear();

        coordX = 1;
        coordY = (minY+maxY)/2;
        std::cerr<<" [PATH Y]minY = "<<minY<<std::endl;
        std::cerr<<" [PATH Y]maxY = "<<maxY<<std::endl;
        std::cerr<<" [PATH Y]coordY = "<<coordY<<std::endl;

    } 

void flag_callback(const std_msgs::Bool::ConstPtr &obstacleDetected){
    if(obstacleDetected->data){
        straight = false;
        //std::cerr << "error straight if" << std::endl;
    }
    else{
        straight = true;
        //std::cerr << "error straight else" << std::endl;

    }
}
*/