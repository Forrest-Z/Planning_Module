#pragma once
#include "DataWarehouse.cpp"
#include "LocalPlan.cpp"
#include "cstring"
using namespace std;
class StateMachine{
public:
    enum State{
        Idle=0,
        Drive,
        Avoid,
        Park,
        StaticObs,
        StaticObsRe,
        ESTOP,
        Dynamic,
        Static_big,
        Redu,
    };
    string getName(){
        string re="NULL";

        switch (mode)
        {
        case Idle:
            re = "Idle";
            break;
        case StaticObs:
            re = "StaticObs";
            break;
        case StaticObsRe:
            re = "StaticObsRe";
            break;
        case Dynamic:
            re = "Dynamic";
            break;
        case Drive:
            re = "Drive";
            break;
        case Static_big:
            re= "Static_big";
            break;
        case Redu:
        re = "REDU";
            break;
        default:
        re="LINE";
        break;
        }
        return re;
    }
    enum State mode;
    DataWarehouse* dw;
   
    StateMachine( DataWarehouse* dw ){

        ROS_INFO("[Constructor] StateMachine_ON__LocalPlan__DW");

        this->dw=dw;

        dw->ignoreLane=false;

        mode=State::Idle;
        //st = new Idle_State();
        cout<<"11"<<endl;

    }
    // 정적 장애물을 탐지했을 때의 대응 및 상태변경 
    void static_Opstacle(){
        mode=State::Avoid;
        ROS_INFO( "[DRIVE -> AVOID] avoid mode");
        ROS_INFO("[Ignore] Ignore Lane");
        dw->ignoreLane=true;
    }
    // 돌발 장애물을 탐지했을 때의 대응 및 상태변경 
    void dynamic_Opstacle(){
    }
    //장애물 회피 완료 했을 때
    void obastacle_Complete(){
        start();
        ROS_INFO("[COMPLETE] Avoiding Complete!");
       // lp->setPathModeHDmap();
        dw->ignoreLane=false;
    }

    void action(){
        ROS_INFO("In Action------------");
    }
    // void stop(){
    //     switch(mode){
    //         case State::Idle:
    //             ROS_INFO("[IDLE -> IDLE] already stop");
    //         break;
    //         case State::Drive:
    //             ROS_INFO("[DRIVE -> IDLE] stop the car");
    //             mode=State::Idle;
    //         break;
    //         case State::Avoid:
    //             ROS_INFO("[CRAWL -> IDLE] stop the car");
    //             mode=State::Idle;
    //         break;
    //     }
    // }
    void start(){
        switch(mode){
                case State::Idle:
                    ROS_INFO( "[IDLE -> DRIVING] start");
                    mode=State::Drive;
                break;
                case State::Drive:
                    ROS_INFO( "[DRIVING -> DRIVING] already driving");
                break;
                case State::Avoid:
                    ROS_INFO("[AVOID -> DRIVING] Avoid Complete");
                    mode=State::Drive;
                break;
                default:
                    ROS_INFO("Start !?!?");
                break;
            }
    }
    // void avoid(){
    //     switch(mode){
    //         case State::Idle:
    //             ROS_INFO( "[IDLE -> AVOID] localPath Planning");
    //             mode=State::Avoid;
    //         break;
    //         case State::Drive:
    //             ROS_INFO( "[DRIVING -> AVOID] avoid mode");
    //             mode=State::Avoid;
    //         break;
    //         // case State::Avoid:
    //         //     ROS_INFO( "[AVOID -> AVOID] already avoid");
    //         // break;
    //         default:
    //             ROS_INFO("Avoid !?!?");
    //         break;
    //     }
    // }
    // // 서행 필요할 때
    // void crawl(){
    //     dw->maxVelo=5;
    //      switch(mode){
    //         // case State::Idle:
    //         //     ROS_INFO("[IDLE -> IDLE] already stop");
    //         // break;
    //         case State::Drive:
    //             ROS_INFO("[DRIVE -> CRAWL] Crawling Mode On!");
    //         break;
    //         // case State::Avoid:
    //         //     ROS_INFO("[AVOID -> IDLE] stop the car");
    //         //     mode=State::Idle;
    //         // break;
    //         // case State::Change:
    //         //     ROS_INFO("[AVOID -> IDLE] stop the car");
    //         //     mode=State::Idle;
    //         // break;
    //         default:
    //             ROS_INFO("[CRAWL] default");
    //         break;
    //     }
    // }
    void printCurMode(){
        cout<<"[Mode]   ---  "<<getName()<<endl;
    }
};