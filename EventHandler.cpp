#pragma once
#include "Machine/StateMachine.cpp"   
#include "DataWarehouse.cpp"
#include "ros/ros.h"
#include "Path/LocalPlan.cpp"
#include "iostream"
using namespace std;
// Event Handler는 특정 이벤트를 탐지하고, 머신에 상황을 알릴 뿐 어떤 값도 제어하지 않는 것이 원칙
class EventHandler{

public:
    StateMachine* machine;
    DataWarehouse* dw;
    LocalPlan* lp;
    bool temp=false;
    std::vector<geometry_msgs::Point>  obsMin;
    EventHandler(LocalPlan* lo, DataWarehouse* dw){
        ROS_INFO("[Constructor] EventHandler_ON");
        
        machine = new StateMachine(dw);
        this->lp=lo;
        this->dw=dw;
    }
    int missionIdx=-1;
    int curMissionIdx=0;
    void listen_Data(){
        //------------------------------ F L A G ----------------------------
        //-------------------미션구역 진입
        missionIdx=isFlag();
        if( missionIdx != -1){
            cout<<"In  Mission"<<endl;
            switch(missionIdx){
                case 1:
                    cout<<"PARKING!!"<<endl;
                    // machine->mode=StateMachine::State::Park;
                    // dw->encoder=true;
                    dw->fp.finished[curMissionIdx]=true;
                    dw->canParking=true;
                    break;
                case 2:
                    cout<<"StaticObs!!"<<endl;
                    dw->maxVelo=dw->missionVelo;
                    if( machine->mode==StateMachine::State::Drive ){
                        machine->mode=StateMachine::State::StaticObsRe;     
                    }
                    break;
                case 3:
                    cout<<"DynamicObs!!"<<endl;
                    dw->maxVelo=dw->missionVelo;
                    machine->mode=StateMachine::State::Dynamic;
                    break;
                case 4:
                    cout<<"Return!!"<<endl;
                    machine->mode=StateMachine::State::Drive;
                    dw->encoder=false;
                    for(int i=0;i<curMissionIdx;i++)
                    dw->fp.finished[i]=true;
                    // dw->fp.finished[curMissionIdx]=true;
                    dw->maxVelo=dw->std_Velo;
                    dw->stopLine=false;
                    dw->brake=false;
                    temp=false;
                    break;
                case 5:
                    cout<<"ESTOP!!!!!!!"<<endl;
                    machine->mode=StateMachine::State::ESTOP;
                    break;
                case 6:
                    cout<<"STOPLINE!!!!!!!"<<endl;
                    dw->maxVelo=dw->missionVelo;
                    if(!temp) dw->stopLine=true;
                    temp=true;
                    // dw->fp.finished[curMissionIdx]=true;
                    break;
                case 7:
                   cout<<"Static Big Obs!!"<<endl;
                    dw->maxVelo=dw->missionVelo;
                    if( machine->mode==StateMachine::State::Drive ){
                        machine->mode=StateMachine::State::StaticObsRe;     
                    }
                    break;

                case 8:
                    cout<<"REDUCE!!!!!!!"<<endl;
                    dw->maxVelo=dw->missionVelo;
                    machine->mode=StateMachine::State::Redu;     

                    // dw->stopLine=true;
                    // dw->fp.finished[curMissionIdx]=true;
                    // machine->mode=StateMachine::State::ESTOP;
                    break;
                default:
                cout<<"WHY??"<<endl;
                    break;    
            }
        }else{
            machine->mode=StateMachine::State::Drive;
        }
        if(dw->stopLine){
            if(dw->light==DataWarehouse::GREEN || dw->light==DataWarehouse::LEFT ){
                //GO!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                cout<<"GPGPGOGPGOGOGO!!!!!!!!!!!!!!!!!!!!!1!!!!!!!!!!!!!1"<<endl;
                dw->fp.finished[curMissionIdx]=true;
                dw->maxVelo=dw->std_Velo;
                machine->mode=StateMachine::State::Drive;
                dw->brake=false;
            }else if(dw->light==DataWarehouse::RED){
                cout<<"STOP!!!!!!!!!!!!!!!!!!!!!1!!!!!!!!!!!!!1"<<endl;
                machine->mode=StateMachine::State::ESTOP;
                dw->brake=true;
            }
           
        }else{

        }
        //표지판 인식
        // 서행모드가 필요한 표지판
        if(dw->sign==DataWarehouse::CROSSWALK || \
            dw->sign==DataWarehouse::SCHOOLZONE || \
            dw->sign==DataWarehouse::SPEEDBUMP ){
            ROS_INFO("[VISION] Traffic sign detected!!");
            // traffic_Sign_Crawl();
        }else if(dw->sign==DataWarehouse::NULLSIGN){
           //표지판 사라짐
           //machine->start();
            
        }else{
            //또다른 무언가~~~
        }
        //로그 깔끔하게 하려고 미리 null줌. 실제는 정지선에서 멈출 때까지 가져가야함
        
        //차선 스티어링 값
        //if 스티어링값에 대한 차량조정

        //if 차선 인식 무시 모드인가~
        //-------------------------------LiDAR-------------------------------
        // 조건 추가(pathMaker에서 현재 노드==정적 장애물 미션 노드일경우)
        // if(dw->obstacleDetected){
        //     ROS_INFO("[2DRiDAR] Obastacle");
        //     ROS_INFO("[WARNNING] Static Obastacle Detected!");
        //     //정적 장애물 처리
        //     machine->static_Opstacle();
        
            
        // }else{
        //     if(machine->mode==StateMachine::State::Avoid){
        //         ROS_INFO("[2DRiDAR] Avoid Complete");
        //         machine->obastacle_Complete();
        //     }
        // }
        // // 조건 추가(pathMaker에서 현재 노드==동적 장애물 미션 노드일경우)
        // if(dw->obstacleDetected){
        //     ROS_INFO("[2DRiDAR] Obastacle");
        //     ROS_INFO("[WARNNING] Dynamic Obastacle Detected!");
        //     //동적 장애물 처리
        //     machine->dynamic_Opstacle();    
            
        // }else{
        //     if(machine->mode==StateMachine::State::Avoid){
        //         ROS_INFO("[2DRiDAR] Avoid Complete");
        //         machine->obastacle_Complete();
        //     }
        // }

        /*
        if 앞에 장애물있을 때
            if 동적장애물, 못피함
                상태를 Stop으로 바꾸고
            else if 정적 장애물
        */
        //-------------------------------GPS&IMU-------------------------------

        //GPS 꺼져있을 때 & 켜져있을 때 이동 어떻게하나?
        // if(dw->GPSbreakdown){
        //     ROS_INFO("[WARNNING] GPS BreakDdown!!!");
        //     if(!dw->obstacleDetected) machine->GPSBreakdown();
        //     //GPS가 꺼져있으면, 차선인식 주행 상태로 변경
        // }else{
        //     // ROS_INFO("[EH] GPS WORKS!!!");
        //     // ROS_INFO("[GPS] GPS Working");
        //     // if(dw->obstacleDetected) lp->setPathModeLocal();
        //     // else lp->setPathModeHDmap();
        //     //HD맵을 통해 글로벌 패스를 받아 GPS와 연동하며 로컬패스를 생성
        // }
         
        
    }

    //미션구역 진입했는지 위치 체크
    int isFlag(){
        int re=-1;
        for(int i=0;i<dw->fp.size;i++){
            //cout<<"flag X "<<dw->fp.x[i]<<" Y "<<dw->fp.y[i]<<endl;
            //cout<<"COORD X "<<dw->globalCoordX<<" Y "<<dw->globalCoordY<<endl;

            float subX = dw->globalCoordX-dw->fp.x[i];
            float subY = dw->globalCoordY-dw->fp.y[i];
            float dist = subX*subX+subY*subY;
            // cout<<"flag dist "<<dist<<" rad "<<dw->fp.rad[i]<<endl;
            if(dw->fp.finished[i]){
                cout<<"[EH] "<<i<<" MISSION Finish!!"<<endl;
            }else if(dist <= dw->fp.rad[i]*dw->fp.rad[i] || dw->fp.started[i]){
                dw->fp.started[i]=true;
                cout<<"[EH] "<<i<<" IN MISSION!!"<<endl;
                switch(dw->fp.fType[i]){
                    //None
                    case 'x':
                        curMissionIdx=i;
                        re= -2;
                        break;
                    //Parking
                    case 'p':
                        curMissionIdx=i;
                        re= 1;
                        break;
                    //Static Obstacle
                    case 's':
                         curMissionIdx=i;
                        //  cout<<"curMi = "<<curMissionIdx<<endl;
                        //  for(;;);
                        re= 2;
                         break;
                    //Dynamic Obstacle
                    case 'd':
                        curMissionIdx=i;
                        re= 3;
                         break;
                    // Return
                    case 'r':
                        cout<<"RE!"<<endl;
                        re= 4;
                         break;
                    // E-stop
                    case 'v':
                        re= 5;
                         break;
                    //STOPLINE
                    case 'l':
                        curMissionIdx=i;
                        re= 6;
                        break;
                    case 'b':
                        curMissionIdx=i;
                        re= 7;
                        break;
                    
                    // REDUEE
                    case 'e':
                        curMissionIdx=i;
                        re= 8;
                        break;
                }
            }
        }
        // cout<<"[EH]No MISSION!!"<<endl;
        return re;
    }
    // ----------------------------------신호등 탐지했을 때의 대응 및 상태변경 
    void traffic_Light_RED(){
        ROS_INFO("[VISION] TRAFFIC_Light RED");
        //if() -> 정지선 조건 추가해야함
        // machine->stop();
    }
    void traffic_Light_GREEN(){
        //traffic light가 Red->Green으로 바뀌면
        if (true){}
        ROS_INFO("[VISION] TRAFFIC_Light GREEN");
        machine->start();
        dw->light=DataWarehouse::tLight::NULLLIGHT;
    }
    void traffic_Light_LEFT(){
        ROS_INFO("[VISION] TRAFFIC_Light LEFT");
        //machine->stop();
    }
};