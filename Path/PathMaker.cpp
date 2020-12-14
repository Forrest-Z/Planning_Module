#pragma once
#include "ros/ros.h"
#include "LocalPlan.cpp"
#include "GlobalPlan.cpp"
#include "Machine/StateMachine.cpp"
#include "DataWarehouse.cpp"
#include "cmath"
class PathMaker{
public:
    DataWarehouse* dw;
    bool finished=false;
    double moveCoordX;
    double moveCoordY;
    double moveCoordX2;
    double moveCoordY2;
    double moveCoordX3;
    double moveCoordY3;
    double nearCoordX;
    double nearCoordY;
    double backCoordX;
    double backCoordY;

    float lineX;
    float lineY;

    float staticY;
    float staticX=2;

    float heading;
    LocalPlan* lp;
    GlobalPlan* gp;
    StateMachine* machine;
    enum PlaningMode{
        HDmap=0,
        LineCatch,
        Local,
        None,
        Parking,
        LocalStatic,
        Stop,
    };
    enum PlaningMode pMode = HDmap;
    //주행모드
    /*
        1. HD map의 글로벌 좌표를 통한 이동
            - 모든 기기가 정상적으로 작동중이며, 장애물 없음
            - gps와 slam을 통해 위치 파악해서 웨이포인트 이동
        
        2. 차선인식을 통한 직선주행
            - GPS가 작동하지 않는 상태
            - slam을 통해 위치 파악하지만, 차선인식을 우선으로 이동
        
        3. 로컬맵 사용
            - 정적 장애물이 존재해서 피해야함.
            - 차선인식 기능 제거(차선을 무시하고 위치를 틀어야함)
            - 장애물을 회피하는 과정을 local path로 만들고, 
                회피가 완료되면 일반주행 재개
    */
    PathMaker(DataWarehouse *dwPtr){
        dw=dwPtr;
        ROS_INFO("[Constructor] PathMaker_ON");
        gp = new GlobalPlan(dw);
        lp = new LocalPlan();
    }
    void setMachine(StateMachine* ma){
        machine=ma;
        machine->printCurMode();
    }
    void changeMode(int idx){
        //lp->setPMAdd(this);
        //pMode=(PathMaker::PlaningMode)idx;
    }
    void sendTopic(){
        // moveCoordX, moveCoordY, heading 보냄
    }
   
    int makePath(){
        //--------------------------------도착했냐?------------------------------------------
        if(gp->finished){
            cout<<"[PM] FINISHED!!!"<<endl;
            finished=true;
            return 0;
        }

        //--------------------------어떤 방법 이용해서 좌표 찍을까~~----------------------------
        if(machine->mode==StateMachine::State::Drive){
            if (dw->GPSbreakdown) {
                pMode=PlaningMode::LineCatch;
            }else{
                pMode=PlaningMode::HDmap;
            }
            
        }else if(machine->mode==StateMachine::State::StaticObs){
            pMode=PlaningMode::LocalStatic;
        }else if(machine->mode==StateMachine::State::Idle){
            pMode=PlaningMode::None;
        }else if(machine->mode==StateMachine::State::Park){
            // pMode=PlaningMode::Parking;
        }else if(machine->mode==StateMachine::State::StaticObsRe){
            pMode=PlaningMode::HDmap;
        }else if(machine->mode==StateMachine::State::ESTOP){
            pMode=PlaningMode::Stop;
        }
        //---------------------------점좌표 제작-------------------------------------------
        switch(pMode){
            case PlaningMode::HDmap:
                getPlanHDmap();
                return 0;
                break;
            case PlaningMode::LineCatch:
                getPlanLineCatch();
                return 1;
                break;
            case PlaningMode::Local:
                getPlanLocal();
                break;
            case PlaningMode::Parking:
                // getPlanParkingEncoder();
                break;
            case PlaningMode::Stop:
                stopAndWait();
                break;
            case PlaningMode::LocalStatic:
                getSoheeAlgo();
                break;
        }
        return 0;
    }
    void getPlanHDmap(){
        
        ROS_INFO("[Planning] Get Path by HDmap");
        cout<<"[PM] Link Name = "<<gp->pathLinkStr[gp->curLinkNum]<<" IDX = "<<gp->curLinkNum<<endl;
        gp->findPath(dw->globalCoordX, dw->globalCoordY);
        cout<<setprecision(15)<<"Cur Coord  "<<dw->globalCoordX<<" , "<<dw->globalCoordY<<endl;

        moveCoordX=gp->coordX;
        moveCoordY=gp->coordY;

        moveCoordX2=gp->coordX2;
        moveCoordY2=gp->coordY2;

        moveCoordX3=gp->coordX3;
        moveCoordY3=gp->coordY3;

        nearCoordX=gp->nearCoordX;
        nearCoordY=gp->nearCoordY;

        backCoordX=gp->backCoordX;
        backCoordY=gp->backCoordY;


        //GPS키면 없애야함
        // tempMove();
        // dw->globalCoordX=gp->globalCoordX;
        // dw->globalCoordY=gp->globalCoordY;
    }
    void getPlanLineCatch(){
        ROS_INFO("[Planning] Get Path by LineCatch");
        cout<<"line coord = "<<lineX<<"  "<<lineY<<endl;
        cout<<"Cur Coord  "<<dw->globalCoordX<<" , "<<dw->globalCoordY<<endl;

        moveCoordX=lineX;
        moveCoordY=lineY;

        moveCoordX2=lineX*1.3;
        moveCoordY2=lineY*1.3;
    }
    void getPlanParkingEncoder(){

        // velocity 3 fix

        ROS_INFO("[Planning] Get Path by ParkingEncoder");

        //gp->findPath(dw->globalCoordX, dw->globalCoordY);
        gp->coordX=0.0;
        gp->coordY=0.0;

        moveCoordX=0;
        moveCoordY=0;
        
        moveCoordX2=0;
        moveCoordY2=0;

        nearCoordX=0;
        nearCoordY=0;


        if(dw->canParking){
            cout<<"CAN PARK!!"<<endl;
            //Do Parking

        }
        //GPS키면 없애야함
        // dw->globalCoordX=gp->globalCoordX;
        // dw->globalCoordY=gp->globalCoordY;
        // tempMove();
        cout<<"Cur Coord  "<<dw->globalCoordX<<" , "<<dw->globalCoordY<<endl;       
    }
    void getPlanLocal(){
        ROS_INFO("[Planning] Get Path by LocalMap");
        dw->encoder=1;
        // lp->getObstacleAvoidPath();
        // moveCoordX=lp->coordX;
        // moveCoordY=lp->coordY;
        gp->coordX=0.0;
        gp->coordY=0.1;

        moveCoordX=gp->coordX;
        moveCoordY=gp->coordY;
        

        // tempMove();
        // dw->globalCoordX=gp->globalCoordX;
        // dw->globalCoordY=gp->globalCoordY;
        
        
        cout<<"Cur Coord  "<<dw->globalCoordX<<" , "<<dw->globalCoordY<<endl;
    }
    void getSoheeAlgo(){
        cout<<"SO HEE ENCODER"<<endl;
        gp->findPath(dw->globalCoordX, dw->globalCoordY);
        
        //cout<<gp->pathX[gp->curPathIdx]<<" , "<<gp->pathY[gp->curPathIdx]<<endl;
        //cout<<gp->count<<" , Cur Coord"<<gp->globalCoordX<<" , "<<gp->globalCoordY<<endl;
        cout<<"Cur Coord  "<<dw->globalCoordX<<" , "<<dw->globalCoordY<<endl;

        moveCoordX=gp->coordX;
        moveCoordY=gp->coordY;

        moveCoordX2=gp->coordX2;
        moveCoordY2=gp->coordY2;

        moveCoordX3=gp->coordX3;
        moveCoordY3=gp->coordY3;

        nearCoordX=gp->nearCoordX;
        nearCoordY=gp->nearCoordY;
    }


    void printStatus(){
        /*
            목적 경로 전체 출력
        */
        //현재 노드 출력, 현재 위치 출력
       
        
        cout<<"Current Node = "<<gp->pathNodeName[gp->curLinkNum]<<endl;
        cout<<setprecision(15)<<"Current COORD = "<<gp->globalCoordX<<" , "<<gp->globalCoordY<<endl;
        //목적 경로 전체 출력
       
    }
    void stopAndWait(){
        cout<<"S T O P S T O P S T O P S T O P S T O P S T O P"<<endl;
        return;
    }
    // 지금 GPS없어서 그냥 만들어놓은거고 나중에 GPS받는 방향으로 바꿔야됨
    void tempMove(){
        cout<<"MOVE TO "<<moveCoordX<<" , "<<moveCoordY<<endl;
        gp->globalCoordX += moveCoordX;
        gp->globalCoordY += moveCoordY;
        lp->globalCoordX += moveCoordX;
        lp->globalCoordY += moveCoordY;
    }
    //보낼 메시지 만듬
    const double getCoordX(){return moveCoordX;}
    const double getCoordY(){return moveCoordY;}

    const double getCoordX2(){return moveCoordX2;}
    const double getCoordY2(){return moveCoordY2;}

    const double getCoordX3(){return moveCoordX3;}
    const double getCoordY3(){return moveCoordY3;}

    const double getNearCoordX(){return nearCoordX;}
    const double getNearCoordY(){return nearCoordY;}

    const double getBackCoordX(){return backCoordX;}
    const double getBackCoordY(){return backCoordY;}

    
};