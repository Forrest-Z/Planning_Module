#pragma once
#include "ros/ros.h"
#include <geometry_msgs/Point.h>
#include <iostream>
#include <string>

using namespace std;
class DataWarehouse{

    class FlagPoint{
    public:
 
        float x[100];
        float y[100];
        int rad[100];
        int size=0;
        bool finished[100];
        bool started[100];
        char fType[100];
        FlagPoint(){
            for(int i=0;i<100;i++){
                x[i]=y[i]=0;
                started[i]=finished[i]=false;
            }
        }
    };
    
public:
    DataWarehouse(){
        ROS_INFO("[Constructor] DataWarehouse_ON");
        // openFlagFile();
    }
    float dis_stan=0;
    //-------------------------------SITUATION_ATTRIBUTE------------------
    //--상황값은 무조건 상태에 의해서 결정됨.

    //장애물 회피를 위한 차선인식 무시
    bool ignoreLane=false;

    //무조건 멈춰야됨
    bool STOP=false;

    //최대속도 (보통 = 18, 미션 = 5)
    float std_Velo=14;      //기본 값
    float maxVelo=14;       //토픽으로 쏘는 값
    float missionVelo=7;   //미션 값

    //-------------------------------VISION-------------------------------
    //신호등 관련 값들
    enum tLight{
        NULLLIGHT = 9999,
        RED = 2,
        GREEN = 1,
        LEFT = 1
    };
    enum tLight light = NULLLIGHT;

    //표지판 인식
    enum tSign{
        NULLSIGN = 9999,
        CROSSWALK=0,
        SCHOOLZONE,
        SPEEDBUMP,
    };
    enum tSign sign = NULLSIGN;
    //차선인식에서 얻은 스티어링 디그리
    float steering=0;
    bool stopLine=false;
    bool brake=false;
    //-------------------------------LiDAR-------------------------------
    //주차
    bool canParking=false;
    // int parkingNum;

    //장애물 인지
    bool obstacleDetected=false;

    //std::vector<float> obsMin;
    //std::vector<float> obsMax;
    std::vector<geometry_msgs::Point>  obsMin;
    std::vector<geometry_msgs::Point>  obsMax;
    std::vector<geometry_msgs::Point>  obsCen;
    //std::vector<float> obsMin;
    
    //-------------------------------GPS&IMU-------------------------------
    //현재 위치 위경도값
    double globalCoordX=-2000;
    double globalCoordY=-2000;
    //현재 헤딩값
    float heading=0;
    //현재 속도
    int velocity=0;
    //GPS작동 여부
    bool GPSbreakdown=false;
    bool encoder=false;
    //------------------------------HDMAP--------------------------------
    // JSON으로 직렬화된 값을 정리함. 기본적으로 features에 접근한 상태.
    // Json::Value rootNode;
    // Json::Value rootLink;
    // Json::Value totalNode;
    // Json::Value totalLink;
    FlagPoint fp;

    //파일 읽는 경로
    std::string path="/home/korus/catkin_ws/src/Control_pkg/src";

    // 데이터 파일 여는 과정
     void openFlagFile(){
        //메모리 할당 효율 너무안좋은듯. 나중에 리팩토링 필요

        string str = "";
        char c[100];
        int n,preN,idx,flgIdx;
       
        ifstream flagReadFile;
        flagReadFile.open(path+"/flag.txt",ios::app);
        if (flagReadFile.is_open()) {
            cout << "open File" << endl;
        }
        else cout << "open Failed" << endl;

        n=preN=idx=0;


        while (!flagReadFile.eof()) {
            flagReadFile.getline(c, 100);
            str = c;
            cout<<"type = "<<str.substr(0, 1)<<endl;
            fp.fType[idx]=c[0];
            n = str.find(" ");
            preN = n;
            n = str.find(" ", n + 1);
            cout<<"SIZE = "<<str.substr(preN+1, n- preN)<<endl;
            fp.rad[idx]=stoi(str.substr(preN+1, n- preN));
            preN = n;
            n = str.find(" ", n + 1);
            fp.x[idx]=stof(str.substr(preN+1, n- preN));
            fp.y[idx]=stof(str.substr(n+1));
            cout << " X : " << fp.x[idx] << endl;
            cout << " Y : " << fp.y[idx] << endl;
            idx++; 
            fp.size=idx;
	    }   
    } 
    void setSpeed(float std, float mission){
        std_Velo=std;      //기본 값
        maxVelo=std;       //토픽으로 쏘는 값
        missionVelo=mission;
    }
};