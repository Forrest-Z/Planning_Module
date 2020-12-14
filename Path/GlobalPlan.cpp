#pragma once
#include "ros/ros.h"
#include <fstream>
#include <iostream>
#include <string>
#include "jsoncpp.cpp"
#include <stdlib.h>
#include "DataWarehouse.cpp"
#include "cmath"
#include "UTM.h"

using namespace std;
class GlobalPlan{

public:
    // 모든 행위 정지 - 함부로 건들면 안됨
    bool stop=false;

    //끝남
    bool finished=false;
    // 경로 노드 이름이 순서대로 적힘 - INPUT 하드코딩
    std::string pathNodeName[100];
    int nodeSize=2;
    //FlagPoint fp;
    
    
    // 경로 링크 순서대로 저장하는 변수 - 이 링크를 계속 따라감
    string pathLinkStr[10000];
    int pathLink[10000];
    int pathLinkSize=1;
    // 현재 진행중인 링크 Idx
    int curLinkNum=0;

    // JSON으로 직렬화된 값을 정리함. 기본적으로 features에 접근한 상태.
    Json::Value rootNode;
    Json::Value rootLink;
    Json::Value totalNode;
    Json::Value totalLink;

    //현재 링크를 좁은 간격으로 나눠서 각각의 경로의 좌표를 string으로 저장
    std::string pathX[50000];
    std::string pathY[50000];
    int pathQueueSize=0;
    int curPathIdx=0;
 
    // 현재 차량의 위도, 경도
    double globalCoordX = -2000;
    double globalCoordY = -2000;

    // 제어에 주는 값 - 다음 위치의 상대좌표. 단위 = 100000M
    double coordX=0.001;
    double coordY=0;

    double coordX2=0.001;
    double coordY2=0;

    double coordX3=0.001;
    double coordY3=0;

    double nearCoordX=0.001;
    double nearCoordY=0;

    double backCoordX=0;
    double backCoordY=0;

    //Idx변경 때 사용
    double preDesti=0;

    bool noLink=false;

    DataWarehouse* dw;
    bool coord_come=false; //오프셋 들어옴
    double offX=0;
    double offY=0;

    //파일 읽는 경로
    std::string  path="/home/korus/catkin_ws/src/Control_pkg/src";
    GlobalPlan(DataWarehouse* dwPara){
        this->dw=dwPara;
        ROS_INFO("[Constructor] GlobalPlan_ON");
        openNodeDataFile();
        getCurPathCOORD(pathLink[0]);    
        printCurPath();
        printAllNode();
        // for(;;);
        //---------------Heading Offset-----------
        //getCurDestiIdx();
        //getHeadingOffset();
    }

    ofstream out;
    void linkCOORDtoUTM(){
        out.open(path+"/link_UTM2.json");

        double lat[29]={37.242745,37.241682,37.241701,37.24021,37.2402,37.24017,37.23997,37.23996,37.23993,37.240026,37.240406,37.24095,37.241326,37.24146,37.24150,37.24155,37.24241,37.24245,37.24250,37.24276,37.24276,37.24276,37.24071,37.24065,37.24064,37.24034,37.24031,37.24026,37.23990};
        double lon[29] = {126.774262,126.77436254,126.77388934,126.773471, 126.773512,126.773571,126.774042,126.774079,126.774148,126.77431112,126.77435461,126.774317,126.7744516,126.774480,126.774481,126.774477,126.774457,126.774458,126.774454,126.773988,126.773939,126.773874,126.774062,126.773839,126.774013,126.773845,126.773818,126.773795,126.773557};
        for(int i=0;i<29;i++){
                double a,b;
                a=b=0;
                LatLonToUTMXY(lat[i], lon[i], 52, a, b);
                // out<<setprecision(15)<<"AF "<<a<<" , "<<b<<endl; 
        // offX=302459.72;
        // offY=4123708.77
                out<<setprecision(15)<<a-302459.72<<" "<<b-4123708.77<<endl;
        }        
    }
    void openNodeDataFile(){  
        Json::Reader reader;
        
        //--------------------------------READ DATA-----------------------------
        //-----------------------------LINK DATA-----------------------------
        ifstream readLink;
        string temp;
        string linkData="";
        readLink.open(path+"/linkJSON_test.json",ios::app);
        // readLink.open(path+"/linkJSON_test.json",ios::app);
        if(readLink.is_open()){
            ROS_INFO("[FILE_READ] Link file is opened");
            while(getline(readLink, temp)){
                linkData.append(temp);
            }
            //----------------------------------파싱파싱 파파싱
            reader.parse(linkData, rootLink);
            totalLink = rootLink["features"];
            //--------------------COORD to UTM-------------------

        }else{
            ROS_INFO("[ERROR] READ LINK FAIL!!");
        }
        //-----------------------------NODE DATA-----------------------------
        ifstream readNode;
        readNode.open(path+"/nodeJSON_test.json",ios::app);
        string nodeData="";
        if(readNode.is_open()){
            ROS_INFO("[FILE_READ] Node file is opened");
            while(getline(readNode, temp)){
                nodeData.append(temp);
            }
            //----------------------------------파싱파싱 파파싱----------------------
            reader.parse(nodeData, rootNode);
            totalNode = rootNode["features"];

        }else{
            ROS_INFO("[ERROR] READ NODE FAIL!!");
        }
        //-----------------------------Path DATA-----------------------------
        ifstream readPath;
        readPath.open(path+"/path.txt",ios::app);
        string pathData="";
        
        nodeSize=0;
        if(readPath.is_open()){
            ROS_INFO("[FILE_READ] Path file is opened");
            while(getline(readPath, temp)){
                pathNodeName[nodeSize]=temp;
                cout<<" Node~ "<<pathNodeName[nodeSize]<<endl;
                nodeSize++;
            }
            //----------------------------------파싱파싱 파파싱----------------------
        }else{
            ROS_INFO("[ERROR] READ PATH FAIL!!");
        }
        pathLinkSize=nodeSize-1;        
         for(int i=0;i<pathLinkSize;i++){
            pathLink[i] = getLinkIdxByTFNode(pathNodeName[i],pathNodeName[i+1]);
            if(pathLink[i]!=-1)
            pathLinkStr[i] = totalLink[pathLink[i]]["properties"]["ID"].asString();
            cout<<i<<" LINK = "<<pathLink[i]<<endl;
        }

        //------------------------위경도 데이터를 UTM으로 변환해서 사용 (애초에 utm이면 주석처리 ㄱㄱ)
        // linkCOORDtoUTM();
    }

    bool isFindLink=false;
    void setLink(){
        double min=0;
        curLinkNum=0;
        while(true){
            curPathIdx=0;
            min = getCurDestiIdx();
            cout<<"curLink = "<<curLinkNum<<"   path Idx = "<<curPathIdx<<"  min = "<<min<<endl;
            if(min<2) {
                // for(;;);
                return;
            }
            curLinkNum++;
            getCurPathCOORD(pathLink[curLinkNum]);  
        }
    }
    // 경로 찾기
    void findPath(double dwCoordX, double dwCoordY){

        globalCoordX=dwCoordX;
        globalCoordY=dwCoordY;
        // //         -2000  NOStart
        if(globalCoordX==globalCoordY && globalCoordY==-2000){
            cout<<"coord pls"<<endl;
            return;
        }
        // else{
        //     if(!isFindLink){
        //         setLink();
        //         isFindLink=true;
        //     }
        // }
        
        if(!finished){
            getCurPathCOORD(pathLink[curLinkNum]);  
            cout<<"next"<<endl;
            getCurDestiIdx();      
            if(isArrived()){
                cout<<"ARRIVED"<<endl;
                changeCulNode();
            }
            noLink=false;
            getMoveDist(); 
            //cout<<"-----------PATH FIND END------------------"<<endl;
        }
    }

    // Offset 적용
    void adapt_offset(){
        // filoty
        // offX=346208;
        // offY=4069632;
        //kcity
        // offX=302459.72;
        // offY=4123708.77;
        // 3gong
        // offX = 346391.9;
        // offY = 4070174.2;
        // pg
     offX = 346850.5;
        offY = 4069908.3;
        
        // narea
        // offX = 346768;
        // offY = 4070385; 
        // Sanhak
        // offX = 346243;
        // offY = 4069674; 
        for(int i=0;i<pathQueueSize-1;i++){
            // cout<<"OFFSET i = "<<i<<"  pathQueueSize = "<<pathQueueSize<<endl;
            pathX[i] = to_string(stod(pathX[i]) - offX);
            pathY[i] = to_string(stod(pathY[i]) - offY);     
        }
        // printCurPath();
        // cout<<"OFFX"<<offX<<endl;
        // cout<<"OFFY"<<offY<<endl;
        // cout<<"af "<<pathX[0]<<endl;
    }

    //노드 도착했나 안했나 확인
    bool isArrived(){
        cout<<"cur IDX = "<<curPathIdx<<endl;
        double destiX = std::stod(pathX[curPathIdx]);
        double destiY = std::stod(pathY[curPathIdx]);
        double subX = abs(globalCoordX-destiX);
        double subY = abs(globalCoordY-destiY);
        // cout<<subX<<"   ~~    "<<subY<<endl;
        cout<<"sub = "<<subX+subY<<endl;
        if(subX+subY>0.3 && curPathIdx<pathQueueSize-2){
            cout<<"[GlobalPath] NO CHANGE"<<endl;
            return false;
        }else{
            cout<<"[GlobalPath] PATH CHANGED!"<<endl;
            return true;
        }
    }

    //노드에 도착했을 경우 다음 목적 노드를 바꿈
    void changeCulNode(){
        //-----------------------BEF----------------------
        // curPathIdx++;
        cout<<" cur Idx = "<<curPathIdx<<" Qsize = "<<pathQueueSize<<endl;
        //0start--,
        if(curPathIdx>=pathQueueSize-65){
            //finished=true;
            curPathIdx=0;
            curLinkNum++;
            if(curLinkNum==pathLinkSize){
                cout<<"[GlobalPath] Path Finished!!  "<<endl;
                for(;;);
            }
            getCurPathCOORD(pathLink[curLinkNum]);  
            cout<<"[GlobalPath] Link Finished!!  "<<endl;
        }
     }

    
    //현재 링크 위에서 가장 가까운 경로 구하기
    double getCurDestiIdx(){
        double min = 99999;
        int minIdx=curPathIdx;

        if(coordX==coordY && coordX==0) {curPathIdx++;}
        cout<<"cur "<<curPathIdx<<"  pathQueueSize = "<<pathQueueSize<<endl;
        for(int i=curPathIdx;i<pathQueueSize-1;i++){
            double desti= pow(globalCoordX-stod(pathX[i]), 2) + pow(globalCoordY-stod(pathY[i]), 2);
            // cout<<"DESTI = "<<desti<<" idx "<<i<<endl;
             if(desti<min){
                min=desti;
                minIdx=i;
            }
        }
        curPathIdx=minIdx;

        // cout<<"DESTI = "<<pow(globalCoordX-stod(pathX[curPathIdx]), 2) + pow(globalCoordY-stod(pathY[curPathIdx]), 2)<<" curIdx "<<curPathIdx<<endl;
        return min;
    }

    //좌표2개 구하기
    int getNextIdx(double interval){
        int re=0;
        for(int i=curPathIdx+1;i<pathQueueSize;i++){
            re=i;
            double subX = stod(pathX[i])-stod(pathX[curPathIdx]);
            double subY = stod(pathY[i])-stod(pathY[curPathIdx]);
            if(subX*subX + subY*subY >interval*interval){
                return i;
            }
        }
        cout<<"Something wrong"<<endl;
        // for(;;);ssssss
        return re;
    }
    int getPreIdx(double interval){
        int re=0;
        for(int i=curPathIdx;i>=0;i--){
            re=i;
            double subX = stod(pathX[i])-stod(pathX[curPathIdx]);
            double subY = stod(pathY[i])-stod(pathY[curPathIdx]);
            if(subX*subX + subY*subY >interval*interval){
                return i;
            }
        }
        // cout<<"Something wrong"<<endl;
        // for(;;);
        return re;
    }
    int pathPointIdx[10];
    double pathPointX[10];
    double pathPointY[10];
    //이동할 좌표 구하기
    void getMoveDist(){
        int nextPath =20;
        //------------------Path by Curve---------------------------------
        int scdTargetIdx = curPathIdx+nextPath;
        int firTargetIdx = curPathIdx+nextPath/2;
        int forTargetIdx = curPathIdx+nextPath/2;

        int nearTargetIdx = curPathIdx+1;

        int backTargetIdx = curPathIdx-3;

        //---------------AFTER----------------------------------
        // firTargetIdx=getNextIdx(0);
        firTargetIdx=getNextIdx(0.7);
        scdTargetIdx=getNextIdx(1.2);
        forTargetIdx=getNextIdx(5);
        backTargetIdx=getPreIdx(0.5);
        if(scdTargetIdx>= pathQueueSize) {
            scdTargetIdx = pathQueueSize-1;
        }
        if(firTargetIdx>=pathQueueSize){
            firTargetIdx= pathQueueSize-1;
        }
        if(forTargetIdx>=pathQueueSize){
            forTargetIdx= pathQueueSize-1;
        }
        if(nearTargetIdx>=pathQueueSize){
            nearTargetIdx= pathQueueSize-1;
        }
        if(nearTargetIdx<0) nearTargetIdx=0;
        if(backTargetIdx<0) backTargetIdx=0;

        float interval = 5;
        pathPointIdx[0]=nearTargetIdx;
        for(int i=1;i<10;i++){
            pathPointIdx[i]=getNextIdx((i+1)*interval);
        }
        for(int i=0;i<10;i++){
            if(pathPointIdx[i] >= pathQueueSize)
            {
                pathPointIdx[i] = pathQueueSize-1;
                cout<<"UPON PATH"<<endl;
            }
            pathPointX[i]= stod(pathX[pathPointIdx[i]]) - globalCoordX;
            pathPointY[i]= stod(pathY[pathPointIdx[i]]) - globalCoordY;
        }

        double dirLat= stod(pathX[firTargetIdx]) - globalCoordX;
        double dirLon= stod(pathY[firTargetIdx]) - globalCoordY;
        if(dirLat<0.000002 && dirLat>-0.000002)dirLat=0;
        if(dirLon<0.000002 && dirLon>-0.000002)dirLon=0;
        coordX=dirLat;
        coordY=dirLon;
        

        dirLat= stod(pathX[scdTargetIdx]) - globalCoordX;
        dirLon= stod(pathY[scdTargetIdx]) - globalCoordY;
        if(dirLat<0.000002 && dirLat>-0.000002)dirLat=0;
        if(dirLon<0.000002 && dirLon>-0.000002)dirLon=0;
        coordX2=dirLat;
        coordY2=dirLon;

        dirLat= stod(pathX[forTargetIdx]) - globalCoordX;
        dirLon= stod(pathY[forTargetIdx]) - globalCoordY;
        if(dirLat<0.000002 && dirLat>-0.000002)dirLat=0;
        if(dirLon<0.000002 && dirLon>-0.000002)dirLon=0;
        coordX3=dirLat;
        coordY3=dirLon;


        dirLat= stod(pathX[nearTargetIdx]) - globalCoordX;
        dirLon= stod(pathY[nearTargetIdx]) - globalCoordY;
        nearCoordX=dirLat;      
        nearCoordY=dirLon;

        dirLat= stod(pathX[backTargetIdx]) - globalCoordX;
        dirLon= stod(pathY[backTargetIdx]) - globalCoordY;
        backCoordX=dirLat;      
        backCoordY=dirLon;

        printCoord();
    }
    void printCoord(){
        cout<<"IDX "<<curPathIdx<<endl;
        cout<<setprecision(15)<<"Des coord = "<<coordX<<" , "<<coordY<<endl;
        cout<<setprecision(15)<<"Des coord2 = "<<coordX2<<" , "<<coordY2<<endl;
        cout<<setprecision(15)<<"Near coord = "<<nearCoordX<<" , "<<nearCoordY<<endl;
        cout<<setprecision(15)<<"[GP] coord = "<<globalCoordX<<" , "<<globalCoordY<<endl;
        //cout<<"rem coord = "<<coordX2 - globalCoordX<<" , "<<coordY2-globalCoordY<<endl;
    }
    
    // 지금 GPS없어서 그냥 만들어놓은거고 나중에 GPS받는 방향으로 바꿔야됨
    void tempMove(){
        cout<<"MOVE TO "<<coordX<<" , "<<coordY<<endl;
        globalCoordX += coordX;
        globalCoordY += coordY;
    }
    int interval=20; //cm단위, 경로 인터벌
    // 목적 노드의 번호를 사용해서 현재 링크의 좌표를 쭉 얻어옴
    void getCurPathCOORD(int linkIdx){
        pathQueueSize=0;
        cout<<"Link idx = "<<linkIdx<<endl;
        if(linkIdx==-1) return;
        Json::Value coord = totalLink[linkIdx]["geometry"]["coordinates"][0];


        int queueSize=0;
        double dis=0,disX=0,disY=0;
        int m=0;
        cout<<"size = "<<endl;
        for(int i=0;i<coord.size()-1;i++){
            // cout<<"Idx = "<<i<<endl;
            double x0,y0,x1,y1;
            x0=coord[i][0].asDouble();
            y0=coord[i][1].asDouble();

            x1=coord[i+1][0].asDouble();
            y1=coord[i+1][1].asDouble();

            disX = x1-x0;
            disY = y1-y0;

            dis = sqrt((x1-x0)*(x1-x0) + (y1-y0)*(y1-y0));
            m = dis*100/interval;
            if(m==0) m=1;
            for(int ii=0;ii<m;ii++){
                pathX[queueSize]=to_string(coord[i][0].asDouble() + disX*ii/m);
                pathY[queueSize]=to_string(coord[i][1].asDouble() + disY*ii/m); 
                // cout<<pathX[queueSize]<<" "<<pathY[queueSize]<<endl;
                queueSize++;
            }
        }
        int nextLinkNum = curLinkNum+1;
        if(nextLinkNum<pathLinkSize-1){
            Json::Value nextCoord = totalLink[pathLink[nextLinkNum]]["geometry"]["coordinates"][0];
            int to40=0;
            for(int i=0;i<nextCoord.size()-1;i++){
                // cout<<"Idx = "<<i<<endl;
                double x0,y0,x1,y1;
                x0=nextCoord[i][0].asDouble();
                y0=nextCoord[i][1].asDouble();

                x1=nextCoord[i+1][0].asDouble();
                y1=nextCoord[i+1][1].asDouble();

                disX = x1-x0;
                disY = y1-y0;

                dis = sqrt((x1-x0)*(x1-x0) + (y1-y0)*(y1-y0));
                m = dis*100/interval;
                if(m==0) m=1;
                for(int ii=0;ii<m;ii++){
                    pathX[queueSize]=to_string(nextCoord[i][0].asDouble() + disX*ii/m);
                    pathY[queueSize]=to_string(nextCoord[i][1].asDouble() + disY*ii/m); 
                    // cout<<pathX[queueSize]<<" "<<pathY[queueSize]<<endl;
                    queueSize++;
                    to40++;
                    if(to40>=40){
                        i=ii=9999999;
                    }
                }
            }
        }
        else{
            for(int ii=0;ii<40;ii++){
                if(m==0) m=1;
                pathX[queueSize]=to_string(coord[coord.size()-2][0].asDouble() + disX*ii/m);
                pathY[queueSize]=to_string(coord[coord.size()-2][1].asDouble() + disY*ii/m);
                // cout<<pathX[queueSize]<<" "<<pathY[queueSize]<<endl;
                queueSize++; 
            }
        }
        
            // for(;;);
        pathQueueSize=queueSize;
        adapt_offset();
    }

    // totalLink의 ToNode와 nodeId사용해서 link의 idx찾기
    int getLinkIdxByTFNode(string fromNodeID,string toNodeID){
        int re=0;
        cout<<"FROM = "<<fromNodeID<<"TO = "<<toNodeID<<endl;
        //----------------------------From, To 이용해서 노드 찾기--------------------------------------
        for(int i=0;i<totalLink.size();i++){
            if(totalLink[i]["properties"]["ToNodeID"]==toNodeID && totalLink[i]["properties"]["FromNodeID"]==fromNodeID){
                return i;
            }
        }
       cout<<"ERROR - No NODE"<<endl;
       return -1;
    }
    
    //----------------------------------------------------  PRINT---------------------------------------------------------
    // 모든 노드, 링크 출력
    void printAllNode(){
        cout<<"--------------Node List------------------------"<<endl;
        for(int i=0;i<pathLinkSize;i++){
            cout<<"Node "<<pathNodeName[i]<<endl;
             if(pathLink[i]!=-1) cout<<"      Link "<<totalLink[pathLink[i]]["properties"]["ID"];   //현대 - LINKID 자율 ID
             else cout<<"      Link  - NULL"<<endl;   //현대 - LINKID 자율 ID
        }
        cout<<"Node "<<pathNodeName[pathLinkSize]<<endl;
        cout<<"--------------------------------------------------"<<endl;
    }
    // 링크 내의 경로 출력
    void printCurPath(){
        cout<<"--------------Current Path------------------------"<<endl;
        for(int i=0;i<pathQueueSize;i++){
           cout<<setprecision(15)<<pathX[i]<<" , "<<pathY[i]<<endl;
        }
        cout<<"--------------------------------------------------"<<endl;
        cout<<"size = "<<pathQueueSize<<endl;
        // for(;;);
    }

    void printAll(){
        cout<<"x, y"<<endl;
        for(int i=0;i<pathLinkSize;i++){
            int idx = pathLink[i];
            cout<<"idx = "<<idx<<endl;
            if(0 < idx || idx <= totalLink.size()) Json::Value printCoord = totalLink[idx]["geometry"]["coordinates"][i];
        }
    }

    ofstream outPath;
    string outPathFileName="/pathpath.txt";
    //전체 경로 파일 출력
    void pathFilePrint(){
        outPath.open(path+outPathFileName);
        if(!outPath.is_open()){
            cout<<"FAILED"<<endl;
        }else{
            cout<<"SUCCESS"<<endl;
        }
        for(int i=0;i<pathLinkSize;i++){
            int idx = pathLink[i];
            cout<<"idx = "<<idx<<endl;
            if(0 < idx || idx <= totalLink.size()) {
                Json::Value coordArr = totalLink[idx]["geometry"]["coordinates"][0];
                for(int ii=0;ii<coordArr.size();ii++){

                    string line = "[ "+coordArr[ii][0].asString()+" , "+coordArr[ii][1].asString()+"],\n";
                    // outPath<<coordArr[ii][0]<<" "<<coordArr[ii][1]<<endl;
                    outPath<<line;
                }
            outPath<<endl;
            }

        }
    }
        void openDataFile(){
        //메모리 할당 효율 너무안좋은듯. 나중에 리팩토링 필요
        ifstream readFile;
        readFile.open(path+"/pathFile.txt",ios::app);
        if (readFile.is_open()) {
            cout << "open File" << endl;
        }
        else cout << "open Failed" << endl;
        string str = "";
        char c[100];
        int n,preN,idx,flgIdx;
        n=preN=idx=flgIdx=0;
        while (!readFile.eof()) {
            readFile.getline(c, 100);
            str = c;
            n = str.find(" ");
            pathX[idx] = str.substr(0, n-2);
            pathY[idx] = str.substr(n+1, n+7);
            idx++;
	    }
        pathQueueSize=idx;  
    }
};


/**/