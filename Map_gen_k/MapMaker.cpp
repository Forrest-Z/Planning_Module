#include "string"
#include "fstream"
#include "iostream"
#include "tf2_msgs/TFMessage.h"
#include "ros/ros.h"
#include "UTM.cpp"
#include "sensor_msgs/NavSatFix.h"
using namespace std;

float preX=-99999;
float preY=-99999;
float preX2=-99999;
float preY2=-99999;


double interval = 0.2;

ofstream writeFile;
ofstream utmWriteFile;

void writePositionLine(const float posX,const float posY){
    if(preX==-99999 || posX==-99999) {
        cout<<"Data Please~~"<<endl;
        preX=posX;
        preY=posY;
        return;
    }
    double subX = preX-posX;
    double subY = preY-posY;
    cout<<"sub X = "<<subX<<endl;
    if( subX*subX + subY*subY < interval*interval){
        cout<<"Too short to write"<<endl;
        return;
    }
    string str="["+to_string(posX)+" , "+to_string(posY)+"],";
    if(writeFile.is_open()){
        cout<<"Write "<<str<<endl;
        writeFile<<str<<endl;
        preX=posX;
        preY=posY;
    }
}
void writePositionLine2(const float posX,const float posY){

    if(preX2==-99999 || posX==-99999) {
        cout<<"Data Please~~"<<endl;
        preX2=posX;
        preY2=posY;
        return;
    }
    double subX = preX2-posX;
    double subY = preY2-posY;
     cout<<"sub X = "<<subX<<endl;
    if( subX*subX + subY*subY < interval*interval){
        cout<<subX*subX + subY*subY<<"  Too short to write"<<endl;
        return;
    }
    string str=to_string(posX)+" "+to_string(posY);

    if(utmWriteFile.is_open()){
        cout<<"Write "<<str<<endl;
        utmWriteFile<<str<<endl;
        preX2=posX;
        preY2=posY;
    }
}
void slamCallback(tf2_msgs::TFMessage msg){
    static float posX,posY;
    cout<<"---------------------SLAM COME-------------"<<endl;
    posX= msg.transforms[0].transform.translation.x;
    posY= msg.transforms[0].transform.translation.y;
    cout<<"pose "<<posX<<" "<<posY<<endl;
    writePositionLine(posX, posY);
}
void gpsCallback(sensor_msgs::NavSatFix msgs){
    static float posX,posY;
    cout<<"---------------------GPS COME-------------"<<endl;
    double a,b;
    a=b=0;
    LatLonToUTMXY(msgs.latitude, msgs.longitude, 52, a, b);
    cout<<"ROW!!!  "<<a<<" "<<b<<endl;
    posX= a;
    posY= b;

    writePositionLine2(posX, posY);
}
int main(int argc, char** argv){
    ros::init(argc,argv,"MapMaker");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/fix",100,gpsCallback);
    ros::Subscriber simulSub = nh.subscribe("/tf",100, slamCallback);
    ros::Rate loop(100); 

    string path="/home/korus/catkin_ws/src/Control_pkg/data/";
    string fileName="path_gen.txt";
    string utmFileName="utm_path_gen.txt";
    
    writeFile.open(path+fileName);
    utmWriteFile.open(path+utmFileName);


    if(writeFile.is_open()){
        cout<<"YEAH~~"<<endl;
    }

    while(ros::ok){       
        ros::spinOnce();
        // system("clear");
        loop.sleep();
    }
    ros::shutdown();

    return 0;
}