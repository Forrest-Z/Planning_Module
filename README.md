# Planning_Module

![main](https://user-images.githubusercontent.com/55074554/102064247-6a74d900-3e3a-11eb-9b44-2c0ea2d77908.gif) 

 이 레파지토리는 무인차량의 경로추종 프레임워크를 단순화하기 위해 제작되었습니다. \
필자가 2020 국제 창작 자동차 대회에 출전할 때 직접 제작했었던 Korus팀의 판단 프레임워크를 일부 수정했음을 밝힙니다.


1. GPS, IMU를 사용한 일반적인 방법 
2. LiDAR, Camera를 사용한 SLAM

 위 두 개의 환경에서 미리 주어진 전역경로 상에서 측위->판단->제어로 이어지는 시스템만을 구성하기 위해서, \
해당되지 않는 토픽이나 모듈은 생략하거나 삭제했습니다.


## 목차

1. [프레임워크 구성](#1장-프레임워크-구성)
2. [주요 기능](#2장-주요-기능) \
2.1 [경로생성 및 사용](#2_1-경로생성-및-사용) \
2.2 [SLAM, GPS 토픽 처리](#2_2-SLAM-GPS-토픽-처리)
3. [경로 출력](#3장-경로-출력)



---

## 1장 프레임워크 구성
![Framework](이미지 URL)
Camrera, LiDAR, GPS, IMU 등의 센서로부터 데이터를 입력받아 지역경로를 생성한 뒤 제어파트에 전달합니다.  
Control_pkg는 메인 루프가 주기적으로 반복되며 다른 모듈을 호출하는 패키지의 핵심 모듈입니다.  
모든 input data는 콜백함수를 통해 Data Warehouse에 저장되고, EventHandler에서는 매 주기마다 호출되며   
외부 환경에 따라 차량의 상태나 행동을 결정합니다.(본 프로젝트에서는 비활성화)  
Path Maker는 읽어들인 전역경로와 현재 차량의 상태를 기반으로 지역경로를 생성합니다.

## 2장 주요 기능
## 2_1 경로생성 및 사용
  
본 모듈에서는 JSON형태로 직렬화된 경로 데이터를 파싱해서 사용한다. 
일반적으로 국토지리정보원에서 제공하는 정밀 도로지도의 shape파일을 직렬화하면 geoJson파일을 얻을 수 있는데, 확장자만 json으로 변경한 형태이다.  
  
~경로 json파일 캡쳐이미지
  
  
Link json파일의 구조는 위와 같다. 여러 속성이 있지만 실제로 사용하는 것은 몇 되지 않는다.  
ID는 Link의 ID이고 L_LinID와 R_LinkID는 해당 Link의 좌, 우 차선 Link의 ID이다. (없으면 null로 표시됨)
From, To NodeID는 해당 Link의 시작노드와 끝 노드의 ID이며, 아래 geometry의 coordinates속성은 해당 경로를 적절히 보간한 위치 데이터이다.  
차량은 주기마다 자신의 현재 위치와 자신이 현재 주행중인 경로의 데이터를 비교해가며 자신의 지역경로를 생성하게 된다.  
차선을 변경 할 때는 R,L LinkID를 탐색하고, 링크의 ToNode 지점에 일정 거리 이상 가까워지면 다음 경로의 LinkID를 받아와 사용하게 된다.

## 2_2 SLAM GPS 토픽 처리

사실상 SLAM과 GPS 어느 방법을 사용하더라도, 자신의 위치와 현재 경로를 비교하는 작업의 큰 틀은 유사하다.  
세부적으로 다른점은 크게 2가지로 나눌 수 있다.  

1. SLAM은 미리 제작된 맵에서의 상대좌표를 사용하게 되고, GPS는 위도와 경도를 사용한다.
2. SLAM은 현재 위치와 동시에 차량의 Heading을 제공하지만 GPS는 직접 구해야한다.
  
위 두 문제를 전처리과정에서 해결해준다면 같은 이름의 토픽으로 제공받아도 정상적으로 작동한다.  
GPS 데이터만을 사용해서 Heading을 구한다면 저속 혹은 정차중인 상태에서 차량의 heading이 정상적으로 연산되지 않는다.  
imu를 포함해서 칼만필터 등을 이용해 퓨전한다면 GPS의 Heading문제를 해결할 수 있다.  

위도, 경도로 현재 위치가 표현되는 경우 이를 UTM좌표계에 맞춰 변환해 주는것이 권장된다.
미터단위의 직관성과 사용의 편의성을 생각한다면 매우 자연스러운 선택이고, 위경도 데이터를 그대로 사용한다면 미터단위로 진행되는 모든 알고리즘을 수정해주는 작업이 필요하다. 추가로 판단 모듈에서 현재 위치가 어떤 단위로 제공되고있는지도 파악해야하는 번거로움이 있다.  

특히 이러한 인터페이스를 일원화 하는것은 매우 중요하다.  
어떤 센서에서 어떤 데이터를 제공하는지 일일히 명세하고, 이를 판단에 사용하게 된다면 프레임워크의 유지보수와 확장성에 제한이 발생한다.  
이러한 데이터들은 외부 인지, 측위 모듈에서 모든 데이터를 정리한 다음, 정해진 양식을 통해 데이터를 일관성있게 전달해야 한다.



## 3장 경로 출력

로보틱스 환경에서 경로는 보통 두 가지로 표현된다.

1. 샘플링된 좌표들의 집함
2. 일정한 거리까지의 3차함수

현재 차량의 위치에서 가장 가까운 전역경로에 수렴할 수 있는 지역경로를 만들어야 한다.

![local](https://user-images.githubusercontent.com/55074554/102064236-6648bb80-3e3a-11eb-83db-891d2527b5f7.gif)

지역경로를 생성하는 알고리즘은 Frenet Optimal Trajectory알고리즘을 사용했다. [논문링크](http://video.udacity-data.com.s3.amazonaws.com/topher/2017/July/595fd482_werling-optimal-trajectory-generation-for-dynamic-street-scenarios-in-a-frenet-frame/werling-optimal-trajectory-generation-for-dynamic-street-scenarios-in-a-frenet-frame.pdf)  
본 프레임워크는 경로를 샘플링한 후 좌표의 집합을 전송하거나, 샘플링된 좌표를 3차함수로 근사시켜 제어측에 제공한다.
