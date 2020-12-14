# Planning_Module

 이 레파지토리는 무인차량의 경로추종 프레임워크를 단순화하기 위해 제작되었습니다. \
필자가 2020 국제 창작 자동차 대회에 출전할 때 직접 제작했었던 Korus팀의 판단 프레임워크를 일부 수정했음을 밝힙니다.


1. GPS, IMU를 사용한 일반적인 방법 
2. LiDAR, Camera를 사용한 SLAM

 위 두 개의 환경에서 미리 주어진 전역경로 상에서 측위->판단->제어로 이어지는 시스템만을 구성하기 위해서, \
해당되지 않는 토픽이나 모듈은 생략하거나 삭제했습니다.

주변 차량이나, 인지파트에서 입력된 데이터를 제외하고

## 목차

1. [프레임워크 구성](#1장-프레임워크-구성)
2. [주요 기능](#2장-주요-기능) \
2.1 [경로생성 및 사용](#2_1-경로생성-및-사용) \
2.2 [SLAM, GPS 토픽 처리](#2_2-SLAM,-GPS-토픽-처리)
3. [경로 출력](#3장-경로-출력)



---

## 1장 프레임워크 구성
//기술보고서의 프레임워크 사진

Camrera, LiDAR, GPS, IMU 등의 센서로부터 데이터를 입력받아 지역경로를 생성한 뒤 제어파트에 전달합니다.  
Control_pkg는 메인 루프가 주기적으로 반복되며 다른 모듈을 호출하는 패키지의 핵심 모듈입니다.  
모든 input data는 콜백함수를 통해 Data Warehouse에 저장되고, EventHandler에서는 매 주기마다 호출되며   
외부 환경에 따라 차량의 상태나 행동을 결정합니다.(본 프로젝트에서는 비활성화)  
Path Maker는 읽어들인 전역경로와 현재 차량의 상태를 기반으로 지역경로를 생성합니다.

## 2장 주요 기능
## 2_1 경로생성 및 사용
## 2_2 SLAM, GPS 토픽 처리
## 3장 경로 출력