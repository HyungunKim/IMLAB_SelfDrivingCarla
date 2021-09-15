# 프로젝트 소개
국민대학교 지능형 모빌리티 연구실 - 칼라 시뮬레이터를 활용한 자율주행 연구.
([View in English](#about-this-project))
## 빠른 시작 가이드
* [칼라 시뮬레이션](https://github.com/carla-simulator/carla/blob/master/Docs/download.md) 실행
* local_lane_matching/roadmodule.py 코드 13 줄의 xml tree 생성을 원하는 맵으로 지정
* local_lane_matching/map_info.py 실행


   ![image](https://user-images.githubusercontent.com/51437350/133353947-44f509e0-c104-4ea5-8d97-79b796d2233c.png)
* 실행 화면
* 실행 결과 ex) `Location(x=321.607025, y=59.2123120, z=0.038266) is in road 10, lane -1`
  


## 파일 설명

# About this project
Kookmin University Intelligent Mobility Laboratory - Self Driving Car with Carla Project.
([한글로 보기](#프로젝트-소개))
## Quick Start Guide
* [Start Carla](https://github.com/carla-simulator/carla/blob/master/Docs/download.md)
* in local_lane_matching/roadmodule.py line 13, change xml tree to load the your map
* run local_lane_matching/map_info.py in terminal

   ![image](https://user-images.githubusercontent.com/51437350/133353947-44f509e0-c104-4ea5-8d97-79b796d2233c.png)
* screen shot
* command line output ex) `Location(x=321.607025, y=59.2123120, z=0.038266) is in road 10, lane -1`
