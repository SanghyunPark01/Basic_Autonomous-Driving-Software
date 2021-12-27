# Basic_Autonomous-Driving-Software  
2021년도 2학기 프로젝트  
프로젝트에 대한 내용은 아래 링크 참고.  
[프로젝트 소개 및 과정](https://sanghyunpark01.github.io/categories/Basic_Autodrive_Software)  
  
---  
## 🛠️환경설정 및 버전
|환경설정|버전|
|:---:|:---:|
|OS|Ubuntu 18.04|
|사용언어|C++|
|사용 IDE|QT Creater|
|ROS|Melodic|
|Cuda|10.1|
|CuDNN|7.6.5|
|OpenCV|3.4.0|  

## 🛠️ROS Package  
|ROS Package Name|Description|
|:---:|:---:|
|darknet_ros|yolov3 와 ROS를 연동하기 위한 패키지|
|video_stream_opencv|카메라로만 실행이 가능한 darknet_ros를 video로 실행시키기 위한 패키지|
|command_pkg|darknet_ros를 통해 yolov3에서 인식된 결과 값을 받아 인식된 값을 출력해주는 패키지|  

## 🛠️System Architecture  
<p align="center"><img src="https://user-images.githubusercontent.com/77342519/147436501-8dc7c385-a0dc-4616-891f-2b1f94d4f976.png" width="800px"></p>  

## 🛠️Test  
### Test 1
<p align="center"><img src="https://user-images.githubusercontent.com/77342519/147437038-5502d2c8-e4e1-42a3-99f4-e5e3ec6689a0.gif"></p>  

### Test 2 (신호등 바뀌는 모습)  
프레임 마다 값이 바뀜
<p align="center"><img src="https://user-images.githubusercontent.com/77342519/147437403-c6c53b67-4aca-48ad-a71d-6757beec9992.gif"></p>  


### Test 3 (속도제한 표지판이 바뀌는 모습)
속도제한 표지판은 바뀌고 인식이 안되도 이전 결과값 유지
<p align="center"><img src="https://user-images.githubusercontent.com/77342519/147437414-8d966ae5-5993-45a0-8ce1-494f784e2c9d.gif"></p>  

---  

## 🛠️자세한 내용은 Blog 참고
[프로젝트 소개 및 과정](https://sanghyunpark01.github.io/categories/Basic_Autodrive_Software)  

