#include "ros/ros.h"
#include "std_msgs/String.h"
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <vector>
using namespace std;

//class
class Box{
protected:
  double x1, x2, y1, y2;
  double size;
  double Gx,Gy;
  double pp;
  string name;
  int bid;
  int valid;
public:
  Box(){};
  ~Box(){};
  void Set_i(string s,double p, double a, double b, double c, double d);
  void Set_id(int t){bid=t;}
  void Get_boundingbox_i(void);
  void Set_valid(void);
  void Test_print(void);
  int pull_bid(void){return bid;}
  double pull_pp(void){return pp;}
  int pull_valid(void){return valid;}
  string pull_name(void){return name;}
};

//variable
int box_count;
string vv;
string tt;
vector<Box> bbox_t;
vector<Box> bbox_v;
//
void Box :: Get_boundingbox_i()
{
  double width, height;
  width=x2-x1;
  height=y2-y1;
  size=width*height;
  Gx=(x1+x2)/2;
  Gy=(y1+y2)/2;
}
void Box :: Test_print(){
  cout<< "name : " << name << ", size : " << size << endl;
  cout<< "probability : "<< pp << endl;
  cout<< "Center of Box : ("<< Gx << ", " << Gy << ")" << endl;
  cout<< "id : "<< bid<<endl;
  cout<< "valid :"<<valid<<endl;
}
void Box :: Set_i(string s,double p, double a, double b, double c, double d){
  name=s;pp=p; x1=a; x2=b; y1=c; y2=d;
}
void Box :: Set_valid(void){
  int v=1;
  /*if(bid==0){
    if(Gy>300) v=0;
    if(Gx>800||Gx<200) v=0;
  }else if(bid==1){
    if(Gx<500) v=0;
  }*/
  if(pp<0.921)v=0;
  valid=v;
}
string msg_t(void){
  string msg1;
  int s=bbox_t.size();
  for(int i=0;i<s;i++){
    if(bbox_t[0].pull_name()!=bbox_t[i].pull_name())msg1="...";
    else {
      msg1=bbox_t[i].pull_name();
      tt=msg1;
    }
  }


  return msg1;
}
string msg_v(void){
  string msg2;
  int s=bbox_v.size();
  for(int i=0;i<s;i++){
    if(bbox_v[0].pull_name()!=bbox_v[i].pull_name())msg2="...";
    else {
      msg2=bbox_v[i].pull_name();
      vv=msg2;
    }
  }
  return msg2;
}
void Command_msg(void){

  cout<<"-------------------------"<<endl;
  if(bbox_t.size()!=0){
    if(msg_t()=="traffic_L")cout<<"Detect Left traffic signal";
    else if(msg_t()=="traffic_R")cout<<"Detect Red traffic signal, slow down";
    else if(msg_t()=="traffic_G")cout<<"Detect Green traffic signal";
    else cout<<msg_t();}
  else if(bbox_t.size()==0){
    cout<<"Go straight";
  }

  cout<<endl;
  //-----------------------------
  if(bbox_v.size()!=0){
    if(msg_v()=="v_30")cout<<"Limit 30Km/h";
    else if(msg_v()=="v_50")cout<<"Limit 50km/h";
    else cout<<msg_v();}
  else if(bbox_v.size()==0){
    if(vv=="v_30")cout<<"Limit 30Km/h";
    else if(vv=="v_50")cout<<"Limit 50km/h";
    else cout<<msg_v();}
  cout<<endl<<"-------------------------"<<endl;
}
void Algorithm_main(const darknet_ros_msgs::BoundingBoxesConstPtr& boundingbox)
{
  box_count=boundingbox->bounding_boxes.size();
  for(int i=0;i<box_count;i++){
      Box B;
      B.Set_i(boundingbox->bounding_boxes[i].Class, boundingbox->bounding_boxes[i].probability,boundingbox->bounding_boxes[i].xmax,boundingbox->bounding_boxes[i].xmin,boundingbox->bounding_boxes[i].ymax,boundingbox->bounding_boxes[i].ymin);
      B.Get_boundingbox_i();
      B.Set_valid();
      int k=boundingbox->bounding_boxes[i].id;
      if(k==0||k==1||k==2)B.Set_id(0);
      else B.Set_id(1);
      if(B.pull_bid()==0&&B.pull_valid()==1){
        bbox_t.push_back(B);
      }else if(B.pull_bid()==1&&B.pull_valid()==1){
        bbox_v.push_back(B);
      }else{
        bbox_t.clear();
        bbox_v.clear();
      }

  }

  Command_msg();

  /*for(int i=0;i<bbox_t.size();i++){
    bbox_t[i].Test_print();
    cout<<endl;
  }
  for(int i=0;i<bbox_v.size();i++){
    bbox_v[i].Test_print();
    cout<<endl;
  }*/
  bbox_t.clear();
  bbox_v.clear();
  box_count=0;
  cout<<"\033[2J\033[1;1H"; //clear terminal
}
void boundingbox_callback_test(const darknet_ros_msgs::BoundingBoxesConstPtr& boundingbox)
{
  for(int i=0;i<boundingbox->bounding_boxes.size();i++){
    int k=boundingbox->bounding_boxes[i].id;
    if(k==0||k==1||k==2)cout<<"0";
    else cout<<"1";
    cout<< endl;
  }
}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "sub_box_command");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("/darknet_ros/bounding_boxes", 1000, Algorithm_main); //choose [boundingbox_callback_test,Algorithm_main]

  ros::spin();

  return 0;
}
