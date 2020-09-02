#include <ros.h>
#include <std_msgs/String.h>

ros::NodeHandle nh;

int run_once = 0;
std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);

char target[3] = "go";

void messageCb( const std_msgs::String& toggle_msg){
  digitalWrite(13, HIGH-digitalRead(13));
  str_msg.data = toggle_msg.data;
  chatter.publish( &str_msg );
  if (strcmp(toggle_msg.data,"go")==0){
    run_once=1;
  }
}

ros::Subscriber<std_msgs::String> sub("toggle_led", &messageCb );

int res = 0;
int go_time = 200;
int pause_time = 1000;

void setup() {
  nh.initNode();
  nh.advertise(chatter);
  nh.subscribe(sub);
  pinMode(13, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(12, OUTPUT);
  pinMode(7, INPUT_PULLUP);
  pinMode(8, INPUT_PULLUP);
}

void forward(){
  digitalWrite(11, LOW);
  digitalWrite(12, HIGH);   
  for (int i = 0; i < go_time; i++) {
    res = digitalRead(7);
    if(res==0){
      break;
    }
    delay(1);
  }
  digitalWrite(11, LOW);   
  digitalWrite(12, LOW); 
}

void backward(){
  digitalWrite(11, HIGH);   
  digitalWrite(12, LOW);   
  for (int i = 0; i < go_time; i++) {
    res = digitalRead(8);
    if(res==0){
      break;
    }
    delay(1);  
  }
  digitalWrite(11, LOW);   
  digitalWrite(12, LOW);   
}

// the loop function runs over and over again forever
void loop() {
  nh.spinOnce();
  if (run_once==1) {
    run_once=0;
    for (int i = 0; i < 1; i++) {
      forward();
      delay(pause_time);
      backward();
      delay(pause_time);
    }
  }
  delay(100);
}
