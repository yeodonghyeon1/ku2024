#define ip1 3 //1->3pin
#define ip2 5 // 2->5pin
#define ip3 6 // 3->6pin
//#define ip4 9 // 4->9pin
#define ip5 10 // 5->10pin
#define ip6 11 // 6->10pin

#include <Servo.h>
#include <ros.h>
#include <std_msgs/UInt16.h>
#include <Adafruit_NeoPixel.h>

ros::NodeHandle nh;

int ch1; 
int ch2; 
int ch3;
int ch5; 
int ch6;
int val1;
int val2;
int relaypin = 10;

int rpin = 22;
int gpin = 24;
int bpin = 26;

int neoPixelPin = 18;
int numPixels = 144;
Adafruit_NeoPixel strip = Adafruit_NeoPixel(numPixels, neoPixelPin, NEO_GRB + NEO_KHZ800); // Fixed this line

void setColor(int r, int g, int b) {
  for (int i = 0; i < numPixels; i++) {
    strip.setPixelColor(i, r, g, b);
  }
  strip.show();
}

Servo thruster1;
Servo thruster2;

void thruster_l(const std_msgs::UInt16& cmd_msg) {
  thruster1.writeMicroseconds(cmd_msg.data); // 1100-1900
}

void thruster_r(const std_msgs::UInt16& cmd_msg) {
  thruster2.writeMicroseconds(cmd_msg.data); // 1100-1900
}

ros::Subscriber<std_msgs::UInt16> sub1("thrusterL", thruster_l);
ros::Subscriber<std_msgs::UInt16> sub2("thrusterR", thruster_r);

void setup() {
  strip.begin();
  strip.show();
  strip.clear();

  pinMode(rpin, OUTPUT);
  pinMode(gpin, OUTPUT);
  pinMode(bpin, OUTPUT);

  nh.initNode();
  nh.subscribe(sub1);
  nh.subscribe(sub2);

  pinMode(ip1, INPUT);
  pinMode(ip2, INPUT);
  pinMode(ip3, INPUT);
  pinMode(ip5, INPUT);
  pinMode(9, INPUT);
  pinMode(8, INPUT);
  pinMode(relaypin, INPUT); // Changed led to relaypin

  thruster1.attach(13); // left L
  thruster2.attach(12); // right R

  thruster1.writeMicroseconds(1500);
  thruster2.writeMicroseconds(1500);

  delay(7000);
}

void loop() {
  Serial.println(ch1);
  ch5 = pulseIn(ip5, HIGH);

  if (ch5 < 1450) {
    digitalWrite(8, HIGH);
    digitalWrite(9, LOW);
    autonomous();
    blinkGreen();
  } else if (ch5 > 1450) {
    digitalWrite(8, LOW);
    digitalWrite(9, HIGH);
    rc();
    blinkRed();
  }
}


void blinkGreen(){
  digitalWrite(gpin, HIGH);
  delay(100);
  digitalWrite(gpin, LOW);
  delay(100);
}

void blinkRed(){
  digitalWrite(rpin, HIGH);
  delay(100);
  digitalWrite(rpin, LOW);
  delay(100);
}
