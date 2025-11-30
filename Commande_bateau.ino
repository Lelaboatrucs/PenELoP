#include <RPLidar.h>
#define RPLIDAR_MOTOR 3 // The PWM pin for control the speed of RPLIDAR's motor (MOTOCTRL).

RPLidar lidar;

float minDistance = 100000;
float angleAtMinDist = 0;
float xRef=1000;
float thetaRef=270;
float Kex=0.18;
float Ketheta=4;
float ex= 0;
float etheta= 0;
float alpha= 0;
float prevalpha= 0;

// sercomoteur 

#include <Servo.h>
Servo myServo;
int entalpha = (int)alpha+90;   


void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);  // For RPLidar
  lidar.begin(Serial1);
  pinMode(RPLIDAR_MOTOR, OUTPUT);  // set pin modes

  myServo.attach(9);
myServo.write(entalpha);
}



void loop() {
  if (IS_OK(lidar.waitPoint())) {
    //perform data processing here... 
    float distance = lidar.getCurrentPoint().distance;
    float angle = lidar.getCurrentPoint().angle;  // 0-360 deg
    
    if (lidar.getCurrentPoint().startBit) {
       // a new scan, display the previous data...
       printData2(angleAtMinDist, minDistance);
       minDistance = 100000;
       angleAtMinDist = 0;
    } else {
       if ( distance > 0 &&  distance < minDistance) {
        if ( angle > 225 &&  angle < 315 ){
          minDistance = distance;
          angleAtMinDist = angle;
          }
          
       }
    }
  }
  else {
    analogWrite(RPLIDAR_MOTOR, 255); //stop the rplidar motor
    // Try to detect RPLIDAR
    rplidar_response_device_info_t info;
    if (IS_OK(lidar.getDeviceInfo(info, 100))) {
       // Detected
       lidar.startScan();
       analogWrite(RPLIDAR_MOTOR, 255);
       delay(1000);
    }
  }
}

void printData(float angle, float distance)
{

  Serial.print("dist: ");
  Serial.print(distance);
  Serial.print("  angle: ");
  Serial.println(angle);
}
void printData2(float angle, float distance)
{
  // calcul erreurs
ex= xRef-distance;
etheta=thetaRef-angle;
  // calcul alpha
alpha=Kex*ex-Ketheta*etheta;

//saturation alpha
if (alpha>=90){
  alpha=90;
}
if (alpha<=-90){
  alpha=-90;
}
//filtrage alpha
alpha=0.2*alpha+0.8*prevalpha;
prevalpha=alpha;
entalpha = ((int)alpha)+90;  
  Serial.print("dist: ");
  Serial.print(distance);
  Serial.print("  angle: ");
  Serial.print(angle);
  Serial.print("  ex: ");
  Serial.print(ex);
  Serial.print("  etheta: ");
  Serial.print(etheta);
  Serial.print("  alpha: ");
  Serial.println(alpha);
  myServo.write(entalpha);

}