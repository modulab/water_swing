
// Include the library
#include <AS5X47.h>

// Define where the CSN Pin in connected. 
int leagan1SelectPin = 9;
int leagan2SelectPin = 10;

int valvaGenerala1Pin = 8;
int valvaGenerala2Pin = 6;
int valvaGenerala3Pin = 4;
int valvaLeagan1Pin = 7; // de modificat
int valvaLeagan2Pin = 5; // de modificat

int pot1 = A0;

long idleTime =10 * 1000;
long lastIdle = 0;

float degreesConversionConstant = 180/3.14;
float radianConversionConstant = 3.14/180;
float g =9.81;
float L = 3.0;
float Lcap = 2.0;
float th = sqrt(2 * Lcap/g);

float Lp = 3.0;
float thp = sqrt(2 * Lp / g);
float perioadaT = 2 * 3.14 * sqrt(L / g);
float omega = sqrt(g / L);


void checkIdle(){
  if (millis()-lastIdle > idleTime) {
      digitalWrite(valvaGenerala1Pin, LOW);
      digitalWrite(valvaGenerala2Pin, LOW);
      digitalWrite(valvaGenerala3Pin, LOW);
  } else {
      //digitalWrite(valvaGenerala1Pin, HIGH);
      //digitalWrite(valvaGenerala2Pin, HIGH);
      //digitalWrite(valvaGenerala3Pin, HIGH);
  }
}

class Leagan {
    public:
    //0 - unknown
    //1 - dreapta
    //-1 stanga
    int directie = 0; 
    boolean calibrating = true;
    boolean first = false;
    float verticalAngle;
    float unghi;

    int indexCalibrate = 0;
    
    float phi0;
    float phiFuture;

    float delta;
    float lastDelta=0;
    
    float lastVal;
    float phiFutureDegrees = 0;
    float distanceAngle = 0;

    int valvaPin;

    Leagan(int pin) {
      valvaPin = pin;
    }

    void calibrate(float unghi) {

      if (abs(lastVal-unghi)>1) {    
        calibrating = true;
        indexCalibrate = 0;
        lastVal = unghi;
        return;
      }
      
      if (indexCalibrate == 20) {
        calibrating = false;
        verticalAngle = unghi;
        Serial.println("Calibratig done!");
      }
      indexCalibrate++;  
    }
      
 

  float lowpass(float x, float y_1)
  {
    float alpha = 0.1;
    float y = y_1 + alpha * (x - y_1);
    return y;
    //return x;
  }


 
  void go(float unghi) {
      float phi0radian = 0;
      
      if (!first) {
        lastVal = unghi;
        first = true;      
      } else {
        
        //salt and pepper noise
        delta = unghi - lastVal;
        if (abs(delta) > 20) { //salt and pepper noise
          //unghi = lastVal;
        }
        
        unghi = lowpass(unghi, lastVal);
        
        //if (valvaPin==valvaLeagan1Pin) {
        //  Serial.print(String(unghi)+",");
        //} else {
        //  Serial.println(String(unghi));
        //}
        
        //delta = abs(unghi) - abs(lastVal);
        delta = unghi - lastVal;

        float max =0;

        float semn = delta * lastDelta;
        //if ((semn < 0) && (abs(unghi)>3)) {  // SE SCHIMBA SEMN
        if (semn < 0) {  // SE SCHIMBA SEMN
          
            if (delta > 0) {
              directie = 1;
            }
    
            if (delta < 0) {
              directie = -1;
            }
        }

        // phi0 maxim se updateaza cand se gaseste maxim sau unghiul a depasit vechiul maxim
       // if (((semn < 0)) || (abs(phi0) < abs(unghi))) {
        if (semn < 0) {
  
          
          phi0 = unghi;
          phi0radian = phi0 * radianConversionConstant;
          phiFuture = phi0radian * cos(omega*thp);
          
  
          //if (abs(phi0) < 2) {
          //    directie = 0;
          //}

          phiFutureDegrees = phiFuture * degreesConversionConstant;
          distanceAngle = abs(phi0) - abs(phiFutureDegrees);
          
        } // END SE SCHIMBA SEMN


        // phi0 e maximul calculat intr-o parte. dar se va folosi si in partea cealalta. 
        // asa ca trebuie sa i se schimbe semnul
        if (unghi>0) {
          phi0 = abs(phi0);
        } else {
          phi0 = -abs(phi0);
        }

        if (valvaPin==valvaLeagan1Pin) {
           Serial.print(String(distanceAngle)+",");
          //Serial.println(String(directie));
          Serial.println(String(abs(phi0 + directie*(abs(phi0)-abs(unghi))) - distanceAngle));
        } 
        //else {
        //  Serial.println(String(unghi));
        //}

        //if ((abs(phi0)-directie*unghi) + abs(phi0) - distanceAngle > 10 ) {

        int val = analogRead(pot1);
        val = map(val, 0, 1023, 0, 2);
        
        if (( abs(phi0 + directie*(abs(phi0)-abs(unghi))) - distanceAngle < 3+val ) && (abs(phi0)>10)){
             digitalWrite(valvaPin, HIGH);
            //Serial.println("ON");
        } else {
            digitalWrite(valvaPin, LOW);
            //Serial.println("OFF");
        }
        
        lastDelta = delta;        
        lastVal = unghi;
      }
      
  }
 
  void debug_directie()
  {
    if (directie == 0) {
      Serial.println("unknown");
    }
    if (directie == 1) {
      Serial.println("dreapta");
    }
    if (directie == -1) {
      Serial.println("stanga");
    }  
  }

};


Leagan leagan1(valvaLeagan1Pin);
Leagan leagan2(valvaLeagan2Pin);


AS5X47 xx = AS5X47(leagan1SelectPin);
AS5X47 xx1 = AS5X47(leagan2SelectPin);


void scoateAer(int valva, int t) {
  digitalWrite(valva, HIGH);
  delay(t);
  digitalWrite(valva, LOW);
}


void setup() {
  pinMode(valvaGenerala1Pin, OUTPUT);
  pinMode(valvaGenerala2Pin, OUTPUT);
  pinMode(valvaGenerala3Pin, OUTPUT);
  
  pinMode(valvaLeagan1Pin, OUTPUT);
  pinMode(valvaLeagan2Pin, OUTPUT);

  digitalWrite(valvaGenerala1Pin, LOW);
  digitalWrite(valvaGenerala2Pin, LOW);
  digitalWrite(valvaGenerala3Pin, LOW);
  
  digitalWrite(valvaLeagan1Pin, LOW);
  digitalWrite(valvaLeagan2Pin, LOW);
  Serial.begin(9600);

  //scoateAer(valvaGenerala1Pin, 5000);
  //scoateAer(valvaLeagan1Pin, 5000);
  //scoateAer(valvaGenerala2Pin, 5000);
  //scoateAer(valvaLeagan2Pin, 5000);
  //scoateAer(valvaGenerala3Pin, 5000);
  
}


void loop() {
  //
  //selecteaza encoder leagan1
  //
  //digitalWrite(leagan2SelectPin, LOW);
  //gitalWrite(leagan1SelectPin, HIGH);

  float angle = xx.readAngle();
//  Serial.print(String(angle)+",");

  checkIdle();

  if (leagan1.calibrating) {
    leagan1.calibrate(angle);
  }
  else
  {
    float unghiRel = angle - leagan1.verticalAngle;
    leagan1.go(unghiRel);
    if (abs(unghiRel)>3) {
      lastIdle = millis();
    }
    //leagan1.debug_directie();
  }




  angle = xx1.readAngle();
 // Serial.println(angle);

  if (leagan2.calibrating) {
    leagan2.calibrate(angle);
  }
  else
  {
    float unghiRel = angle - leagan2.verticalAngle;
    leagan2.go(unghiRel);
    if (abs(unghiRel)>3) {
      lastIdle = millis();
    }
    //leagan2.debug_directie();
  }

  
  delay(50);
}
