#include <Pixy2.h>

Pixy2 pixy;
unsigned long xPos;
unsigned long yPos;
unsigned long height;

String goLeft = "0";
String goRight = "1";
String goForward = "2";

void setup() {
  Serial.begin(9600);
  pixy.init();
  pixy.setLamp(0,0);
}

void loop() {
  // put your main code here, to run repeatedly:
  pixy.ccc.getBlocks(true, 1, 10);
  if (pixy.ccc.numBlocks){
    xPos = pixy.ccc.blocks[0].m_x;
    if (xPos <= 140) {
      Serial.print(goLeft); //ball left
    } else if (xPos >= 170) {
      Serial.print(goRight); //ball right
    } else {
      Serial.print(goForward); //ball center
    }
  }
  Serial.flush();
}
