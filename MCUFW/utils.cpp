#include "utils.h"
#include "Arduino.h"

void print_frame(char frame[], char len){
  for(char i = 0; i < len; i++){
    Serial.print(frame[i], HEX);
    Serial.print(' ');
  }
  Serial.println();
}
