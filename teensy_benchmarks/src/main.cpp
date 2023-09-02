/**
 * Blink
 *
 * Turns on an LED on for one second,
 * then off for one second, repeatedly.
 */
#include "Arduino.h"

#include <stdio.h>
#include "osqp.h"
#include "mpc_mat_workspace.h"

#ifndef LED_BUILTIN
#define LED_BUILTIN 13
#endif

void setup()
// int main()
{
  initialize LED digital pin as an output.
  pinMode(LED_BUILTIN, OUTPUT);

  // start serial terminal
  Serial.begin(9600);
  while(!Serial){ // wait to connect
    continue;
  }

  OSQPInt exitflag;

  printf( "Embedded test program for vector updates.\n");
  
  unsigned long start = micros();
  exitflag = osqp_solve( &mpc_mat_solver );
  unsigned long end = micros();
  Serial.println(end-start);


  if( exitflag > 0 ) {
    // printf( "  OSQP errored: %s\n", osqp_error_message(exitflag));
    Serial.println("ERROR");
  } else {
    // printf( "  Solved workspace with no error.\n" );
    Serial.println("Success!");
  }
}

void loop()
{
  // turn the LED on (HIGH is the voltage level)
  digitalWrite(LED_BUILTIN, HIGH);

  // wait for a second
  delay(100);

  // turn the LED off by making the voltage LOW
  digitalWrite(LED_BUILTIN, LOW);

   // wait for a second
  delay(100);
}