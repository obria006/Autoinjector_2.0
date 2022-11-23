/*
This code handles serial communication between the arduino and the computer, and
it controls the voltage commands sent to the electronic pressure controller and
solenoid valves.

This code is updated from the original Autoinjector v1.0 `pythonarduinotriggeropen.ino`.
This doesn't include the exact same code that was in Autoinjector v1.0. However, from the
best of my knowledge, the code that I removed from Autoinjector 1.0 wasn't even functionally
used by Autoinjector 1.0. Therefore, I believe this code is sufficient for Autoinjector 1.0
and Autoinjector 2.0 while reducing bloat.
*/


// VARIABLES NECESSARY FOR AUTOINJECTOR 2.0
String readString; // String command from computer
String bpstring, bpval, bp1, bp2, outputbp; // Parsed strings for backpressure
String purgestring; // Strings for purging pressure
int bpvolt; // digital backpressure value command
bool lvOpen, hvOpen; // Indicators of low/high pressure valve open


void setup() {
  /* Starts serial communication and declares pin modes for valves and
  electronic pressure controller */
  Serial.begin(9600);
  pinMode(11, OUTPUT); // Electronic pressure controller
  pinMode(2, OUTPUT); // Low pressure solenoid valve
  pinMode(3, OUTPUT); // High pressure solenoid valve
  
}


void valveLowOpen() {
  /* Open the low pressure valve */
  // Serial.println("Opening low pressure valve");
  digitalWrite(2,HIGH); // open valve
  lvOpen = true;
}


void valveLowClose() {
  /* Close the low pressure valve */
  // Serial.println("Closing low pressure valve");
  digitalWrite(2,LOW); // close valve
  lvOpen = false;  
}


void valveHighOpen() {
  /* Open the high pressure valve */
  // Serial.println("Opening high pressure valve");
  digitalWrite(3,HIGH); // open valve
  hvOpen = true;
}


void valveHighClose() {
  /* Close the high pressure valve */
  // Serial.println("Closing high pressure valve");
  digitalWrite(3,LOW); // close valve
  hvOpen = false;
}


void parseToBackpressure() {
  /* Parses string from serial communication to a digital value that is sent
  to the pressure controller. Backpressure commands are encapuslated by 'c' and
  'C'. Example: bc100C will be parsed to send 100 to the pressure controller */

  // Read backpressure value from serial string between 'c' and 'C'
  bp1 = readString.indexOf("c");
  bp2 = readString.indexOf("C");
  outputbp = readString.substring(bp1.toInt()+1,bp2.toInt());

  // Convert from string to integer and send to pressure contoller
  bpvolt = outputbp.toInt();
  analogWrite(11,bpvolt);

  // Open the solenoid valve if pressure > 0
  if (bpvolt > 0){
    valveLowOpen();
  }
  else {
    delay(500); // Wait for pressure to equilibrium to 0 before closing valve
    valveLowClose();
  }
}


void purgePressure() {
  /* ONLY FOR AUTOINJECTOR 2.0

  Cycles low pressure valve off and high pressure valve on to apply a pulse
  of high pressure in attempt to unclog pipette. */
  bool isOpen = lvOpen;
  valveLowClose();
  delay(200);
  valveHighOpen();
  delay(500);
  valveHighClose();
  delay(400);
  if (isOpen == true){ // Reopen valve only if was open to begin with
    valveLowOpen();
  }
}


void loop() {
  /* Main loop for controlling pressure control system. Continuously loops
  until data is received over Serial. When serial data available, reads in the
  entirety of the data and appends data to a String. Then depending on the
  content of the string it takes different actions. Loop will only handle 1
  command at a time. For instance, if you send 2 commands in the same string,
  then only the first command will be handled - the second command will be ignored.
  Specifically, it will only handle commands where the first character of the
  entire string matches a specified letter. Backpressure must start with 'b', and
  purge must start with 'x'.
  
  The loop expects strings to take a certain form:
    backpressure: sets a constant backpressure for keeping unclogged during
    experiment. Strings should be in form of "bc___C". 'b' is required to be
    the first character which indicates that the command is backpressure. Then
    the string expects the digital pressure value to be contained between 'c'\
    and 'C'. Example "bc100Cpressure!100!" will send 100 to the pressure
    controller. 

    purge: AUTOINJECTOR 2.0 ONLY. string must start with 'x'. If the first
    character of the string is 'x' then arduino will cycle valves to attempt
    unclogging.
  */

  //runs when no data is here
  while (!Serial.available()) {
    } 

  //when something is being sent... buffer is 30ms
  while (Serial.available()) {
    delay(30);

    //interprets serial info
    if (Serial.available() > 0) {
      char c = Serial.read();
      readString += c;
    }
  }

  // if information is sent, print recieved data
  if (readString.length() > 0) {
    Serial.print("Arduino recieved: ");
    Serial.println(readString);

    // finds the index of the command charcaters for backpressure, purge, and trigger
    bpstring = readString.indexOf("b");
    purgestring = readString.indexOf("x");

    // ONLY RUN COMMAND IN FIRST CHARACTER SLOT
    // send pressure command to electronic pressure controller
    if (bpstring == "0") {
      parseToBackpressure();
    }
    // Cylces low/high pressure valves to try unclog pipette
    if (purgestring == "0") {
      purgePressure();
    }

  // Delay in loop to cycle every 100ms
  delay(100);

  //clear strings
  readString = ""; 
  bpstring = "";
  purgestring = "";
  }
}
