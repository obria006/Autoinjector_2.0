/*
This code handles serial communication between the arduino and the computer, and
it controls the voltage commands sent to the electronic pressure controller and
solenoid valves.

This code is updated from the original Autoinjector v1.0 `pythonarduinotriggeropen.ino`.
This version of the code is considered LEGACY because it contains code from the
original Autoinjector v1.0 `pythonarduinotriggeropen.ino` file for backwards 
compatability. To the best of my knowledge, the code from Autoinjector 1.0 isn't
actually necessary even in the Autoinjector 1.0 version. For instance, I don't believe
the `tstring` is used for "triggering" in Autoinjector 1.0 (I couldn't find any python
code for Autoinjector 1.0 that sends a `tstring` to the arduino). Likewise, the pressure
command "pressure!___!" I don't believe does anything functionally except print values.
This "pressure!___!" section of arduino code does set the `apv` variable which is used in
the `tstring` section... but like I said the `tstring` section is never accessed by the
arduino code. So to the best of my knowledge this version of arduino code isn't necessary
but I've included it just to be safe.
*/

// VARIABLES NECESSARY FOR AUTOINJECTOR 2.0
String readString; // String command from computer
String bpstring, bpval, bp1, bp2, outputbp; // Parsed strings for backpressure
String purgestring; // Strings for purging pressure
int bpvolt; // digital backpressure value command
bool lvOpen, hvOpen; // Indicators of low/high pressure valve open

// VARIABLES FOR AUTOINJECTOR 1.0 COMPATABLIITY
String pstring, val, v1, v2, output; // pressure strings from AUTOINJECTOR 1.0
String tstring,ninject_index1,ninject_index2,ninjectstring; // trigger strings from AUTOINJECTOR 1.0
String pulsewidth_index1, pulsewidth_index2, pulsewidthstring; // trigger pulse strings from AUTOINJECTOR 1.0
int apv, ninject,pulsewidth,n; //AINJ 1.0. inject pressure volt,num inject,pulsewidth of inject, conuter


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
  Serial.println("Opening low pressure valve");
  digitalWrite(2,HIGH); // open valve
  lvOpen = true;
}


void valveLowClose() {
  /* Close the low pressure valve */
  Serial.println("Closing low pressure valve");
  digitalWrite(2,LOW); // close valve
  lvOpen = false;  
}


void valveHighOpen() {
  /* Open the high pressure valve */
  Serial.println("Opening high pressure valve");
  digitalWrite(3,HIGH); // open valve
  hvOpen = true;
}


void valveHighClose() {
  /* Close the high pressure valve */
  Serial.println("Closing high pressure valve");
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

  // FROM AUTOINJECTOR 1.0 - UNNECESSARY AND COULD BE DELETED
  // Reads pressure value between !__! and prints value
  v1=readString.indexOf("!");
  v2=readString.lastIndexOf("!");
  val=readString.substring(v1.toInt()+1,v2.toInt());
  apv=val.toInt();
  output = apv;
  delay(50);
  Serial.print("Compensation pressure set to:");
  Serial.println(outputbp);
  Serial.print("Injection pressure set to:");
  Serial.println(output);
  delay(50);
}


void parseToTrigger() {
  /* AUTOINJECTOR 1.0 ONLY - UNNECESSARY AND COULD BE REMOVED (EVEN IN 1.0)
  
  Parses the string from serial to extract the number of pulses and duration
  then sends pulse commands to the electronic pressure controller. Number of 
  injections encapsulated between 'n__N' and pulse duration encapsulated
  between 'w__W'. Example: "n5Nw100w means 5 pulses at 100ms.*/

  n = 0;
  // ninject for number of injections ex. "n06N" means 6 injections
  // pulsewidth for pulse width time of triggering in ms ex. "w100W" means 100ms

  // Parse string between "n__N" for number of pulses
  ninject_index1 = readString.indexOf("n");
  ninject_index2 = readString.indexOf("N");
  ninjectstring = readString.substring(ninject_index1.toInt()+1,ninject_index2.toInt());
  ninject = ninjectstring.toInt();

  // Parse string between "w__W" for duration of pusles
  pulsewidth_index1 = readString.indexOf("w");
  pulsewidth_index2 = readString.indexOf("W");
  pulsewidthstring = readString.substring(pulsewidth_index1.toInt()+1,pulsewidth_index2.toInt());
  pulsewidth = pulsewidthstring.toInt();

  Serial.print("(pulsewidth,ninject) = ");
  Serial.print("(");
  Serial.print(pulsewidth);
  Serial.print(",");
  Serial.print(ninject);
  Serial.println(")");
  
  // Run the pressure pulses while returning to backpressure between pulse
  while (n < ninject) {
    analogWrite(11, apv); // inject at full pressure
    delay(pulsewidth);
    analogWrite(11, bpvolt); // stop injection but still have compensation pressure to prevent backflow
    
    output = "trigger" +String(n);
    n = n + 1;
    Serial.print("injected ");
    Serial.println(n);
  }
  Serial.println("Injection Complete");
}

void purgePressure() {
  /* ONLY FOR AUTOINJECTOR 2.0

  Cycles low pressure valve off and high pressure valve on to apply a pulse
  of high pressure in attempt to unclog pipette. */
  bool isOpen = lvOpen;
  valveLowClose();
  delay(250);
  valveHighOpen();
  delay(500);
  valveHighClose();
  delay(250);
  if (isOpen == true){
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
  entire string matches a specified letter. Backpressure must start with 'b', purge
  must start with 'x', and trigger must start with 't'.
  
  The loop expects strings to take a certain form:
    backpressure: sets a constant backpressure for keeping unclogged during
    experiment. Strings should be in form of "bc___Cpressure!___!". 'b' is
    required to be the first character which indicates that the command is
    backpressure. Then the string expects the digital pressure value to be
    contained between 'c' and 'C' and '!' and '!'. Example "bc100Cpressure!100!"
    will send 100 to the pressure controller. Realistically, the second half of
    the string ("pressure!___!") is unnecessary as it doesn't impact the
    pressure value being sent - it only is used to print the value. I would
    remove this code, but it is kept for compatability with AUTOINJECTOR 1.0.

    purge: AUTOINJECTOR 2.0 ONLY. string must start with 'x'. If the first
    character of the string is 'x' then arduino will cycle valves to attempt
    unclogging.

    trigger: AUTOINJECTOR 1.0 ONLY. KEPT FOR BACK COMPATABLIITY. I think this
    could be removed because I don't think it's used even in AUTOINJECTOR 1.0, but
    i'm keeping it just in case. Trigger sends n pressure pulses with duration
    of w. Trigger expects a string in the form of: "tn___Nw___W". The number of
    pulses is contained between 'n' and 'N' and the duration in ms of the pulse
    is contained between 'w' and 'W'. Example: "tn5Nw100W" will apply 5 pulses of
    100ms.
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
    Serial.println("Arduino recieved:");
    Serial.println(readString);

    // finds the index of the command charcaters for backpressure, purge, and trigger
    bpstring = readString.indexOf("b");
    purgestring = readString.indexOf("x");
    tstring = readString.indexOf("t");

    // ONLY RUN COMMAND IN FIRST CHARACTER SLOT
    // send pressure command to electronic pressure controller
    if (bpstring == "0") {
      parseToBackpressure();
    }
    // Cylces low/high pressure valves to try unclog pipette
    if (purgestring == "0") {
      purgePressure();
    }
    // Runs series of pressure pulses
    if (tstring == "0"){
      parseToTrigger();
    }

  // Delay in loop to cycle every 100ms
  delay(100);

  //clear strings
  readString = ""; 
  bpstring = "";
  purgestring = "";
  ninject = 0;
  tstring = "";
  }
}
