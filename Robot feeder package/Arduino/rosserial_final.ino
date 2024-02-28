#include <ros.h>
#include <SoftwareSerial.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>


ros::NodeHandle nh;
std_msgs::String sys_init_msg;  // Variable type declaration
std_msgs::String no_signal;
std_msgs::String plate_selected;
std_msgs::String systemstatus;
std_msgs::Bool sys_init_state;
std_msgs::Int16 food_selection_ard;
std_msgs::Int16 choice1;
std_msgs::Int16 choice2;
std_msgs::Int16 choice3;
std_msgs::Int16 choice4;
std_msgs::Int16 choice5;


byte com = 0; // Declaration of com variable which is used to perform serial communication with the voice module
bool sys_init;
int buttonValue;
int readVal;
SoftwareSerial mySerial(2,3); // Declaration of SoftwareSerial ports


char ns[30] = "No signal";  // Messages to appear in terminal
char ps[30] = "Plate selected...";


ros::Publisher foodSelect("food_selection_arduino", &food_selection_ard); // Initialization of publishers
ros::Publisher systemStatus("system_status_arduino", &systemstatus);


void messageCb3( const std_msgs::Bool& sys_init_state){
  if (sys_init_state.data == true){
    sys_init = true;
  }
  else{
    sys_init =false;
  }
}
ros::Subscriber<std_msgs::Bool> systemInit("system_init_arduino", &messageCb3 );


void setup()
{ 
  Serial.begin(9600);
  mySerial.begin(9600);     // Method to set baudrate
  nh.getHardware()->setBaud(9600);  // Method to set baudrate
  
  pinMode(5, INPUT);    // Analogue A0 Pin
  pinMode(2, INPUT);    // SoftwareSerial TX Pin
  pinMode(3, OUTPUT);   // SoftwareSerial RX Pin
  
  nh.initNode();
  nh.advertise(chatter);
  nh.advertise(foodSelect);
  nh.advertise(systemStatus);
  nh.subscribe(systemInit);

  mySerial.write(0xAA); // Initialization of voice module
  mySerial.write(byte(0x00)); // This line and the previous line adds up to AA00, which makes the voice module enter waiting mode
  delay(100);
  mySerial.write(0xAA);
  mySerial.write(0x37); // This line and previous line adds up to AA37, which changes the voice module to compact mode
  delay(100);
  mySerial.write(0xAA);
  mySerial.write(byte(0x00)); // This line and the previous line adds up to AA00, which makes the voice module enter waiting mode
  delay(100);
  mySerial.write(0xAA);
  mySerial.write(0x21); // This line and previous line adds up to AA21, which imports group 1 of the 3 groups of voice commands recorded
  delay(100);
}

void loop()
{  
  no_signal.data = ns;  // Storing of texts inside the string variable no_signal
  plate_selected.data = ps; // Storing of texts inside the string variable plat_selected
  
  choice1.data = 1; // Declaring the int16 variable choice1 as the value 1
  choice2.data = 2; // Declaring the int16 variable choice2 as the value 2
  choice3.data = 3; // Declaring the int16 variable choice3 as the value 3
  choice4.data = 4; // Declaring the int16 variable choice4 as the value 4
  choice5.data = 5; // Declaring the int16 variable choice5 as the value 5
  
  if (sys_init == true ){
    while (sys_init == true){ // Constantly read for any inputs from the control box which contains buttons and microphone
      readVal = analogRead(A0); // Read button input analog values
      com = mySerial.read();    //  Read SoftwareSerial's pins
      if ((com == 0x11) || (readVal>=100 && readVal<=150)){   // If the word "one" is heard by the voice module or the button which is linked to the resistor producing an output between 100 and 150 is pressed
        foodSelect.publish( &choice1); // Publish a int value to the topic /food_selection_arduino
        systemStatus.publish( &plate_selected ); // Publish the a string to the /topic system_status_arduino
        sys_init = false; // Reset rosserial system_status as false so the control box will not be active during other executions of the feeding process
        continue;
      }
      else if ((com == 0x12) || (readVal>=300 && readVal<=370)){  // If the word "two" is heard by the voice module or the button which is linked to the resistor producing an output between 300 and 370 is pressed
        foodSelect.publish( &choice2); //food_selection_arduino
        systemStatus.publish( &plate_selected ); //system_status_arduino
        sys_init = false;
        continue;
      }
      else if ((com == 0x13) || (readVal>=420 && readVal<=470)){  // If the word "three" is heard by the voice module or the button which is linked to the resistor producing an output between 420 and 470 is pressed
        foodSelect.publish( &choice3); //food_selection_arduino
        systemStatus.publish( &plate_selected ); //system_status_arduino
        sys_init = false;
        continue;
      }
      else if ((com == 0x14) || (readVal>=610 && readVal<=660)){  // If the word "four" is heard by the voice module or the button which is linked to the resistor producing an output between 610 and 660 is pressed
        foodSelect.publish( &choice4); //food_selection_arduino
        systemStatus.publish( &plate_selected ); //system_status_arduino
        sys_init = false;
        continue;
      }    
      else if ((com == 0x15)){  // If the word rice is heard by the voice module
        foodSelect.publish( &choice5 ); //food_selection_arduino
        systemStatus.publish( &plate_selected ); //system_status_arduino
        sys_init = false;
        continue;
      }
      nh.spinOnce();
      delay(10);
    }
  }

  else{
    systemStatus.publish( &no_signal );
    delay(1000);
  }
  
  nh.spinOnce();
}
