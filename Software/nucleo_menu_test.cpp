#include "mbed.h"
#include "C12832.h"

typedef enum {initialisation, home, commands, commandSent, cameraViewState, sensorDataState, debug, nextState} ProgramState;

class Joystick {                                                        //Begin Joystick class definition
    
private:                                                                //Data member declaration
    DigitalIn up, down, left, right, fire;                              //Declaration of DigitalIn objects
    
public:
    Joystick (PinName u, PinName d, PinName l, PinName r, PinName f) :  //Constructor
        up(u), down(d), left(l), right(r), fire(f){}                    //Pins assigned to each direction
    
    bool upPressed(void){                                               //Member functions return value of joystick pin
        return up;
    }
    
    bool downPressed(void){
        return down;
    }
    
    bool leftPressed(void){
        return left;
    }
    
    bool rightPressed(void){
        return right;
    }
    
    bool firePressed(void){
        return fire;
    }
};

class Controller 
{
    ProgramState currentState;
    int commandCounter;

public:
    Controller() {init();} //Constructor

    void init(){
        currentState = initialisation;
        commandCounter = 0;
    }

    void changeState (ProgramState state){
    	currentState = state;
    }

    ProgramState returnState(){
      	return currentState;
    }

    void incrementCommandCounter (){
        commandCounter++;
    }

    void decrementCommandCounter(){
        commandCounter--;
    }
    
    int returnCommandCounter(){
        return commandCounter;
    }

    void sendCommand(char* command){
        incrementCommandCounter();
        ; //send necessary command
    }

    void cancelCommand(){
        decrementCommandCounter();
    }
};

int currentOption = 0;
const char* homeOptions[4] = {"Commands", "Camera View", "Sensor Data", "Debug"};
const char* commandOptions[6] = {"Move", "Return", "Pick Up/Put Down", "Attack", "CANCEL", "BACK"};
const char* sensorList[3] = {"GPS", "Temp", "Pressure"};
float GPSData = 52;
float tempData = 20.7;
float barData = 1.02;
float sensorData[3] = {GPSData, tempData, barData};

Controller controller();


void homeOptionUpISR()
{
    if (currentOption == 0)
    {
        currentOption = sizeof(homeOptions)-1;
    }
    else
    {
        currentOption--;
    }

}

void homeOptionDownISR()
{
    if (currentOption == sizeof(homeOptions-1))
    {
        currentOption = 0;
    }
    else
    {
        currentOption++;
    }
}

void homeOptionSelectISR()
{
    controller.changeState(nextState);
}

void commandOptionUpISR()
{
    if (currentOption == 0) 
    {
        currentOption = sizeof(commandOptions)-1;
    }
    else
    {
        currentOption--;
    }

}

void commandOptionDownISR()
{
    if (currentOption == sizeof(commandOptions)-1)
    {
        currentOption = 0;
    }
    else
    {
        currentOption++;
    }
}

void commandOptionSelectISR()
{   
    if (commandOptions[currentOption]=="BACK"){
        controller.changeState(initialisation);
    }
    else if (commandOptions[currentOption]=="CANCEL"){
        controller.cancelCommand();
    }
    else controller.sendCommand(commandOptions[currentOption]);
}

void returnToHomeISR()
{
    controller.changeState(initialisation);
}

int main()
{
    C12832 LCD(D11, D13, D12, D7, D10);                          

    Joystick jstick(A2, A3, A4, A5, D4); 

    InterruptIn up(A2);
    InterruptIn down(A3);
    InterruptIn fire(D4);
    
    int goToState = 0;

    while(1) {
    switch (controller.returnState()) {
    case (initialisation) :                                       
        LCD.cls();
        controller.changeState("home");                                       
    case (home) :                                            
        LCD.locate(0,0);
        LCD.printf("Options:");
        for (int i=0; i < sizeof(homeOptions); i++){
            LCD.printf("\n  -")
            LCD.printf(homeOptions[i])
            if (currentOption == i) {
                LCD.printf(" <---");
            }
        }
        up.rise(&homeOptionUpISR);
        down.rise(&homeOptionDownISR);
        fire.rise(&homeOptionSelectISR);
    break;
    case("Commands") :                                                 
        LCD.locate(0,0);
        LCD.printf("Commands:")
        for (int i=0; i < sizeof(commandOptions); i++){
            LCD.printf("\n  -")
            LCD.printf(commandOptions[i])
            if (currentOption == i) {
                LCD.printf(" <---");
            }
        LCD.printf("\n\nCommands queued: %d", controller.returnCommandCounter())
        }
        up.rise(&commandOptionUpISR);
        down.rise(&commandOptionDownISR);
        fire.rise(&commandOptionSelectISR);                                          
    break;
    case ("Command Sent") :
        LCD.cls();
        LCD.locate(0,0)
        LCD.printf("\n \"%s\" command sent.", commandOptions[currentOption]);
        LCD.printf("\n\n Commands queued: %d", controller.returnCommandCounter());
        wait(2);
        LCD.cls();
        controller.changeState("Commands");
    break;
    case("Camera View") :                                               
        LCD.locate(0,0);
        LCD.printf("Camera Feed:");
        LCD.printf("\n\n\n\n  BACK <---");
        fire.rise(&returnToHomeISR);
    break;
    case("Sensor Data") :                                               
        LCD.locate(0,0);
        LCD.printf("Sensors:");
        LCD.printf("\n  -");
        for (int i=0; i < sizeof(sensorList); i++){
            LCD.printf("\n  -");
            LCD.printf(sensorList[i]);
            LCD.printf(" : %f", sensorData[i]);
        }
        LCD.printf("\n\n  BACK <---");
        fire.rise(&returnToHomeISR);
    break;
    case(nextState) :
        LCD.cls();
        goToState = currentOption;
        currentOption = 0;
        controller.changeState(homeOptions[goToState]);
    default :
        controller.changeState(initialisation);                                             
 	}
 }
}



