#include "mbed.h"
#include "C12832.h"

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


class Controller {
  
  enum ProgramState {initialisation, home, commands, camera, sensors};       //Definition of the enum that refers to the states of the program
  
  private:
    ProgramState currentState = initialisation;

  public:
    Controller () : {} //Constructor

    void changeState (state){
    	currentState = state;
    }

    ProgramState returnState(){
      	return currentState;
    }
}


int main()
{
    C12832 LCD(D11, D13, D12, D7, D10);                          

    Joystick jstick(A2, A3, A4, A5, D4); 
    Controller controller();

    while(1) {
    switch (controller.returnState()) {
    case (initialisation) :                                       
        ;                                       
    case (home) :                                            
        ;                                          
    break;
    case(commands) :                                                 
        ;                                          
    break;
    case(camera) :                                               
        ;
    break;
    case(sensors) :                                               
        ;
    break;
    default :
        state = initialisation;                                             
 }
 }
}



