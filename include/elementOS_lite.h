#pragma once
#include "vex.h"

#define RESET   "\033[0m"
#define BLACK   "\033[30m"      /* Black */
#define RED     "\033[31m"      /* Red */
#define GREEN   "\033[32m"      /* Green */
#define YELLOW  "\033[33m"      /* Yellow */
#define BLUE    "\033[34m"      /* Blue */
#define MAGENTA "\033[35m"      /* Magenta */
#define CYAN    "\033[36m"      /* Cyan */
#define WHITE   "\033[37m"      /* White */

#define HEX_BLACK   "#000000"      /* Black */
#define HEX_RED     "#ff0000"      /* Red */
#define HEX_GREEN   "#008000"      /* Green */
#define HEX_YELLOW  "#ffff00"      /* Yellow */
#define HEX_BLUE    "#0000ff"      /* Blue */
#define HEX_MAGENTA "#ff00ff"      /* Magenta */
#define HEX_CYAN    "#00ffff"      /* Cyan */
#define HEX_WHITE   "#ffffff"      /* White */

#define SINGLE_LINE_HIGHT 15
#define SINGLE_LETTER_WIDTH 10

/// @brief elementOS主函数
void EosMain();

/// @brief 初始化函数
void Initialize();

/// @brief 自动路线函数
void AutoRoute();

namespace eos{
    inline brain*            BRAIN = NULL;
    inline controller*       CONTROLLER = NULL;
    inline competition*      COMPETITION = NULL;
    inline brain::lcd*       LCD = NULL;    
    inline digital_out*      BUZZER = NULL;

    inline timer GLOBAL_TIMER = timer();

    inline int ALLIANCE = 0; //0 = red , 1 = blue;

    inline bool READY = false;
    inline bool INTTIALIZE_READY = false;
    inline bool AUTODEBUG = false;
    inline bool OPDEBUG = false;
    inline bool BUZZER_ENABLE = false;

    /* 系统操作 */

    inline void SystemInitialize(brain* pbrain , controller* pcontroller , competition* pcompetition , digital_out* pbuzzer){
        BRAIN = pbrain; CONTROLLER = pcontroller; COMPETITION = pcompetition;
        LCD = &BRAIN->Screen;
        GLOBAL_TIMER.clear();
        BUZZER = pbuzzer;
    }

    inline void SystemWait(){wait(50,msec);}
    inline void SystemWait(uint32_t time){wait(time,msec);}
    inline double SystemTime(timeUnits t){return GLOBAL_TIMER.time(t);}
    
    inline void SystemExit(){vexSystemExitRequest();}

    inline int SystemBattery(){
        return int(eos::BRAIN->Battery.capacity()) >= 100 ? 99 : int(eos::BRAIN->Battery.capacity());
    }

    inline void BuzzerDrive(){
        waitUntil(eos::INTTIALIZE_READY);

        //开机启动音
        repeat(3){
            BUZZER->set(true);
            wait(100,msec);
            BUZZER->set(false);
            wait(50,msec);
        }

        while(true){
            if(BUZZER_ENABLE){
                BUZZER->set(true);
                wait(200,msec);
                BUZZER->set(false);
                BUZZER_ENABLE = false;
            }
            wait(10,msec);
        }
    }

    /* SD卡操作 */



    /* 遥控器操作 */

    inline void ClearControllerScreen(){CONTROLLER->Screen.clearScreen();}
    inline void ClearControllerLine(int column){CONTROLLER->Screen.clearLine(column);}
    
    template<class T>
    inline void ControllerPrint(T value , int c , int l){
        CONTROLLER->Screen.setCursor(c , l);
        CONTROLLER->Screen.print(value);
    }
    template void ControllerPrint(int value , int c , int l);
    template void ControllerPrint(double value , int c , int l);
    template void ControllerPrint(const char* value , int c , int l);

    inline bool A(){return CONTROLLER->ButtonA.pressing();}
    inline bool B(){return CONTROLLER->ButtonB.pressing();}
    inline bool X(){return CONTROLLER->ButtonX.pressing();}
    inline bool Y(){return CONTROLLER->ButtonY.pressing();}
    inline bool L1(){return CONTROLLER->ButtonL1.pressing();}
    inline bool L2(){return CONTROLLER->ButtonL2.pressing();}
    inline bool R1(){return CONTROLLER->ButtonR1.pressing();}
    inline bool R2(){return CONTROLLER->ButtonR2.pressing();}
    inline bool Up(){return CONTROLLER->ButtonUp.pressing();}
    inline bool Down(){return CONTROLLER->ButtonDown.pressing();}
    inline bool Left(){return CONTROLLER->ButtonLeft.pressing();}
    inline bool Right(){return CONTROLLER->ButtonRight.pressing();}
    inline int A1(){return CONTROLLER->Axis1.position();}
    inline int A2(){return CONTROLLER->Axis2.position();}
    inline int A3(){return CONTROLLER->Axis3.position();}
    inline int A4(){return CONTROLLER->Axis4.position();}

        /* 用单键或双键控制一个bool变量的值 */
    inline void SingleButtonControl(bool &_status , bool(*button)(void)){
        if(button()){
            if(!_status){
                _status = true;
                do {                                                                         
                    wait(5, msec);                                                             
                } while (button());
            }else if(_status){
                _status = false;
                do {                                                                         
                    wait(5, msec);                                                             
                } while (button());
            }
        }
    }
    inline void DoubleButtonControl(bool &_status , bool(*button_t)(void) , bool(*button_f)(void)){
        if(button_t()){_status = true;}
        else if(button_f()){_status = false;}
    }

    /* 主控操作 */

    inline void ClearBrainScreen(){BRAIN->Screen.clearScreen();}
    inline void ClearBrainLine(int column){BRAIN->Screen.clearLine(column);}

    inline void BrainDisplayImage(int x , int y , const char* name){
        BRAIN->Screen.drawImageFromFile(name , x , y);
    }

    class brain_button{
        private:

            int x , y;
            int h , w;
            const char* text;
            const char* button_color;
            const char* font_color;

        public:

            brain_button(int _x , int _y , int _h , int _w , const char* _text , const char* _button_color , const char* _font_color){
                x = _x; y = _y; text = _text; button_color = _button_color;
                h = _h; w = _w; font_color = _font_color;
            }
            brain_button(){}

            void SetText(const char* t){text = t;}
            void SetButtonColor(const char* c){button_color = c;}
            void SetFontColor(const char* c){font_color = c;}
            void SetSize(int _h , int _w){h = _h; w = _w;}
            void SetPosition(int _x , int _y){x = _x; y = _y;}

            void DisplayButton(){
                eos::BRAIN->Screen.drawRectangle(this->x , this->y , this->w , this->h , this->button_color);
                eos::LCD->setPenColor(font_color);
                eos::LCD->printAt( 
                    this->x + int((this->w - strlen(text) * SINGLE_LETTER_WIDTH) / 2) , 
                    this->y + SINGLE_LINE_HIGHT + int((this->h - SINGLE_LINE_HIGHT) / 2) , 
                    false ,
                    text
                );
                eos::LCD->setPenColor(HEX_WHITE);
            }

            bool IsPressed(){
                if(
                    eos::LCD->xPosition() >= this->x && eos::LCD->xPosition() <= this->x + this->w && 
                    eos::LCD->yPosition() >= this->y && eos::LCD->yPosition() <= this->y + this->h &&
                    eos::LCD->pressing()
                ){
                    return true;
                }else{
                    return false;
                }
            }
    };

    /* 终端操作 */

    inline void TerminalClear(){std::cout << "\033c";}
    inline void TerminalEndl(){std::cout << std::endl;}

    template<class T>
    inline void TerminalPrint(T value , const char* color){
        std::cout << color << value << RESET << std::endl;
    }
    template void TerminalPrint(int value , const char* color);
    template void TerminalPrint(double value , const char* color);
    template void TerminalPrint(const char* value , const char* color);

    template<class T>
    inline void TerminalPrintWithData(const char* value , T data , const char* color){
        std::cout << color << value << data << RESET << std::endl;
    }
    template void TerminalPrintWithData(const char* value , int data , const char* color);
    template void TerminalPrintWithData(const char* value , double data , const char* color);
    template void TerminalPrintWithData(const char* value , const char* data , const char* color);

}