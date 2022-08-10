#include "HalWin32.h"
#include "HalWin32_Button.h"
#include <windows.h>
#include <time.h>
#include <thread>


void ButtonUpdateTask() {
    while (1) {
        HalWin32Button_Update();
        Sleep(10);
    }
}

void HalWin32::Hal_Init()
{
    HalWin32Button_Init();

    std::thread* t = new std::thread(ButtonUpdateTask);
}

void HalWin32::Hal_Update()
{

}
