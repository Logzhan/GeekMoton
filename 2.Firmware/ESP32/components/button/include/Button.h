#ifndef __BUTTON_H
#define __BUTTON_H

#ifdef __cplusplus
extern "C" {
#endif

void button_init();
void Button_Update();
void getKeyPadState(int* ok, int* up, int* down);

#ifdef __cplusplus
}
#endif

#endif
