#ifndef _LV_GEEK_OS_H_
#define _LV_GEEK_OS_H_

#ifdef __cplusplus
extern "C" {
#endif

// GeekOS 初始化函数
void GeekOS_Init(void);
// GeekOS 系统退出函数
void GeekOS_Uninit(void);
// 页面切换
void PageSwitchByName(const char* name);
// 退出当前页面
void ExitCurrentPages(void);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif
