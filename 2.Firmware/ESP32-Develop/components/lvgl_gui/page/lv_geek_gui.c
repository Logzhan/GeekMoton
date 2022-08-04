
#if defined(LV_LVGL_H_INCLUDE_SIMPLE)
#include "lvgl.h"
#else
#include "../lvgl/lvgl.h"
#endif

//#include "battery.h"
#include "lv_geek_gui.h"

/*********************
 *      DEFINES
 *********************/
// GEEK GUI 界面退出动画
#define LV_GEEK_GUI_ANIM_Y                 (LV_VER_RES / 20)
#define LV_GEEK_GUI_ANIM_DELAY             (40)
#define LV_GEEK_GUI_ANIM_TIME              (150)
#define LV_GEEK_GUI_ANIM_TIME_BG           (300)

/**********************
 *  STATIC PROTOTYPES
 **********************/
static void geek_page_anim_out_all(lv_obj_t* obj, uint32_t delay);
static void geek_page_anim_anim_in(lv_obj_t * obj, uint32_t delay);
/**********************
 *  STATIC VARIABLES
 **********************/
lv_obj_t* scr = NULL;
lv_obj_t* scr_func = NULL;
lv_obj_t* bat_precent;
lv_obj_t* sd_card;
lv_obj_t* battery;
lv_obj_t* wifi;
lv_obj_t* bg_img;
/*状态栏更新任务句柄*/
//static lv_task_t * taskTopBarUpdate;

enum{
	MAIN_PAGE = 0,                  // 主菜单
	FUNC_PAGE = 1,                  // 功能菜单
}geek_page_def;

int page_idx = MAIN_PAGE;           // 默认主菜单的页面索引是0

/**----------------------------------------------------------------------
* Function    : init_status_bar
* Description : 初始化状态栏
* Author      : zhanli&719901725@qq.com
* Date        : 2021/12/31 zhanli
*---------------------------------------------------------------------**/
void init_status_bar(){
    // 设定电池百分比
	bat_precent = lv_label_create(scr);
	lv_label_set_recolor(bat_precent, true);
	lv_label_set_text_fmt(bat_precent, "#f1f1f1 %d%%", 90);
	lv_obj_set_pos(bat_precent, 205, 5);
	geek_page_anim_anim_in(bat_precent, 0);

	// 显示电池图标
	sd_card = lv_label_create(scr);
	lv_label_set_recolor(sd_card, true);
	lv_label_set_text_fmt(sd_card, "#f1f1f1 %s", LV_SYMBOL_BATTERY_FULL);
	lv_obj_set_pos(sd_card, 182, 5);
	geek_page_anim_anim_in(sd_card, 0);

	// 显示SD卡图标
	battery = lv_label_create(scr);
	lv_label_set_recolor(battery, true);
	lv_label_set_text_fmt(battery, "#f1f1f1 %s", LV_SYMBOL_SD_CARD);
	lv_obj_set_pos(battery, 32, 5);
	geek_page_anim_anim_in(battery, 0);

	// 显示WIFI图标
	wifi = lv_label_create(scr);
	lv_label_set_recolor(wifi, true);
	lv_label_set_text_fmt(wifi, "#f1f1f1 %s", LV_SYMBOL_WIFI);
	lv_obj_set_pos(wifi, 5, 5);
	geek_page_anim_anim_in(wifi, 0);

}

// /**----------------------------------------------------------------------
// * Function    : update_staus_bar_task
// * Description : 更新状态栏信息任务
// * Author      : zhanli&719901725@qq.com
// * Date        : 2021/12/31 zhanli
// *---------------------------------------------------------------------**/
// void update_staus_bar_task(lv_task_t * task){
// 	// 获取当前电池电量
// 	int bat = battery_get_capacity();

//     lv_label_set_text_fmt(bat_precent, "#f1f1f1 %d%%", bat);

//     const char * battSymbol[] =
//     {
//         LV_SYMBOL_BATTERY_EMPTY,
//         LV_SYMBOL_BATTERY_1,
//         LV_SYMBOL_BATTERY_2,
//         LV_SYMBOL_BATTERY_3,
//         LV_SYMBOL_BATTERY_FULL
//     };
//     uint8_t bat_level = bat / 20;
//     if(bat_level > 4)bat_level = 4;

//     lv_label_set_text_fmt(sd_card, "#f1f1f1 %s", battSymbol[bat_level]);
// }

void update_motion_info(){
    LV_IMG_DECLARE(run_ico_img);

    lv_obj_t* img2 = lv_img_create(scr);
	lv_img_set_src(img2, &run_ico_img);
	lv_obj_set_pos(img2, 155, 110);
	//geek_page_anim_anim_in(img2, 0);

    lv_obj_t* label1 = lv_label_create(scr);
	lv_label_set_recolor(label1, true);
	lv_label_set_text_fmt(label1, "#f1f1f1 / %d", 12040);
	lv_obj_set_pos(label1, 180, 115);
	//geek_page_anim_anim_in(label1, 0);
}

void update_time_info()
{
    LV_FONT_DECLARE(Morganite_100);

    static lv_style_t font_style1;
	lv_style_init(&font_style1);
	//lv_style_set_text_font(&font_style1, &Morganite_100);
	lv_style_set_text_color(&font_style1, lv_color_hex(0xf1f1f1));

	lv_obj_t* font_label1 = lv_label_create(scr);
	lv_obj_add_style(font_label1, &font_style1, LV_STATE_DEFAULT);
	lv_label_set_text_fmt(font_label1, "%d:%d", 15,34);
	lv_obj_align(font_label1, LV_ALIGN_CENTER, 0, 0);
}
/**----------------------------------------------------------------------
* Function    : init_main_page_bg
* Description : 初始化主页面的背景图片
* Author      : zhanli&719901725@qq.com
* Date        : 2021/12/31 zhanli
*---------------------------------------------------------------------**/
void init_main_page_bg(){
	LV_IMG_DECLARE(bg_img_main);
	scr = lv_scr_act();
	bg_img = lv_img_create(scr);
	lv_img_set_src(bg_img, &bg_img_main);
	lv_obj_align(bg_img, LV_ALIGN_CENTER,0, 0);
}

void geek_main_page_init(){
    // 状态栏初始化
    init_status_bar();
    update_motion_info();
    update_time_info();
    // // 创建定时更新状态栏任务
    // taskTopBarUpdate = lv_task_create(update_staus_bar_task, 200, LV_TASK_PRIO_LOW, NULL);
    // update_staus_bar_task(taskTopBarUpdate);
}

void geek_gui_init(void)
{
    scr = lv_disp_get_scr_act(NULL);
	// GUI背景初始化
    init_main_page_bg();
	// 主页面功能组件初始化
    geek_main_page_init();
}


static void geek_page_anim_out_all(lv_obj_t* obj, uint32_t delay) {
	// lv_obj_t* child = lv_obj_get_child_back(obj, NULL);
	// while (child) {
	// 	if (child != lv_scr_act() && child != bg_img) {
	// 		lv_anim_t a;
	// 		lv_anim_init(&a);
	// 		lv_anim_set_var(&a, child);
	// 		lv_anim_set_time(&a, LV_GEEK_GUI_ANIM_TIME);
	// 		lv_anim_set_delay(&a, delay);
	// 		lv_anim_set_exec_cb(&a, (lv_anim_exec_xcb_t)lv_obj_set_y);
	// 		if (lv_obj_get_y(child) < 80) {
	// 			lv_anim_set_values(&a, lv_obj_get_y(child),
	// 				lv_obj_get_y(child) - LV_GEEK_GUI_ANIM_Y);
	// 		}
	// 		else {
	// 			lv_anim_set_values(&a, lv_obj_get_y(child),
	// 				lv_obj_get_y(child) + LV_GEEK_GUI_ANIM_Y);

	// 			delay += LV_GEEK_GUI_ANIM_DELAY;
	// 		}
	// 		lv_anim_set_ready_cb(&a, lv_obj_del_anim_ready_cb);
	// 		lv_anim_start(&a);
	// 	}
	// 	child = lv_obj_get_child_back(obj, child);
	// }
}

static void geek_page_anim_anim_in(lv_obj_t * obj, uint32_t delay)
{
    if (obj != lv_scr_act()) {
        lv_anim_t a;
        lv_anim_init(&a);
        lv_anim_set_var(&a, obj);
        lv_anim_set_time(&a, LV_GEEK_GUI_ANIM_TIME);
        lv_anim_set_delay(&a, delay);
        lv_anim_set_exec_cb(&a, (lv_anim_exec_xcb_t) lv_obj_set_y);
        lv_anim_set_values(&a, lv_obj_get_y(obj) - LV_GEEK_GUI_ANIM_Y,
                lv_obj_get_y(obj));
        lv_anim_start(&a);

        lv_obj_fade_in(obj, LV_GEEK_GUI_ANIM_TIME - 50, delay);
    }
}


void enter_func_page(){
	// if(page_idx == MAIN_PAGE){
	// 	lv_task_del(taskTopBarUpdate);
	// 	geek_page_anim_out_all(scr, 0);
	// 	page_idx = FUNC_PAGE;

	// 	scr_func = lv_scr_act();

	// 	lv_obj_t* label1 = lv_label_create(scr, NULL);
	// 	lv_label_set_recolor(label1, true);
	// 	lv_label_set_text(label1, "#f1f1f1 hello function!");
	// 	lv_obj_align(label1, NULL, LV_ALIGN_CENTER, 0, 0);
	// 	geek_page_anim_anim_in(label1, 0);
	// 	return;
	// }
	// if(page_idx == FUNC_PAGE){
	// 	geek_page_anim_out_all(scr_func, 0);
	// 	geek_main_page_init();
	// 	page_idx = MAIN_PAGE;
	// }
}