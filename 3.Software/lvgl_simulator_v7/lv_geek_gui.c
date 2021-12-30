
#include "../../lv_examples.h"
#include "lv_geek_gui.h"
#include "images.h"

/*********************
 *      DEFINES
 *********************/
// GEEK GUI 界面退出动画
#define LV_GEEK_GUI_ANIM_Y                 (240 / 20)
#define LV_GEEK_GUI_ANIM_DELAY             (40)
#define LV_GEEK_GUI_ANIM_TIME              (150)
#define LV_GEEK_GUI_ANIM_TIME_BG           (300)

/**********************
 *  STATIC PROTOTYPES
 **********************/
static void geek_page_anim_out_all(lv_obj_t* obj, uint32_t delay);
/**********************
 *  STATIC VARIABLES
 **********************/
lv_obj_t* scr = NULL;
lv_obj_t* bat_precent;
lv_obj_t* sd_card;
lv_obj_t* battery;
lv_obj_t* wifi;
lv_obj_t* bg_img;
/*状态栏更新任务句柄*/
static lv_task_t * taskTopBarUpdate;

/**----------------------------------------------------------------------
* Function    : init_status_bar
* Description : 初始化状态栏
* Author      : zhanli&719901725@qq.com
* Date        : 2021/12/31 zhanli
*---------------------------------------------------------------------**/
void init_status_bar(){

	bat_precent = lv_label_create(scr, NULL);
	lv_label_set_recolor(bat_precent, true);
	lv_label_set_text_fmt(bat_precent, "#f1f1f1 %d%%", 90);
	lv_obj_set_pos(bat_precent, 205, 5);


	// 显示电池图标
	sd_card = lv_label_create(scr, NULL);
	lv_label_set_recolor(sd_card, true);
	lv_label_set_text_fmt(sd_card, "#f1f1f1 %s", LV_SYMBOL_BATTERY_FULL);
	lv_obj_set_pos(sd_card, 182, 5);

	// 显示SD卡图标
	//battery = lv_label_create(scr, NULL);
	//lv_label_set_recolor(battery, true);
	//lv_label_set_text_fmt(battery, "#f1f1f1 %s", LV_SYMBOL_SD_CARD);
	//lv_obj_set_pos(battery, 32, 5);

	// 显示WIFI图标
	wifi = lv_label_create(scr, NULL);
	lv_label_set_recolor(wifi, true);
	lv_label_set_text_fmt(wifi, "#f1f1f1 %s", LV_SYMBOL_WIFI);
	lv_obj_set_pos(wifi, 5, 5);
}

/**----------------------------------------------------------------------
* Function    : update_staus_bar_task
* Description : 更新状态栏信息任务
* Author      : zhanli&719901725@qq.com
* Date        : 2021/12/31 zhanli
*---------------------------------------------------------------------**/
void update_staus_bar_task(lv_task_t * task){
    static int bat = 0;
    bat++;
    if(bat > 100){
        //geek_page_anim_out_all(scr, 0);
        bat = 0;
    }
    lv_label_set_text_fmt(bat_precent, "#f1f1f1 %d%%", bat);

    const char * battSymbol[] =
    {
        LV_SYMBOL_BATTERY_EMPTY,
        LV_SYMBOL_BATTERY_1,
        LV_SYMBOL_BATTERY_2,
        LV_SYMBOL_BATTERY_3,
        LV_SYMBOL_BATTERY_FULL
    };
    uint8_t bat_level = bat / 20;
    if(bat_level > 4)bat_level = 4;

    lv_label_set_text_fmt(sd_card, "#f1f1f1 %s", battSymbol[bat_level]);
}

void update_motion_info(){
    LV_IMG_DECLARE(run_ico_img);

    lv_obj_t* img2 = lv_img_create(scr, NULL);
	lv_img_set_src(img2, &run_ico_img);
	lv_obj_set_pos(img2, 160, 110);

    lv_obj_t* label1 = lv_label_create(scr, NULL);
	lv_label_set_recolor(label1, true);
	lv_label_set_text_fmt(label1, "#f1f1f1 / %d", 1204);
	lv_obj_set_pos(label1, 185, 115);

}

void update_time_info()
{
    LV_FONT_DECLARE(Morganite_100);

    static lv_style_t font_style1;
	lv_style_init(&font_style1);
	lv_style_set_text_font(&font_style1, LV_STATE_DEFAULT, &Morganite_100);
	lv_style_set_text_color(&font_style1, LV_STATE_DEFAULT, lv_color_hex(0xf1f1f1));

	lv_obj_t* font_label1 = lv_label_create(scr, NULL);
	lv_obj_add_style(font_label1, LV_LABEL_PART_MAIN, &font_style1);
	lv_label_set_text_fmt(font_label1, "%d:%d", 15,34);
	lv_obj_align(font_label1, NULL, LV_ALIGN_CENTER, 0, 0);

}
/**----------------------------------------------------------------------
* Function    : init_main_page_bg
* Description : 初始化主页面的背景图片
* Author      : zhanli&719901725@qq.com
* Date        : 2021/12/31 zhanli
*---------------------------------------------------------------------**/
void init_main_page_bg(){
	scr = lv_scr_act();
	bg_img = lv_img_create(scr, NULL);
	lv_img_set_src(bg_img, &bg_img_main);
	lv_obj_align(bg_img, NULL, LV_ALIGN_CENTER, 0, 0);
}

void geek_main_page_init(){
    // 主页面背景初始化
    init_main_page_bg();
    // 状态栏初始化
    init_status_bar();

    update_motion_info();

    update_time_info();

    // 创建定时更新状态栏任务
    //taskTopBarUpdate = lv_task_create(update_staus_bar_task, 2000, LV_TASK_PRIO_LOW, NULL);
    //update_staus_bar_task(taskTopBarUpdate);
    //// 页面退出动画
    //geek_page_anim_out_all(scr, 0);
}

void geek_gui_init(void)
{
    scr = lv_disp_get_scr_act(NULL);
    geek_main_page_init();
}

static void geek_page_anim_out_all(lv_obj_t* obj, uint32_t delay) {
	lv_obj_t* child = lv_obj_get_child_back(obj, NULL);
	while (child) {
		if (child != lv_scr_act() && child != bg_img) {
			lv_anim_t a;
			lv_anim_init(&a);
			lv_anim_set_var(&a, child);
			lv_anim_set_time(&a, 12);
			lv_anim_set_delay(&a, delay);
			lv_anim_set_exec_cb(&a, (lv_anim_exec_xcb_t)lv_obj_set_y);
			if (lv_obj_get_y(child) < 80) {
				lv_anim_set_values(&a, lv_obj_get_y(child),
					lv_obj_get_y(child) - 12);
			}
			else {
				lv_anim_set_values(&a, lv_obj_get_y(child),
					lv_obj_get_y(child) + 12);

				delay += LV_GEEK_GUI_ANIM_DELAY;
			}
			lv_anim_set_ready_cb(&a, lv_obj_del_anim_ready_cb);
			lv_anim_start(&a);
		}
		child = lv_obj_get_child_back(obj, child);
	}
}
