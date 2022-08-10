#include "TemplateView.h"
#include <stdarg.h>
#include <stdio.h>

using namespace Page;

void TemplateView::Create(lv_obj_t* root)
{
    lv_obj_remove_style_all(root);
    lv_obj_set_size(root, LV_HOR_RES, LV_VER_RES);
    lv_obj_set_style_bg_color(root, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(root, LV_OPA_COVER, 0);

    lv_obj_t* label = lv_label_create(root);
    lv_label_set_recolor(label, true);
    lv_obj_align(label, LV_ALIGN_TOP_MID, 0, 20);
    lv_label_set_text(label, "");
    ui.labelTitle = label;

    label = lv_label_create(root);
    lv_label_set_recolor(label, true);
    lv_label_set_text(label, "");
    lv_obj_center(label);
    ui.labelTick = label;


    lv_group_t* group = lv_group_get_default();
    lv_group_set_wrap(group, true);
    lv_group_add_obj(group, root);
    
}
