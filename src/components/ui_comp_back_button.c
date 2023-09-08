// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.3.0
// LVGL version: 8.3.6
// Project name: pel

#include "../ui.h"

// COMPONENT Back Button

lv_obj_t * ui_Back_Button_create(lv_obj_t * comp_parent)
{

    lv_obj_t * cui_Back_Button;
    cui_Back_Button = lv_btn_create(comp_parent);
    lv_obj_set_width(cui_Back_Button, 100);
    lv_obj_set_height(cui_Back_Button, 50);
    lv_obj_set_align(cui_Back_Button, LV_ALIGN_CENTER);
    lv_obj_add_flag(cui_Back_Button, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_clear_flag(cui_Back_Button, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    lv_obj_t ** children = lv_mem_alloc(sizeof(lv_obj_t *) * _UI_COMP_BACK_BUTTON_NUM);
    children[UI_COMP_BACK_BUTTON_BACK_BUTTON] = cui_Back_Button;
    lv_obj_add_event_cb(cui_Back_Button, get_component_child_event_cb, LV_EVENT_GET_COMP_CHILD, children);
    lv_obj_add_event_cb(cui_Back_Button, del_component_child_event_cb, LV_EVENT_DELETE, children);
    ui_comp_Back_Button_create_hook(cui_Back_Button);
    return cui_Back_Button;
}

