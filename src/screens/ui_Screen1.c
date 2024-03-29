// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.3.0
// LVGL version: 8.3.6
// Project name: pel

#include "../ui.h"

void ui_Screen1_screen_init(void)
{
    ui_Screen1 = lv_obj_create(NULL);
    lv_obj_clear_flag(ui_Screen1, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_power = lv_slider_create(ui_Screen1);
    lv_slider_set_range(ui_power, 0, 255);
    lv_obj_set_width(ui_power, 249);
    lv_obj_set_height(ui_power, 10);
    lv_obj_set_x(ui_power, lv_pct(0));
    lv_obj_set_y(ui_power, lv_pct(30));
    lv_obj_set_align(ui_power, LV_ALIGN_CENTER);

    lv_obj_set_style_bg_color(ui_power, lv_color_hex(0x1CEC11), LV_PART_INDICATOR | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_power, 255, LV_PART_INDICATOR | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_color(ui_power, lv_color_hex(0x2012DE), LV_PART_INDICATOR | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_main_stop(ui_power, 0, LV_PART_INDICATOR | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_stop(ui_power, 255, LV_PART_INDICATOR | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_dir(ui_power, LV_GRAD_DIR_HOR, LV_PART_INDICATOR | LV_STATE_DEFAULT);

    ui_VALUE = lv_label_create(ui_Screen1);
    lv_obj_set_width(ui_VALUE, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_VALUE, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_VALUE, 0);
    lv_obj_set_y(ui_VALUE, lv_pct(23));
    lv_obj_set_align(ui_VALUE, LV_ALIGN_CENTER);
    lv_label_set_text(ui_VALUE, "");

    ui_HEADER = lv_label_create(ui_Screen1);
    lv_obj_set_width(ui_HEADER, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_HEADER, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_HEADER, 0);
    lv_obj_set_y(ui_HEADER, lv_pct(15));
    lv_obj_set_align(ui_HEADER, LV_ALIGN_CENTER);
    lv_label_set_text(ui_HEADER, "POWER");

    ui_tempbar = lv_bar_create(ui_Screen1);
    lv_obj_set_width(ui_tempbar, 15);
    lv_obj_set_height(ui_tempbar, 65);
    lv_obj_set_x(ui_tempbar, -125);
    lv_obj_set_y(ui_tempbar, -40);
    lv_obj_set_align(ui_tempbar, LV_ALIGN_CENTER);

    lv_obj_set_style_bg_color(ui_tempbar, lv_color_hex(0xFF0000), LV_PART_INDICATOR | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_tempbar, 255, LV_PART_INDICATOR | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_color(ui_tempbar, lv_color_hex(0x0300FE), LV_PART_INDICATOR | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_main_stop(ui_tempbar, 15, LV_PART_INDICATOR | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_stop(ui_tempbar, 252, LV_PART_INDICATOR | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_dir(ui_tempbar, LV_GRAD_DIR_VER, LV_PART_INDICATOR | LV_STATE_DEFAULT);

    ui_Label2 = lv_label_create(ui_Screen1);
    lv_obj_set_width(ui_Label2, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label2, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Label2, -122);
    lv_obj_set_y(ui_Label2, -83);
    lv_obj_set_align(ui_Label2, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label2, "t°");

    ui_tempval = lv_label_create(ui_Screen1);
    lv_obj_set_width(ui_tempval, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_tempval, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_tempval, -125);
    lv_obj_set_y(ui_tempval, -18);
    lv_obj_set_align(ui_tempval, LV_ALIGN_CENTER);
    lv_label_set_text(ui_tempval, "");
    lv_obj_set_style_text_font(ui_tempval, &lv_font_montserrat_14, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_add_event_cb(ui_power, ui_event_power, LV_EVENT_ALL, NULL);

}
