// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.4.2
// LVGL version: 8.3.11
// Project name: Smart_Gadget

#include "ui.h"
#include "ui_helpers.h"
#include <zephyr/logging/log.h>

///////////////////// VARIABLES ////////////////////
void upanim_Animation(lv_obj_t * TargetObject, int delay);
void hour_Animation(lv_obj_t * TargetObject, int delay);
void min_Animation(lv_obj_t * TargetObject, int delay);
void sec_Animation(lv_obj_t * TargetObject, int delay);
void scrolldot_Animation(lv_obj_t * TargetObject, int delay);


// SCREEN: ui_Splash
void ui_Splash_screen_init(void);
void ui_event_Splash(lv_event_t * e);
lv_obj_t * ui_Splash;
lv_obj_t * ui_Demo;
lv_obj_t * ui_Smart_Gadget;
lv_obj_t * ui_SLS_Logo;


// SCREEN: ui_Clock
void ui_Clock_screen_init(void);
void ui_event_Clock(lv_event_t * e);
lv_obj_t * ui_Clock;
lv_obj_t * ui_Clock_Panel;
lv_obj_t * ui_Dot1;
lv_obj_t * ui_Dot2;
lv_obj_t * ui_Dot3;
lv_obj_t * ui_Dot4;
lv_obj_t * ui_Dot5;
lv_obj_t * ui_Dot6;
lv_obj_t * ui_Dot7;
lv_obj_t * ui_Dot8;
lv_obj_t * ui_Clock_Number1;
lv_obj_t * ui_Clock_Number2;
lv_obj_t * ui_Clock_Number3;
lv_obj_t * ui_Clock_Number4;
lv_obj_t * ui_Min;
lv_obj_t * ui_Hour;
lv_obj_t * ui_Sec;
lv_obj_t * ui_Clock_Center;
lv_obj_t * ui_Clock_Number;
lv_obj_t * ui_Date;
lv_obj_t * ui_Scrolldots;


// SCREEN: ui_Call
void ui_Call_screen_init(void);
void ui_event_Call(lv_event_t * e);
lv_obj_t * ui_Call;
lv_obj_t * ui_Elena;
lv_obj_t * ui_Incoming;
lv_obj_t * ui_Call_Incon1;
lv_obj_t * ui_Phone1;
lv_obj_t * ui_Call_Incon2;
lv_obj_t * ui_Phone2;
lv_obj_t * ui_Avatar;
lv_obj_t * ui_Scrolldots1;


// SCREEN: ui_Chat
void ui_Chat_screen_init(void);
void ui_event_Chat(lv_event_t * e);
lv_obj_t * ui_Chat;
lv_obj_t * ui_Chat_container;
lv_obj_t * ui_Chat_date;
lv_obj_t * ui_C1;
lv_obj_t * ui_Chat_Panel1;
lv_obj_t * ui_Chat1;
lv_obj_t * ui_Chat_Icon1;
lv_obj_t * ui_C2;
lv_obj_t * ui_Chat_Panel2;
lv_obj_t * ui_Chat2;
lv_obj_t * ui_Chat_Icon2;
lv_obj_t * ui_Delifered;
lv_obj_t * ui_C3;
lv_obj_t * ui_Chat_Panel3;
lv_obj_t * ui_Chat3;
lv_obj_t * ui_Chat_Icon3;
lv_obj_t * ui_Scrolldots2;


// SCREEN: ui_Music_Player
void ui_Music_Player_screen_init(void);
void ui_event_Music_Player(lv_event_t * e);
lv_obj_t * ui_Music_Player;
lv_obj_t * ui_Music_Title;
lv_obj_t * ui_Author;
lv_obj_t * ui_Play_btn;
lv_obj_t * ui_Play;
lv_obj_t * ui_Album;
lv_obj_t * ui_Backward;
lv_obj_t * ui_Forward;
lv_obj_t * ui_Scrolldots3;


// SCREEN: ui_Weather
void ui_Weather_screen_init(void);
void ui_event_Weather(lv_event_t * e);
lv_obj_t * ui_Weather;
lv_obj_t * ui_Pary_Cloud;
lv_obj_t * ui_New_York;
lv_obj_t * ui_Cloud;
lv_obj_t * ui_Celsius;
lv_obj_t * ui_Weather_Icons;
lv_obj_t * ui_w1;
lv_obj_t * ui_w2;
lv_obj_t * ui_w3;
lv_obj_t * ui_W1_Num;
lv_obj_t * ui_W2_Num;
lv_obj_t * ui_W3_Num;
lv_obj_t * ui_Scrolldots4;


// SCREEN: ui_Alarm
void ui_Alarm_screen_init(void);
void ui_event_Alarm(lv_event_t * e);
lv_obj_t * ui_Alarm;
lv_obj_t * ui_Alarm_container;
lv_obj_t * ui_Set_alarm;
lv_obj_t * ui_Alarm_Comp;
lv_obj_t * ui_Alarm_Comp1;
lv_obj_t * ui_Alarm_Comp2;
lv_obj_t * ui_Alarm_Comp3;
lv_obj_t * ui_Scrolldots5;
void ui_event____initial_actions0(lv_event_t * e);
lv_obj_t * ui____initial_actions0;
const lv_img_dsc_t * ui_imgset_chatbox[1] = {&ui_img_chatbox2_png};
const lv_img_dsc_t * ui_imgset_weather_[3] = {&ui_img_weather_1_png, &ui_img_weather_2_png, &ui_img_weather_3_png};

///////////////////// TEST LVGL SETTINGS ////////////////////
#if LV_COLOR_DEPTH != 16
    #error "LV_COLOR_DEPTH should be 16bit to match SquareLine Studio's settings"
#endif
#if LV_COLOR_16_SWAP !=1
    #error "LV_COLOR_16_SWAP should be 1 to match SquareLine Studio's settings"
#endif

///////////////////// ANIMATIONS ////////////////////
void upanim_Animation(lv_obj_t * TargetObject, int delay)
{
    ui_anim_user_data_t * PropertyAnimation_0_user_data = lv_mem_alloc(sizeof(ui_anim_user_data_t));
    PropertyAnimation_0_user_data->target = TargetObject;
    PropertyAnimation_0_user_data->val = -1;
    lv_anim_t PropertyAnimation_0;
    lv_anim_init(&PropertyAnimation_0);
    lv_anim_set_time(&PropertyAnimation_0, 200);
    lv_anim_set_user_data(&PropertyAnimation_0, PropertyAnimation_0_user_data);
    lv_anim_set_custom_exec_cb(&PropertyAnimation_0, _ui_anim_callback_set_y);
    lv_anim_set_values(&PropertyAnimation_0, -30, 0);
    lv_anim_set_path_cb(&PropertyAnimation_0, lv_anim_path_ease_out);
    lv_anim_set_delay(&PropertyAnimation_0, delay + 0);
    lv_anim_set_deleted_cb(&PropertyAnimation_0, _ui_anim_callback_free_user_data);
    lv_anim_set_playback_time(&PropertyAnimation_0, 0);
    lv_anim_set_playback_delay(&PropertyAnimation_0, 0);
    lv_anim_set_repeat_count(&PropertyAnimation_0, 0);
    lv_anim_set_repeat_delay(&PropertyAnimation_0, 0);
    lv_anim_set_early_apply(&PropertyAnimation_0, false);
    lv_anim_set_get_value_cb(&PropertyAnimation_0, &_ui_anim_callback_get_y);
    lv_anim_start(&PropertyAnimation_0);
    ui_anim_user_data_t * PropertyAnimation_1_user_data = lv_mem_alloc(sizeof(ui_anim_user_data_t));
    PropertyAnimation_1_user_data->target = TargetObject;
    PropertyAnimation_1_user_data->val = -1;
    lv_anim_t PropertyAnimation_1;
    lv_anim_init(&PropertyAnimation_1);
    lv_anim_set_time(&PropertyAnimation_1, 100);
    lv_anim_set_user_data(&PropertyAnimation_1, PropertyAnimation_1_user_data);
    lv_anim_set_custom_exec_cb(&PropertyAnimation_1, _ui_anim_callback_set_opacity);
    lv_anim_set_values(&PropertyAnimation_1, 0, 255);
    lv_anim_set_path_cb(&PropertyAnimation_1, lv_anim_path_linear);
    lv_anim_set_delay(&PropertyAnimation_1, delay + 0);
    lv_anim_set_deleted_cb(&PropertyAnimation_1, _ui_anim_callback_free_user_data);
    lv_anim_set_playback_time(&PropertyAnimation_1, 0);
    lv_anim_set_playback_delay(&PropertyAnimation_1, 0);
    lv_anim_set_repeat_count(&PropertyAnimation_1, 0);
    lv_anim_set_repeat_delay(&PropertyAnimation_1, 0);
    lv_anim_set_early_apply(&PropertyAnimation_1, true);
    lv_anim_start(&PropertyAnimation_1);

}
void hour_Animation(lv_obj_t * TargetObject, int delay)
{
    ui_anim_user_data_t * PropertyAnimation_0_user_data = lv_mem_alloc(sizeof(ui_anim_user_data_t));
    PropertyAnimation_0_user_data->target = TargetObject;
    PropertyAnimation_0_user_data->val = -1;
    lv_anim_t PropertyAnimation_0;
    lv_anim_init(&PropertyAnimation_0);
    lv_anim_set_time(&PropertyAnimation_0, 1000);
    lv_anim_set_user_data(&PropertyAnimation_0, PropertyAnimation_0_user_data);
    lv_anim_set_custom_exec_cb(&PropertyAnimation_0, _ui_anim_callback_set_image_angle);
    lv_anim_set_values(&PropertyAnimation_0, 0, 2800);
    lv_anim_set_path_cb(&PropertyAnimation_0, lv_anim_path_ease_out);
    lv_anim_set_delay(&PropertyAnimation_0, delay + 0);
    lv_anim_set_deleted_cb(&PropertyAnimation_0, _ui_anim_callback_free_user_data);
    lv_anim_set_playback_time(&PropertyAnimation_0, 0);
    lv_anim_set_playback_delay(&PropertyAnimation_0, 0);
    lv_anim_set_repeat_count(&PropertyAnimation_0, 0);
    lv_anim_set_repeat_delay(&PropertyAnimation_0, 0);
    lv_anim_set_early_apply(&PropertyAnimation_0, false);
    lv_anim_start(&PropertyAnimation_0);
    ui_anim_user_data_t * PropertyAnimation_1_user_data = lv_mem_alloc(sizeof(ui_anim_user_data_t));
    PropertyAnimation_1_user_data->target = TargetObject;
    PropertyAnimation_1_user_data->val = -1;
    lv_anim_t PropertyAnimation_1;
    lv_anim_init(&PropertyAnimation_1);
    lv_anim_set_time(&PropertyAnimation_1, 300);
    lv_anim_set_user_data(&PropertyAnimation_1, PropertyAnimation_1_user_data);
    lv_anim_set_custom_exec_cb(&PropertyAnimation_1, _ui_anim_callback_set_opacity);
    lv_anim_set_values(&PropertyAnimation_1, 0, 255);
    lv_anim_set_path_cb(&PropertyAnimation_1, lv_anim_path_linear);
    lv_anim_set_delay(&PropertyAnimation_1, delay + 0);
    lv_anim_set_deleted_cb(&PropertyAnimation_1, _ui_anim_callback_free_user_data);
    lv_anim_set_playback_time(&PropertyAnimation_1, 0);
    lv_anim_set_playback_delay(&PropertyAnimation_1, 0);
    lv_anim_set_repeat_count(&PropertyAnimation_1, 0);
    lv_anim_set_repeat_delay(&PropertyAnimation_1, 0);
    lv_anim_set_early_apply(&PropertyAnimation_1, true);
    lv_anim_start(&PropertyAnimation_1);

}
void min_Animation(lv_obj_t * TargetObject, int delay)
{
    ui_anim_user_data_t * PropertyAnimation_0_user_data = lv_mem_alloc(sizeof(ui_anim_user_data_t));
    PropertyAnimation_0_user_data->target = TargetObject;
    PropertyAnimation_0_user_data->val = -1;
    lv_anim_t PropertyAnimation_0;
    lv_anim_init(&PropertyAnimation_0);
    lv_anim_set_time(&PropertyAnimation_0, 1000);
    lv_anim_set_user_data(&PropertyAnimation_0, PropertyAnimation_0_user_data);
    lv_anim_set_custom_exec_cb(&PropertyAnimation_0, _ui_anim_callback_set_image_angle);
    lv_anim_set_values(&PropertyAnimation_0, 0, 2100);
    lv_anim_set_path_cb(&PropertyAnimation_0, lv_anim_path_ease_out);
    lv_anim_set_delay(&PropertyAnimation_0, delay + 0);
    lv_anim_set_deleted_cb(&PropertyAnimation_0, _ui_anim_callback_free_user_data);
    lv_anim_set_playback_time(&PropertyAnimation_0, 0);
    lv_anim_set_playback_delay(&PropertyAnimation_0, 0);
    lv_anim_set_repeat_count(&PropertyAnimation_0, 0);
    lv_anim_set_repeat_delay(&PropertyAnimation_0, 0);
    lv_anim_set_early_apply(&PropertyAnimation_0, false);
    lv_anim_start(&PropertyAnimation_0);
    ui_anim_user_data_t * PropertyAnimation_1_user_data = lv_mem_alloc(sizeof(ui_anim_user_data_t));
    PropertyAnimation_1_user_data->target = TargetObject;
    PropertyAnimation_1_user_data->val = -1;
    lv_anim_t PropertyAnimation_1;
    lv_anim_init(&PropertyAnimation_1);
    lv_anim_set_time(&PropertyAnimation_1, 200);
    lv_anim_set_user_data(&PropertyAnimation_1, PropertyAnimation_1_user_data);
    lv_anim_set_custom_exec_cb(&PropertyAnimation_1, _ui_anim_callback_set_opacity);
    lv_anim_set_values(&PropertyAnimation_1, 0, 255);
    lv_anim_set_path_cb(&PropertyAnimation_1, lv_anim_path_linear);
    lv_anim_set_delay(&PropertyAnimation_1, delay + 0);
    lv_anim_set_deleted_cb(&PropertyAnimation_1, _ui_anim_callback_free_user_data);
    lv_anim_set_playback_time(&PropertyAnimation_1, 0);
    lv_anim_set_playback_delay(&PropertyAnimation_1, 0);
    lv_anim_set_repeat_count(&PropertyAnimation_1, 0);
    lv_anim_set_repeat_delay(&PropertyAnimation_1, 0);
    lv_anim_set_early_apply(&PropertyAnimation_1, true);
    lv_anim_start(&PropertyAnimation_1);

}
void sec_Animation(lv_obj_t * TargetObject, int delay)
{
    ui_anim_user_data_t * PropertyAnimation_0_user_data = lv_mem_alloc(sizeof(ui_anim_user_data_t));
    PropertyAnimation_0_user_data->target = TargetObject;
    PropertyAnimation_0_user_data->val = -1;
    lv_anim_t PropertyAnimation_0;
    lv_anim_init(&PropertyAnimation_0);
    lv_anim_set_time(&PropertyAnimation_0, 60000);
    lv_anim_set_user_data(&PropertyAnimation_0, PropertyAnimation_0_user_data);
    lv_anim_set_custom_exec_cb(&PropertyAnimation_0, _ui_anim_callback_set_image_angle);
    lv_anim_set_values(&PropertyAnimation_0, 0, 3600);
    lv_anim_set_path_cb(&PropertyAnimation_0, lv_anim_path_linear);
    lv_anim_set_delay(&PropertyAnimation_0, delay + 0);
    lv_anim_set_deleted_cb(&PropertyAnimation_0, _ui_anim_callback_free_user_data);
    lv_anim_set_playback_time(&PropertyAnimation_0, 0);
    lv_anim_set_playback_delay(&PropertyAnimation_0, 0);
    lv_anim_set_repeat_count(&PropertyAnimation_0, LV_ANIM_REPEAT_INFINITE);
    lv_anim_set_repeat_delay(&PropertyAnimation_0, 0);
    lv_anim_set_early_apply(&PropertyAnimation_0, false);
    lv_anim_start(&PropertyAnimation_0);
    ui_anim_user_data_t * PropertyAnimation_1_user_data = lv_mem_alloc(sizeof(ui_anim_user_data_t));
    PropertyAnimation_1_user_data->target = TargetObject;
    PropertyAnimation_1_user_data->val = -1;
    lv_anim_t PropertyAnimation_1;
    lv_anim_init(&PropertyAnimation_1);
    lv_anim_set_time(&PropertyAnimation_1, 1000);
    lv_anim_set_user_data(&PropertyAnimation_1, PropertyAnimation_1_user_data);
    lv_anim_set_custom_exec_cb(&PropertyAnimation_1, _ui_anim_callback_set_opacity);
    lv_anim_set_values(&PropertyAnimation_1, 0, 255);
    lv_anim_set_path_cb(&PropertyAnimation_1, lv_anim_path_linear);
    lv_anim_set_delay(&PropertyAnimation_1, delay + 0);
    lv_anim_set_deleted_cb(&PropertyAnimation_1, _ui_anim_callback_free_user_data);
    lv_anim_set_playback_time(&PropertyAnimation_1, 0);
    lv_anim_set_playback_delay(&PropertyAnimation_1, 0);
    lv_anim_set_repeat_count(&PropertyAnimation_1, 0);
    lv_anim_set_repeat_delay(&PropertyAnimation_1, 0);
    lv_anim_set_early_apply(&PropertyAnimation_1, true);
    lv_anim_start(&PropertyAnimation_1);

}
void scrolldot_Animation(lv_obj_t * TargetObject, int delay)
{
    ui_anim_user_data_t * PropertyAnimation_0_user_data = lv_mem_alloc(sizeof(ui_anim_user_data_t));
    PropertyAnimation_0_user_data->target = TargetObject;
    PropertyAnimation_0_user_data->val = -1;
    lv_anim_t PropertyAnimation_0;
    lv_anim_init(&PropertyAnimation_0);
    lv_anim_set_time(&PropertyAnimation_0, 300);
    lv_anim_set_user_data(&PropertyAnimation_0, PropertyAnimation_0_user_data);
    lv_anim_set_custom_exec_cb(&PropertyAnimation_0, _ui_anim_callback_set_y);
    lv_anim_set_values(&PropertyAnimation_0, 30, -8);
    lv_anim_set_path_cb(&PropertyAnimation_0, lv_anim_path_ease_out);
    lv_anim_set_delay(&PropertyAnimation_0, delay + 0);
    lv_anim_set_deleted_cb(&PropertyAnimation_0, _ui_anim_callback_free_user_data);
    lv_anim_set_playback_time(&PropertyAnimation_0, 0);
    lv_anim_set_playback_delay(&PropertyAnimation_0, 0);
    lv_anim_set_repeat_count(&PropertyAnimation_0, 0);
    lv_anim_set_repeat_delay(&PropertyAnimation_0, 0);
    lv_anim_set_early_apply(&PropertyAnimation_0, true);
    lv_anim_start(&PropertyAnimation_0);

}

///////////////////// FUNCTIONS ////////////////////
void ui_event_Splash(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t * target = lv_event_get_target(e);
    if(event_code == LV_EVENT_SCREEN_LOADED) {
        upanim_Animation(ui_SLS_Logo, 100);
        upanim_Animation(ui_Smart_Gadget, 200);
        upanim_Animation(ui_Demo, 300);
        _ui_screen_change(&ui_Clock, LV_SCR_LOAD_ANIM_FADE_ON, 200, 1400, &ui_Clock_screen_init);
    }
}
void ui_event_Clock(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t * target = lv_event_get_target(e);
    if(event_code == LV_EVENT_SCREEN_LOAD_START) {
        upanim_Animation(ui_Clock_Panel, 100);
        upanim_Animation(ui_Clock_Number, 300);
        upanim_Animation(ui_Date, 200);
        scrolldot_Animation(ui_Scrolldots, 0);
    }
    if(event_code == LV_EVENT_GESTURE &&  lv_indev_get_gesture_dir(lv_indev_get_act()) == LV_DIR_LEFT) {
        lv_indev_wait_release(lv_indev_get_act());
        _ui_screen_change(&ui_Call, LV_SCR_LOAD_ANIM_FADE_ON, 10, 0, &ui_Call_screen_init);
    }
    if(event_code == LV_EVENT_GESTURE &&  lv_indev_get_gesture_dir(lv_indev_get_act()) == LV_DIR_RIGHT) {
        lv_indev_wait_release(lv_indev_get_act());
        _ui_screen_change(&ui_Alarm, LV_SCR_LOAD_ANIM_FADE_ON, 0, 0, &ui_Alarm_screen_init);
    }
    if(event_code == LV_EVENT_SCREEN_LOAD_START) {
        hour_Animation(ui_Hour, 200);
        min_Animation(ui_Min, 400);
    }
}
void ui_event_Call(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t * target = lv_event_get_target(e);
    if(event_code == LV_EVENT_SCREEN_LOADED) {
        upanim_Animation(ui_Avatar, 50);
        upanim_Animation(ui_Elena, 200);
        upanim_Animation(ui_Incoming, 300);
        upanim_Animation(ui_Call_Incon1, 200);
        upanim_Animation(ui_Call_Incon2, 300);
        scrolldot_Animation(ui_Scrolldots1, 0);
    }
    if(event_code == LV_EVENT_GESTURE &&  lv_indev_get_gesture_dir(lv_indev_get_act()) == LV_DIR_LEFT) {
        lv_indev_wait_release(lv_indev_get_act());
        _ui_screen_change(&ui_Chat, LV_SCR_LOAD_ANIM_FADE_ON, 0, 0, &ui_Chat_screen_init);
    }
    if(event_code == LV_EVENT_GESTURE &&  lv_indev_get_gesture_dir(lv_indev_get_act()) == LV_DIR_RIGHT) {
        lv_indev_wait_release(lv_indev_get_act());
        _ui_screen_change(&ui_Clock, LV_SCR_LOAD_ANIM_FADE_ON, 0, 0, &ui_Clock_screen_init);
    }
}
void ui_event_Chat(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t * target = lv_event_get_target(e);
    if(event_code == LV_EVENT_SCREEN_LOADED) {
        upanim_Animation(ui_Chat_date, 100);
        upanim_Animation(ui_C1, 200);
        upanim_Animation(ui_C2, 300);
        upanim_Animation(ui_C3, 400);
        scrolldot_Animation(ui_Scrolldots2, 0);
    }
    if(event_code == LV_EVENT_GESTURE &&  lv_indev_get_gesture_dir(lv_indev_get_act()) == LV_DIR_RIGHT) {
        lv_indev_wait_release(lv_indev_get_act());
        _ui_screen_change(&ui_Call, LV_SCR_LOAD_ANIM_FADE_ON, 0, 0, &ui_Call_screen_init);
    }
    if(event_code == LV_EVENT_GESTURE &&  lv_indev_get_gesture_dir(lv_indev_get_act()) == LV_DIR_LEFT) {
        lv_indev_wait_release(lv_indev_get_act());
        _ui_screen_change(&ui_Music_Player, LV_SCR_LOAD_ANIM_FADE_ON, 0, 0, &ui_Music_Player_screen_init);
    }
}
void ui_event_Music_Player(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t * target = lv_event_get_target(e);
    if(event_code == LV_EVENT_SCREEN_LOADED) {
        upanim_Animation(ui_Album, 100);
        upanim_Animation(ui_Music_Title, 200);
        upanim_Animation(ui_Author, 300);
        upanim_Animation(ui_Play_btn, 200);
        upanim_Animation(ui_Forward, 300);
        upanim_Animation(ui_Backward, 400);
        scrolldot_Animation(ui_Scrolldots3, 0);
    }
    if(event_code == LV_EVENT_GESTURE &&  lv_indev_get_gesture_dir(lv_indev_get_act()) == LV_DIR_RIGHT) {
        lv_indev_wait_release(lv_indev_get_act());
        _ui_screen_change(&ui_Chat, LV_SCR_LOAD_ANIM_FADE_ON, 0, 0, &ui_Chat_screen_init);
    }
    if(event_code == LV_EVENT_GESTURE &&  lv_indev_get_gesture_dir(lv_indev_get_act()) == LV_DIR_LEFT) {
        lv_indev_wait_release(lv_indev_get_act());
        _ui_screen_change(&ui_Weather, LV_SCR_LOAD_ANIM_FADE_ON, 0, 0, &ui_Weather_screen_init);
    }
}
void ui_event_Weather(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t * target = lv_event_get_target(e);
    if(event_code == LV_EVENT_SCREEN_LOADED) {
        upanim_Animation(ui_Cloud, 100);
        upanim_Animation(ui_Pary_Cloud, 200);
        upanim_Animation(ui_Celsius, 300);
        upanim_Animation(ui_New_York, 400);
        upanim_Animation(ui_Weather_Icons, 300);
        scrolldot_Animation(ui_Scrolldots4, 0);
    }
    if(event_code == LV_EVENT_GESTURE &&  lv_indev_get_gesture_dir(lv_indev_get_act()) == LV_DIR_RIGHT) {
        lv_indev_wait_release(lv_indev_get_act());
        _ui_screen_change(&ui_Music_Player, LV_SCR_LOAD_ANIM_FADE_ON, 0, 0, &ui_Music_Player_screen_init);
    }
    if(event_code == LV_EVENT_GESTURE &&  lv_indev_get_gesture_dir(lv_indev_get_act()) == LV_DIR_LEFT) {
        lv_indev_wait_release(lv_indev_get_act());
        _ui_screen_change(&ui_Alarm, LV_SCR_LOAD_ANIM_FADE_ON, 0, 0, &ui_Alarm_screen_init);
    }
}
void ui_event_Alarm(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t * target = lv_event_get_target(e);
    if(event_code == LV_EVENT_SCREEN_LOADED) {
        upanim_Animation(ui_Set_alarm, 100);
        upanim_Animation(ui_Alarm_Comp, 200);
        upanim_Animation(ui_Alarm_Comp1, 300);
        upanim_Animation(ui_Alarm_Comp2, 400);
        upanim_Animation(ui_Alarm_Comp3, 500);
        scrolldot_Animation(ui_Scrolldots5, 0);
    }
    if(event_code == LV_EVENT_GESTURE &&  lv_indev_get_gesture_dir(lv_indev_get_act()) == LV_DIR_RIGHT) {
        lv_indev_wait_release(lv_indev_get_act());
        _ui_screen_change(&ui_Weather, LV_SCR_LOAD_ANIM_FADE_ON, 0, 0, &ui_Weather_screen_init);
    }
    if(event_code == LV_EVENT_GESTURE &&  lv_indev_get_gesture_dir(lv_indev_get_act()) == LV_DIR_LEFT) {
        lv_indev_wait_release(lv_indev_get_act());
        _ui_screen_change(&ui_Clock, LV_SCR_LOAD_ANIM_FADE_ON, 0, 0, &ui_Clock_screen_init);
    }
}
void ui_event____initial_actions0(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t * target = lv_event_get_target(e);
    if(event_code == LV_EVENT_SCREEN_LOAD_START) {
        sec_Animation(ui_Sec, 0);
    }
}

///////////////////// SCREENS ////////////////////

void ui_init(void)
{

   LV_EVENT_GET_COMP_CHILD = lv_event_register_id();

    lv_disp_t * dispp = lv_disp_get_default();
    lv_theme_t * theme = lv_theme_basic_init(dispp);
    lv_disp_set_theme(dispp, theme);
    ui_Splash_screen_init();
    ui_Clock_screen_init();
    ui_Call_screen_init();
    ui_Chat_screen_init(); 
    ui_Music_Player_screen_init();
    ui_Weather_screen_init();
    ui_Alarm_screen_init();
    ui____initial_actions0 = lv_obj_create(NULL);
    lv_obj_add_event_cb(ui____initial_actions0, ui_event____initial_actions0, LV_EVENT_ALL, NULL);

    lv_disp_load_scr(ui____initial_actions0); 
    lv_disp_load_scr(ui_Splash);

}
