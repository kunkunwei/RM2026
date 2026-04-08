//
// Created by RM UI Designer
// Static Edition
//

#include <string.h>

#include "ui_interface.h"

ui_5_frame_t ui_g_Ungroup_0;

ui_interface_number_t *ui_g_Ungroup_EGY_NUM = (ui_interface_number_t*)&(ui_g_Ungroup_0.data[0]);
ui_interface_ellipse_t *ui_g_Ungroup_LAUNCH3508 = (ui_interface_ellipse_t*)&(ui_g_Ungroup_0.data[1]);
ui_interface_ellipse_t *ui_g_Ungroup_AIM = (ui_interface_ellipse_t*)&(ui_g_Ungroup_0.data[2]);
ui_interface_ellipse_t *ui_g_Ungroup_AUTOROTATE = (ui_interface_ellipse_t*)&(ui_g_Ungroup_0.data[3]);

void _ui_init_g_Ungroup_0() {
    for (int i = 0; i < 4; i++) {
        ui_g_Ungroup_0.data[i].figure_name[0] = 0;
        ui_g_Ungroup_0.data[i].figure_name[1] = 0;
        ui_g_Ungroup_0.data[i].figure_name[2] = i + 0;
        ui_g_Ungroup_0.data[i].operate_type = 1;
    }
    for (int i = 4; i < 5; i++) {
        ui_g_Ungroup_0.data[i].operate_type = 0;
    }

    ui_g_Ungroup_EGY_NUM->figure_type = 6;
    ui_g_Ungroup_EGY_NUM->operate_type = 1;
    ui_g_Ungroup_EGY_NUM->layer = 0;
    ui_g_Ungroup_EGY_NUM->color = 1;
    ui_g_Ungroup_EGY_NUM->start_x = 538;
    ui_g_Ungroup_EGY_NUM->start_y = 155;
    ui_g_Ungroup_EGY_NUM->width = 4;
    ui_g_Ungroup_EGY_NUM->font_size = 40;
    ui_g_Ungroup_EGY_NUM->number = 1;

    ui_g_Ungroup_LAUNCH3508->figure_type = 3;
    ui_g_Ungroup_LAUNCH3508->operate_type = 1;
    ui_g_Ungroup_LAUNCH3508->layer = 0;
    ui_g_Ungroup_LAUNCH3508->color = 1;
    ui_g_Ungroup_LAUNCH3508->start_x = 1484;
    ui_g_Ungroup_LAUNCH3508->start_y = 590;
    ui_g_Ungroup_LAUNCH3508->width = 20;
    ui_g_Ungroup_LAUNCH3508->rx = 10;
    ui_g_Ungroup_LAUNCH3508->ry = 10;

    ui_g_Ungroup_AIM->figure_type = 3;
    ui_g_Ungroup_AIM->operate_type = 1;
    ui_g_Ungroup_AIM->layer = 0;
    ui_g_Ungroup_AIM->color = 1;
    ui_g_Ungroup_AIM->start_x = 1485;
    ui_g_Ungroup_AIM->start_y = 536;
    ui_g_Ungroup_AIM->width = 20;
    ui_g_Ungroup_AIM->rx = 10;
    ui_g_Ungroup_AIM->ry = 10;

    ui_g_Ungroup_AUTOROTATE->figure_type = 3;
    ui_g_Ungroup_AUTOROTATE->operate_type = 1;
    ui_g_Ungroup_AUTOROTATE->layer = 0;
    ui_g_Ungroup_AUTOROTATE->color = 1;
    ui_g_Ungroup_AUTOROTATE->start_x = 1483;
    ui_g_Ungroup_AUTOROTATE->start_y = 646;
    ui_g_Ungroup_AUTOROTATE->width = 20;
    ui_g_Ungroup_AUTOROTATE->rx = 10;
    ui_g_Ungroup_AUTOROTATE->ry = 10;


    ui_proc_5_frame(&ui_g_Ungroup_0);
    SEND_MESSAGE((uint8_t *) &ui_g_Ungroup_0, sizeof(ui_g_Ungroup_0));
}

void _ui_update_g_Ungroup_0() {
    for (int i = 0; i < 4; i++) {
        ui_g_Ungroup_0.data[i].operate_type = 2;
    }

    ui_proc_5_frame(&ui_g_Ungroup_0);
    SEND_MESSAGE((uint8_t *) &ui_g_Ungroup_0, sizeof(ui_g_Ungroup_0));
}

void _ui_remove_g_Ungroup_0() {
    for (int i = 0; i < 4; i++) {
        ui_g_Ungroup_0.data[i].operate_type = 3;
    }

    ui_proc_5_frame(&ui_g_Ungroup_0);
    SEND_MESSAGE((uint8_t *) &ui_g_Ungroup_0, sizeof(ui_g_Ungroup_0));
}

ui_string_frame_t ui_g_Ungroup_1;
ui_interface_string_t* ui_g_Ungroup_AutoRotateText = &(ui_g_Ungroup_1.option);

void _ui_init_g_Ungroup_1() {
    ui_g_Ungroup_1.option.figure_name[0] = 0;
    ui_g_Ungroup_1.option.figure_name[1] = 0;
    ui_g_Ungroup_1.option.figure_name[2] = 4;
    ui_g_Ungroup_1.option.operate_type = 1;

    ui_g_Ungroup_AutoRotateText->figure_type = 7;
    ui_g_Ungroup_AutoRotateText->operate_type = 1;
    ui_g_Ungroup_AutoRotateText->layer = 0;
    ui_g_Ungroup_AutoRotateText->color = 3;
    ui_g_Ungroup_AutoRotateText->start_x = 1525;
    ui_g_Ungroup_AutoRotateText->start_y = 665;
    ui_g_Ungroup_AutoRotateText->width = 2;
    ui_g_Ungroup_AutoRotateText->font_size = 20;
    ui_g_Ungroup_AutoRotateText->str_length = 6;
    strcpy(ui_g_Ungroup_AutoRotateText->string, "Rotate");


    ui_proc_string_frame(&ui_g_Ungroup_1);
    SEND_MESSAGE((uint8_t *) &ui_g_Ungroup_1, sizeof(ui_g_Ungroup_1));
}

void _ui_update_g_Ungroup_1() {
    ui_g_Ungroup_1.option.operate_type = 2;

    ui_proc_string_frame(&ui_g_Ungroup_1);
    SEND_MESSAGE((uint8_t *) &ui_g_Ungroup_1, sizeof(ui_g_Ungroup_1));
}

void _ui_remove_g_Ungroup_1() {
    ui_g_Ungroup_1.option.operate_type = 3;

    ui_proc_string_frame(&ui_g_Ungroup_1);
    SEND_MESSAGE((uint8_t *) &ui_g_Ungroup_1, sizeof(ui_g_Ungroup_1));
}
ui_string_frame_t ui_g_Ungroup_2;
ui_interface_string_t* ui_g_Ungroup_ShootText = &(ui_g_Ungroup_2.option);

void _ui_init_g_Ungroup_2() {
    ui_g_Ungroup_2.option.figure_name[0] = 0;
    ui_g_Ungroup_2.option.figure_name[1] = 0;
    ui_g_Ungroup_2.option.figure_name[2] = 5;
    ui_g_Ungroup_2.option.operate_type = 1;

    ui_g_Ungroup_ShootText->figure_type = 7;
    ui_g_Ungroup_ShootText->operate_type = 1;
    ui_g_Ungroup_ShootText->layer = 0;
    ui_g_Ungroup_ShootText->color = 3;
    ui_g_Ungroup_ShootText->start_x = 1525;
    ui_g_Ungroup_ShootText->start_y = 610;
    ui_g_Ungroup_ShootText->width = 2;
    ui_g_Ungroup_ShootText->font_size = 20;
    ui_g_Ungroup_ShootText->str_length = 5;
    strcpy(ui_g_Ungroup_ShootText->string, "Shoot");


    ui_proc_string_frame(&ui_g_Ungroup_2);
    SEND_MESSAGE((uint8_t *) &ui_g_Ungroup_2, sizeof(ui_g_Ungroup_2));
}

void _ui_update_g_Ungroup_2() {
    ui_g_Ungroup_2.option.operate_type = 2;

    ui_proc_string_frame(&ui_g_Ungroup_2);
    SEND_MESSAGE((uint8_t *) &ui_g_Ungroup_2, sizeof(ui_g_Ungroup_2));
}

void _ui_remove_g_Ungroup_2() {
    ui_g_Ungroup_2.option.operate_type = 3;

    ui_proc_string_frame(&ui_g_Ungroup_2);
    SEND_MESSAGE((uint8_t *) &ui_g_Ungroup_2, sizeof(ui_g_Ungroup_2));
}
ui_string_frame_t ui_g_Ungroup_3;
ui_interface_string_t* ui_g_Ungroup_AimText = &(ui_g_Ungroup_3.option);

void _ui_init_g_Ungroup_3() {
    ui_g_Ungroup_3.option.figure_name[0] = 0;
    ui_g_Ungroup_3.option.figure_name[1] = 0;
    ui_g_Ungroup_3.option.figure_name[2] = 6;
    ui_g_Ungroup_3.option.operate_type = 1;

    ui_g_Ungroup_AimText->figure_type = 7;
    ui_g_Ungroup_AimText->operate_type = 1;
    ui_g_Ungroup_AimText->layer = 0;
    ui_g_Ungroup_AimText->color = 3;
    ui_g_Ungroup_AimText->start_x = 1526;
    ui_g_Ungroup_AimText->start_y = 555;
    ui_g_Ungroup_AimText->width = 2;
    ui_g_Ungroup_AimText->font_size = 20;
    ui_g_Ungroup_AimText->str_length = 3;
    strcpy(ui_g_Ungroup_AimText->string, "Aim");


    ui_proc_string_frame(&ui_g_Ungroup_3);
    SEND_MESSAGE((uint8_t *) &ui_g_Ungroup_3, sizeof(ui_g_Ungroup_3));
}

void _ui_update_g_Ungroup_3() {
    ui_g_Ungroup_3.option.operate_type = 2;

    ui_proc_string_frame(&ui_g_Ungroup_3);
    SEND_MESSAGE((uint8_t *) &ui_g_Ungroup_3, sizeof(ui_g_Ungroup_3));
}

void _ui_remove_g_Ungroup_3() {
    ui_g_Ungroup_3.option.operate_type = 3;

    ui_proc_string_frame(&ui_g_Ungroup_3);
    SEND_MESSAGE((uint8_t *) &ui_g_Ungroup_3, sizeof(ui_g_Ungroup_3));
}

void ui_init_g_Ungroup() {
    _ui_init_g_Ungroup_0();
    _ui_init_g_Ungroup_1();
    _ui_init_g_Ungroup_2();
    _ui_init_g_Ungroup_3();
}

void ui_update_g_Ungroup() {
    _ui_update_g_Ungroup_0();
    _ui_update_g_Ungroup_1();
    _ui_update_g_Ungroup_2();
    _ui_update_g_Ungroup_3();
}

void ui_remove_g_Ungroup() {
    _ui_remove_g_Ungroup_0();
    _ui_remove_g_Ungroup_1();
    _ui_remove_g_Ungroup_2();
    _ui_remove_g_Ungroup_3();
}

