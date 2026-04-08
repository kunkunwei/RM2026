//
// Created by RM UI Designer
// Static Edition
//

#include <string.h>

#include "ui_interface.h"

ui_5_frame_t ui_g_Ungroup_0;

ui_interface_line_t *ui_g_Ungroup_engbar_bg = (ui_interface_line_t*)&(ui_g_Ungroup_0.data[0]);
ui_interface_line_t *ui_g_Ungroup_engbar = (ui_interface_line_t*)&(ui_g_Ungroup_0.data[1]);
ui_interface_number_t *ui_g_Ungroup_NewNumber = (ui_interface_number_t*)&(ui_g_Ungroup_0.data[2]);
ui_interface_round_t *ui_g_Ungroup_NewRound = (ui_interface_round_t*)&(ui_g_Ungroup_0.data[3]);

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

    ui_g_Ungroup_engbar_bg->figure_type = 0;
    ui_g_Ungroup_engbar_bg->operate_type = 1;
    ui_g_Ungroup_engbar_bg->layer = 0;
    ui_g_Ungroup_engbar_bg->color = 3;
    ui_g_Ungroup_engbar_bg->start_x = 1296;
    ui_g_Ungroup_engbar_bg->start_y = 225;
    ui_g_Ungroup_engbar_bg->width = 60;
    ui_g_Ungroup_engbar_bg->end_x = 1792;
    ui_g_Ungroup_engbar_bg->end_y = 225;

    ui_g_Ungroup_engbar->figure_type = 0;
    ui_g_Ungroup_engbar->operate_type = 1;
    ui_g_Ungroup_engbar->layer = 1;
    ui_g_Ungroup_engbar->color = 2;
    ui_g_Ungroup_engbar->start_x = 1301;
    ui_g_Ungroup_engbar->start_y = 230;
    ui_g_Ungroup_engbar->width = 50;
    ui_g_Ungroup_engbar->end_x = 1786;
    ui_g_Ungroup_engbar->end_y = 230;

    ui_g_Ungroup_NewNumber->figure_type = 6;
    ui_g_Ungroup_NewNumber->operate_type = 1;
    ui_g_Ungroup_NewNumber->layer = 3;
    ui_g_Ungroup_NewNumber->color = 7;
    ui_g_Ungroup_NewNumber->start_x = 1325;
    ui_g_Ungroup_NewNumber->start_y = 266;
    ui_g_Ungroup_NewNumber->width = 2;
    ui_g_Ungroup_NewNumber->font_size = 20;
    ui_g_Ungroup_NewNumber->number = 145;

    ui_g_Ungroup_NewRound->figure_type = 2;
    ui_g_Ungroup_NewRound->operate_type = 1;
    ui_g_Ungroup_NewRound->layer = 0;
    ui_g_Ungroup_NewRound->color = 5;
    ui_g_Ungroup_NewRound->start_x = 263;
    ui_g_Ungroup_NewRound->start_y = 557;
    ui_g_Ungroup_NewRound->width = 20;
    ui_g_Ungroup_NewRound->r = 91;


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


void ui_init_g_Ungroup() {
    _ui_init_g_Ungroup_0();
}

void ui_update_g_Ungroup() {
    _ui_update_g_Ungroup_0();
}

void ui_remove_g_Ungroup() {
    _ui_remove_g_Ungroup_0();
}

