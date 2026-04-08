#include "TaskList.h"
#include "Referee.h"
#include "ui.h"
#include "ShootTask.h"
#include "ui_interface.h"
#include <cstring>

using namespace Device;
uint8_t color =8; // 初始颜色根据摩擦轮状态设置

// 根据裁判系统数据更新ui_self_id
static void update_ui_self_id_from_referee() {
    Referee &referee = Referee::getInstance();
    
    // 获取GameRobotState数据
    auto &robot_state = referee.getRefereeInfo<RefereeType::GameRobotState>();
    
    // 根据robot_id设置ui_self_id
    // 红方英雄: 1, 蓝方英雄: 11
    // 注意：UI系统需要的是发送者ID，对于英雄机器人：
    // 红方英雄发送者ID = 1, 蓝方英雄发送者ID = 11
    ui_set_self_id(robot_state.robot_id);
    
    // 调试信息（可选）
    // printf("UI Self ID updated to: %d\n", robot_state.robot_id);
}
// 简单封装：将形状填好并立即发送
static void send_5_figures(ui_5_frame_t &frame) {
    ui_proc_5_frame(&frame);
    SEND_MESSAGE(reinterpret_cast<uint8_t *>(&frame), sizeof(frame));
}

static void send_1_figure(ui_1_frame_t &frame) {
    ui_proc_1_frame(&frame);
    SEND_MESSAGE(reinterpret_cast<uint8_t *>(&frame), sizeof(frame));
}

static void send_string(const char name[3], uint8_t op_type, uint8_t layer, uint8_t color, uint8_t size,
                        uint8_t width, uint16_t x, uint16_t y, const char *text) {
    ui_string_frame_t frame{};
    auto *str = &frame.option;
    std::memcpy(str->figure_name, name, 3);
    str->operate_type = op_type;      // add=1, modify=2
    str->figure_type = 7;       // string
    str->layer = layer;
    str->color = color;
    str->font_size = size;
    str->str_length = 0;        // real length set by ui_proc
    str->width = width;
    str->start_x = x;
    str->start_y = y;
    std::strncpy(str->string, text, sizeof(str->string));
    ui_proc_string_frame(&frame);
    SEND_MESSAGE(reinterpret_cast<uint8_t *>(&frame), sizeof(frame));
}

// 绘制固定UI：一条瞄准线、四个电量圆点、一个矩形瞄准框和 5 个字符标签
static void draw_hero_ui() {
    // 瞄准线 + 4 个圆点
    ui_5_frame_t f1{};
    auto *line = reinterpret_cast<ui_interface_line_t *>(&f1.data[0]);
    std::memcpy(line->figure_name, "L00", 3);
    line->operate_type = 1;
    line->figure_type = 0;  // line
    line->layer = 9;
    line->color = 4;        // 紫红
    line->width = 2;
    line->start_x = 571; line->start_y = 470;
    line->end_x = 1345;  line->end_y = 470;

    for (int i = 1; i <= 4; ++i) {
        auto *c = reinterpret_cast<ui_interface_round_t *>(&f1.data[i]);
        char name[3] = {'C', static_cast<char>('0' + i), '0'}; // C10,C20...
        std::memcpy(c->figure_name, name, 3);
        c->operate_type = 1;
        c->figure_type = 2; // circle
        c->layer = 9;
        c->color = 3;       // 橙色
        c->width = 15;
        c->start_y = 820;
        c->r = 15;
    }
    auto *c1 = reinterpret_cast<ui_interface_round_t *>(&f1.data[1]); c1->start_x = 230;
    auto *c2 = reinterpret_cast<ui_interface_round_t *>(&f1.data[2]); c2->start_x = 280;
    auto *c3 = reinterpret_cast<ui_interface_round_t *>(&f1.data[3]); c3->start_x = 330;
    auto *c4 = reinterpret_cast<ui_interface_round_t *>(&f1.data[4]); c4->start_x = 380;
    send_5_figures(f1);

    // 矩形瞄准框
    ui_1_frame_t f_rect{};
    auto *rect = reinterpret_cast<ui_interface_rect_t *>(&f_rect.data[0]);
    std::memcpy(rect->figure_name, "R00", 3);
    rect->operate_type = 1;
    rect->figure_type = 1;  // rectangle
    rect->layer = 8;
    rect->color = 4;        // 紫红
    rect->width = 3;
    rect->start_x = 900; rect->start_y = 460;
    rect->end_x = 1020; rect->end_y = 400;
    send_1_figure(f_rect);

    // 文本标签
    send_string("GYO", 1, 8, 1, 24, 4, 60, 890, "GYRO");
    send_string("CAP", 1, 8, 1, 24, 3, 60, 830, "CAP");
    // SHOOT标签初始颜色为白色（摩擦轮关闭）
    send_string("SHT", 1, 8, color, 24, 5, 60, 770, "SHOOT");
    send_string("AUT", 1, 8, 1, 24, 4, 60, 710, "AUTO");
    send_string("PRM", 1, 8, 1, 24, 5, 910, 880, "PRIME");
}


void ClientUI_Task(void const * argument){

    Referee &hreferee = Referee::getInstance();
    bool last_referee_exist = hreferee.RefereeExist();
    bool ui_inited = false;
    bool last_frib_state = false;
    uint32_t ui_update_counter = 0;
    auto &hVT03 = VT03::getInstance();

    static bool last_G = false;
    static bool last_V = false;

    while (1)
    {
        bool Fric = ShootFSM::getIsFribOpened();
        if (Fric){color=2;}
        else{color=8;}
        // color = Fric ? 2 : 8; // 摩擦轮打开为绿色，关闭为白色
        bool ref_exist = hreferee.RefereeExist();
        
        // 检测裁判系统连接状态变化
        if(last_referee_exist != ref_exist) {
            last_referee_exist = ref_exist;
            if(ref_exist){
                osDelay(200);
                // 根据裁判系统数据更新ui_self_id
                update_ui_self_id_from_referee();
                ui_remove_g_Ungroup(); // 清理旧 UI
                osDelay(100);
                draw_hero_ui();
                // osDelay(100);
                ui_inited = true;
            }
        }

        // 裁判系统已连接但UI未初始化
        if(ref_exist && !ui_inited){
            // 根据裁判系统数据更新ui_self_id
            update_ui_self_id_from_referee();
            // draw_hero_ui();
            osDelay(100);
            ui_inited = true;
        }

        if (last_frib_state!=Fric)
        {
            // Only update SHOOT label color (operate_type = 2 for modify)
            send_string("SHT", 2, 8, color, 24, 5, 60, 770, "SHOOT");
            // update other label status or just delay
            osDelay(100);
        }
        // 更新上次摩擦轮状态
        last_frib_state = Fric;

        bool G = hVT03.getState()->key_code & (1 << 10);
        bool V = hVT03.getState()->key_code & (1 << 14);

        if (G && !last_G) {
            draw_hero_ui();
        } else if (V && !last_V) {
            ui_remove_g_Ungroup();
        }

        last_G = G;
        last_V = V;

        osDelay(50);
    }

}
