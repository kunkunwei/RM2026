#ifndef BUZZER_MUSIC_H
#define BUZZER_MUSIC_H

#include "main.h"

// 音符频率定义

// 低音区
#define NOTE_C3  131
#define NOTE_D3  147
#define NOTE_E3  165
#define NOTE_F3  175
#define NOTE_G3  196
#define NOTE_A3  220
#define NOTE_B3  247

// 中音区
#define NOTE_C4  262
#define NOTE_D4  294
#define NOTE_E4  330
#define NOTE_F4  349
#define NOTE_G4  392
#define NOTE_A4  440
#define NOTE_B4  494

// 高音区
#define NOTE_C5  523
#define NOTE_D5  587
#define NOTE_E5  659
#define NOTE_F5  698
#define NOTE_G5  784
#define NOTE_A5  880
#define NOTE_B5  988

//更高音区
#define NOTE_C6  1047
#define NOTE_D6  1175
#define NOTE_E6  1319
#define NOTE_F6  1397
#define NOTE_G6  1568
#define NOTE_A6  1760
#define NOTE_B6  1976

// 定义音符和持续时间
typedef struct {
    uint16_t freq;
    uint16_t duration;
} music_note_t;

// “鸡你太美”音乐简谱 - 修正版
// “鸡你太美”音乐简谱 - 最终修正版
// “鸡你太美”音乐简谱 - 社区验证版
static const music_note_t jntm_score[] = {
    {NOTE_A4, 90}, {NOTE_A4, 90}, {NOTE_A4, 90}, {NOTE_G4, 180},
    {NOTE_A4, 90}, {NOTE_G4, 90}, {NOTE_A4, 180}, {0, 90},
    {NOTE_E4, 90}, {NOTE_E4, 90}, {NOTE_E4, 90}, {NOTE_D4, 180},
    {NOTE_E4, 90}, {NOTE_D4, 90}, {NOTE_E4, 180}, {0, 180},
};


void play_music(const music_note_t* score, uint16_t score_size);

#endif //BUZZER_MUSIC_H
