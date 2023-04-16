#include <Servo.h>

#include <SPI.h>
#include <U8g2lib.h>
#include "max6675.h"

#define GRAPH_WIDTH 128
#define GRAPH_HEIGHT 32
#define TEMP_SCALE 4
#define TEMP_SHIFT_F 17
#define TICK_MS 1000
#define AVG_TICKS 4
#define GRAPH_TICKS 16

#define DISPLAY_CS_PIN 9
#define DISPLAY_DC_PIN 6
#define DISPLAY_RES_PIN 8

#define TEMP_CLK_PIN 3
#define TEMP_CS_PIN 4
#define TEMP_DO_PIN 5

#define VALVE_SERVO_PIN 7
#define VALVE_MIN 50
#define VALVE_MAX 160

U8G2_SSD1309_128X64_NONAME0_F_4W_HW_SPI u8g2(U8G2_R0, DISPLAY_CS_PIN, DISPLAY_DC_PIN, DISPLAY_RES_PIN);
MAX6675 thermocouple(TEMP_CLK_PIN, TEMP_CS_PIN, TEMP_DO_PIN);
Servo valve;

int16_t s_temp_f(int16_t s_temp) {
  return (s_temp * 9 + (5 * TEMP_SCALE / 2)) / (5 * TEMP_SCALE) + 32 - TEMP_SHIFT_F;
}

int16_t temp_f(int16_t temp) {
  return s_temp_f(temp * TEMP_SCALE);
}

int16_t s_temp_from_f(int16_t temp_f) {
  return ((temp_f - 32 + TEMP_SHIFT_F) * 5 * TEMP_SCALE + 4) / 9;
}

uint8_t temp_graph[GRAPH_WIDTH];
uint8_t valve_graph[GRAPH_WIDTH];
bool graph_full = false;

int16_t temp_avg_buf[AVG_TICKS];

uint8_t current_graph_idx = 0;
uint8_t current_avg_idx = 0;
uint8_t graph_tick_cnt = 0;

int32_t temp_avg_sum = 0;
uint16_t valve_position = 65535;
int32_t current_err = 0;

uint16_t target_temp =
  s_temp_from_f(225); // PROD
  //s_temp_from_f(76);  // TEST

int32_t P_term = 8;
int32_t D_term = 4096;

char strbuf[15];


void setup() {
  //Serial.begin(57600);

  valve.attach(VALVE_SERVO_PIN);
  u8g2.begin();
  //u8g2.setFont(u8g2_font_5x7_mf);
  //u8g2.setDrawColor(2);
  int16_t temp = (int16_t)(thermocouple.readCelsius() * TEMP_SCALE);
  for (uint8_t i = 0; i < AVG_TICKS; i++) {
    temp_avg_buf[i] = temp;
  }
}

void loop() {
  read_temp();
  adjust_valve();

  u8g2.clearBuffer();
  //u8g2.drawHLine(0, 32, 128);
  u8g2.setFont(u8g2_font_5x7_mf);
  sprintf(strbuf, "T:%uF\0", s_temp_f(temp_avg_sum / AVG_TICKS));
  u8g2.drawStr(0, 18, strbuf);
  //Serial.println(strbuf);  
  sprintf(strbuf, "V:%u%%\0", (valve_position / 256) * 101 / 256);
  u8g2.drawStr(0, 50, strbuf);
  draw_graphs();
  u8g2.sendBuffer();

  if (graph_tick_cnt >= GRAPH_TICKS) {
    current_graph_idx += 1;
    if (current_graph_idx >= GRAPH_WIDTH) {
      current_graph_idx = 0;
      graph_full = true;
    }
    graph_tick_cnt = 0;
  } else graph_tick_cnt += 1;

  delay(TICK_MS);
}

void draw_graphs() {
  uint8_t t_max = 0;
  uint8_t t_min = 255;
  for (uint8_t i = 0; i < (graph_full ? GRAPH_WIDTH : current_graph_idx + 1); i++) {
    if (temp_graph[i] < t_min) t_min = temp_graph[i];
    if (temp_graph[i] > t_max) t_max = temp_graph[i];
  }
  if (t_max - t_min < GRAPH_HEIGHT) {
    uint8_t mid = (t_max/2) + (t_min/2);
    t_max = mid + (GRAPH_HEIGHT/2);
    t_min = mid - (GRAPH_HEIGHT/2);
  }
  uint16_t scale = (t_max - t_min);
  uint8_t tgt = (uint8_t)(target_temp / TEMP_SCALE);

  u8g2.setFont(u8g2_font_4x6_mf);
  sprintf(strbuf, "MAX:%u\0", temp_f((int16_t)t_max));
  u8g2.drawStr(0, 6, strbuf);
  sprintf(strbuf, "MIN:%u\0", temp_f((int16_t)t_min));
  u8g2.drawStr(0, 30, strbuf);
  sprintf(strbuf, "TGT:%uF\0", temp_f((int16_t)tgt));
  u8g2.drawStr(96, 30, strbuf);

 
  if (tgt >= t_min && tgt <= t_max) {
    uint8_t display_value = (uint8_t)((uint16_t)(tgt - t_min) * GRAPH_HEIGHT / scale);
    draw_dotted_hline(32, GRAPH_HEIGHT - display_value - 1);
  }

  for (uint8_t i = 0; i < (graph_full ? GRAPH_WIDTH : (current_graph_idx + 1)); i++) {
    uint8_t graph_x = (i > current_graph_idx) ? (i - current_graph_idx) : (GRAPH_WIDTH - 1 + i - current_graph_idx);
    //graph_x = GRAPH_WIDTH - graph_x - 1;
    //if (graph_index >= GRAPH_WIDTH) graph_index -= GRAPH_WIDTH;
    uint8_t display_value = (uint8_t)((uint16_t)(temp_graph[i] - t_min) * GRAPH_HEIGHT / scale);
    u8g2.drawPixel(graph_x, GRAPH_HEIGHT - display_value - 1);
    display_value = valve_graph[i] / (256 / GRAPH_HEIGHT);
    u8g2.drawPixel(graph_x, (2 * GRAPH_HEIGHT) - display_value - 1);
  }
  
}

void read_temp() {
  temp_avg_buf[current_avg_idx] = (int16_t)(thermocouple.readCelsius() * TEMP_SCALE);
  current_avg_idx += 1;
  if (current_avg_idx >= AVG_TICKS) current_avg_idx = 0;  
  temp_avg_sum = 0;
  for (uint8_t i = 0; i < AVG_TICKS; i++) temp_avg_sum += temp_avg_buf[i];
  temp_graph[current_graph_idx] = (uint8_t)(temp_avg_sum / (AVG_TICKS * TEMP_SCALE));
}


void adjust_valve() {
  int32_t error = target_temp * AVG_TICKS - temp_avg_sum;
  int16_t valve_adj = (int16_t)((P_term * error + D_term * (error - current_err))/(AVG_TICKS * TEMP_SCALE));
  current_err = error;
  if (valve_adj < 0 && valve_position < -valve_adj) valve_position = 0;
  else if (valve_adj > 0 && 65535-valve_position < valve_adj) valve_position = 65535;
  else valve_position += valve_adj;
  valve.write((valve_position  / 256) * (VALVE_MAX - VALVE_MIN) / 256 + VALVE_MIN);
  valve_graph[current_graph_idx] = (uint8_t)(valve_position / 256);
}

void draw_dotted_hline(uint8_t start, uint8_t y) {
  for (uint8_t i = start; i < 128; i+=2) u8g2.drawPixel(i, y);
}