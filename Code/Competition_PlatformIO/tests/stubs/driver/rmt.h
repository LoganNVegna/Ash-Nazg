#pragma once
#include <Arduino.h>
#include <vector>
#include <assert.h>
using esp_err_t=int;using TickType_t=uint32_t;using gpio_num_t=int;
enum rmt_channel_t {RMT_CHANNEL_2=2,RMT_CHANNEL_3=3,RMT_CHANNEL_MAX=8};
constexpr int ESP_OK=0,ESP_ERR_INVALID_ARG=1,RMT_MODE_TX=0,RMT_IDLE_LEVEL_LOW=0;
constexpr int ESP_ERR_TIMEOUT=2,ESP_ERR_INVALID_STATE=3;
struct rmt_item32_t {union {uint32_t val;struct {uint32_t duration0:15;uint32_t level0:1;uint32_t duration1:15;uint32_t level1:1;};};};
struct rmt_config_t {rmt_channel_t channel;int rmt_mode;gpio_num_t gpio_num;int mem_block_num,clk_div;
    struct {bool loop_en,carrier_en;int idle_level;bool idle_output_en;} tx_config;};
struct MockFrame {int channel;uint16_t packet;};
extern std::vector<MockFrame> trace;
inline esp_err_t rmt_config(const rmt_config_t* c) {
    assert((c->channel==2 && c->gpio_num==17)||(c->channel==3 && c->gpio_num==18));
    assert(c->clk_div==3 && c->tx_config.idle_level==RMT_IDLE_LEVEL_LOW);return ESP_OK;
}
inline int mockFault[8]{},mockInstalls[8]{},mockUninstalls[8]{},mockWrites[8]{};
inline esp_err_t rmt_driver_install(rmt_channel_t c,size_t,int){++mockInstalls[c];return ESP_OK;}
inline esp_err_t rmt_driver_uninstall(rmt_channel_t c){++mockUninstalls[c];return ESP_OK;}
inline esp_err_t rmt_tx_stop(rmt_channel_t){return ESP_OK;}
inline esp_err_t rmt_wait_tx_done(rmt_channel_t c,TickType_t){return mockFault[c];}
inline esp_err_t rmt_write_items(rmt_channel_t c,const rmt_item32_t* p,int n,bool) {
    assert(n==17 && p[16].duration0==0 && p[16].duration1==0);uint16_t word=0;++mockWrites[c];
    for(int i=0;i<16;++i) {
        assert(p[i].level0==1 && p[i].level1==0);
        assert(p[i].duration0+p[i].duration1==44);
        assert(p[i].duration0==14 || p[i].duration0==29);
        word=uint16_t((word<<1)|(p[i].duration0>p[i].duration1));
    }
    auto body=word>>4;assert(((body^(body>>4)^(body>>8))&15)==(word&15));
    trace.push_back({int(c),word});return ESP_OK;
}
