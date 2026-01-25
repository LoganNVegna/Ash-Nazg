#include "DShotRMT.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

DShotRMT::DShotRMT(gpio_num_t gpio, rmt_channel_t rmtChannel) {
  dshot_config.gpio_num = gpio;
  dshot_config.pin_num = static_cast<uint8_t>(gpio);
  dshot_config.rmt_channel = rmtChannel;
  dshot_config.mem_block_num = static_cast<uint8_t>(RMT_CHANNEL_MAX - static_cast<uint8_t>(rmtChannel));
  buildTxRmtItem(DSHOT_NULL_PACKET);
}

DShotRMT::DShotRMT(uint8_t pin, uint8_t channel) {
  dshot_config.gpio_num = static_cast<gpio_num_t>(pin);
  dshot_config.pin_num = pin;
  dshot_config.rmt_channel = static_cast<rmt_channel_t>(channel);
  dshot_config.mem_block_num = RMT_CHANNEL_MAX - channel;
  buildTxRmtItem(DSHOT_NULL_PACKET);
}

DShotRMT::DShotRMT(uint8_t pin) {
  dshot_config.gpio_num = static_cast<gpio_num_t>(pin);
  dshot_config.pin_num = pin;
  dshot_config.rmt_channel = static_cast<rmt_channel_t>(RMT_CHANNEL_MAX - 1);
  dshot_config.mem_block_num = RMT_CHANNEL_MAX - 1;
  buildTxRmtItem(DSHOT_NULL_PACKET);
}

DShotRMT::~DShotRMT() {
  rmt_driver_uninstall(dshot_config.rmt_channel);
}

DShotRMT::DShotRMT(DShotRMT const &) {
  // Copy not fully supported; avoid use.
}

bool DShotRMT::begin(dshot_mode_t dshot_mode, bool is_bidirectional) {
  dshot_config.mode = dshot_mode;
  dshot_config.clk_div = DSHOT_CLK_DIVIDER;
  dshot_config.name_str = dshot_mode_name[dshot_mode];
  dshot_config.is_bidirectional = is_bidirectional;

  switch (dshot_config.mode) {
    case DSHOT150:
      dshot_config.ticks_per_bit = 64;
      dshot_config.ticks_zero_high = 24;
      dshot_config.ticks_one_high = 48;
      break;
    case DSHOT300:
      dshot_config.ticks_per_bit = 32;
      dshot_config.ticks_zero_high = 12;
      dshot_config.ticks_one_high = 24;
      break;
    case DSHOT600:
      dshot_config.ticks_per_bit = 16;
      dshot_config.ticks_zero_high = 6;
      dshot_config.ticks_one_high = 12;
      break;
    case DSHOT1200:
      dshot_config.ticks_per_bit = 8;
      dshot_config.ticks_zero_high = 3;
      dshot_config.ticks_one_high = 6;
      break;
    default:
      dshot_config.ticks_per_bit = 0;
      dshot_config.ticks_zero_high = 0;
      dshot_config.ticks_one_high = 0;
      break;
  }

  dshot_config.ticks_zero_low = (dshot_config.ticks_per_bit - dshot_config.ticks_zero_high);
  dshot_config.ticks_one_low = (dshot_config.ticks_per_bit - dshot_config.ticks_one_high);

  dshot_tx_rmt_config.rmt_mode = RMT_MODE_TX;
  dshot_tx_rmt_config.channel = dshot_config.rmt_channel;
  dshot_tx_rmt_config.gpio_num = dshot_config.gpio_num;
  dshot_tx_rmt_config.mem_block_num = dshot_config.mem_block_num;
  dshot_tx_rmt_config.clk_div = dshot_config.clk_div;
  dshot_tx_rmt_config.tx_config.loop_en = false;
  dshot_tx_rmt_config.tx_config.carrier_en = false;
  dshot_tx_rmt_config.tx_config.idle_output_en = true;

  if (dshot_config.is_bidirectional) {
    dshot_tx_rmt_config.tx_config.idle_level = RMT_IDLE_LEVEL_HIGH;
  } else {
    dshot_tx_rmt_config.tx_config.idle_level = RMT_IDLE_LEVEL_LOW;
  }

  rmt_config(&dshot_tx_rmt_config);
  return rmt_driver_install(dshot_tx_rmt_config.channel, 0, 0);
}

void DShotRMT::sendPacket(uint16_t value) {
  dshot_packet_t dshot_rmt_packet = {};
  if (value > DSHOT_THROTTLE_MAX) value = DSHOT_THROTTLE_MAX;
  dshot_rmt_packet.throttle_value = value;
  dshot_rmt_packet.telemetric_request = NO_TELEMETRIC;
  dshot_rmt_packet.checksum = calculateCRC(dshot_rmt_packet);
  sendRmtPaket(dshot_rmt_packet);
}

void DShotRMT::sendThrottleValue(uint16_t throttle_value) {
  // Match dshot300 example: throttle 48–2047 (DSHOT_THROTTLE_MIN..MAX)
  if (throttle_value < DSHOT_THROTTLE_MIN) throttle_value = DSHOT_THROTTLE_MIN;
  if (throttle_value > DSHOT_THROTTLE_MAX) throttle_value = DSHOT_THROTTLE_MAX;
  sendPacket(throttle_value);
}

void DShotRMT::sendMotorStop() {
  sendPacket(0);
}

void DShotRMT::sendCommand(dshot_cmd_t cmd, uint8_t repeat) {
  const uint16_t v = (uint16_t)cmd;
  if (v > 47) return;
  for (uint8_t i = 0; i < repeat; i++) {
    sendPacket(v);
    rmt_wait_tx_done(dshot_config.rmt_channel, pdMS_TO_TICKS(1));
    delayMicroseconds(200);
  }
}

void DShotRMT::sendThrottle3D(int16_t throttle_3d) {
  if (throttle_3d > 999) throttle_3d = 999;
  if (throttle_3d < -999) throttle_3d = -999;
  uint16_t raw;
  if (throttle_3d < 0)
    raw = (uint16_t)(1000 - throttle_3d);
  else
    raw = (uint16_t)throttle_3d;
  if (raw > DSHOT_THROTTLE_MAX - DSHOT_THROTTLE_MIN)
    raw = DSHOT_THROTTLE_MAX;
  else
    raw += DSHOT_THROTTLE_MIN;
  sendThrottleValue(raw);
}

void DShotRMT::sendRmtPaket(const dshot_packet_t &dshot_packet) {
  buildTxRmtItem(parseRmtPaket(dshot_packet));
  rmt_write_items(dshot_tx_rmt_config.channel, dshot_tx_rmt_item, DSHOT_PACKET_LENGTH, false);
}

rmt_item32_t *DShotRMT::buildTxRmtItem(uint16_t parsed_packet) {
  if (dshot_config.is_bidirectional) {
    for (int i = 0; i < DSHOT_PAUSE_BIT; i++, parsed_packet <<= 1) {
      if (parsed_packet & 0x8000) {
        dshot_tx_rmt_item[i].duration0 = dshot_config.ticks_one_low;
        dshot_tx_rmt_item[i].duration1 = dshot_config.ticks_one_high;
      } else {
        dshot_tx_rmt_item[i].duration0 = dshot_config.ticks_zero_low;
        dshot_tx_rmt_item[i].duration1 = dshot_config.ticks_zero_high;
      }
      dshot_tx_rmt_item[i].level0 = 0;
      dshot_tx_rmt_item[i].level1 = 1;
    }
    dshot_tx_rmt_item[DSHOT_PAUSE_BIT].level0 = 1;
    dshot_tx_rmt_item[DSHOT_PAUSE_BIT].level1 = 0;
  } else {
    for (int i = 0; i < DSHOT_PAUSE_BIT; i++, parsed_packet <<= 1) {
      if (parsed_packet & 0x8000) {
        dshot_tx_rmt_item[i].duration0 = dshot_config.ticks_one_high;
        dshot_tx_rmt_item[i].duration1 = dshot_config.ticks_one_low;
      } else {
        dshot_tx_rmt_item[i].duration0 = dshot_config.ticks_zero_high;
        dshot_tx_rmt_item[i].duration1 = dshot_config.ticks_zero_low;
      }
      dshot_tx_rmt_item[i].level0 = 1;
      dshot_tx_rmt_item[i].level1 = 0;
    }
    dshot_tx_rmt_item[DSHOT_PAUSE_BIT].level0 = 0;
    dshot_tx_rmt_item[DSHOT_PAUSE_BIT].level1 = 1;
  }
  dshot_tx_rmt_item[DSHOT_PAUSE_BIT].duration0 = 0;
  dshot_tx_rmt_item[DSHOT_PAUSE_BIT].duration1 = DSHOT_PAUSE;
  return dshot_tx_rmt_item;
}

uint16_t DShotRMT::calculateCRC(const dshot_packet_t &dshot_packet) {
  const uint16_t packet = (dshot_packet.throttle_value << 1) | dshot_packet.telemetric_request;
  if (dshot_config.is_bidirectional) {
    const uint16_t x = packet ^ (packet >> 4) ^ (packet >> 8);
    return (~x) & 0x0F;
  }
  return (packet ^ (packet >> 4) ^ (packet >> 8)) & 0x0F;
}

uint16_t DShotRMT::parseRmtPaket(const dshot_packet_t &dshot_packet) {
  uint16_t crc = calculateCRC(dshot_packet);
  uint16_t parsed = (dshot_packet.throttle_value << 1) | dshot_packet.telemetric_request;
  return (parsed << 4) | crc;
}
