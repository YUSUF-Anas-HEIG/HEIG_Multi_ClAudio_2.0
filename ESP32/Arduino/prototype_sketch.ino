#include "AudioTools.h"
#include "AudioTools/AudioLibs/A2DPStream.h"

AudioInfo info32(44100, 2, 32);
AudioInfo info16(44100, 2, 16);
BluetoothA2DPSource a2dp_source;
I2SStream i2s;
FormatConverterStream conv(i2s);
const int BYTES_PER_FRAME = 4;

static A2DPNoVolumeControl no_vol;


#define GAIN 65

#define NOISE_THRESHOLD 500 

int16_t apply_gain_and_gate(int32_t sample) {
  
  //if (abs(sample) < NOISE_THRESHOLD) return 0; 
  sample *= GAIN;
  
  if (sample > 32767) return 32767;
  if (sample < -32768) return -32768;

  return (int16_t)sample;
}

int32_t get_sound_data(Frame* data, int32_t frameCount) {
  int count = conv.readBytes((uint8_t*)data, frameCount * BYTES_PER_FRAME) / BYTES_PER_FRAME;

  for (int i = 0; i < count; i++) {
    data[i].channel1 = apply_gain_and_gate(data[i].channel1);
    data[i].channel2 = apply_gain_and_gate(data[i].channel2);
  }
  return count;
}
// Arduino Setup
void setup(void) {
  //Serial.begin(115200);
  //AudioToolsLogger.begin(Serial, AudioToolsLogLevel::Info);

  // setup conversion
  conv.begin(info32, info16);

  
  //Serial.println("starting I2S...");
  auto cfg = i2s.defaultConfig(RX_MODE);
  cfg.i2s_format = I2S_STD_FORMAT; // or try with I2S_LSB_FORMAT
  
  cfg.copyFrom(info32);
  cfg.is_master = true;
  i2s.begin(cfg);

  //Serial.println("starting A2DP...");
  //a2dp_source.set_auto_reconnect(false);
   
  a2dp_source.set_volume_control(&no_vol);
  a2dp_source.set_volume(100);
  a2dp_source.set_task_priority(configMAX_PRIORITIES - 3);

  std::vector <const char*> peer = {"WH-1000XM4", "WH-XB910N", "UGREEN-BT509"};
  
  a2dp_source.start(peer, get_sound_data);

  //Serial.println("A2DP started");
}

// Arduino loop - repeated processing
void loop() {  }