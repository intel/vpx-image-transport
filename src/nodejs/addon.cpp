// Add your copyright and license header

#include "gen/nan__vpx_encoder.h"
#include "vpx_encoder_creator.h"

void initModule(v8::Local<v8::Object> exports) {
  NanVPXEncoder::Init(exports);
  Nan::Export(exports, "createEncoder", CreateEncoder);
}

NODE_MODULE(node_vpx, initModule);
