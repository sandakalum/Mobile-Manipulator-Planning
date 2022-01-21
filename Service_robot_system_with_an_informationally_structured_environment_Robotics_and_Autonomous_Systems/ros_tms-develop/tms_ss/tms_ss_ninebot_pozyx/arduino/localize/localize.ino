
#include <Pozyx.h>
#include <Pozyx_definitions.h>
#include <Wire.h>
#include <math.h>

typedef struct{
  uint16_t id;
  double x;
  double y;
} anchor_t;

////////////////// PARAMETERS //////////////////

//const uint8_t num_anchors = 16; // the number of anchors
const uint8_t num_anchors = 4; // the number of anchors

anchor_t anchors[num_anchors] = {
  {0x6e23, 0, 0},
  {0x6e49, 18, 0},
  {0x6044, 0, 18},
  {0x6050, 18, 18}
};

/*
anchor_t anchors[num_anchors] = {
                                 {0x6044,-27.476,-8.039},
                                 {0x6e15,-11.096,2.612},
                                 {0x6e30,3.423,13.252},
                                 {0x6e58,19.015,27.244},
                                 {0x6e22,-25.448,-16.174},
                                 {0x6e39,-8.735,-15.017},
                                 {0x6037,11.108,0.657},
                                 {0x6e49,35.743,4.081},
                                 {0x6e11,46.601,29.822},
                                //  {0x6e11,70.839,60.474},
                                 {0x6e0b,61.462,39.014},
                                 {0x694c,66.839,49.511},
                                 {0x6e08,72.77,74.072},
                                 {0x6954,48.319,18.575},
                                //  {0x6954,76.838,51.398},
                                 {0x6e5c,62.389,31.236},
                                 {0x6050,81.744,41.521},
                                 //{0x6050,71.906,43.546},
                                 {0x6023,82.578,57.5}
                                 };
*/


bool remote = true;
// bool remote = true;
uint16_t remote_id = 0x6e69;

uint8_t algorithm = POZYX_POS_ALG_UWB_ONLY;
uint8_t dimension = POZYX_2D;
int32_t height = 1000;

////////////////////////////////////////////////

void setup(){
  Serial.begin(115200);
  Pozyx.begin();

  if(!remote) remote_id = NULL;

  AnchorsCalibration();
  
  delay(1000);
}

void loop(){

  coordinates_t position;
  quaternion_t orientation;

  int status;
  if(remote){
    status = Pozyx.doRemotePositioning(remote_id, &position, dimension, height, algorithm);
  }else{
    status = Pozyx.doPositioning(&position, dimension, height, algorithm);
  }

  Pozyx.getQuaternion(&orientation, remote_id);
  
  if(status == POZYX_SUCCESS){
    printCoordinates(position, orientation, remote_id);
  }else{
    Serial.println(F("Positioning Error"));
    AnchorsCalibration();
  }
}

void printCoordinates(coordinates_t coor, quaternion_t quat, uint16_t remote_id){
  uint16_t network_id = 0;
  if(remote) network_id = remote_id;
  
  Serial.print(network_id);
  Serial.print(",");
  Serial.print(coor.x);
  Serial.print(",");
  Serial.print(coor.y);
  Serial.print(",");
  Serial.print(coor.z);
  Serial.print(",");
  Serial.print(quat.x);
  Serial.print(",");
  Serial.print(quat.y);
  Serial.print(",");
  Serial.print(quat.z);
  Serial.print(",");
  Serial.println(quat.weight);
}

void AnchorsCalibration(){
  Pozyx.clearDevices(remote_id);
  Pozyx.setPositionAlgorithm(algorithm, dimension, remote_id);
  for(int i = 0; i < num_anchors; i++){
    device_coordinates_t anchor;
    anchor.network_id = anchors[i].id;
    anchor.flag = 0x1;
    anchor.pos.x = (int32_t)round(anchors[i].x * 1000);
    anchor.pos.y = (int32_t)round(anchors[i].y * 1000);
    anchor.pos.z = 0;
    Pozyx.addDevice(anchor, remote_id);
  }
  if(num_anchors > 4){
    Pozyx.setSelectionOfAnchors(POZYX_ANCHOR_SEL_AUTO, num_anchors, remote_id);
  } 
}