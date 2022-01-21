#include <Pozyx.h>
#include <Pozyx_definitions.h>

// Please read the ready-to-localize tuturial together with this example.
// https://www.pozyx.io/Documentation/Tutorials/ready_to_localize
/**
  The Pozyx ready to localize tutorial (c) Pozyx Labs
  
  Please read the tutorial that accompanies this sketch: https://www.pozyx.io/Documentation/Tutorials/ready_to_localize/Arduino
  
  This tutorial requires at least the contents of the Pozyx Ready to Localize kit. It demonstrates the positioning capabilities
  of the Pozyx device both locally and remotely. Follow the steps to correctly set up your environment in the link, change the
  parameters and upload this sketch. Watch the coordinates change as you move your device around!
*/
#include <Pozyx.h>
#include <Pozyx_definitions.h>
#include <Wire.h>

////////////////////////////////////////////////
////////////////// PARAMETERS //////////////////
////////////////////////////////////////////////
int num_tags = 1;
uint16_t tags[1] = {0x6924};

//int num_tags = 5;
//uint16_t tags[5] = {0x6077,0x6924,0x6e3d,0x6908,0x6905};   

//int num_tags = 1;
//uint16_t tags[1] = {0x6e3d};

//int num_tags = 3;
//uint16_t tags[3] = {0x6077,0x6924,0x6e3d};   

//int num_tags = 2;
//uint16_t tags[2] = {0x607B, 0x6076};

//uint8_t num_anchors = 8;                                    // the number of anchors
//uint16_t anchors[8] = {0x605B, 0x680A, 0x6822, 0x6031, 0x6173, 0x683B, 0x685D, 0x603B};     // the network id of the anchors: change these to the network ids of your anchors.
//int32_t anchors_x[8] = {985, 6498, 9786, 14738, 14643, 11058, 4834, 208};               // anchor x-coorindates in mm
//int32_t anchors_y[8] = {4659, 6181, 5781, 5553, 357, 117, 117, 3018};                  // anchor y-coordinates in mm
//int32_t heights[8] = {2400, 2425, 2525, 2505, 2375, 2320, 2365, 2410};              // anchor z-coordinates in mm
//uint8_t num_anchors = 7;                                    // the number of anchors
//uint16_t anchors[7] = {0x680A, 0x6822, 0x683B, 0x685D, 0x603B, 0x605B, 0x6031};     // the network id of the anchors: change these to the network ids of your anchors.
//int32_t anchors_x[7] = {6498, 9786,11058, 4834, 4507, 8699, 7525};               // anchor x-coorindates in mm
//int32_t anchors_y[7] = {6181, 5781, 117, 117, 2877, 117, 5564};                  // anchor y-coordinates in mm
//int32_t heights[7] = {2425, 2525, 2320, 2365, 170, 200, 85};              // anchor z-coordinates in mm
//uint8_t num_anchors = 7;                                    // the number of anchors
//uint16_t anchors[7] = {0x6822, 0x683B, 0x685D, 0x603B, 0x605B, 0x6031, 0x6173};     // the network id of the anchors: change these to the network ids of your anchors.
//int32_t anchors_x[7] = {9786,11058, 4834, 4507, 8699, 7525, 7365};               // anchor x-coorindates in mm
//int32_t anchors_y[7] = {5781, 117, 117, 2877, 117, 5564, 2246};                  // anchor y-coordinates in mm
//int32_t heights[7] = {2525, 2320, 2365, 170, 200, 85, 3115};              // anchor z-coordinates in mm
//
//uint8_t num_anchors = 8;                                    // the number of anchors
//uint16_t anchors[8] = {0x6822, 0x683B, 0x685D, 0x603B, 0x605B, 0x6031, 0x6173, 0x680A};     // the network id of the anchors: change these to the network ids of your anchors.
//int32_t anchors_x[8] = {9786,11058, 4834, 4507, 8699, 7525, 7365, 6498};               // anchor x-coorindates in mm
//int32_t anchors_y[8] = {5781, 117, 117, 2877, 117, 5564, 2246, 6181};                  // anchor y-coordinates in mm
//int32_t heights[8] = {2525, 2320, 2365, 170, 200, 85, 3115, 2400};              // anchor z-coordinates in mm

//uint8_t num_anchors = 4;                                    // the number of anchors
//uint16_t anchors[4] = {0x6822, 0x6031, 0x685D, 0x603B};     // the network id of the anchors: change these to the network ids of your anchors.
//int32_t anchors_x[4] = {9786, 7525, 7500, 9800};               // anchor x-coorindates in mm
//int32_t anchors_y[4] = {5781, 5564, 117, 117};                  // anchor y-coordinates in mm
//int32_t heights[4] = {2520, 85, 2700, 190};              // anchor z-coordinates in mm

//uint8_t num_anchors = 8;                                    // the number of anchors
//uint16_t anchors[8] = {0x6822, 0x6031, 0x605B, 0x603B, 0x685D, 0x6173, 0x683B, 0x680A};     // the network id of the anchors: change these to the network ids of your anchors.
//int32_t anchors_x[8] = {9786, 7525, 7500, 9800, 4834, 7365, 11058, 6498};               // anchor x-coorindates in mm
//int32_t anchors_y[8] = {5781, 5564, 117, 117, 117, 2246, 117, 6181};                  // anchor y-coordinates in mm
//int32_t heights[8] = {2520, 85, 2700, 190, 2365, 3115, 2320, 2425};              // anchor z-coordinates in mm

//uint8_t num_anchors = 8;                                    // the number of anchors
//uint16_t anchors[8] = {0x605B, 0x685D, 0x603B, 0x683B, 0x6822, 0x6031, 0x680A, 0x6173};     // the network id of the anchors: change these to the network ids of your anchors.
//int32_t anchors_x[8] = {4500, 4805, 8033, 11179, 9778, 7531, 6316, 7365};               // anchor x-coorindates in mm
//int32_t anchors_y[8] = {2884, 146, 143, 153, 5824, 5571, 6215, 2246};                  // anchor y-coordinates in mm
//int32_t heights[8] = {995, 2192, 452, 1370, 2533, 85, 1798, 3115};              // anchor z-coordinates in mm
//
//uint8_t num_anchors = 6;                                    // the number of anchors
//uint16_t anchors[6] = {0x605B, 0x685D, 0x683B, 0x6822,  0x680A, 0x6173};     // the network id of the anchors: change these to the network ids of your anchors.
//int32_t anchors_x[6] = {4500, 4805,11179, 9778, 6316, 7365};               // anchor x-coorindates in mm
//int32_t anchors_y[6] = {2884, 146, 153, 5824, 6215, 2246};                  // anchor y-coordinates in mm
//int32_t heights[6] = {995, 2192,1370, 2533, 1798, 3115};              // anchor z-coordinates in mm

uint8_t num_anchors = 4;                                    // the number of anchors
uint16_t anchors[4] = {0x694c, 0x6037, 0x6e0b,  0x6e15};     // the network id of the anchors: change these to the network ids of your anchors.
int32_t anchors_x[4] = {10000, 10000, 0, 0};               // anchor x-coorindates in mm
int32_t anchors_y[4] = {0,6500, 0, 6500};                 // anchor y-coordinates in mm
int32_t heights[4] = {1500,1500, 1500, 1500};              // anchor z-coordinates in mm

//uint8_t algorithm = POZYX_POS_ALG_TRACKING; 
uint8_t algorithm = POZYX_POS_ALG_UWB_ONLY;             // positioning algorithm to use
uint8_t dimension = POZYX_2D;//POZYX_3D;                           // positioning dimension
int32_t height = 1300;                                  // height of device, required in 2.5D positioning

////////////////////////////////////////////////

void setup(){
  Serial.begin(115200);
  
  Pozyx.begin();
  for (int i=0; i<num_tags;i++){
    setAnchorsManual(tags[i]);
  }
  
  delay(1000);
}

void loop(){
  for (int i = 0; i < num_tags; i++){
    coordinates_t position;
//    euler_angles_t angles;
    quaternion_t q;
    int status = Pozyx.doRemotePositioning(tags[i], &position, dimension, height, algorithm);
    Pozyx.getQuaternion(&q,tags[i]);
    if (status == POZYX_SUCCESS){
      printCoordinates(position, q, tags[i]);
    }else{
//      printErrorCode("positioning", tags[i]);
      setAnchorsManual(tags[i]);
    }
  }
}

// prints the coordinates for either humans or for processing
void printCoordinates(coordinates_t coor,quaternion_t q, uint16_t network_id){
  Serial.print(network_id);
  Serial.print(",");
  Serial.print(coor.x);
  Serial.print(",");
  Serial.print(coor.y);
  Serial.print(",");
  Serial.print(coor.z);
  Serial.print(",");
  Serial.print(q.x);
  Serial.print(",");
  Serial.print(q.y);
  Serial.print(",");
  Serial.print(q.z);
  Serial.print(",");
  Serial.print(q.weight);
  Serial.print("\n");
}

// function to manually set the anchor coordinates
void setAnchorsManual(uint16_t tag){
  int status = Pozyx.clearDevices(tag);
  for(int j = 0; j < num_anchors; j++){
    device_coordinates_t anchor;
    anchor.network_id = anchors[j];
    anchor.flag = 0x1; 
    anchor.pos.x = anchors_x[j];
    anchor.pos.y = anchors_y[j];
    anchor.pos.z = heights[j];
    status &= Pozyx.addDevice(anchor, tag);
  }
  Pozyx.setSelectionOfAnchors(POZYX_ANCHOR_SEL_AUTO,num_anchors,tag);
}
