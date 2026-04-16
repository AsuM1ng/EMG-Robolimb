#ifndef __SDO_FRAMES_H
#define __SDO_FRAMES_H

#include "sys.h"
#include "can.h"

#define SDO_MAX_DATA_LEN 8

typedef struct {
    UNS8 len;
    UNS8 data[SDO_MAX_DATA_LEN];
} SDO_Frame;



extern const SDO_Frame SDO_ACTIVATE_INIT;
extern const SDO_Frame SDO_ACTIVATE_PPM;
extern const SDO_Frame SDO_ACTIVATE_SETV2000;
extern const SDO_Frame SDO_ACTIVATE_SETV1000;
extern const SDO_Frame SDO_ACTIVATE_SETV100;
extern const SDO_Frame SDO_DISABLE;
extern const SDO_Frame SDO_ENABLE;
extern const SDO_Frame SDO_GO;
extern const SDO_Frame SDO_TARGET_POS_NODE10000;
extern const SDO_Frame SDO_TARGET_POS_NODE1000;
extern const SDO_Frame SDO_TARGET_POS_NODE2000;
extern const SDO_Frame SDO_TARGET_POS_NODE5000;
extern const SDO_Frame SDO_TARGET_POS_NODEUN10000;
extern const SDO_Frame SDO_TARGET_POS_NODEUN1000;
extern const SDO_Frame SDO_TARGET_POS_NODEUN2000;
extern const SDO_Frame SDO_TARGET_POS_NODEUN5000;
extern const SDO_Frame SDO_TARGET_POS_NODE100;
extern const SDO_Frame SDO_TARGET_POS_NODE200;
extern const SDO_Frame SDO_TARGET_POS_NODE500;
extern const SDO_Frame SDO_TARGET_POS_NODEUN100;
extern const SDO_Frame SDO_TARGET_POS_NODEUN200;
extern const SDO_Frame SDO_TARGET_POS_NODEUN500;

UNS8 send_sdo_to_node(UNS8 node_id, const SDO_Frame *frame);

#endif
