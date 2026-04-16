#include "sys.h"
#include "main.h"
#include "sdo_frames.h"

int main(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
    delay_init(168);

    CAN1_Init(&Master_Data,1000000);
    TIM2_Init();
    USART1_Init(115200);

    setNodeId(&Master_Data, 0x01);
    setState(&Master_Data, Initialisation);
    setState(&Master_Data, Operational);

    while(1)
    {
        /*
         * 依次发往 0x601~0x604。
         * 0x01：肩ROLL正方向向下
				 * 0x02：肩YAW正方向外旋
			   * 0x03：肩PITCH正方向向下
			   * 0x04：肘关节正方向向下
         */
			
			  delay_ms(3000);
			////////////////////////准备////////////////////////
        send_sdo_to_node(0x01, &SDO_ACTIVATE_INIT);
				send_sdo_to_node(0x02, &SDO_ACTIVATE_INIT);
        send_sdo_to_node(0x03, &SDO_ACTIVATE_INIT);
				send_sdo_to_node(0x04, &SDO_ACTIVATE_INIT);
			  delay_ms(500);
			
        send_sdo_to_node(0x01, &SDO_ACTIVATE_PPM);
				send_sdo_to_node(0x02, &SDO_ACTIVATE_PPM);
        send_sdo_to_node(0x03, &SDO_ACTIVATE_PPM);
				send_sdo_to_node(0x04, &SDO_ACTIVATE_PPM);
        delay_ms(500);
			
        send_sdo_to_node(0x01, &SDO_ACTIVATE_SETV100);
				send_sdo_to_node(0x02, &SDO_ACTIVATE_SETV1000);
        send_sdo_to_node(0x03, &SDO_ACTIVATE_SETV1000);
				send_sdo_to_node(0x04, &SDO_ACTIVATE_SETV1000);
        delay_ms(500);
				
				send_sdo_to_node(0x01, &SDO_DISABLE);
				send_sdo_to_node(0x02, &SDO_DISABLE);
        send_sdo_to_node(0x03, &SDO_DISABLE);
        send_sdo_to_node(0x04, &SDO_DISABLE);
        delay_ms(500);
			////////////////////////伸手////////////////////////
				send_sdo_to_node(0x02, &SDO_ENABLE);
				send_sdo_to_node(0x03, &SDO_ENABLE);
        send_sdo_to_node(0x04, &SDO_ENABLE);
        delay_ms(500);
				
				send_sdo_to_node(0x02, &SDO_TARGET_POS_NODEUN200);
				send_sdo_to_node(0x03, &SDO_TARGET_POS_NODEUN500);
        send_sdo_to_node(0x04, &SDO_TARGET_POS_NODE1000);
        delay_ms(500);
				
				send_sdo_to_node(0x02, &SDO_GO);
				send_sdo_to_node(0x03, &SDO_GO);
        send_sdo_to_node(0x04, &SDO_GO);
        delay_ms(8000);
				
			////////////////////////抬臂////////////////////////
				send_sdo_to_node(0x01, &SDO_ENABLE);
				send_sdo_to_node(0x02, &SDO_ENABLE);
        send_sdo_to_node(0x04, &SDO_ENABLE);
        delay_ms(500);

				send_sdo_to_node(0x01, &SDO_TARGET_POS_NODEUN100);
				send_sdo_to_node(0x02, &SDO_TARGET_POS_NODEUN200);
        send_sdo_to_node(0x04, &SDO_TARGET_POS_NODEUN2000);
        delay_ms(500);

				send_sdo_to_node(0x01, &SDO_GO);
				send_sdo_to_node(0x02, &SDO_GO);
        send_sdo_to_node(0x04, &SDO_GO);
        delay_ms(5000);

				
			////////////////////////放臂////////////////////////
				send_sdo_to_node(0x01, &SDO_ENABLE);
				send_sdo_to_node(0x02, &SDO_ENABLE);
        send_sdo_to_node(0x04, &SDO_ENABLE);
        delay_ms(500);

				send_sdo_to_node(0x01, &SDO_TARGET_POS_NODE100);
				send_sdo_to_node(0x02, &SDO_TARGET_POS_NODE200);
        send_sdo_to_node(0x04, &SDO_TARGET_POS_NODE2000);
        delay_ms(500);

				send_sdo_to_node(0x01, &SDO_GO);
				send_sdo_to_node(0x02, &SDO_GO);
        send_sdo_to_node(0x04, &SDO_GO);
        delay_ms(5000);
			////////////////////////缩手////////////////////////


//        send_sdo_to_node(0x01, &SDO_DISABLE);
//				send_sdo_to_node(0x02, &SDO_DISABLE);
//        send_sdo_to_node(0x03, &SDO_DISABLE);
//        delay_ms(500);

				send_sdo_to_node(0x02, &SDO_ENABLE);
				send_sdo_to_node(0x03, &SDO_ENABLE);
        send_sdo_to_node(0x04, &SDO_ENABLE);
        delay_ms(500);
				
				send_sdo_to_node(0x02, &SDO_TARGET_POS_NODE200);
				send_sdo_to_node(0x03, &SDO_TARGET_POS_NODE500);
        send_sdo_to_node(0x04, &SDO_TARGET_POS_NODEUN1000);
        delay_ms(500);
				
				send_sdo_to_node(0x02, &SDO_GO);
				send_sdo_to_node(0x03, &SDO_GO);
        send_sdo_to_node(0x04, &SDO_GO);
        delay_ms(8000);
				
    }
}
