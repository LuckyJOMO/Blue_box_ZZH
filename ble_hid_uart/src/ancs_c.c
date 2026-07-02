/*
 * Copyright (c) 2023 Shanghai Panchip Microelectronics Co.,Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/types.h>
#include <stddef.h>
#include <errno.h>
#include <zephyr.h>
#include <sys/printk.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/conn.h>
#include <bluetooth/uuid.h>
#include <bluetooth/gatt.h>
#include <sys/byteorder.h>

#include "ancs_c.h"
#include "controller_ota.h"  // 添加 controller_ota.h 以获取 TGT_SendMultiData 函数声明

extern uint8_t qrcode[100];
extern uint8_t qrcode_length;
extern uint8_t	ANCS_FLAG;	
/**@brief String literals for the iOS notification Category id types. Used then printing to UART. */
static char const *lit_catid[] = {
	"Other",
	"Incoming Call",
	"Missed Call",
	"Voice Mail",
	"Social",
	"Schedule",
	"Email",
	"News",
	"Health And Fitness",
	"Business And Finance",
	"Location",
	"Entertainment"
};

/**@brief String literals for the iOS notification event types. Used then printing to UART. */
static char const *lit_eventid[] = {
	"Added",
	"Modified",
	"Removed"
};

/**@brief String literals for the iOS notification attribute types. Used when printing to UART. */
static char const *lit_attrid[] = {
	"App Identifier",
	"Title",
	"Subtitle",
	"Message",
	"Message Size",
	"Date",
	"Positive Action Label",
	"Negative Action Label"
};
static uint8_t s_attr_buf[CFG_ANCS_ATTRIBUTE_MAXLEN];
static uint16_t s_attr_size;
static uint16_t s_buf_index;
static int s_uid;

struct bt_conn *ancs_c_conn;

static struct bt_uuid_16 uuid = BT_UUID_INIT_16(0);
static struct bt_uuid_128 ancs_srvc_uuid = BT_UUID_INIT_128(ANCS_SRVC_UUID);
static struct bt_uuid_128 ancs_ntf_source_uuid = BT_UUID_INIT_128(ANCS_NTF_SOURCE_UUID);
static struct bt_uuid_128 ancs_control_point_uuid = BT_UUID_INIT_128(ANCS_CONTROL_POINT_UUID);
static struct bt_uuid_128 ancs_data_sourcec_uuid = BT_UUID_INIT_128(ANCS_DATA_SOURCE_UUID);

static uint16_t write_control_point_handle;
static uint16_t ntf_handle;
static uint16_t data_handle;
static struct bt_gatt_write_params write_control_point_params;
static struct bt_gatt_discover_params discover_params;
static struct bt_gatt_subscribe_params subscribe_params_src;
static struct bt_gatt_subscribe_params subscribe_params_data;

static uint8_t notify_source_func(struct bt_conn *conn,
				  struct bt_gatt_subscribe_params *params,
				  const void *data, uint16_t length)
{
	if (!data) {
		printk("[UNSUBSCRIBED]\n");
		params->value_handle = 0U;
		return BT_GATT_ITER_STOP;
	}

	ancs_decode_notification_source((uint8_t *)data, length);

	return BT_GATT_ITER_CONTINUE;
}

static uint8_t notify_data_func(struct bt_conn *conn,
				struct bt_gatt_subscribe_params *params,
				const void *data, uint16_t length)
{
	if (!data) {
		printk("[UNSUBSCRIBED]\n");
		params->value_handle = 0U;
		return BT_GATT_ITER_STOP;
	}

 //	 show_reg3((uint8_t *)data, length);
 ////////************发送变量初始化**********//////////////				
		char send_content[255]={0};
		char *content_copy;
	    char rec_name_type[50] ={0x00};
		uint8_t send_length;
// // ////////************截取软件名字**********//////////////			
        content_copy = (char*)data;		
        send_length = (((uint8_t)content_copy[6]) + (((uint8_t)content_copy[7])<<8));				 	 
//		printf("ancelen=%d\r\n",send_length);
 		memcpy(rec_name_type,&(content_copy[8]),send_length);
// 		printf("rec_name_type=%s\r\n",rec_name_type);
//  ////////************发送软件名字+内容给MCU**********//////////////					 	 	 				        
	    if((bool)strstr(rec_name_type, "com.apple.mobilephone")) 
		{
			char *send_name = "MOBILEPHONE:";
			memcpy(send_content,send_name,12);
		    memcpy(send_content+12,data,length);
			// for(int i=0;i<(length+12);i++)
			// {
			// 	 printf("0x%02x ",send_content[i]);
		    // }
			TGT_SendMultiData((uint8_t *)send_content, length+12);					 
		}
				 
		if((bool)strstr(rec_name_type, "com.tencent.xin")) 
		{
			char *send_name = "WECHAT:";
			memcpy(send_content,send_name,7);
			memcpy(send_content+7,data,length);
			// for(int i=0;i<(length + 7);i++)
		    // {
			// 	printf("0x%02x ",send_content[i]);
			// }					 
			TGT_SendMultiData((uint8_t *)send_content, length+7);	
		}
				 
		if((bool)strstr(rec_name_type, "com.facebook.Facebook")) 
		{
			char *send_name = "FACEBOOK:";
		    memcpy(send_content,send_name,9);
			memcpy(send_content+9,data,length);
			// for(int i=0;i<(param->packet_size + 9);i++)
			// {
			// 	 printf("0x%02x ",send_content[i]);
			// }					 
			TGT_SendMultiData((uint8_t*)send_content, length+9);
		}
				 
		if((bool)strstr(rec_name_type, "com.burbn.instagram")) 
		{
			char *send_name = "INSTAGRAM:";
			memcpy(send_content,send_name,10);
			memcpy(send_content+10,data,length);
			// for(int i=0;i<(length+10);i++)
			// {
			//   printf("0x%02x ",send_content[i]);
			// }					 
			TGT_SendMultiData((uint8_t*)send_content, length+10);
		}
				 
		if((bool)strstr(rec_name_type, "com.atebits.Tweetie2")) 
		{
			char *send_name = "TWITTER:";
			memcpy(send_content,send_name,8);
			memcpy(send_content+8,data,length);
			// for(int i=0;i<(length+8);i++)
		    // {
			// 	printf("0x%02x ",send_content[i]);
			// }					 
			TGT_SendMultiData((uint8_t*)send_content, length+8);
		}				 
 ////////************发送变量初始化**********//////////////	     
 //	ancs_decode_data_source((uint8_t *)data, length);

	return BT_GATT_ITER_CONTINUE;
}

static void write_rsp(struct bt_conn *conn, uint8_t err,
		      struct bt_gatt_write_params *params)
{
	if ((err > 0) && (err != 0xA2) && (err != 0xA3)) {
		printk("write err code 0x%02x\n", err);
	}
}

static uint8_t discover_func(struct bt_conn *conn,
			     const struct bt_gatt_attr *attr,
			     struct bt_gatt_discover_params *params)
{
	int err;

	if (!attr) {
		printk("Discover complete\n");
		(void)memset(params, 0, sizeof(*params));


		return BT_GATT_ITER_STOP;
	}

	printk("[ATTRIBUTE] handle %u\n", attr->handle);

	if (!bt_uuid_cmp(discover_params.uuid, &ancs_srvc_uuid.uuid)) {
		discover_params.uuid = &ancs_control_point_uuid.uuid;
		discover_params.start_handle = attr->handle + 1;
		discover_params.type = BT_GATT_DISCOVER_CHARACTERISTIC;

		err = bt_gatt_discover(conn, &discover_params);
		if (err) {
			printk("Discover failed (err %d)\n", err);
		}
	} else if (!bt_uuid_cmp(discover_params.uuid, &ancs_control_point_uuid.uuid)) {
		write_control_point_handle = attr->handle;
		discover_params.uuid = &ancs_ntf_source_uuid.uuid;
		discover_params.start_handle = attr->handle + 1;
		discover_params.type = BT_GATT_DISCOVER_CHARACTERISTIC;

		err = bt_gatt_discover(conn, &discover_params);
		if (err) {
			printk("Discover failed (err %d)\n", err);
		}
	} else if (!bt_uuid_cmp(discover_params.uuid, &ancs_ntf_source_uuid.uuid)) {
		memcpy(&uuid, BT_UUID_GATT_CCC, sizeof(uuid));
		ntf_handle = attr->handle;
		discover_params.uuid = &uuid.uuid;
		discover_params.start_handle = attr->handle;
		discover_params.type = BT_GATT_DISCOVER_DESCRIPTOR;
		subscribe_params_src.value_handle = bt_gatt_attr_value_handle(attr);

		err = bt_gatt_discover(conn, &discover_params);
		if (err) {
			printk("Discover failed (err %d)\n", err);
		}
	} else if ((!bt_uuid_cmp(discover_params.uuid, BT_UUID_GATT_CCC)) && (attr->handle == (ntf_handle + 2))) {
		subscribe_params_src.notify = notify_source_func;
		subscribe_params_src.value = BT_GATT_CCC_NOTIFY;
		subscribe_params_src.ccc_handle = attr->handle;

		err = bt_gatt_subscribe(conn, &subscribe_params_src);
		if (err && err != -EALREADY) {
			printk("Subscribe failed (err %d)\n", err);
		} else {
			printk("[SUBSCRIBED] %d\n", subscribe_params_src.ccc_handle);
		}

		discover_params.uuid = &ancs_data_sourcec_uuid.uuid;
		discover_params.start_handle = attr->handle + 1;
		discover_params.type = BT_GATT_DISCOVER_CHARACTERISTIC;

		err = bt_gatt_discover(conn, &discover_params);
		if (err) {
			printk("Discover failed (err %d)\n", err);
		}
	} else if (!bt_uuid_cmp(discover_params.uuid, &ancs_data_sourcec_uuid.uuid)) {
		memcpy(&uuid, BT_UUID_GATT_CCC, sizeof(uuid));
		data_handle = attr->handle;
		discover_params.uuid = &uuid.uuid;
		discover_params.start_handle = attr->handle;
		discover_params.type = BT_GATT_DISCOVER_DESCRIPTOR;
		subscribe_params_data.value_handle = bt_gatt_attr_value_handle(attr);

		err = bt_gatt_discover(conn, &discover_params);
		if (err) {
			printk("Discover failed (err %d)\n", err);
		}
	} else if ((!bt_uuid_cmp(discover_params.uuid, BT_UUID_GATT_CCC)) && (attr->handle == (data_handle + 2))) {
		subscribe_params_data.notify = notify_data_func;
		subscribe_params_data.value = BT_GATT_CCC_NOTIFY;
		subscribe_params_data.ccc_handle = attr->handle;

		err = bt_gatt_subscribe(conn, &subscribe_params_data);
		if (err && err != -EALREADY) {
			printk("Subscribe failed (err %d)\n", err);
		} else {
			printk("[SUBSCRIBED] %d\n", subscribe_params_data.ccc_handle);
		}
	}

	return BT_GATT_ITER_STOP;
}

void ble_ancs_c_discovery_service(struct bt_conn *conn)
{
	int err;

	discover_params.uuid = &ancs_srvc_uuid.uuid;
	discover_params.func = discover_func;
	discover_params.start_handle = BT_ATT_FIRST_ATTTRIBUTE_HANDLE;
	discover_params.end_handle = BT_ATT_LAST_ATTTRIBUTE_HANDLE;
	discover_params.type = BT_GATT_DISCOVER_PRIMARY;

	err = bt_gatt_discover(conn, &discover_params);
	if (err) {
		printk("Discover failed(err %d)\n", err);
		return;
	}
}

static void ancs_notify_attr_print(void)
{
	uint8_t commd_id = s_attr_buf[0];
	uint16_t UID;
	uint8_t attr_id;
	uint16_t attr_size = 0;

	memcpy(&UID, &s_attr_buf[1], 4);
	memcpy(&attr_size, &s_attr_buf[6], 2);

	if (commd_id == CTRL_POINT_GET_NTF_ATTRIBUTE) {
		attr_id = s_attr_buf[5];
		printk("UID=%d, ATTR_ID: %s, ATTR_SIZE=%d: ", UID, lit_attrid[attr_id], attr_size);
		for (uint16_t idx = 0; idx < attr_size; idx++) {
			printk("%c", s_attr_buf[8 + idx]);
		}
		printk("\n");
	} else if (commd_id == CTRL_POINT_GET_APP_ATTRIBUTE) {
		for (uint16_t idx = 0; idx < attr_size; idx++) {
			printk("%c", s_attr_buf[idx]);
		}
		printk("\n");
	}
}

void ancs_decode_data_source(uint8_t *p_data, uint16_t length)
{

	//  printk("decode\n");

	/* It's the begginning of a attr info. */
	if (s_buf_index == 0) {
		memcpy(&s_attr_size, &p_data[6], 2);
		memcpy(s_attr_buf, p_data, length);
		/* It's the end of the attr info, a complete attr info is already stored in the buffer. */
		if (s_attr_size == (length - 8)) {
			ancs_notify_attr_print();
			s_buf_index = 0;
		} else {    /* It's not the end of the attr info. */
			s_buf_index = length;
		}
	} else {   /*  It isn't the begginning of a attr info. */
		memcpy(&s_attr_buf[s_buf_index], p_data, length);
		/* It's the end of the attr info, print the buffer. */
		if (s_attr_size == (s_buf_index - 8) + length) {
			ancs_notify_attr_print();
			s_buf_index = 0;
		} else { /* It's not the end of the attr info. */
			s_buf_index = s_buf_index + length;
		}
	}
}

void ancs_set_uid(uint32_t uid)
{
	s_uid = uid;
}

uint32_t ancs_get_uid(void)
{
	return s_uid;
}


void ancs_notify_ctrpoint_get(uint32_t uid)   //获得包括APP名字、发件人、消息内容的详细信息
{
	uint8_t buf[12];

	buf[0] = CTRL_POINT_GET_NTF_ATTRIBUTE;
	memcpy(&buf[1], &uid, 4);
	buf[5] = 0x00;    //APP_ID知道软件名字
  
	
	buf[6] = 0x01;    //title知道发件人名字
	buf[7] = 0xff;
	buf[8] = 0xff; 
	
	buf[9] = 0x03;   //message知道信息内容
	buf[10] = 0xff;
	buf[11] = 0xff; 

    ancs_c_write_control_point(buf,12);  
    //printk("point12=0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x",buf[0],buf[1],buf[2],buf[3],buf[4],buf[5],buf[6],buf[7],buf[8],buf[9],buf[10],buf[11]);				

}


void notification_content_print(ntf_source_pdu_t *p_notif)
{
	// // printk("Notification\n");
	// // printk("Event:       %s\n", lit_eventid[p_notif->event_id]);
    // printk("Category ID: %s\n", lit_catid[p_notif->category_id]);
	// printk("Category Cnt:%u\n", p_notif->category_count);
	// // printk("UID:         %u\n", p_notif->notification_uid);
	// // printk("Flags:\n");
	// if (p_notif->event_flags.silent == 1) {
	// 	printk("Silent\n");
	// }
	// if (p_notif->event_flags.important == 1) {
	// 	printk("Important\n");
	// }
	// if (p_notif->event_flags.pre_existing == 1) {
	// 	printk("Pre-existing\n");
	// }
	// if (p_notif->event_flags.positive_action == 1) {
	// 	printk("Positive Action\n");
	// }
	// if (p_notif->event_flags.negative_action == 1) {
	// 	printk("Negative Action\n");
	// }

///////////*************发送ctr_point函数*******////////////////////
	if(strstr(lit_eventid[p_notif->event_id],"Removed"))
	{
        // printk("Removed\n");
	}
	else
	{ 
        // printk("ANCS_FLAG=%d\n",ANCS_FLAG);
	   if(ANCS_FLAG == 3)
	   {
		        // printk("ctrget\n");
		  ancs_notify_ctrpoint_get(p_notif->notification_uid);		
	   }
	}
}

void ancs_decode_notification_source(uint8_t *p_data, uint16_t length)
{
	ntf_source_pdu_t pdu;

    // printk("decode_notification_source\n");

	memcpy(&pdu, p_data, length);
	notification_content_print(&pdu);

	ancs_set_uid((uint32_t) pdu.notification_uid);
}

void ancs_c_write_control_point(uint8_t *p_data, uint16_t length)
{
	write_control_point_params.handle = write_control_point_handle + 1;
	write_control_point_params.func = write_rsp;
	write_control_point_params.offset = 0U;
	write_control_point_params.data = p_data;
	write_control_point_params.length = length;

	if (bt_gatt_write(ancs_c_conn, &write_control_point_params) < 0) {
		printk("write fail\n");
		bt_conn_unref(ancs_c_conn);
	}
}

void ancs_notify_attr_get(uint32_t uid, char noti_attr)
{
	int len = 0;
	uint8_t buf[8];

	buf[0] = CTRL_POINT_GET_NTF_ATTRIBUTE;
	memcpy(&buf[1], &uid, 4);
	buf[5] = noti_attr;

	if ((noti_attr == ANCS_NOTIF_ATTR_ID_TITLE) || (noti_attr == ANCS_NOTIF_ATTR_ID_SUBTITLE)
	    || (noti_attr == ANCS_NOTIF_ATTR_ID_MESSAGE)) {
		len = CFG_ANCS_ATTRIBUTE_MAXLEN;
		buf[6] = (len & 0xff);
		buf[7] = (len >> 8) & 0xff;
		ancs_c_write_control_point(buf, 8);
	} else {
		ancs_c_write_control_point(buf, 6);
	}
}

void ancs_action_perform(uint32_t uid, int action)
{
	uint8_t buf[6];

	buf[0] = CTRL_POINT_PERFORM_NTF_ACTION;
	memcpy(&buf[1], &uid, 4);
	buf[5] = action;
	ancs_c_write_control_point(buf, 6);
}
