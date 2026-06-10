/*
 * Copyright (c) 2023 Shanghai Panchip Microelectronics Co.,Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _ANCS_H_
#define _ANCS_H_

#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/conn.h>
#include <bluetooth/uuid.h>
#include <bluetooth/gatt.h>
#include <sys/byteorder.h>
#include <stdint.h>

/** Maximum allowed value for attribute length */
#ifndef CFG_ANCS_ATTRIBUTE_MAXLEN
#define CFG_ANCS_ATTRIBUTE_MAXLEN 500
#endif

/** Attribute ID element without maximum length */
#define ANCS_ATTR(ID) ((uint32_t) 0x80000000 | ((uint8_t) ID))

/** Attribute ID element with maximum length */
#define ANCS_ATTR_MAXLEN(ID, LEN) ((uint32_t) 0x80000000 | ((uint8_t) ID) | ((uint16_t) LEN << 8))

#define ANCS_SRVC_UUID                       0xd0, 0x00, 0x2d, 0x12, 0x1e, 0x4b, 0x0f, 0xa4, \
	0x99, 0x4e, 0xce, 0xb5, 0x31, 0xf4, 0x05, 0x79                /**< UUID of Apple notification center service. */
#define ANCS_NTF_SOURCE_UUID                 0xbd, 0x1d, 0xa2, 0x99, 0xe6, 0x25, 0x58, 0x8c, \
	0xd9, 0x42, 0x01, 0x63, 0x0d, 0x12, 0xbf, 0x9f                /**< UUID of notification source. */
#define ANCS_CONTROL_POINT_UUID              0xd9, 0xd9, 0xaa, 0xfd, 0xbd, 0x9b, 0x21, 0x98, \
	0xa8, 0x49, 0xe1, 0x45, 0xf3, 0xd8, 0xd1, 0x69                /**< UUID of control point. */
#define ANCS_DATA_SOURCE_UUID                0xfb, 0x7b, 0x7c, 0xce, 0x6a, 0xb3, 0x44, 0xbe, \
	0xb5, 0x4b, 0xd6, 0x24, 0xe9, 0xc6, 0xea, 0x22                /**< UUID of data source. */

/**@brief Event types that are passed from client to application on an event. */
typedef enum {
	BLE_ANCS_C_EVT_INVALID,                /**< ANCS Client invalid event type. */
	BLE_ANCS_C_EVT_DISCOVERY_CPLT,         /**< ANCS Client has found ANCS service and its characteristics. */
	BLE_ANCS_C_EVT_DISCOVERY_FAILED,       /**< ANCS Client found ANCS service failed because of invalid operation or no found at the peer. */
	BLE_ANCS_C_EVT_NTF_SOURCE_NTF_ENABLED, /**< ANCS Client has enable notification for notification source. */
	BLE_ANCS_C_EVT_DATA_SOURCE_NTF_ENABLED,/**< ANCS Client has enable notification for data source. */
	BLE_ANCS_C_EVT_NTF_SOURCE_RECEIVE,     /**< ANCS Client has receive notification from notification source. */
	BLE_ANCS_C_EVT_DATA_SOURCE_RECEIVE,    /**< ANCS Client has receive notification from data source. */
	BLE_ANCS_C_EVT_WRITE_OP_ERR,
} ble_ancs_c_evt_type_t;

/**@brief  ancs handle structure. */
typedef struct {
	uint16_t ancs_service_handle;                   /**< Handle of ancs service as provided by a discovery. */
	uint16_t ancs_ntf_source_handle;                /**< Handle of ancs  notification source characteristic as provided by a discovery. */
	uint16_t ancs_ntf_source_cccd_handle;           /**< Handle of CCCD of ancs  control point characteristic as provided by a discovery. */
	uint16_t ancs_control_point_handle;             /**< Handle of ancs  control point characteristic as provided by a discovery. */
	uint16_t ancs_data_source_handle;               /**< Handle of ancs  data source characteristic as provided by a discovery. */
	uint16_t ancs_data_source_cccd_handle;          /**< Handle of CCCD of ancs data source characteristic as provided by a discovery. */
} ancs_c_att_handles_t;

/**@brief IDs for iOS notification attributes. */
typedef enum {
	ANCS_NOTIF_ATTR_ID_APP_IDENTIFIER = 0,          /**< Identify that the attribute data is of an "App Identifier" type. */
	ANCS_NOTIF_ATTR_ID_TITLE,                       /**< Identify that the attribute data is a "Title". */
	ANCS_NOTIF_ATTR_ID_SUBTITLE,                    /**< Identify that the attribute data is a "Subtitle". */
	ANCS_NOTIF_ATTR_ID_MESSAGE,                     /**< Identify that the attribute data is a "Message". */
	ANCS_NOTIF_ATTR_ID_MESSAGE_SIZE,                /**< Identify that the attribute data is a "Message Size". */
	ANCS_NOTIF_ATTR_ID_DATE,                        /**< Identify that the attribute data is a "Date". */
	ANCS_NOTIF_ATTR_ID_POSITIVE_ACTION_LABEL,       /**< The notification has a "Positive action" that can be executed associated with it. */
	ANCS_NOTIF_ATTR_ID_NEGATIVE_ACTION_LABEL,       /**< The notification has a "Negative action" that can be executed associated with it. */
} ancs_notification_attr_t;

/**@brief Category IDs for iOS notifications. */
typedef enum {
	ANCS_CATEGORY_ID_OTHER,                 /**< The iOS notification belongs to the "other" category.  */
	ANCS_CATEGORY_ID_INCOMING_CALL,         /**< The iOS notification belongs to the "Incoming Call" category. */
	ANCS_CATEGORY_ID_MISSED_CALL,           /**< The iOS notification belongs to the "Missed Call" category. */
	ANCS_CATEGORY_ID_VOICE_MAIL,            /**< The iOS notification belongs to the "Voice Mail" category. */
	ANCS_CATEGORY_ID_SOCIAL,                /**< The iOS notification belongs to the "Social" category. */
	ANCS_CATEGORY_ID_SCHEDULE,              /**< The iOS notification belongs to the "Schedule" category. */
	ANCS_CATEGORY_ID_EMAIL,                 /**< The iOS notification belongs to the "E-mail" category. */
	ANCS_CATEGORY_ID_NEWS,                  /**< The iOS notification belongs to the "News" category. */
	ANCS_CATEGORY_ID_HEALTH_AND_FITNESS,    /**< The iOS notification belongs to the "Health and Fitness" category. */
	ANCS_CATEGORY_ID_BUSINESS_AND_FINANCE,  /**< The iOS notification belongs to the "Buisness and Finance" category. */
	ANCS_CATEGORY_ID_LOCATION,              /**< The iOS notification belongs to the "Location" category. */
	ANCS_CATEGORY_ID_ENTERTAINMENT          /**< The iOS notification belongs to the "Entertainment" category. */
} ancs_category_id_t;

/**@brief Event IDs for iOS notifications. */
typedef enum {
	ANCS_EVENT_ID_NOTIFICATION_ADDED,       /**< The iOS notification was added. */
	ANCS_EVENT_ID_NOTIFICATION_MODIFIED,    /**< The iOS notification was modified. */
	ANCS_EVENT_ID_NOTIFICATION_REMOVED      /**< The iOS notification was removed. */
} ancs_evt_id_t;

/**@brief ID for actions that can be performed for iOS notifications. */
typedef enum {
	ACTION_ID_POSITIVE = 0,                 /**< Positive action. */
	ACTION_ID_NEGATIVE                      /**< Negative action. */
} ancs_c_action_id_t;

/**@brief ctrl point command that can be performed for iOS notifications. */
typedef enum {
	CTRL_POINT_GET_NTF_ATTRIBUTE = 0,/**< Request attributes to be sent from the NP to the NC for a given notification. */
	CTRL_POINT_GET_APP_ATTRIBUTE,    /**< Request attributes to be sent from the NP to the NC for a given iOS app. */
	CTRL_POINT_PERFORM_NTF_ACTION,   /**< Request an action to be performed on a given notification, for example, dismiss an alarm. */
} ancs_c_ctrl_point_t;


/**@brief notification flags that can be performed for iOS notifications. */
typedef struct {
	uint8_t silent          : 1;            /**< If this flag is set, the notification has a low priority. */
	uint8_t important       : 1;            /**< If this flag is set, the notification has a high priority. */
	uint8_t pre_existing    : 1;            /**< If this flag is set, the notification is pre-existing. */
	uint8_t positive_action : 1;            /**< If this flag is set, the notification has a positive action that can be taken. */
	uint8_t negative_action : 1;            /**< If this flag is set, the notification has a negative action that can be taken. */
} ancs_ntf_flags_t;

/**@brief iOS notification structure. */
typedef struct {
	ancs_evt_id_t event_id;                 /**< Whether the notification was added, removed, or modified. */
	ancs_ntf_flags_t event_flags;           /**< Whether the notification was added, removed, or modified. */
	ancs_category_id_t category_id;         /**< Classification of the notification type, for example, email or location. */
	uint8_t category_count;                 /**< Current number of active notifications for this category ID. */
	uint32_t notification_uid;              /**< Notification UID. */
} ntf_source_pdu_t;


extern struct bt_conn *ancs_c_conn;

/**
 *****************************************************************************************
 * @brief To access phone's all services about ANCS.
 *****************************************************************************************
 */
void ble_ancs_c_discovery_service(struct bt_conn *conn);

/**
 *****************************************************************************************
 * @brief Decode notification source message.
 *
 * @param[in] p_data: Pointer to the parameters of the read request.
 * @param[in] length: The Length of read value
 *****************************************************************************************
 */
void ancs_decode_notification_source(uint8_t *p_data, uint16_t length);

/**
 *****************************************************************************************
 * @brief Decode data source message.
 *
 * @param[in] p_data: Pointer to the parameters of the read request.
 * @param[in] length: Length of read data
 *****************************************************************************************
 */
void ancs_decode_data_source(uint8_t *p_data, uint16_t length);

/**
 *****************************************************************************************
 * @brief Get notification attribute
 *
 * @param[in] uid: The UID of notify message
 * @param[in] noti_attr: The notification attribute
 *
 *****************************************************************************************
 */
void ancs_notify_attr_get(uint32_t uid, char noti_attr);

/**
 *****************************************************************************************
 * @brief ancs perform action
 *
 * @param[in] uid: The UID of notify message
 * @param[in] action: The action status defined by specification
 *
 *****************************************************************************************
 */
void ancs_action_perform(uint32_t uid, int action);

/**
 *****************************************************************************************
 * @brief set phone call notify message uid
 *
 * @param[in] uid: the uid of notify message
 *****************************************************************************************
 */
void ancs_set_uid(uint32_t uid);

/**
 *****************************************************************************************
 * @brief get ancs phone call UID
 *
 * @return phone call notify message UID
 *****************************************************************************************
 */
uint32_t ancs_get_uid(void);

///获取APP_ID软件名字、title发件人名字、message信息内容///
void ancs_notify_ctrpoint_get(uint32_t uid);
///获取APP_ID软件名字、title发件人名字、message信息内容///

#endif
