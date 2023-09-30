#ifndef BLUETOOTHLE_H_
#define BLUETOOTHLE_H_

//Bluetooth
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/services/hrs.h>

void connected(struct bt_conn *conn, uint8_t err);
void disconnected(struct bt_conn *conn, uint8_t reason);
void auth_cancel(struct bt_conn *conn);

void BT_Init(void);
void BT_Adv_Start(void);


#endif