#include "bluetoothLE.h"

const struct bt_data advertising_data[] = { 
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_UUID16_ALL, 
				  BT_UUID_16_ENCODE(BT_UUID_HRS_VAL) ),
};

 void connected(struct bt_conn *conn, uint8_t err){
	if(err){
		printk("Connection failed: err -> %d \n ", err);
	}
	else{
		printk("Connected \n");
	}
}
 void disconnected(struct bt_conn *conn, uint8_t reason){
		printk("Disconnected: reason -> %d \n ", reason);
	}
 void auth_cancel(struct bt_conn *conn)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printk("Pairing cancelled: %s\n", addr);
}

 void BT_Init(void){
	int	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
	}
	else{
		printk("Bluetooth initalize success \n");
	}
}
 void BT_Adv_Start(void){
	int err = bt_le_adv_start(BT_LE_ADV_CONN_NAME, advertising_data, 
                            ARRAY_SIZE( advertising_data),NULL, 0);
	if (err)
	{
		printk("Advertising start faliled: %d \n", err );
		return;
	}
	else{
		printk("Advertising start success \n" );
	}
	
}




