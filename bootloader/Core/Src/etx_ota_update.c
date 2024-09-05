/*
 * etx_ota_update.c
 *
 *  Created on: Aug 12, 2024
 *      Author: amitsingh14
 */

#include "etx_ota_update.h"
#include "main.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern CRC_HandleTypeDef hcrc;

uint8_t Rx_Buffer[ETX_OTA_PACKET_MAX_SIZE];
ETX_OTA_STATE_ ota_state = ETX_OTA_STATE_IDLE;
uint32_t fw_total_size = 0, fw_recieved_size = 0, fw_recieved_crc = 0;
;

meta_info *cfg_flash = (meta_info*) (ETX_CONFIG_FLASH_ADDR);

uint16_t etx_receive_chunk(uint8_t *buf, uint16_t max_len);
ETX_OTA_EX_ etx_process_data(uint8_t *buf, uint16_t len);
HAL_StatusTypeDef write_data_to_flash_app(uint8_t *data, uint16_t data_len);
HAL_StatusTypeDef write_data_to_slot(uint8_t *data, uint16_t data_len,
bool is_first_block);
uint32_t calculate_crc32(uint8_t *pbuff, uint32_t len);
static HAL_StatusTypeDef write_cfg_to_flash(meta_info *cfg);
void load_new_app(void);
void etx_ota_send_resp(uint8_t type);

ETX_OTA_EX_ etx_ota_download_and_flash(void) {

	ETX_OTA_EX_ ret = ETX_OTA_EX_OK;
	uint16_t len;

	memset(Rx_Buffer, 0, ETX_OTA_PACKET_MAX_SIZE);

	printf("Waiting for the OTA data...\r\n");

	ota_state = ETX_OTA_STATE_START;

	while (ota_state != ETX_OTA_STATE_IDLE) {
		len = etx_receive_chunk(Rx_Buffer, ETX_OTA_PACKET_MAX_SIZE);

		if (len != 0u) {
			//ret = etx_process_data( Rx_Buffer, len );
			printf("Frame length= %d\n", len);
			ret = etx_process_data(Rx_Buffer, len);
			//break;

		} else {
			//didn't received data. break.
			ret = ETX_OTA_EX_ERR;
		}

		//Send ACK or NACK
		if( ret != ETX_OTA_EX_OK )
		 {
			 printf("Sending NACK\r\n");
			 etx_ota_send_resp( ETX_OTA_NACK );
			 break;
		 }
		 else
		 {
			 printf("Sending ACK\r\n");
			 etx_ota_send_resp( ETX_OTA_ACK );
		 }

	}

	return ret;
}

uint32_t calculate_crc32(uint8_t *pbuff, uint32_t len) {
	uint32_t crc = 0xFF;
	__HAL_CRC_DR_RESET(&hcrc);
	for (uint32_t i = 0; i < len; i++) {
		uint32_t data = pbuff[i];
		crc = HAL_CRC_Accumulate(&hcrc, &data, 1);
	}
	//sprintf(log_buff,"crc: 0x%lx",crc);
	//DEBUG_LOG(TAG, log_buff);
	return crc;
}

uint16_t etx_receive_chunk(uint8_t *buf, uint16_t max_len) {
	int16_t ret;
	uint16_t index = 0u;
	uint16_t data_len = 0u;

	ret = HAL_UART_Receive(&huart1, &buf[index], 1, HAL_MAX_DELAY);

	//printf("sof=0x%x\n",buf[index]);
	if (ret != HAL_OK) {
		printf("Read error\n");
		index = 0u;
		return index;
	}

	if (buf[index] != ETX_OTA_SOF) {
		printf("SOF not detected\n");
		ret = ETX_OTA_EX_ERR;
		index = 0u;
		return index;
	}
	index++;

	ret = HAL_UART_Receive(&huart1, &buf[index], 1, HAL_MAX_DELAY);
	if (ret != HAL_OK) {
		printf("Read error\n");
		index = 0u;
		return index;
	}
	//printf("cmd = 0x%x\n",buf[index]);
	index++;

	ret = HAL_UART_Receive(&huart1, &buf[index], 2, HAL_MAX_DELAY);
	if (ret != HAL_OK) {
		printf("Read error\n");
		index = 0u;
		return index;
	}
	data_len = *((uint16_t*) &buf[index]);
	//printf("len=%d\n",data_len);
	index += 2;

	for (uint16_t i = 0; i < data_len; i++) {
		ret = HAL_UART_Receive(&huart1, &buf[index++], 1, HAL_MAX_DELAY);
		if (ret != HAL_OK) {
			printf("Read error\n");
			index = 0u;
			return index;
		}
		//printf("data = %d\n",buf[index-1]);
	}

	ret = HAL_UART_Receive(&huart1, &buf[index], 4, HAL_MAX_DELAY);
	if (ret != HAL_OK) {
		printf("Read error\n");
		index = 0u;
		return index;
	}
	//printf("crc=0x%x\n",*((uint32_t *)&buf[index]));
	uint32_t packet_crc = *((uint32_t*) &buf[index]);
	index += 4;

	ret = HAL_UART_Receive(&huart1, &buf[index], 1, HAL_MAX_DELAY);
	if (ret != HAL_OK) {
		printf("Read error\n");
		index = 0u;
		return index;
	}
	//printf("end of frame = 0x%x\n",buf[index]);

	if (buf[index] != ETX_OTA_EOF) {
		printf("EOF not detected\n");
		ret = ETX_OTA_EX_ERR;
		index = 0u;
		return index;
	}

	uint32_t cal_crc = calculate_crc32((uint8_t*) &buf[4], data_len);
	if (cal_crc != packet_crc) {
		printf(
				"Chunk's CRC mismatch [Cal CRC = 0x%08lX] [Rec CRC = 0x%08lX]\r\n",
				cal_crc, packet_crc);
		;
		index = 0u;
		return index;
	}

	if (max_len < index) {
		printf(
				"Received more data than expected. Expected = %d, Received = %d\r\n",
				max_len, index);
		index = 0u;
	}

	return index;
}

ETX_OTA_EX_ etx_process_data(uint8_t *buf, uint16_t len) {

	ETX_OTA_EX_ ret = ETX_OTA_EX_ERR;

	if (buf == NULL || len == 0) {
		printf("There is not data present\n");
		return ret;
	}

	switch (ota_state) {

	case ETX_OTA_STATE_IDLE: {
		printf("In idle state");
		ret = ETX_OTA_EX_OK;
	}
		break;
	case ETX_OTA_STATE_START: {
		ETX_OTA_COMMAND_ *cmd = (ETX_OTA_COMMAND_*) buf;
		if (cmd->packet_type == ETX_OTA_PACKET_TYPE_CMD) {
			if (cmd->cmd == ETX_OTA_CMD_START) {
				printf("Received OTA START Command\n");
				ota_state = ETX_OTA_STATE_HEADER;
				ret = ETX_OTA_EX_OK;
			}
		}
	}
		break;
	case ETX_OTA_STATE_HEADER: {
		ETX_OTA_HEADER_ *header = (ETX_OTA_HEADER_*) buf;
		if (header->packet_type == ETX_OTA_PACKET_TYPE_HEADER) {
			fw_total_size = header->meta_data.package_size;
			fw_recieved_crc = header->meta_data.package_crc;
			printf("file_size= %ld\n", fw_total_size);
			printf("Received OTA Header\n");
			ota_state = ETX_OTA_STATE_DATA;
			ret = ETX_OTA_EX_OK;
		}
	}
		break;
	case ETX_OTA_STATE_DATA: {
		ETX_OTA_DATA_ *dat = (ETX_OTA_DATA_*) buf;
		uint16_t dat_len = dat->data_len;
		HAL_StatusTypeDef ex;

		if (dat->packet_type == ETX_OTA_PACKET_TYPE_DATA) {
			printf("data_size= %d\n", dat->data_len);
			//printf("data_size= %d\n",dat->dat[0]);
			//printf("data_size= %d\n",dat->dat[3]);
			//fw_recieved_size += dat_len;
			bool is_first_block = false;
			if (fw_recieved_size == 0) {
				//This is the first block
				is_first_block = true;
				/* Read the configuration */
				//ETX_GNRL_CFG_ cfg;
				//memcpy(&cfg, cfg_flash, sizeof(ETX_GNRL_CFG_));
				/* Before writing the data, reset the available slot */
				//cfg.slot_table[slot_num_to_write].is_this_slot_not_valid = 1u;
				/* write back the updated config */
				//ret = write_cfg_to_flash(&cfg);
				//if (ret != ETX_OTA_EX_OK) {
				//	break;
				//}
			}

			ex = write_data_to_slot(buf + 4, dat_len, is_first_block);

			printf("Received OTA DATA\n");
			if (ex == HAL_OK) {
				printf("[%ld/%ld]\r\n",
						fw_recieved_size / ETX_OTA_DATA_MAX_SIZE,
						fw_total_size / ETX_OTA_DATA_MAX_SIZE);

				if (fw_recieved_size >= fw_total_size) {
					ota_state = ETX_OTA_STATE_END;
				}

				ret = ETX_OTA_EX_OK;
			}
		}
	}
		break;
	case ETX_OTA_STATE_END: {
		ETX_OTA_COMMAND_ *cmd = (ETX_OTA_COMMAND_*) buf;
		if (cmd->packet_type == ETX_OTA_PACKET_TYPE_CMD) {
			if (cmd->cmd == ETX_OTA_CMD_END) {
				printf("Received OTA END Command\r\n");

				printf("Validating the received Binary...\r\n");

				uint32_t slot_addr = ETX_SLOT_ADDR;

				//Calculate and verify the CRC
				uint32_t cal_crc = calculate_crc32((uint8_t*) slot_addr,
						fw_total_size);
				if (cal_crc != fw_recieved_crc) {
					printf("ERROR: FW CRC Mismatch\r\n");
					break;
				}
				printf("Done!!!\r\n");

				meta_info cfg;
				memcpy(&cfg, cfg_flash, sizeof(meta_info));

				cfg.package_size = fw_total_size;
				cfg.package_crc = fw_recieved_crc;
				cfg.update_available = 1u;

				ret = write_cfg_to_flash(&cfg);

				if (ret == ETX_OTA_EX_OK) {
					ota_state = ETX_OTA_STATE_IDLE;
					ret = ETX_OTA_EX_OK;
				}
			}
		}
	}
		break;
	default: {
		/* Should not come here */
		ret = ETX_OTA_EX_ERR;
	}
		break;
	}

	return ret;
}

void etx_ota_send_resp(uint8_t type) {
	ETX_OTA_RESP_ rsp = {
			.sof = ETX_OTA_SOF,
			.packet_type = ETX_OTA_PACKET_TYPE_RESPONSE,
			.data_len = 1u,
			.status = type,
			.eof = ETX_OTA_EOF };

	rsp.crc = calculate_crc32((uint8_t*)&rsp.status, 1);
	//send response
	HAL_UART_Transmit(&huart1, (uint8_t*) &rsp, sizeof(ETX_OTA_RESP_),
	HAL_MAX_DELAY);
}

HAL_StatusTypeDef write_data_to_flash_app(uint8_t *data, uint16_t data_len) {
	HAL_StatusTypeDef ret;

	ret = HAL_FLASH_Unlock();
	if (ret != HAL_OK) {
		return ret;
	}

	FLASH_WaitForLastOperation( HAL_MAX_DELAY);

	__HAL_FLASH_CLEAR_FLAG(
			FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR);

	printf("Erasing the Flash memory...\r\n");
	//Erase the Flash
	FLASH_EraseInitTypeDef EraseInitStruct;
	uint32_t SectorError;
	EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
	EraseInitStruct.Sector = FLASH_SECTOR_4;
	EraseInitStruct.NbSectors = 1;
	EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;

	ret = HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError);
	if (ret != HAL_OK) {
		printf("Flash erase Error\r\n");
		return ret;
	}

	for (uint32_t i = 0; i < data_len; i++) {
		ret = HAL_FLASH_Program( FLASH_TYPEPROGRAM_BYTE,
				(ETX_APP_FLASH_ADDR + i), data[i]);

		if (ret != HAL_OK) {
			printf("Flash Write Error\r\n");
			break;
		}
	}

	if (ret != HAL_OK) {
		return ret;
	}

	ret = HAL_FLASH_Lock();
	if (ret != HAL_OK) {
		return ret;
	}

	FLASH_WaitForLastOperation( HAL_MAX_DELAY);

	return ret;
}

HAL_StatusTypeDef write_data_to_slot(uint8_t *data, uint16_t data_len,
bool is_first_block) {
	HAL_StatusTypeDef ret;

	ret = HAL_FLASH_Unlock();
	if (ret != HAL_OK) {
		return ret;
	}

	if (is_first_block) {
		printf("Erasing the Flash memory...\r\n");
		//Erase the Flash
		FLASH_EraseInitTypeDef EraseInitStruct;
		uint32_t SectorError;
		EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
		EraseInitStruct.Sector = FLASH_SECTOR_5;
		EraseInitStruct.NbSectors = 1;
		EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;

		ret = HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError);
		if (ret != HAL_OK) {
			return ret;
		}
	}

	for (int i = 0; i < data_len; i++) {
		ret = HAL_FLASH_Program( FLASH_TYPEPROGRAM_BYTE,
				(ETX_SLOT_ADDR + fw_recieved_size), data[i]);
		if (ret == HAL_OK) {
			//update the data count
			fw_recieved_size += 1;
		} else {
			printf("Flash Write Error\r\n");
			break;
		}
	}

	if (ret != HAL_OK) {
		return ret;
	}

	ret = HAL_FLASH_Lock();
	if (ret != HAL_OK) {
		return ret;
	}

	return ret;
}

static HAL_StatusTypeDef write_cfg_to_flash(meta_info *cfg) {
	HAL_StatusTypeDef ret;

	if (cfg == NULL) {
		ret = HAL_ERROR;
		return ret;
	}

	ret = HAL_FLASH_Unlock();
	if (ret != HAL_OK) {
		return ret;
	}

	//Check if the FLASH_FLAG_BSY.
	FLASH_WaitForLastOperation( HAL_MAX_DELAY);

	//Erase the Flash
	FLASH_EraseInitTypeDef EraseInitStruct;
	uint32_t SectorError;

	EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
	EraseInitStruct.Sector = FLASH_SECTOR_2;
	EraseInitStruct.NbSectors = 1;                    //erase only sector 4
	EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;

	// clear all flags before you write it to flash
	__HAL_FLASH_CLEAR_FLAG(
			FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR);

	ret = HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError);
	if (ret != HAL_OK) {
		return ret;
	}

	//write the configuration
	uint8_t *data = (uint8_t*) cfg;
	for (uint32_t i = 0u; i < sizeof(meta_info); i++) {
		ret = HAL_FLASH_Program( FLASH_TYPEPROGRAM_BYTE,
		ETX_CONFIG_FLASH_ADDR + i, data[i]);
		if (ret != HAL_OK) {
			printf("Config Flash Write Error\r\n");
			break;
		}
	}

	//Check if the FLASH_FLAG_BSY.
	FLASH_WaitForLastOperation( HAL_MAX_DELAY);

	if (ret != HAL_OK) {
		return ret;
	}

	ret = HAL_FLASH_Lock();
	if (ret != HAL_OK) {
		return ret;
	}

	return ret;
}

void load_new_app(void) {
	bool is_update_available = false;
	//uint8_t           slot_num;
	HAL_StatusTypeDef ret;
	/* Read the configuration */
	meta_info cfg;
	memcpy(&cfg, cfg_flash, sizeof(meta_info));
	/*
	 * Check the slot whether it has a new application.
	 */
	/*for( uint8_t i = 0; i < ETX_NO_OF_SLOTS; i++ )
	 {
	 if( cfg.slot_table[i].should_we_run_this_fw == 1u )
	 {
	 printf("New Application is available in the slot %d!!!\r\n", i);
	 is_update_available               = true;
	 slot_num                          = i;
	 //update the slot
	 cfg.slot_table[i].is_this_slot_active    = 1u;
	 cfg.slot_table[i].should_we_run_this_fw  = 0u;
	 break;
	 }
	 }*/
	if (cfg.update_available == 1u) {
		is_update_available = true;
		cfg.update_available = 0u;
	}

	if (is_update_available) {
		//make other slots inactive
		/*for( uint8_t i = 0; i < ETX_NO_OF_SLOTS; i++ )
		 {
		 if( slot_num != i )
		 {
		 //update the slot as inactive
		 cfg.slot_table[i].is_this_slot_active = 0u;
		 }
		 }
		 uint32_t slot_addr;
		 if( slot_num == 0u )
		 {
		 slot_addr = ETX_APP_SLOT0_FLASH_ADDR;
		 }
		 else
		 {
		 slot_addr = ETX_APP_SLOT1_FLASH_ADDR;
		 }*/
		uint32_t slot_addr = ETX_SLOT_ADDR;
		//Load the new app or firmware to app's flash address
		ret = write_data_to_flash_app((uint8_t*) slot_addr, cfg.package_size);
		if (ret != HAL_OK) {
			printf("App Flash write Error\r\n");
		} else {
			/* write back the updated config */
			ret = write_cfg_to_flash(&cfg);
			if (ret != HAL_OK) {
				printf("Config Flash write Error\r\n");
			}
		}
	} else {
		//Find the active slot in case the update is not available
		/*for( uint8_t i = 0; i < ETX_NO_OF_SLOTS; i++ )
		 {
		 if( cfg.slot_table[i].is_this_slot_active == 1u )
		 {
		 slot_num = i;
		 break;
		 }
		 }*/
	}
	//Verify the application is corrupted or not
	printf("Verifying the Application...");
	FLASH_WaitForLastOperation( HAL_MAX_DELAY);
	//Verify the application
	uint32_t cal_data_crc = calculate_crc32((uint8_t*) ETX_APP_FLASH_ADDR,
			cfg.package_size);
	FLASH_WaitForLastOperation( HAL_MAX_DELAY);
	//Verify the CRC
	if (cal_data_crc != cfg.package_crc) {
		printf("ERROR!!!\r\n");
		printf("Invalid Application. HALT!!!\r\n");
		while (1)
			;
	}
	printf("Done!!!\r\n");
}
