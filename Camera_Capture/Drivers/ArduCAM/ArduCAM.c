#include "ArduCAM.h"
#include "ov2640_regs.h"
#include "ov5642_regs.h"
#include "ov5640_regs.h"

extern I2C_HandleTypeDef hi2c1; // Déclare l'instance de I2C utilisée dans main.c
extern SPI_HandleTypeDef hspi1;  // Handle pour l'interface SPI1
extern UART_HandleTypeDef huart2;

extern uint8_t DMA_Complet;

byte sensor_model = 0;
byte sensor_addr = 0x60;
byte m_fmt = JPEG;

uint8_t vid, pid;

void ArduCAM_Init(byte model) {
	write_reg(0x07, 0x80);
	HAL_Delay(100);
	write_reg(0x07, 0x00);
	HAL_Delay(100);

	set_format(JPEG);

	wrSensorReg8_8(0xff, 0x01);
	rdSensorReg8_8(OV2640_CHIPID_HIGH, &vid);
	rdSensorReg8_8(OV2640_CHIPID_LOW, &pid);
	if ((vid != 0x26) && ((pid != 0x41) || (pid != 0x42))) {
		//Serial.println(F("ACK CMD Can't find OV2640 module! END"));
		HAL_Delay(1000);
	} else {
		//Serial.println(F("ACK CMD OV2640 detected. END"));break;
	}

	wrSensorReg8_8(0xff, 0x01);
	wrSensorReg8_8(0x12, 0x80);
	HAL_Delay(100);
	if (m_fmt == JPEG) {
		wrSensorRegs8_8(OV2640_JPEG_INIT);
		wrSensorRegs8_8(OV2640_YUV422);
		wrSensorRegs8_8(OV2640_JPEG);
		wrSensorReg8_8(0xff, 0x01);
		wrSensorReg8_8(0x15, 0x00);
		wrSensorRegs8_8(OV2640_800x600_JPEG);
		//wrSensorReg8_8(0xff, 0x00);
		//wrSensorReg8_8(0x44, 0x32);
	} else {
		wrSensorRegs8_8(OV2640_QVGA);
	}
}

uint32_t SingleCapTransfer(void) {
	uint32_t length = 0;

	set_frame(1);

	flush_fifo();
	HAL_Delay(100);
	flush_fifo();
	clear_fifo_flag();
	start_capture();
	while (!get_bit(ARDUCHIP_TRIG, CAP_DONE_MASK));

	length = (int) read_fifo_length();
	length--;
	return length;
}

void set_format(byte fmt) {
	if (fmt == BMP)
		m_fmt = BMP;
	else
		m_fmt = JPEG;
}

void set_frame(byte nbr) {
	write_reg(ARDUCHIP_FRAMES, nbr - 1);
}

void set_power_down(){
	write_reg(0x06, 0x02);
}

void set_power_up(){
	write_reg(0x06, 0x01);
}

uint8_t bus_read(uint8_t address) {
	uint8_t txData = address;
	uint8_t rxData = 0x00;

	// Abaisse le signal CS (chip select)
	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);

	// Envoi de l'adresse via SPI
	HAL_SPI_Transmit(&hspi1, &txData, 1, HAL_MAX_DELAY);

	//HAL_Delay(1);

	// Lecture de la donnée via SPI
	HAL_SPI_Receive(&hspi1, &rxData, 1, HAL_MAX_DELAY);

	//HAL_Delay(1);

	// Libère le signal CS
	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);

	return rxData;
}

uint8_t bus_write(uint8_t address, uint8_t value) {
	// Abaisse le signal CS (chip select)
	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);

	// Attente de 10 millisecondes
	//HAL_Delay(1);

	// Envoi de l'adresse via SPI
	HAL_SPI_Transmit(&hspi1, &address, 1, HAL_MAX_DELAY);

	// Envoi de la valeur via SPI
	HAL_SPI_Transmit(&hspi1, &value, 1, HAL_MAX_DELAY);

	// Attente de 10 millisecondes
	//HAL_Delay(1);

	// Libère le signal CS
	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);

	return 1;  // Indique que l'opération s'est bien déroulée
}

uint8_t read_reg(uint8_t addr) {
	uint8_t data;
	data = bus_read(addr & 0x7F);
	return data;
}

void write_reg(uint8_t addr, uint8_t data) {
	bus_write(addr | 0x80, data);
}

uint8_t read_fifo(void) {
	uint8_t data;
	data = bus_read(SINGLE_FIFO_READ);
	return data;
}

void set_fifo_burst() {
	uint8_t value = 0x3C;
	HAL_SPI_Transmit(&hspi1, &value, 1, HAL_MAX_DELAY);
}

void flush_fifo(void) {
	write_reg(ARDUCHIP_FIFO, FIFO_CLEAR_MASK);
}

void start_capture(void) {
	write_reg(ARDUCHIP_FIFO, FIFO_START_MASK);
}

void clear_fifo_flag(void) {
	write_reg(ARDUCHIP_FIFO, FIFO_CLEAR_MASK);
}

uint32_t read_fifo_length(void) {
	uint32_t len1, len2, len3, len = 0;
	len1 = read_reg(FIFO_SIZE1);
	len2 = read_reg(FIFO_SIZE2);
	len3 = read_reg(FIFO_SIZE3) & 0x7f;
	len = ((len3 << 16) | (len2 << 8) | len1) & 0x07fffff;
	return len;
}

void read_burst() {

}

//Set corresponding bit  
void set_bit(uint8_t addr, uint8_t bit) {
	uint8_t temp;
	temp = read_reg(addr);
	write_reg(addr, temp | bit);
}
//Clear corresponding bit 
void clear_bit(uint8_t addr, uint8_t bit) {
	uint8_t temp;
	temp = read_reg(addr);
	write_reg(addr, temp & (~bit));
}

//Get corresponding bit status
uint8_t get_bit(uint8_t addr, uint8_t bit) {
	uint8_t temp;
	temp = read_reg(addr);
	temp = temp & bit;
	return temp;
}

//Set ArduCAM working mode
//MCU2LCD_MODE: MCU writes the LCD screen GRAM
//CAM2LCD_MODE: Camera takes control of the LCD screen
//LCD2MCU_MODE: MCU read the LCD screen GRAM
void set_mode(uint8_t mode) {
	switch (mode) {
	case MCU2LCD_MODE:
		write_reg(ARDUCHIP_MODE, MCU2LCD_MODE);
		break;
	case CAM2LCD_MODE:
		write_reg(ARDUCHIP_MODE, CAM2LCD_MODE);
		break;
	case LCD2MCU_MODE:
		write_reg(ARDUCHIP_MODE, LCD2MCU_MODE);
		break;
	default:
		write_reg(ARDUCHIP_MODE, MCU2LCD_MODE);
		break;
	}
}

void OV2640_set_JPEG_size(uint8_t size) {
	switch (size) {
	case OV2640_160x120:
		wrSensorRegs8_8(OV2640_160x120_JPEG);
		break;
	case OV2640_176x144:
		wrSensorRegs8_8(OV2640_176x144_JPEG);
		break;
	case OV2640_320x240:
		wrSensorRegs8_8(OV2640_320x240_JPEG);
		break;
	case OV2640_352x288:
		wrSensorRegs8_8(OV2640_352x288_JPEG);
		break;
	case OV2640_640x480:
		wrSensorRegs8_8(OV2640_640x480_JPEG);
		break;
	case OV2640_800x600:
		wrSensorRegs8_8(OV2640_800x600_JPEG);
		break;
	case OV2640_1024x768:
		wrSensorRegs8_8(OV2640_1024x768_JPEG);
		break;
	case OV2640_1280x1024:
		wrSensorRegs8_8(OV2640_1280x1024_JPEG);
		break;
	case OV2640_1600x1200:
		wrSensorRegs8_8(OV2640_1600x1200_JPEG);
		break;
	default:
		wrSensorRegs8_8(OV2640_320x240_JPEG);
		break;
	}
}

void OV5640_set_JPEG_size(uint8_t size) {
	switch (size) {
	case OV5640_320x240:
		wrSensorRegs16_8(OV5640_QSXGA2QVGA);
		break;
	case OV5640_352x288:
		wrSensorRegs16_8(OV5640_QSXGA2CIF);
		break;
	case OV5640_640x480:
		wrSensorRegs16_8(OV5640_QSXGA2VGA);
		break;
	case OV5640_800x480:
		wrSensorRegs16_8(OV5640_QSXGA2WVGA);
		break;
	case OV5640_1024x768:
		wrSensorRegs16_8(OV5640_QSXGA2XGA);
		break;
	case OV5640_1280x960:
		wrSensorRegs16_8(OV5640_QSXGA2SXGA);
		break;
	case OV5640_1600x1200:
		wrSensorRegs16_8(OV5640_QSXGA2UXGA);
		break;
	case OV5640_2048x1536:
		wrSensorRegs16_8(OV5640_QSXGA2QXGA);
		break;
	case OV5640_2592x1944:
		wrSensorRegs16_8(OV5640_JPEG_QSXGA);
		break;
	default:
		//320x240
		wrSensorRegs16_8(OV5640_QSXGA2QVGA);
		break;
	}
}

void OV5642_set_JPEG_size(uint8_t size) {
	switch (size) {
	case OV5642_320x240:
		wrSensorRegs16_8(ov5642_320x240);
		break;
	case OV5642_640x480:
		wrSensorRegs16_8(ov5642_640x480);
		break;
	case OV5642_1024x768:
		wrSensorRegs16_8(ov5642_1024x768);
		break;
	case OV5642_1280x960:
		wrSensorRegs16_8(ov5642_1280x960);
		break;
	case OV5642_1600x1200:
		wrSensorRegs16_8(ov5642_1600x1200);
		break;
	case OV5642_2048x1536:
		wrSensorRegs16_8(ov5642_2048x1536);
		break;
	case OV5642_2592x1944:
		wrSensorRegs16_8(ov5642_2592x1944);
		break;
	default:
		wrSensorRegs16_8(ov5642_320x240);
		break;
	}
}

byte wrSensorReg8_8(uint8_t regID, uint8_t regDat) {
	HAL_Delay(1); // Temporisation en millisecondes pour laisser le temps au capteur
	uint8_t data[2] = { regID, regDat };
	//data[0] = regID; // Première valeur de 8 bits
	//data[1] = regDat; // Deuxième valeur de 8 bits

	// Démarre la transmission en envoyant l'adresse du capteur avec HAL I2C
	HAL_I2C_Master_Transmit(&hi2c1, sensor_addr, &data, 2, 1000);

	// Envoie la valeur du registre au capteur
	//HAL_I2C_Master_Transmit(&hi2c1, sensor_addr, &regDat, 1, HAL_MAX_DELAY);

	return 0;  // Succès
}

byte rdSensorReg8_8(uint8_t regID, uint8_t *regDat) {
	HAL_Delay(10);  // Petit délai pour la stabilité du bus

	// Envoi de l'adresse du capteur en mode écriture et ID du registre
	if (HAL_I2C_Master_Transmit(&hi2c1, sensor_addr, &regID, 1, HAL_MAX_DELAY)
			!= HAL_OK) {
		return 1;  // Erreur lors de l'écriture de l'adresse
	}

	HAL_Delay(1);  // Délai pour la synchronisation du bus

	// Relance de l'I2C avec l'adresse du capteur en mode lecture
	if (HAL_I2C_Master_Receive(&hi2c1, sensor_addr | 0x01, regDat, 1,
			HAL_MAX_DELAY) != HAL_OK) {
		return 2;  // Erreur lors de la lecture
	}

	HAL_Delay(1);  // Délai pour s'assurer de la bonne fin de la transaction

	return 0;  // Lecture réussie
}

// I2C Array Write 8-bit address, 8-bit data
int wrSensorRegs8_8(const struct sensor_reg reglist[]) {
	int err = 0;
	uint16_t reg_addr = 0;
	uint16_t reg_val = 0;
	const struct sensor_reg *next = reglist;

	while ((reg_addr != 0xFF) || (reg_val != 0xFF)) // Utilisation de || pour une condition correcte
	{
		reg_addr = next->reg;
		reg_val = next->val;

		err = wrSensorReg8_8(reg_addr, reg_val); // Écriture dans le registre
		if (err != 0) {
			return err; // Retourne l'erreur immédiatement si une écriture échoue
		}

		HAL_Delay(1); // Pause de 1 ms pour garantir la stabilité du bus I2C

		next++; // Passe au prochain registre
	}

	return err; // Retourne 0 si toutes les écritures sont réussies
}

// Fonction d'écriture sur un registre 16 bits adresse, 8 bits données via HAL I2C
byte wrSensorReg16_8(int regID, int regDat) {
	uint8_t data[3];

	// Construire l'adresse du registre et les données
	data[0] = (uint8_t) (regID >> 8);  // Partie haute de l'adresse du registre
	data[1] = (uint8_t) (regID & 0xFF); // Partie basse de l'adresse du registre
	data[2] = (uint8_t) regDat;         // Données à écrire

	// Transmission des données via I2C
	if (HAL_I2C_Master_Transmit(&hi2c1, sensor_addr << 1, data, 3, 1000)
			!= HAL_OK) {
		// Si l'écriture échoue, retourner 0
		return 0;
	}

	return 1;  // Succès
}

// Fonction d'écriture sur un registre 16 bits adresse, 8 bits données via HAL I2C
int wrSensorRegs16_8(const struct sensor_reg reglist[]) {
	int err = 0;

	unsigned int reg_addr;
	unsigned char reg_val;
	const struct sensor_reg *next = reglist;

	while ((reg_addr != 0xffff) | (reg_val != 0xff)) {
		reg_addr = next->reg;
		reg_val = next->val;
		err = wrSensorReg16_8(reg_addr, reg_val);
		HAL_Delay(600);
		next++;
	}
	return err;
}

// Fonction de lecture sur un registre 16 bits adresse, 8 bits données via HAL I2C
byte rdSensorReg16_8(uint16_t regID, uint8_t *regDat) {
	uint8_t data[2];  // Tableau pour stocker l'adresse du registre (16 bits)
	HAL_StatusTypeDef status;

	// Préparer l'adresse du registre (2 octets)
	data[0] = (uint8_t) (regID >> 8);  // Partie haute de l'adresse
	data[1] = (uint8_t) (regID & 0xFF); // Partie basse de l'adresse

	// Envoi de l'adresse du registre
	status = HAL_I2C_Master_Transmit(&hi2c1, sensor_addr + 1, data, 2, 1000);
	if (status != HAL_OK) {
		// Si l'envoi échoue, retour 0
		return 0;
	}

	HAL_Delay(20);  // Délai après la transmission

	// Lecture de la donnée à partir du registre
	status = HAL_I2C_Master_Receive(&hi2c1, (sensor_addr + 1) | 0x01, regDat, 1,
			1000);
	if (status != HAL_OK) {
		// Si la lecture échoue, retour 0
		return 0;
	}

	return 1;  // Succès
}
