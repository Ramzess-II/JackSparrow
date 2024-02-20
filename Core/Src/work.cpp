#include "work.hpp"
#include "helper_3dmath.hpp"
#include "MPU6050.hpp"
#include "MPU6050_6Axis_MotionApps20.hpp"
#include "support.h"
//----------------------- переменные из других файлов --------------------------------//
extern TIM_HandleTypeDef htim2;
extern uint16_t adcBuf[10];
//----------------------- переменные из этого файла ----------------------------------//
uint32_t flag;
MPU6050 mpu;
uint8_t fifoBuffer[45];         // буфер
Quaternion q;
VectorFloat gravity;
float rawAngle[3];
int32_t yaw, pitch, roll, targetAngle, currentAngle;
//------------------------------ функции ---------------------------------------------//
void doWork (void) {
	mpu.initialize();
	mpu.dmpInitialize();
	mpu.setDMPEnabled(true);
	HAL_TIM_Base_Start_IT (&htim2);
    ARGB_Init();  // Initialization
    ARGB_Clear(); // Clear stirp
    startMoveAndLed (20, COUNT_SCREENSAVER);
}

void startMoveAndLed (uint8_t delay, uint32_t count) {
	uint8_t temporary = 0;
	currentAngle = 0;
	targetAngle = 1280;
	for (uint32_t i = 0; i < count; i ++) {
			// Рассчитываем оттенки для каждого светодиода с учетом запаздывания в 90 градусов
			float hue1 = i; // для первого светодиода
			float hue2 = fmod(i + 90, 360.0); // для второго светодиода
			float hue3 = fmod(i + 180, 360.0); // для третьего светодиода
			float hue4 = fmod(i + 270, 360.0); // для четвертого светодиода
			// Преобразуем оттенки в диапазон [0, 255]
			uint8_t hueByte1 = hue1 * 255 / 360;
			uint8_t hueByte2 = hue2 * 255 / 360;
			uint8_t hueByte3 = hue3 * 255 / 360;
			uint8_t hueByte4 = hue4 * 255 / 360;
			// Устанавливаем цвета HSV для каждого светодиода
			ARGB_SetHSV(0, hueByte1, 255, 255); // для первого светодиода
			ARGB_SetHSV(1, hueByte2, 255, 255); // для второго светодиода
			ARGB_SetHSV(2, hueByte3, 255, 255); // для третьего светодиода
			ARGB_SetHSV(3, hueByte4, 255, 255); // для четвертого светодиода
			while (!ARGB_Show());  // Update - Option 2
			temporary = adcBuf[0] % 10;
			if (temporary > 5) {
				if (currentAngle == targetAngle) targetAngle = (HAL_GetTick() % 900);
			} else {
				if (currentAngle == targetAngle) targetAngle = -(adcBuf[5] % 600);
			}
			HAL_Delay(delay);
	}
}

float fixDegrees(float deg) {
  if (deg < 0) {
    return deg + 360; // добавляем 360 к отрицательным углам
  } else {
    return deg;
  }
}

void setRandomPozition (void) {
	if ((HAL_GetTick() % 10000)==0) {
		startMoveAndLed (constrain (adcBuf[0] % 50, 15, 50), constrain(adcBuf[1] % 100, 20, 80) );
	}
}

void Work (void) {
	int32_t deltaYaw = 0;
	static int32_t lastYaw = 0;
	setRandomPozition();
	if (flag) {
		flag = false;
		if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {     // расчёты
		    mpu.dmpGetQuaternion(&q, fifoBuffer);
		    mpu.dmpGetGravity(&gravity, &q);
		    mpu.dmpGetYawPitchRoll(rawAngle, &q, &gravity);
		    yaw = (int32_t)fixDegrees (degrees(rawAngle[0]));
		    pitch = (int32_t)fixDegrees (degrees(rawAngle[1]));
		    roll = (int32_t)fixDegrees (degrees(rawAngle[2]));
		    deltaYaw = yaw - lastYaw;
		    if (deltaYaw > 180) {
		        deltaYaw -= 360;
		    } else if (deltaYaw < -180) {
		        deltaYaw += 360;
		    }
		    lastYaw = yaw;
		    targetAngle += deltaYaw * 3.5;
		    // Определяем отношение угла к каждому цвету
		    float hue;
		    if (yaw < 120) {
		        hue = yaw * (1.0 / 120.0); // Оттенок для перехода от синего к зеленому
		    } else if (yaw < 240) {
		        hue = (yaw - 120.0) * (1.0 / 120.0); // Оттенок для перехода от зеленого к красному
		    } else {
		        hue = (yaw - 240.0) * (1.0 / 120.0); // Оттенок для перехода от красного к синему
		    }
		    // Преобразуем оттенок в диапазон [0, 255]
		    uint8_t hueByte = hue * 255;       // Устанавливаем цвет HSV
		    ARGB_SetHSV(0, hueByte, 255, 255); // Saturation и Value оставляем максимальными
		    ARGB_SetHSV(1, hueByte, 255, 255); // Saturation и Value оставляем максимальными
		    ARGB_SetHSV(2, hueByte, 255, 255); // Saturation и Value оставляем максимальными
		    ARGB_SetHSV(3, hueByte, 255, 255); // Saturation и Value оставляем максимальными
		    while (!ARGB_Show());  // Update - Option 2
		}
	}
	/*if (count >= 100280) {                     //1280
		HAL_TIM_Base_Stop_IT (&htim2);
	}*/
}

//------------------------------ прерывания ------------------------------------------//
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	flag = true;
}

void HAL_TIM_PeriodElapsedCallback (TIM_HandleTypeDef *htim3) {
	HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
	if (currentAngle < targetAngle) {
		HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);
		HAL_GPIO_TogglePin(STEP_GPIO_Port, STEP_Pin);
		currentAngle ++;
	}
	if (currentAngle > targetAngle) {
		HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
		HAL_GPIO_TogglePin(STEP_GPIO_Port, STEP_Pin);
		currentAngle --;
	}
}
//------------------------------ примечания ------------------------------------------//
