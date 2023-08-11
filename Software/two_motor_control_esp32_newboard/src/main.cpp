/*****************************两电机正式调试 电流环控制  zdx_try********************************/
// /*i2c gb1105 与 pwm gb1105 双电机云台测试*/
// #include <SimpleFOC.h>
// #include <HardwareSerial.h> //串口通信用
// #include "driver/uart.h"

// // 串口初始化  串口0烧写程序使用，云台与视觉算法使用串口1
// // HardwareSerial MySerial(1);

// // i2c编码器配置
// // MagneticSensorI2C sensor_i2c = MagneticSensorI2C(AS5600_I2C);
// // TwoWire I2Cone = TwoWire(0);

// MagneticSensorI2C sensor_i2c_yaw = MagneticSensorI2C(MT6701_I2C);
// TwoWire I2Cone = TwoWire(0);

// MagneticSensorI2C sensor_i2c_pitch = MagneticSensorI2C(MT6701_I2C);
// TwoWire I2Ctwo = TwoWire(1);

// // // pwm编码器配置
// // MagneticSensorPWM sensor_pwm = MagneticSensorPWM(15, 4, 904);
// // void doPWM() { sensor_pwm.handlePWM(); }

// // 电机角度限幅
// float yaw_low = -15;   // 4.5
// float yaw_high = 15;   // 7.5
// float pitch_low = -15; //-1.6
// float pitch_high = 15; // 1.6

// // 电机对象实例化
// BLDCMotor motor_pitch = BLDCMotor(6); // 必须设置电机的实际极对数，底层代码采用浮点数运算，估算后的极对数经常因为带有浮点数而错误
// BLDCMotor motor_yaw = BLDCMotor(6);

// BLDCDriver3PWM driver_yaw = BLDCDriver3PWM(25, 33, 32, 26); // new board heartbeat v1.3  (12,14,27,13)  (25,33,32,26)
// BLDCDriver3PWM driver_pitch = BLDCDriver3PWM(12, 14, 27, 13);

// // angle set point variable
// float target_angle_yaw = 0;
// float target_angle_pitch = 0;

// // zdx_try
// float loop_angle_1 = -3;
// float loop_angle_2 = 3;
// float action_angle = 0;
// int tt = 0; // 时间标志
// int tt_period = 2000;

// // instantiate the commander
// Commander command = Commander(Serial);
// void doTarget_yaw(char *cmd) { command.scalar(&target_angle_yaw, cmd); }
// void doTarget_pitch(char *cmd) { command.scalar(&target_angle_pitch, cmd); }
// void doMotor_yaw(char *cmd) { command.motor(&motor_yaw, cmd); }
// void doMotor_pitch(char *cmd) { command.motor(&motor_pitch, cmd); }

// // 串口接收数据函数
// unsigned short i;
// uint8_t j = 0;
// uint8_t camera_buffer[6] = {0, 0, 0, 0, 0, 0};
// int cam_height = 0, cam_width = 0;
// uint8_t temp;
// bool flag = false; // 开始记录标志位
// // void read_usart()
// // {
// //   i = MySerial.available(); // 返回目前串口接收区内的已经接受的数据量
// //   // Serial.println(i);
// //   if (i != 0)
// //   {
// //     while (i--)
// //     {
// //       temp = MySerial.read(); // 读取一个数据并且将它从缓存区删除
// //       if (temp == 0xaa)
// //         flag = true;
// //       if (flag)
// //       {
// //         if (j == 6)
// //           j = 0;
// //         camera_buffer[j] = temp;
// //         j++;
// //       }
// //     }
// //     if ((camera_buffer[0] == 0xaa) && (camera_buffer[5] == 0xbb))
// //     {
// //       int cam_height1 = (camera_buffer[1] << 8 | camera_buffer[2]) - 240;
// //       int cam_width1 = (camera_buffer[3] << 8 | camera_buffer[4]) - 320;
// //       if (cam_height1 == (-240) || cam_width1 == (-320))
// //       {
// //         cam_height = 0;
// //         cam_width = 0;
// //       }
// //       else
// //       {
// //         cam_height = cam_height1;
// //         cam_width = cam_width1;
// //       }
// //       // Serial.printf("height width :%d,%d\n", cam_height, cam_width);
// //       // Serial.println();
// //     }
// //     else
// //     {
// //       cam_height = 0;
// //       cam_width = 0;
// //       // Serial.printf("height width :%d,%d\n", cam_height, cam_width);
// //       // Serial.println("no serial data");
// //     }
// //   }
// //   // else
// //   //   Serial.println("no receive");
// // }

// // PID结构体

// struct PID_version
// {
//   float kp;
//   float ki;
//   float kd;
//   float error_sum;
//   float last_error;
// };

// PID_version PID_pitch;
// PID_version PID_yaw;

// // PID计算函数
// float PID_Cal(int angle_version, int angle_set, PID_version pid)
// {
//   int32_t error = angle_set - angle_version;
//   pid.error_sum += error;
//   int32_t derivative = error - pid.last_error;
//   pid.last_error = error;
//   float velocity = pid.kp * error + pid.ki * pid.error_sum + pid.kd * derivative;

//   // 输出限幅
//   if (velocity >= 20)
//     velocity = 20;
//   if (velocity <= -20)
//     velocity = -20;

//   return velocity;
// }

// // freertos 多线程电机驱动解算
// void TaskOne_motor_yaw(void *xTask1)
// {
//   while (1)
//   {
//     // read_usart();
//     sensor_i2c_yaw.update();
//     // Serial.printf("yaw angle: %f", sensor_i2c_yaw.getAngle());
//     // Serial.println();
//     motor_yaw.loopFOC();
//     // delay(200);
//     /********************************************************/
//     // 偏航电机运动限幅
//     if ((sensor_i2c_yaw.getAngle() >= yaw_high) || (sensor_i2c_yaw.getAngle() <= yaw_low))
//     {
//       motor_yaw.move(0);
//       // Serial.println("yaw cross border!");
//     }
//     else
//     {
//       if ((cam_width <= 15) && (cam_width >= -15))
//       {
//         motor_yaw.move(0);
//         // Serial.println("yaw track ok!");
//       }
//       else
//       {
//         float v1 = PID_Cal(cam_width, 0, PID_yaw);
//         motor_yaw.move(v1);
//         // Serial.printf("yaw velocity:%f", v1);
//         // Serial.println();
//       }
//     }

//     /********************************************************/

//     // motor_yaw.move(target_angle_yaw);
//     //   // motor_yaw.monitor();
//     // command.run();
//   }
// }

// void TaskTwo_motor_pitch(void *xTask2)
// {
//   while (1)
//   {
//     // read_usart();
//     sensor_i2c_pitch.update();

//     // Serial.printf("pitch angle: %f", sensor_i2c_pitch.getAngle());
//     // Serial.println();
//     // delay(10);
//     motor_pitch.loopFOC();
//     // delay(200);

//     /********************************************************/
//     // 俯仰电机运动限幅
//     if ((sensor_i2c_pitch.getAngle() >= pitch_high) || (sensor_i2c_pitch.getAngle() <= pitch_low))
//     {
//       motor_pitch.move(0);
//       // Serial.println("pitch cross border!");
//     }
//     else
//     {
//       if ((cam_height <= 15) && (cam_height >= -15))
//       {
//         motor_pitch.move(0);
//         // Serial.println("pitch track ok!");
//       }
//       else
//       {
//         float v2 = PID_Cal(cam_height, 0, PID_pitch);
//         motor_pitch.move(v2);
//         // Serial.printf("trackiung!  pitch velocity is:%f", v2);
//         // Serial.println();
//       }
//     }
//     /********************************************************/

//     // motor_pitch.move(target_angle_pitch);
//     //    motor_pitch.monitor();
//     // command.run();
//   }
// }

// /*新串口通信测试*/
// #define EX_UART_NUM UART_NUM_1
// #define PATTERN_CHR_NUM (3) /*!< Set the number of consecutive and identical characters received by receiver which defines a UART pattern*/

// #define BUF_SIZE (1024)
// #define RD_BUF_SIZE (BUF_SIZE)
// static QueueHandle_t uart0_queue;
// static const char *TAG = "uart_events";
// void Taskthree_uart(void *xTask3)
// {
//   uart_event_t event;
//   size_t buffered_size;
//   uint8_t *dtmp = (uint8_t *)malloc(RD_BUF_SIZE);
//   while (1)
//   {
//     if (xQueueReceive(uart0_queue, (void *)&event, (TickType_t)portMAX_DELAY))
//     {
//       bzero(dtmp, RD_BUF_SIZE);
//       ESP_LOGI(TAG, "uart[%d] event:", EX_UART_NUM);
//       switch (event.type)
//       {
//       // Event of UART receving data
//       /*We'd better handler data event fast, there would be much more data events than
//       other types of events. If we take too much time on data event, the queue might
//       be full.*/
//       case UART_DATA:
//         // Serial.println();
//         ESP_LOGI(TAG, "[UART DATA]: %d", event.size);
//         uart_read_bytes(EX_UART_NUM, dtmp, event.size, portMAX_DELAY);
//         // Serial.printf("1:%d , 6:%d", dtmp[0], dtmp[5]);
//         if ((dtmp[0] == 0xaa) && (dtmp[5] == 0xbb))
//         {
//           int cam_height1 = (dtmp[1] << 8 | dtmp[2]) - 240;
//           int cam_width1 = (dtmp[3] << 8 | dtmp[4]) - 320;
//           if (cam_height1 == (-240) || cam_width1 == (-320))
//           {
//             cam_height = 0;
//             cam_width = 0;
//           }
//           else
//           {
//             cam_height = cam_height1;
//             cam_width = cam_width1;
//           }
//           // Serial.printf("height:%d , width:%d", cam_height, cam_width);
//           // Serial.println();
//         }
//         ESP_LOGI(TAG, "[DATA EVT]:");
//         break;
//       // Event of HW FIFO overflow detected
//       case UART_FIFO_OVF:
//         ESP_LOGI(TAG, "hw fifo overflow");
//         // If fifo overflow happened, you should consider adding flow control for your application.
//         // The ISR has already reset the rx FIFO,
//         // As an example, we directly flush the rx buffer here in order to read more data.
//         uart_flush_input(EX_UART_NUM);
//         xQueueReset(uart0_queue);
//         break;
//       // Event of UART ring buffer full
//       case UART_BUFFER_FULL:
//         ESP_LOGI(TAG, "ring buffer full");
//         // If buffer full happened, you should consider increasing your buffer size
//         // As an example, we directly flush the rx buffer here in order to read more data.
//         uart_flush_input(EX_UART_NUM);
//         xQueueReset(uart0_queue);
//         break;
//       }
//     }
//   }
//   free(dtmp);
//   dtmp = NULL;
//   vTaskDelete(NULL);
// }

// void setup()
// {
//   // 串口通信初始化
//   // MySerial.begin(115200, SERIAL_8N1, 18, 5); // rxpin  txpin

//   /****************************新串口通信测试****************************/
//   esp_log_level_set(TAG, ESP_LOG_INFO);

//   /* Configure parameters of an UART driver,
//    * communication pins and install the driver */
//   uart_config_t uart_config = {
//       .baud_rate = 115200,
//       .data_bits = UART_DATA_8_BITS,
//       .parity = UART_PARITY_DISABLE,
//       .stop_bits = UART_STOP_BITS_1,
//       .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
//   };
//   // Install UART driver, and get the queue.
//   uart_driver_install(EX_UART_NUM, BUF_SIZE * 2, BUF_SIZE * 2, 20, &uart0_queue, 0);
//   uart_param_config(EX_UART_NUM, &uart_config);

//   // Set UART log level
//   esp_log_level_set(TAG, ESP_LOG_INFO);
//   // Set UART pins (using UART0 default pins ie no changes.)
//   uart_set_pin(EX_UART_NUM, 5, 18, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

//   // Set uart pattern detect function.
//   uart_enable_pattern_det_baud_intr(EX_UART_NUM, '+', PATTERN_CHR_NUM, 9, 0, 0);
//   // Reset the pattern queue length to record at most 20 pattern positions.
//   uart_pattern_queue_reset(EX_UART_NUM, 20);
//   /*********************************************************************/

//   // clear_usart_buffer();
//   //  PID结构体初始化
//   PID_pitch.kp = 0.0035; // 0.0035
//   PID_pitch.ki = 0;      // 0.004
//   PID_pitch.kd = 0.00025;
//   PID_pitch.error_sum = 0;
//   PID_pitch.last_error = 0;
//   PID_yaw.kp = 0.0055; //
//   PID_yaw.ki = 0;
//   PID_yaw.kd = 0.00025;
//   PID_yaw.error_sum = 0;
//   PID_yaw.last_error = 0;

//   // // i2c编码器链接驱动器  yaw
//   I2Cone.begin(4, 21, 400000); //4 21
//   sensor_i2c_yaw.init(&I2Cone);
//   motor_yaw.linkSensor(&sensor_i2c_yaw);
//   // pitch
//   I2Ctwo.begin(2, 15, 400000); //2 15
//   sensor_i2c_pitch.init(&I2Ctwo);
//   motor_pitch.linkSensor(&sensor_i2c_pitch);

//   // pwm编码器链接驱动器
//   // sensor_pwm.init();
//   // sensor_pwm.enableInterrupt(doPWM);
//   // motor_pitch.linkSensor(&sensor_pwm);

//   // driver config
//   // power supply voltage [V]
//   driver_yaw.voltage_power_supply = 8;
//   driver_yaw.init();
//   driver_pitch.voltage_power_supply = 8;
//   driver_pitch.init();
//   // link the motor and the driver
//   motor_yaw.linkDriver(&driver_yaw);
//   motor_pitch.linkDriver(&driver_pitch);

//   // choose FOC modulation (optional)
//   motor_yaw.foc_modulation = FOCModulationType::SinePWM;
//   motor_pitch.foc_modulation = FOCModulationType::SinePWM;

//   // set motion control loop to be used
//   motor_yaw.controller = MotionControlType::torque; // torque,velocity,angle
//   motor_pitch.controller = MotionControlType::torque;
//   motor_yaw.torque_controller = TorqueControlType::voltage;
//   motor_pitch.torque_controller = TorqueControlType::voltage;

//   /************yaw motor pid******************/
//   // velocity loop PID
//   // motor_yaw.PID_velocity.P = 0.25;
//   // motor_yaw.PID_velocity.I = 0.5;
//   // motor_yaw.PID_velocity.D = 0.0;
//   // motor_yaw.PID_velocity.output_ramp = 1000.0;
//   // motor_yaw.PID_velocity.limit = 4.0;
//   motor_yaw.voltage_sensor_align = 3;
//   // motor_yaw.phase_resistance=12.5;
//   // Low pass filtering time constant
//   // motor_yaw.LPF_velocity.Tf = 0.15;
//   // // angle loop PID
//   // motor_yaw.P_angle.P = 9;
//   // motor_yaw.P_angle.I = 0.8;
//   // motor_yaw.P_angle.D = 0;
//   // motor_yaw.P_angle.output_ramp = 0.0;
//   // motor_yaw.P_angle.limit = 20.0;
//   // // Low pass filtering time constant
//   // motor_yaw.LPF_angle.Tf = 0;

//   // Limits
//   // motor_yaw.velocity_limit = 40.0;
//   motor_yaw.voltage_limit = 8.0;
//   // motor_yaw.current_limit = 2.0;
//   /*********************************************/

//   /************pitch motor pid******************/
//   // velocity loop PID
//   // motor_pitch.PID_velocity.P = 0.2;
//   // motor_pitch.PID_velocity.I = 0.05;
//   // motor_pitch.PID_velocity.D = 0;
//   // motor_pitch.PID_velocity.output_ramp = 1000.0;
//   // motor_pitch.PID_velocity.limit = 4.0;
//   motor_pitch.voltage_sensor_align = 2.5;
//   // Low pass filtering time constant
//   // motor_pitch.LPF_velocity.Tf = 0.3;
//   // // angle loop PID
//   // motor_pitch.P_angle.P = 7;
//   // motor_pitch.P_angle.I = 0.8;
//   // motor_pitch.P_angle.D = 0;
//   // motor_pitch.P_angle.output_ramp = 0.0;
//   // motor_pitch.P_angle.limit = 20.0;
//   // // Low pass filtering time constant
//   // motor_pitch.LPF_angle.Tf = 0.1;

//   // Limits
//   // motor_pitch.velocity_limit = 40.0;
//   motor_pitch.voltage_limit = 8.0;
//   // motor_pitch.current_limit = 2.0;
//   /*********************************************/

//   // use monitoring with serial
//   Serial.begin(115200);
//   // comment out if not needed
//   // motor_yaw.useMonitoring(Serial);
//   // motor_pitch.useMonitoring(Serial);

//   // initialize motor_yaw
//   motor_yaw.init();
//   motor_pitch.init();
//   // align sensor and start FOC
//   motor_yaw.initFOC();
//   motor_pitch.initFOC();

//   // add target command T
//   command.add('Y', doTarget_yaw, (char *)"target angle");
//   command.add('P', doTarget_pitch, (char *)"target angle");
//   command.add('M', doMotor_yaw, (char *)"my motor");
//   command.add('N', doMotor_pitch, (char *)"my motor");

//   Serial.println(F("All motor ready!."));
//   Serial.println(F("Set the target angle using serial terminal:"));

//   pinMode(22, OUTPUT);
//   digitalWrite(22, HIGH);

//   xTaskCreatePinnedToCore(TaskOne_motor_yaw, "TaskOne", 10000, NULL, 1, NULL, 0);
//   xTaskCreatePinnedToCore(TaskTwo_motor_pitch, "TaskTwo", 10000, NULL, 2, NULL, 1);
//   xTaskCreatePinnedToCore(Taskthree_uart, "TaskThree", 10000, NULL, 3, NULL, 0);
//   //_delay(1000);
// }

// // angle set point variable
// // float target_angle = 0;

// void loop()
// {
// }

/*****************************两电机正式调试 电流环控制  zdx_try********************************/

/*****************************两电机正式调试 位置环控制  zdx_try********************************/
/*i2c gb1105 与 pwm gb1105 双电机云台测试*/
// #include <SimpleFOC.h>
// #include <HardwareSerial.h> //串口通信用

// // 串口初始化  串口0烧写程序使用，云台与视觉算法使用串口1
// HardwareSerial MySerial(1);

// MagneticSensorI2C sensor_i2c_yaw = MagneticSensorI2C(MT6701_I2C);
// TwoWire I2Cone = TwoWire(0);

// MagneticSensorI2C sensor_i2c_pitch = MagneticSensorI2C(MT6701_I2C);
// TwoWire I2Ctwo = TwoWire(1);

// // 电机角度限幅
// float yaw_low = 2.5;
// float yaw_high = 4.9;
// float pitch_low = -10; //-3.5
// float pitch_high = 10; // 1.5

// // 相机视场角度
// float camera_angle_width = 2.87979;  // 横向视角  165/180*pi
// float camera_angle_height = 1.30899; // 纵向视角  75/180*pi
// float pixel_width = 640.0;           // 横向像素
// float pixel_height = 480.0;          // 纵向像素
// float result_width = 0.0044997;      // 横向计算结果 165/180*pi/640
// float result_height = 0.0027271;     // 纵向计算结果 75/180*pi/480

// // 电机对象实例化
// BLDCMotor motor_pitch = BLDCMotor(6); // 必须设置电机的实际极对数，底层代码采用浮点数运算，估算后的极对数经常因为带有浮点数而错误
// BLDCMotor motor_yaw = BLDCMotor(6);

// BLDCDriver3PWM driver_yaw = BLDCDriver3PWM(25, 33, 32, 26); // new board heartbeat v1.3  (12,14,27,13)  (25,33,32,26)
// BLDCDriver3PWM driver_pitch = BLDCDriver3PWM(12, 14, 27, 13);

// // angle set point variable
// float target_angle_yaw = 0;
// float target_angle_pitch = 0;

// // zdx_try
// float loop_angle_1 = -3;
// float loop_angle_2 = 3;
// float action_angle = 0;
// int tt = 0; // 时间标志
// int tt_period = 2000;

// // instantiate the commander
// Commander command = Commander(Serial);
// void doTarget_yaw(char *cmd) { command.scalar(&target_angle_yaw, cmd); }
// void doTarget_pitch(char *cmd) { command.scalar(&target_angle_pitch, cmd); }
// void doMotor_yaw(char *cmd) { command.motor(&motor_yaw, cmd); }
// void doMotor_pitch(char *cmd) { command.motor(&motor_pitch, cmd); }

// // 串口接收数据函数
// unsigned short i;
// uint8_t j = 0;
// uint8_t camera_buffer[6] = {0, 0, 0, 0, 0, 0};
// float cam_height = 0, cam_width = 0;
// uint8_t temp;
// bool flag = false; // 开始记录标志位
// void read_usart()
// {
//   i = MySerial.available(); // 返回目前串口接收区内的已经接受的数据量
//   // Serial.println(i);
//   if (i != 0)
//   {
//     while (i--)
//     {
//       temp = MySerial.read(); // 读取一个数据并且将它从缓存区删除
//       if (temp == 0xaa)
//         flag = true;
//       if (flag)
//       {
//         if (j == 6)
//           j = 0;
//         camera_buffer[j] = temp;
//         j++;
//       }
//     }
//     if ((camera_buffer[0] == 0xaa) && (camera_buffer[5] == 0xbb))
//     {
//       float cam_height1 = (camera_buffer[1] << 8 | camera_buffer[2]) - 240;
//       float cam_width1 = (camera_buffer[3] << 8 | camera_buffer[4]) - 320;
//       if (cam_height1 == (-240) || cam_width1 == (-320))
//       {
//         cam_height = 0;
//         cam_width = 0;
//       }
//       else
//       {
//         cam_height = cam_height1;
//         cam_width = cam_width1;
//       }
//       Serial.printf("height width :%d,%d\n", cam_height, cam_width);
//       Serial.println();
//     }
//     else
//     {
//       cam_height = 0;
//       cam_width = 0;
//       // Serial.printf("height width :%d,%d\n", cam_height, cam_width);
//       Serial.println("no serial data");
//     }
//   }
//   // else
//   //   Serial.println("no receive");
// }

// void clear_usart_buffer()
// {
//   i = MySerial.available();
//   if (i != 0)
//   {
//     Serial.print("清空串口接收区的缓存......");
//     Serial.println(MySerial.available());
//     while (i--)
//       MySerial.read(); // 读取串口接收回来的数据但是不做处理只给与打印
//   }
//   else
//     Serial.println("串口接收区的缓存为空!!!");
// }

// // PID结构体

// struct PID_version
// {
//   float kp;
//   float ki;
//   float kd;
//   float error_sum;
//   float last_error;
// };

// PID_version PID_pitch;
// PID_version PID_yaw;

// // PID计算函数
// float PID_Cal(int angle_version, int angle_set, PID_version pid)
// {
//   int32_t error = angle_set - angle_version;
//   pid.error_sum += error;
//   int32_t derivative = error - pid.last_error;
//   pid.last_error = error;
//   float velocity = pid.kp * error + pid.ki * pid.error_sum + pid.kd * derivative;

//   // 输出限幅
//   if (velocity >= 20)
//     velocity = 20;
//   if (velocity <= -20)
//     velocity = -20;

//   return velocity;
// }

// // freertos 多线程电机驱动解算
// void TaskOne_motor_yaw(void *xTask1)
// {
//   while (1)
//   {
//     // read_usart();
//     sensor_i2c_yaw.update();
//     float angle_now_yaw = sensor_i2c_yaw.getAngle();
//     // Serial.printf("yaw angle: %f", sensor_i2c_yaw.getAngle());
//     // Serial.println();
//     motor_yaw.loopFOC();
//     // delay(200);
//     /********************************************************/
//     float angle_yaw = sensor_i2c_yaw.getAngle() + camera_angle_width / pixel_width * cam_width;
//     // Serial.printf("cal angle yaw: %f", angle_yaw);

//     // 偏航电机运动限幅
//     // if (sensor_i2c.getAngle() >= yaw_high)
//     // {
//     //   motor_yaw.move(yaw_high);
//     // }
//     // else if (sensor_i2c.getAngle() <= yaw_low)
//     // {
//     //   motor_yaw.move(yaw_low);
//     // }
//     // else
//     // {
//     //   if ((cam_width <= 15) && (cam_width >= -15))
//     //   {
//     //     motor_yaw.move(sensor_i2c.getAngle());
//     //   }
//     //   else
//     //   {
//     //     motor_yaw.move(3.7);
//     //   }
//     // }
//     // motor_yaw.move(3.7);

//     /********************************************************/

//     // motor_yaw.move(target_angle_yaw);
//     //   // motor_yaw.monitor();
//     // command.run();
//   }
// }

// void TaskTwo_motor_pitch(void *xTask2)
// {
//   while (1)
//   {
//     // read_usart();
//     sensor_i2c_pitch.update();
//     float angle_now_pitch = sensor_i2c_pitch.getAngle();
//     // Serial.printf("pitch angle: %f", angle_now_pitch);
//     // Serial.println();
//     motor_pitch.loopFOC();

//     /********************************************************/
//     // float angle_pitch = angle_now_pitch + result_height * cam_height;
//     // Serial.printf("cal : %f", angle_now_pitch);
//     // Serial.println();
//     // //俯仰电机运动限幅
//     // if (angle_now_pitch >= pitch_high)
//     // {
//     //   motor_pitch.move(pitch_high);
//     //   Serial.println("111111");
//     //   continue;
//     // }
//     // else if (angle_now_pitch <= pitch_low)
//     // {
//     //   motor_pitch.move(pitch_low);
//     //   Serial.println("222222");
//     //   continue;
//     // }
//     // else
//     // {
//     //   if ((cam_height <= 15) && (cam_height >= -15))
//     //   {
//     //     motor_pitch.move(target_angle_pitch);
//     //     //Serial.println("pitch track ok!");
//     //   }
//     //   else
//     //   {
//     //     motor_pitch.move(angle_pitch);
//     //     //Serial.printf("pitch tracking angle::%f", angle_pitch);
//     //     //Serial.println();
//     //   }
//     // }
//     /********************************************************/

//     // motor_pitch.move(target_angle_pitch);
//     motor_pitch.monitor();
//     command.run();
//   }
// }

// void Taskthree_uart(void *xTask3)
// {
//   while (1)
//   {
//     read_usart();
//     // Serial.printf("EN");
//     delay(1);
//   }
// }

// void setup()
// {
//   // 串口通信初始化
//   MySerial.begin(115200, SERIAL_8N1, 18, 5); // rxpin  txpin
//   // clear_usart_buffer();
//   //  PID结构体初始化
//   PID_pitch.kp = 0.001; // 0.0062
//   PID_pitch.ki = 0;     // 0.004
//   PID_pitch.kd = 0;
//   PID_pitch.error_sum = 0;
//   PID_pitch.last_error = 0;
//   PID_yaw.kp = 0.005;
//   PID_yaw.ki = 0.004;
//   PID_yaw.kd = 0;
//   PID_yaw.error_sum = 0;
//   PID_yaw.last_error = 0;

//   // i2c编码器链接驱动器  yaw
//   I2Cone.begin(4, 21, 400000);
//   sensor_i2c_yaw.init(&I2Cone);
//   motor_yaw.linkSensor(&sensor_i2c_yaw);
//   // pitch
//   I2Ctwo.begin(23, 15, 400000);
//   sensor_i2c_pitch.init(&I2Ctwo);
//   motor_pitch.linkSensor(&sensor_i2c_pitch);

//   // driver config
//   // power supply voltage [V]
//   driver_yaw.voltage_power_supply = 8;
//   driver_yaw.init();
//   driver_pitch.voltage_power_supply = 8;
//   driver_pitch.init();
//   // link the motor and the driver
//   motor_yaw.linkDriver(&driver_yaw);
//   motor_pitch.linkDriver(&driver_pitch);

//   // choose FOC modulation (optional)
//   motor_yaw.foc_modulation = FOCModulationType::SpaceVectorPWM;
//   motor_pitch.foc_modulation = FOCModulationType::SpaceVectorPWM;

//   // set motion control loop to be used
//   motor_yaw.controller = MotionControlType::angle; // torque,velocity,angle
//   motor_pitch.controller = MotionControlType::angle;
//   // motor_yaw.torque_controller = TorqueControlType::voltage;
//   // motor_pitch.torque_controller = TorqueControlType::dc_current;

//   /************yaw motor pid******************/
//   // velocity loop PID
//   motor_yaw.PID_velocity.P = 0.25;
//   motor_yaw.PID_velocity.I = 0.5;
//   motor_yaw.PID_velocity.D = 0.0;
//   // motor_yaw.PID_velocity.output_ramp = 1000.0;
//   // motor_yaw.PID_velocity.limit = 4.0;
//   motor_yaw.voltage_sensor_align = 3;
//   // motor_yaw.phase_resistance=12.5;
//   // Low pass filtering time constant
//   motor_yaw.LPF_velocity.Tf = 0.15;
//   // // angle loop PID
//   motor_yaw.P_angle.P = 5;
//   motor_yaw.P_angle.I = 0.8;
//   motor_yaw.P_angle.D = 0;
//   // motor_yaw.P_angle.output_ramp = 0.0;
//   // motor_yaw.P_angle.limit = 20.0;
//   // // Low pass filtering time constant
//   // motor_yaw.LPF_angle.Tf = 0;

//   // Limits
//   motor_yaw.velocity_limit = 40.0;
//   // motor_yaw.voltage_limit = 8.0;
//   // motor_yaw.current_limit = 2.0;
//   /*********************************************/

//   /************pitch motor pid******************/
//   // velocity loop PID
//   motor_pitch.PID_velocity.P = 0.2;
//   motor_pitch.PID_velocity.I = 0.05;
//   motor_pitch.PID_velocity.D = 0;
//   // motor_pitch.PID_velocity.output_ramp = 1000.0;
//   // motor_pitch.PID_velocity.limit = 4.0;
//   motor_pitch.voltage_sensor_align = 2.5;
//   // Low pass filtering time constant
//   motor_pitch.LPF_velocity.Tf = 0.3;
//   // // angle loop PID
//   motor_pitch.P_angle.P = 5;
//   motor_pitch.P_angle.I = 0.1;
//   motor_pitch.P_angle.D = 0;
//   // motor_pitch.P_angle.output_ramp = 0.0;
//   // motor_pitch.P_angle.limit = 20.0;
//   // // Low pass filtering time constant
//   // motor_pitch.LPF_angle.Tf = 0.1;

//   // Limits
//   motor_pitch.velocity_limit = 40.0;
//   // motor_pitch.voltage_limit = 8.0;
//   // motor_pitch.current_limit = 2.0;
//   /*********************************************/

//   // use monitoring with serial
//   Serial.begin(115200);
//   // comment out if not needed
//   // motor_yaw.useMonitoring(Serial);
//   motor_pitch.useMonitoring(Serial);

//   // initialize motor_yaw
//   motor_yaw.init();
//   motor_pitch.init();
//   // align sensor and start FOC
//   motor_yaw.initFOC();
//   motor_pitch.initFOC();

//   // motor_yaw.sensor_direction = Direction::CW;

//   // add target command T
//   command.add('Y', doTarget_yaw, (char *)"target angle");
//   command.add('P', doTarget_pitch, (char *)"target angle");
//   command.add('M', doMotor_yaw, (char *)"my motor");
//   command.add('N', doMotor_pitch, (char *)"my motor");

//   Serial.println(F("All motor ready."));
//   Serial.println(F("Set the target angle using serial terminal:"));

//   pinMode(22, OUTPUT);
//   digitalWrite(22, HIGH);

//   // xTaskCreatePinnedToCore(TaskOne_motor_yaw, "TaskOne", 10000, NULL, 1, NULL, 0);
//   // xTaskCreatePinnedToCore(TaskTwo_motor_pitch, "TaskTwo", 10000, NULL, 2, NULL, 1);
//   // xTaskCreatePinnedToCore(Taskthree_uart, "TaskThree", 10000, NULL, 3, NULL, 0);
//   //_delay(1000);
// }

// // angle set point variable
// // float target_angle = 0;

// void loop()
// {
//   sensor_i2c_yaw.update();
//   // float angle_now_yaw = sensor_i2c_yaw.getAngle();
//   Serial.printf("yaw angle: %f", sensor_i2c_yaw.getAngle());
//   Serial.println();
//   motor_yaw.loopFOC();
//   motor_yaw.move(-5.0);
// }

/*****************************两电机正式调试 位置环控制 zdx_try********************************/

/*****************************两电机速度模式测试用例  zdx_try********************************/
/*i2c gb1105 与 pwm gb1105 双电机云台测试*/
#include <SimpleFOC.h>
#include "driver/uart.h"

// i2c编码器配置
// MagneticSensorI2C sensor_i2c = MagneticSensorI2C(AS5600_I2C);
// TwoWire I2Cone = TwoWire(0);

MagneticSensorI2C sensor_i2c_yaw = MagneticSensorI2C(MT6701_I2C);
TwoWire I2Cone = TwoWire(0);

MagneticSensorI2C sensor_i2c_pitch = MagneticSensorI2C(MT6701_I2C);
TwoWire I2Ctwo = TwoWire(1);

// 电机角度限幅
float yaw_low = -15;   // 4.5
float yaw_high = 15;   // 7.5
float pitch_low = -15; //-1.6
float pitch_high = 15; // 1.6

// 电机对象实例化
BLDCMotor motor_pitch = BLDCMotor(6); // 必须设置电机的实际极对数，底层代码采用浮点数运算，估算后的极对数经常因为带有浮点数而错误
BLDCMotor motor_yaw = BLDCMotor(6);

BLDCDriver3PWM driver_yaw = BLDCDriver3PWM(25, 33, 32, 26); // new board heartbeat v1.3  (12,14,27,13)  (25,33,32,26)
BLDCDriver3PWM driver_pitch = BLDCDriver3PWM(12, 14, 27, 13);

// angle set point variable
float target_angle_yaw = 0;
float target_angle_pitch = 0;

// instantiate the commander
Commander command = Commander(Serial);
void doTarget_yaw(char *cmd) { command.scalar(&target_angle_yaw, cmd); }
void doTarget_pitch(char *cmd) { command.scalar(&target_angle_pitch, cmd); }
void doMotor_yaw(char *cmd) { command.motor(&motor_yaw, cmd); }
void doMotor_pitch(char *cmd) { command.motor(&motor_pitch, cmd); }

// 串口接收数据函数
unsigned short i;
uint8_t j = 0;
uint8_t camera_buffer[6] = {0, 0, 0, 0, 0, 0};
int cam_height = 0, cam_width = 0;
uint8_t mode_ctr = 0; // 0--固定模式   1--手动模式   2--自动模式
uint8_t temp;
bool flag = false; // 开始记录标志位

// PID结构体
struct PID_version
{
  float kp;
  float ki;
  float kd;
  float error_sum;
  float last_error;
};

PID_version PID_pitch;
PID_version PID_yaw;
PID_version PID_pitch_fix;
PID_version PID_yaw_fix;

// PID计算函数
float PID_Cal(float angle_version, float angle_set, PID_version pid)
{
  float error = angle_set - angle_version;
  pid.error_sum += error;
  float derivative = error - pid.last_error;
  pid.last_error = error;
  float velocity = pid.kp * error + pid.ki * pid.error_sum + pid.kd * derivative;

  // 输出限幅
  if (velocity >= 30)
    velocity = 30;
  if (velocity <= -30)
    velocity = -30;

  return velocity;
}

// freertos 多线程电机驱动解算
bool yaw_fixing_flag = true;
bool yaw_turn_flag = true;
bool once_flag_mode1 = true;
float angle_now = 0;
float yaw_fixing_angle = 0;
float fixing_yaw = 0;
uint8_t tt_yaw = 0;
uint8_t tt_top_yaw = 10;
void TaskOne_motor_yaw(void *xTask1)
{
  while (1)
  {
    // sensor_i2c_yaw.update();
    if (yaw_fixing_flag)
    {
      yaw_fixing_angle = sensor_i2c_yaw.getAngle();
      yaw_fixing_flag = false;
    }

    fixing_yaw = sensor_i2c_yaw.getAngle();
    // Serial.printf("yaw angle: %f", sensor_i2c_yaw.getAngle());
    // Serial.println();
    /********************************************************/
    // 偏航电机运动限幅
    // if ((sensor_i2c_yaw.getAngle() >= yaw_high) || (sensor_i2c_yaw.getAngle() <= yaw_low))


    /***************************************多种控制模式切换*******************************************/
    switch (mode_ctr)
    {
    case 0:                                                 // 固定模式，电机抱死
      if (motor_yaw.controller != MotionControlType::angle) // 可在中途切换模式
        motor_yaw.controller = MotionControlType::angle;    // torque,velocity,angle
      target_angle_yaw = yaw_fixing_angle;
      // Serial.println("fixing mode");
      break;
    case 1: // 手动模式，程序控制
      if (once_flag_mode1)
      {
        target_angle_yaw = yaw_fixing_angle;
        once_flag_mode1 = false;
      }
      if (motor_yaw.controller != MotionControlType::angle) // 可在中途切换模式
        motor_yaw.controller = MotionControlType::angle;    // torque,velocity,angle
      if (cam_width > 0)
        target_angle_yaw = target_angle_yaw + 0.001;
      else if (cam_width < 0)
        target_angle_yaw = target_angle_yaw - 0.001;

      // Serial.println(target_angle_yaw);
      break;
    case 2: // 自动模式，根据程序识别结果进行转动
      // Serial.println("auto mode");
      if (cam_width == 0)
      {
        if (motor_yaw.controller != MotionControlType::angle) // 可在中途切换模式
          motor_yaw.controller = MotionControlType::angle;    // torque,velocity,angle
        target_angle_yaw = yaw_fixing_angle;
      }
      else
      {
        // target_angle_yaw = PID_Cal(cam_width, 0, PID_yaw);  //pid计算
        yaw_fixing_angle = sensor_i2c_yaw.getAngle(); // 为了让云台在丢失目标后停止在原处
        int camera_offset = 40;
        float speed_yaw = 6;
        if (cam_width > camera_offset)
        {
          // if (motor_yaw.controller != MotionControlType::velocity)
          //   motor_yaw.controller = MotionControlType::velocity; // torque,velocity,angle
          // target_angle_yaw = speed_yaw;   //速度模式
          if (motor_yaw.controller != MotionControlType::angle) // 可在中途切换模式
            motor_yaw.controller = MotionControlType::angle;    // torque,velocity,angle
          target_angle_yaw += 0.002; //角度模式
          yaw_turn_flag = true;
        }
        else if (cam_width < -camera_offset)
        {
          // if (motor_yaw.controller != MotionControlType::velocity)
          //   motor_yaw.controller = MotionControlType::velocity; // torque,velocity,angle
          // target_angle_yaw = -speed_yaw;
          if (motor_yaw.controller != MotionControlType::angle) // 可在中途切换模式
            motor_yaw.controller = MotionControlType::angle;    // torque,velocity,angle
          target_angle_yaw -= 0.002; //角度模式
          yaw_turn_flag = true;
        }
        else
        {
          // if (yaw_turn_flag)
          // {
          //   angle_now = sensor_i2c_yaw.getAngle();
          //   yaw_turn_flag = false;
          // }
          // if (motor_yaw.controller != MotionControlType::angle) // 可在中途切换模式
          //   motor_yaw.controller = MotionControlType::angle;    // torque,velocity,angle
          // target_angle_yaw = angle_now;
          uint8_t nothing = 0;
        }

        // Serial.println("333");

        // Serial.printf("yaw mode:%d", motor_yaw.controller);
        // Serial.println();
      }
      break;
    }
    /*************************************************************************************************/

    /***************************************单纯跟踪程序***********************************************/
    // if (0)
    // {
    //   target_angle_yaw = 0;
    //   // Serial.println("yaw cross border!");
    // }
    // else
    // {
    //   if (cam_width == 0)
    //   {

    //     // target_angle_yaw = -PID_Cal(fixing_yaw, yaw_fixing_angle, PID_yaw_fix);  //速度模式保持初始化角度
    //     if (motor_yaw.controller != MotionControlType::angle) // 可在中途切换模式
    //       motor_yaw.controller = MotionControlType::angle;    // torque,velocity,angle
    //     target_angle_yaw = yaw_fixing_angle;
    //   }
    //   else
    //   {
    //     // target_angle_yaw = PID_Cal(cam_width, 0, PID_yaw);  //pid计算
    //     yaw_fixing_angle = sensor_i2c_yaw.getAngle(); // 为了让云台在丢失目标后停止在原处
    //     int camera_offset = 100;
    //     float speed_yaw = 6;
    //     if (cam_width > camera_offset)
    //     {
    //       if (motor_yaw.controller != MotionControlType::velocity)
    //         motor_yaw.controller = MotionControlType::velocity; // torque,velocity,angle
    //       target_angle_yaw = speed_yaw;
    //       yaw_turn_flag = true;
    //     }
    //     else if (cam_width < -camera_offset)
    //     {
    //       if (motor_yaw.controller != MotionControlType::velocity)
    //         motor_yaw.controller = MotionControlType::velocity; // torque,velocity,angle
    //       target_angle_yaw = -speed_yaw;
    //       yaw_turn_flag = true;
    //     }
    //     else
    //     {
    //       if (yaw_turn_flag)
    //       {
    //         angle_now = sensor_i2c_yaw.getAngle();
    //         yaw_turn_flag = false;
    //       }
    //       if (motor_yaw.controller != MotionControlType::angle) // 可在中途切换模式
    //         motor_yaw.controller = MotionControlType::angle;    // torque,velocity,angle
    //       target_angle_yaw = angle_now;
    //     }

    //     // Serial.println("333");

    //     // Serial.printf("yaw mode:%d", motor_yaw.controller);
    //     // Serial.println();
    //   }
    // }
    /***********************************************************************************/
    motor_yaw.loopFOC();

    // Serial.printf("yaw velocity:%f", target_angle_yaw);
    // Serial.println();
    // switch (motor_yaw.controller)
    // {
    // case 0:
    //   Serial.println("torque mode");
    //   break;
    // case 1:
    //   Serial.println("velocity mode");
    //   break;
    // case 2:
    //   Serial.println("angle mode");
    //   break;
    // }

    /*底层计算方向正负有误，输入时添加符号，和底层对齐！！！！！！！！！！！！！！！！！！！！*/
    motor_yaw.move(-target_angle_yaw);
    // Serial.printf("yaw velocity:%f", target_angle_yaw);
    // Serial.println();
    /********************************************************/

    // motor_yaw.move(target_angle_yaw);
    //   // motor_yaw.monitor();
    // command.run();
  }
}
bool pitch_fixing_flag = true;
bool pitch_turn_flag = true;
bool once_flag_mode2 = true;
float pitch_fixing_angle = 0;
float angle_now_pitch = 0;
uint8_t tt_pitch = 0;
uint8_t tt_top_pitch = 10;
void TaskTwo_motor_pitch(void *xTask2)
{
  while (1)
  {
    // read_usart();
    // sensor_i2c_pitch.update();

    if (pitch_fixing_flag)
    {
      pitch_fixing_angle = sensor_i2c_pitch.getAngle();
      pitch_fixing_flag = false;
    }
    // float fixing_pitch = sensor_i2c_pitch.getAngle();
    // Serial.printf("pitch angle: %f", sensor_i2c_pitch.getAngle());
    // Serial.println();
    // delay(10);
    // unsigned long t1 = millis();
    motor_pitch.loopFOC();
    // Serial.println(millis() - t1);
    // delay(200);

    /********************************************************/
    // 俯仰电机运动限幅
    // if ((sensor_i2c_pitch.getAngle() >= pitch_high) || (sensor_i2c_pitch.getAngle() <= pitch_low))
    /***************************************多种控制模式切换*******************************************/
    switch (mode_ctr)
    {
    case 0:                                                 // 固定模式，电机抱死
      // if (motor_pitch.controller != MotionControlType::angle) // 可在中途切换模式
      //   motor_pitch.controller = MotionControlType::angle;    // torque,velocity,angle
      target_angle_pitch = pitch_fixing_angle;
      // Serial.println("fixing mode");
      break;
    case 1: // 手动模式，程序控制
      if (once_flag_mode2)
      {
        target_angle_pitch = pitch_fixing_angle;
        once_flag_mode2 = false;
      }
      // if (motor_pitch.controller != MotionControlType::angle) // 可在中途切换模式
      //   motor_pitch.controller = MotionControlType::angle;    // torque,velocity,angle
      if (cam_height > 0)
        target_angle_pitch += 0.001;
      else if (cam_height < 0)
        target_angle_pitch -= 0.001;

      Serial.println(target_angle_pitch);
      break;
    case 2: // 自动模式，根据程序识别结果进行转动
      // Serial.println("auto mode");
      if (cam_height == 0)
      {
        // if (motor_pitch.controller != MotionControlType::angle) // 可在中途切换模式
        //   motor_pitch.controller = MotionControlType::angle;    // torque,velocity,angle
        target_angle_pitch = pitch_fixing_angle;
      }
      else
      {
        // target_angle_yaw = PID_Cal(cam_width, 0, PID_yaw);  //pid计算
        pitch_fixing_angle = sensor_i2c_pitch.getAngle(); // 为了让云台在丢失目标后停止在原处
        int camera_offset = 40;
        // float speed_pitch = 6;
        if (cam_height > camera_offset)
        {
          // if (motor_yaw.controller != MotionControlType::velocity)
          //   motor_yaw.controller = MotionControlType::velocity; // torque,velocity,angle
          // target_angle_yaw = speed_yaw;   //速度模式
          // if (motor_pitch.controller != MotionControlType::angle) // 可在中途切换模式
          //   motor_pitch.controller = MotionControlType::angle;    // torque,velocity,angle
          target_angle_pitch -= 0.002; //角度模式
          pitch_turn_flag = true;
        }
        else if (cam_height < -camera_offset)
        {
          // if (motor_yaw.controller != MotionControlType::velocity)
          //   motor_yaw.controller = MotionControlType::velocity; // torque,velocity,angle
          // target_angle_yaw = -speed_yaw;
          // if (motor_pitch.controller != MotionControlType::angle) // 可在中途切换模式
          //   motor_pitch.controller = MotionControlType::angle;    // torque,velocity,angle
          target_angle_pitch += 0.002; //角度模式
          pitch_turn_flag = true;
        }
        else
        {
          // if (pitch_turn_flag)
          // {
          //   angle_now_pitch = sensor_i2c_pitch.getAngle();
          //   pitch_turn_flag = false;
          // }
          // // if (motor_pitch.controller != MotionControlType::angle) // 可在中途切换模式
          // //   motor_pitch.controller = MotionControlType::angle;    // torque,velocity,angle
          // target_angle_pitch = angle_now_pitch;

          // target_angle_pitch = target_angle_pitch;
          uint8_t nothing = 0;
        }

        // Serial.println("333");

        // Serial.printf("yaw mode:%d", motor_yaw.controller);
        // Serial.println();
      }
      break;
    }
    /*************************************************************************************************/


    /***************************************单纯跟踪程序***********************************************/
    // if (0)
    // {
    //   target_angle_pitch = 0;
    //   // Serial.println("pitch cross border!");
    // }
    // else
    // {
    //   if (cam_height == 0)
    //   {
    //     // target_angle_pitch = PID_Cal(fixing_pitch, fixing_angle, PID_pitch_fix);
    //     if (motor_pitch.controller != MotionControlType::angle) // 可在中途切换模式
    //       motor_pitch.controller = MotionControlType::angle;    // torque,velocity,angle
    //     target_angle_pitch = pitch_fixing_angle;
    //   }
    //   // else if ((cam_height <= 15) && (cam_height >= -15))
    //   // {
    //   //   target_angle_pitch = 0;
    //   //   // Serial.println("pitch track ok!");
    //   // }
    //   else
    //   {
    //     // target_angle_pitch = PID_Cal(cam_height, 0, PID_pitch);
    //     // pitch_fixing_angle = sensor_i2c_pitch.getAngle();
    //     pitch_fixing_angle = sensor_i2c_pitch.getAngle(); // 为了让云台在丢失目标后停止在原处
    //     int camera_offset = 100;
    //     float speed_pitch = 2.5;
    //     if (cam_height > camera_offset)
    //     {
    //       if (motor_pitch.controller != MotionControlType::velocity)
    //         motor_pitch.controller = MotionControlType::velocity; // torque,velocity,angle
    //       target_angle_pitch = -speed_pitch;
    //       pitch_turn_flag = true;
    //     }
    //     else if (cam_height < -camera_offset)
    //     {
    //       if (motor_pitch.controller != MotionControlType::velocity)
    //         motor_pitch.controller = MotionControlType::velocity; // torque,velocity,angle
    //       target_angle_pitch = speed_pitch;
    //       pitch_turn_flag = true;
    //     }
    //     else
    //     {
    //       if (pitch_turn_flag)
    //       {
    //         angle_now_pitch = sensor_i2c_pitch.getAngle();
    //         pitch_turn_flag = false;
    //       }
    //       if (motor_pitch.controller != MotionControlType::angle) // 可在中途切换模式
    //         motor_pitch.controller = MotionControlType::angle;    // torque,velocity,angle
    //       target_angle_pitch = angle_now_pitch;
    //     }
    //   }
    // }
    // if(tt_pitch >= tt_top_pitch){
    //   motor_pitch.move(target_angle_pitch);
    //   tt_pitch = 0;
    // }
    // else tt_pitch++;
    // motor_pitch.move(target_angle_pitch);
    // Serial.printf("tracking!  pitch velocity is:%f", target_angle_pitch);
    // Serial.println();
    /********************************************************/
    /*************************************************************************************************/
    motor_pitch.move(target_angle_pitch);
    //    motor_pitch.monitor();
    // command.run();
  }
}

/*新串口通信测试*/
#define EX_UART_NUM UART_NUM_1
#define PATTERN_CHR_NUM (3) /*!< Set the number of consecutive and identical characters received by receiver which defines a UART pattern*/

#define BUF_SIZE (1024)
#define RD_BUF_SIZE (BUF_SIZE)
static QueueHandle_t uart0_queue;
static const char *TAG = "uart_events";
void Taskthree_uart(void *xTask3)
{
  uart_event_t event;
  size_t buffered_size;
  uint8_t *dtmp = (uint8_t *)malloc(RD_BUF_SIZE);
  while (1)
  {
    if (xQueueReceive(uart0_queue, (void *)&event, (TickType_t)portMAX_DELAY))
    {
      bzero(dtmp, RD_BUF_SIZE);
      ESP_LOGI(TAG, "uart[%d] event:", EX_UART_NUM);
      switch (event.type)
      {
      // Event of UART receving data
      /*We'd better handler data event fast, there would be much more data events than
      other types of events. If we take too much time on data event, the queue might
      be full.*/
      case UART_DATA:
        // Serial.println();
        ESP_LOGI(TAG, "[UART DATA]: %d", event.size);
        uart_read_bytes(EX_UART_NUM, dtmp, event.size, portMAX_DELAY);

        /********aa,height_high,height_low,width_high,width_low,bb********/
        // if ((dtmp[0] == 0xaa) && (dtmp[5] == 0xbb))
        // {
        //   int cam_height1 = (dtmp[1] << 8 | dtmp[2]) - 240;
        //   int cam_width1 = (dtmp[3] << 8 | dtmp[4]) - 320;
        //   if (cam_height1 == (-240) || cam_width1 == (-320))
        //   {
        //     cam_height = 0;
        //     cam_width = 0;
        //   }
        //   else
        //   {
        //     cam_height = cam_height1;
        //     cam_width = cam_width1;
        //   }
        //   // Serial.printf("height:%d , width:%d", cam_height, cam_width);
        //   // Serial.println();
        // }
        /****************************************************************/

        /********aa,height_high,height_low,width_high,width_low,mode_switch,bb********/
        if ((dtmp[0] == 0xaa) && (dtmp[6] == 0xbb))
        {
          int cam_height1 = (dtmp[1] << 8 | dtmp[2]) - 240;
          int cam_width1 = (dtmp[3] << 8 | dtmp[4]) - 320;
          mode_ctr = dtmp[5];
          if (cam_height1 == (-240))
            cam_height = 0;
          else
            cam_height = cam_height1;
          if (cam_width1 == (-320))
            cam_width = 0;
          else
            cam_width = cam_width1;

          // Serial.printf("height:%d , width:%d", cam_height, cam_width);
          // Serial.printf("mode_ctr: %d",mode_ctr);
          // Serial.println();
        }
        /****************************************************************/
        ESP_LOGI(TAG, "[DATA EVT]:");
        break;
      // Event of HW FIFO overflow detected
      case UART_FIFO_OVF:
        ESP_LOGI(TAG, "hw fifo overflow");
        // If fifo overflow happened, you should consider adding flow control for your application.
        // The ISR has already reset the rx FIFO,
        // As an example, we directly flush the rx buffer here in order to read more data.
        uart_flush_input(EX_UART_NUM);
        xQueueReset(uart0_queue);
        break;
      // Event of UART ring buffer full
      case UART_BUFFER_FULL:
        ESP_LOGI(TAG, "ring buffer full");
        // If buffer full happened, you should consider increasing your buffer size
        // As an example, we directly flush the rx buffer here in order to read more data.
        uart_flush_input(EX_UART_NUM);
        xQueueReset(uart0_queue);
        break;
      }
    }
  }
  free(dtmp);
  dtmp = NULL;
  vTaskDelete(NULL);
}

void setup()
{
  // 串口通信初始化
  // MySerial.begin(115200, SERIAL_8N1, 18, 5); // rxpin  txpin

  /****************************新串口通信测试****************************/
  esp_log_level_set(TAG, ESP_LOG_INFO);

  /* Configure parameters of an UART driver,
   * communication pins and install the driver */
  uart_config_t uart_config = {
      .baud_rate = 115200,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
  };
  // Install UART driver, and get the queue.
  uart_driver_install(EX_UART_NUM, BUF_SIZE * 2, BUF_SIZE * 2, 20, &uart0_queue, 0);
  uart_param_config(EX_UART_NUM, &uart_config);

  // Set UART log level
  esp_log_level_set(TAG, ESP_LOG_INFO);
  // Set UART pins (using UART0 default pins ie no changes.)
  uart_set_pin(EX_UART_NUM, 5, 18, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

  // Set uart pattern detect function.
  uart_enable_pattern_det_baud_intr(EX_UART_NUM, '+', PATTERN_CHR_NUM, 9, 0, 0);
  // Reset the pattern queue length to record at most 20 pattern positions.
  uart_pattern_queue_reset(EX_UART_NUM, 20);
  /*********************************************************************/

  // clear_usart_buffer();
  //  PID结构体初始化
  PID_pitch.kp = 0.03;  // 0.0035
  PID_pitch.ki = 0.003; // 0.004
  PID_pitch.kd = 0;
  PID_pitch.error_sum = 0;
  PID_pitch.last_error = 0;

  PID_yaw.kp = 0.037; //
  PID_yaw.ki = 0.006;
  PID_yaw.kd = 0;
  PID_yaw.error_sum = 0;
  PID_yaw.last_error = 0;

  // 云台抱死pid
  PID_pitch_fix.kp = 30;
  PID_pitch_fix.ki = 4;
  PID_pitch_fix.kd = 0;
  PID_pitch_fix.error_sum = 0;
  PID_pitch_fix.last_error = 0;

  PID_yaw_fix.kp = 14; //
  PID_yaw_fix.ki = 0;
  PID_yaw_fix.kd = 0;
  PID_yaw_fix.error_sum = 0;
  PID_yaw_fix.last_error = 0;

  // // i2c编码器链接驱动器  yaw
  I2Cone.begin(4, 21, 400000); // 4 21
  sensor_i2c_yaw.init(&I2Cone);
  motor_yaw.linkSensor(&sensor_i2c_yaw);
  // pitch
  I2Ctwo.begin(23, 15, 400000); // 2 15
  sensor_i2c_pitch.init(&I2Ctwo);
  motor_pitch.linkSensor(&sensor_i2c_pitch);

  // driver config
  // power supply voltage [V]
  driver_yaw.voltage_power_supply = 8;
  driver_yaw.init();
  driver_pitch.voltage_power_supply = 8;
  driver_pitch.init();
  // link the motor and the driver
  motor_yaw.linkDriver(&driver_yaw);
  motor_pitch.linkDriver(&driver_pitch);

  // choose FOC modulation (optional)
  motor_yaw.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor_pitch.foc_modulation = FOCModulationType::SpaceVectorPWM;

  // set motion control loop to be used
  motor_yaw.controller = MotionControlType::angle; // torque,velocity,angle
  motor_pitch.controller = MotionControlType::angle;
  motor_yaw.torque_controller = TorqueControlType::voltage;
  motor_pitch.torque_controller = TorqueControlType::voltage;

  /************yaw motor pid******************/
  // velocity loop PID
  motor_yaw.PID_velocity.P = 0.3;
  motor_yaw.PID_velocity.I = 0.05;
  motor_yaw.PID_velocity.D = 0.0;
  motor_yaw.voltage_sensor_align = 2.5;
  // motor_yaw.phase_resistance=12.5;
  // Low pass filtering time constant
  motor_yaw.LPF_velocity.Tf = 0.05;
  // // angle loop PID
  motor_yaw.P_angle.P = 11;
  motor_yaw.P_angle.I = 1.5;
  motor_yaw.P_angle.D = 1;
  // motor_yaw.P_angle.output_ramp = 0.0;
  // motor_yaw.P_angle.limit = 20.0;
  // // Low pass filtering time constant
  // motor_yaw.LPF_angle.Tf = 0;

  // Limits
  // motor_yaw.velocity_limit = 40.0;
  motor_yaw.voltage_limit = 12.0;
  // motor_yaw.current_limit = 2.0;
  /*********************************************/

  /************pitch motor pid******************/
  // velocity loop PID
  motor_pitch.PID_velocity.P = 0.35;
  motor_pitch.PID_velocity.I = 0.05;
  motor_pitch.PID_velocity.D = 0;
  motor_pitch.voltage_sensor_align = 2.5;
  // Low pass filtering time constant
  motor_pitch.LPF_velocity.Tf = 0.15;
  // // angle loop PID
  motor_pitch.P_angle.P = 11;
  motor_pitch.P_angle.I = 1.5;
  motor_pitch.P_angle.D = 0;
  // motor_pitch.P_angle.output_ramp = 0.0;
  // motor_pitch.P_angle.limit = 20.0;
  // // Low pass filtering time constant
  // motor_pitch.LPF_angle.Tf = 0.1;

  // Limits
  // motor_pitch.velocity_limit = 40.0;
  motor_pitch.voltage_limit = 12.0;
  // motor_pitch.current_limit = 2.0;
  /*********************************************/

  // use monitoring with serial
  Serial.begin(115200);
  // comment out if not needed
  // motor_yaw.useMonitoring(Serial);
  // motor_pitch.useMonitoring(Serial);

  // initialize motor_yaw
  motor_yaw.init();
  motor_pitch.init();
  // align sensor and start FOC
  motor_yaw.initFOC();
  motor_pitch.initFOC();

  // add target command T
  command.add('Y', doTarget_yaw, (char *)"target angle");
  command.add('P', doTarget_pitch, (char *)"target angle");
  command.add('M', doMotor_yaw, (char *)"my motor");
  command.add('N', doMotor_pitch, (char *)"my motor");

  Serial.println(F("All motor ready!."));
  Serial.println(F("Set the target angle using serial terminal:"));

  pinMode(22, OUTPUT);
  digitalWrite(22, HIGH);

  xTaskCreatePinnedToCore(TaskOne_motor_yaw, "TaskOne", 10000, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(TaskTwo_motor_pitch, "TaskTwo", 10000, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(Taskthree_uart, "TaskThree", 10000, NULL, 3, NULL, 0);
  //_delay(1000);
}

// angle set point variable
// float target_angle = 0;

void loop()
{
}
/*****************************两电机速度模式测试用例  zdx_try********************************/
