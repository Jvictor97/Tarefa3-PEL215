// Inclusão das bibliotecas utilizadas no código
#include <webots/robot.h>
#include <webots/distance_sensor.h>
#include <webots/motor.h>
#include <math.h>
#include <stdio.h>
#include <webots/supervisor.h>

#define TIME_STEP 64

// Matriz com os valores de logOdds para o grid
double l[8][16];

// Distância máxima medida pelo sensor
double zmax = 5;
// Variáveis para o Mapeamento
double l0 = 0.0, locc = 0.4, lfree = -0.4;

// Definicao da funcao delay
void delay (int time_milisec) {
  double currentTime, initTime, Timeleft;
  double timeValue = (double)time_milisec/1000;
  initTime = wb_robot_get_time();
  Timeleft = 0.00;
  while (Timeleft < timeValue)
  {
    currentTime = wb_robot_get_time();
    Timeleft = currentTime - initTime;
    wb_robot_step(TIME_STEP);
  }
}

// Função inverseSensorModel, como definido em:
// Sebastian THRUN; Wolfram BURGARD; Dieter FOX - PROBABILISTIC ROBOTICS (Tabela 9.2)
double inverseSensorModel(double x, double y, double theta, double xi, double yi, double sensorData[]) {
  
  // Dados dos sensores e configuracao
  double zk, thetak, sensorTheta;
  double minDelta = -1;
  double alpha = 1, beta = 20;
  
  double r = sqrt(pow(xi - x, 2) + pow(yi - y, 2));
  double phi = atan2(yi - y, xi - x) - theta;

  // Ângulos dos sensores
  const double sensorAngles[] = {-90, -50, -30, -10, 10, 30, 50, 90};

  for (int sensorIndex = 0; sensorIndex < 8; sensorIndex++) {
    sensorTheta = sensorAngles[sensorIndex] * M_PI / 180;    
    
    //printf("sensor: %d, sensorTheta: %f, phi: %f, theta: %f, z: %f\n", sensorIndex, sensorTheta, phi, theta, sensorData[sensorIndex]); 
    if (fabs(phi - sensorTheta) < minDelta || minDelta == -1) {
      zk = sensorData[sensorIndex];
      thetak = sensorTheta;
      minDelta = fabs(phi - sensorTheta);
    }
    
  }

  if (r > fmin((double) zmax, zk + alpha / 2) || fabs(phi - thetak) > beta / 2 || zk > zmax)
    return l0; 

  if (zk < zmax && fabs(r - zk) < alpha / 2) 
    return locc;  
  
  if (r <= zk)
    return lfree;
    
  return 0.0;
}

// Função principal do GridMap (nessa função a coordenada 
// Z foi chamada de Y para não confundir com a medida do sensor
void occupancyGridMapping(double x, double y, double theta, double sensorData[]) {
    for (int row = 0; row < 8; row++) {
        for (int column = 0; column < 16; column++) {
            
            double xi = -row + 3.5;
            double yi = column - 7.5;
            
            // Uma célula será considerada como "dentro do campo de visão" caso a distância
            // entre o centro de massa do robô e o da célula seja menor do que zmax e esteja
            // no campo de visao do robo
            double robotCellAngle = atan2(yi - y, xi - x) * 180 / M_PI;
            
            if (sqrt(pow(xi - x, 2) + pow(yi - y, 2)) <= zmax && robotCellAngle >= -90 && robotCellAngle <= 90) {
                //printf("\nrow: %d, column: %d\n", row, column);
                l[row][column] = l[row][column] + inverseSensorModel(x, y, theta, xi, yi, sensorData) - l0;
            }
        }
    }
}

// Função para printar a matriz L
void printMatrix(bool printValues){
  for (int row = 0; row < 8; row++) {
    for (int column = 0; column < 16; column++) {  
       if (printValues)
         printf("%.2f ", l[row][column]);  
       else
         printf("%.2f ", 1 - (1 / (1 + exp(l[row][column]))));  
   }
    printf("\n");
 }
 
  printf("\n\n");
}

// Funcao principal
int main(int argc, char **argv) {
  // Inicialização do webots
  wb_robot_init();
  
  for(int row = 0; row < 8; row++) {
    for(int column = 0; column < 16; column++) {
      l[row][column] = 0.0;
    }
  }
  
  
  // Criando tags para os motores do robo
  WbDeviceTag frontLeftMotor = wb_robot_get_device("front left wheel");   // motor dianteiro esquerdo
  WbDeviceTag frontRightMotor = wb_robot_get_device("front right wheel"); // motor dianteiro direito
  WbDeviceTag backLeftMotor = wb_robot_get_device("back left wheel");    // motor traseiro esquerdo
  WbDeviceTag backRightMotor = wb_robot_get_device("back right wheel");  // motor traseiro direito
  
  // Criando tags para os sensores do robo
  WbDeviceTag so7 = wb_robot_get_device("so7");
  WbDeviceTag so6 = wb_robot_get_device("so6");
  WbDeviceTag so5 = wb_robot_get_device("so5");
  WbDeviceTag so0 = wb_robot_get_device("so0");
  WbDeviceTag so1 = wb_robot_get_device("so1");
  WbDeviceTag so2 = wb_robot_get_device("so2");
  WbDeviceTag so3 = wb_robot_get_device("so3");
  WbDeviceTag so4 = wb_robot_get_device("so4");
  
  // Habilitando os sensores
  wb_distance_sensor_enable(so7, TIME_STEP);
  wb_distance_sensor_enable(so6, TIME_STEP);
  wb_distance_sensor_enable(so5, TIME_STEP);
  wb_distance_sensor_enable(so0, TIME_STEP);
  wb_distance_sensor_enable(so1, TIME_STEP); 
  wb_distance_sensor_enable(so3, TIME_STEP);
  wb_distance_sensor_enable(so4, TIME_STEP);
  wb_distance_sensor_enable(so2, TIME_STEP);
  
  // Configurando os motores
  wb_motor_set_position(frontLeftMotor, INFINITY);
  wb_motor_set_position(frontRightMotor, INFINITY);
  wb_motor_set_position(backLeftMotor, INFINITY);
  wb_motor_set_position(backRightMotor, INFINITY);
  
  // Variaveis para os valores dos sensores
  double so7Value, so6Value, so5Value, so0Value, 
         so1Value, so2Value, so3Value, so4Value;
  
  // Variaveis para as distancias leterais do robo
  double currentRightDistance, currentLeftDistance;
  
  // Distancia minima para as paredes do lado direito
  double minimumDistance = 200.0;
  
  // Variaveis para os erros no controle PID 
  double error, integral = 0.0, errorDifference, oldError = 0.0;
  
  // Constantes definidas para o PID
  double kp = 0.3;
  double kd = 0.0002;
  double ki = 0.000001;

  // Variaveis para o maior valor dos sensores 
  // do lado esquerdo e direito       
  double maxRightSensorValue, maxLeftSensorValue;
  
  // Potencia que sera somada ao motor direito
  double motorPower;
  
  // Inicializacao das velocidades dos motores do robo
  double rightSpeed = 3.0;
  double leftSpeed = 3.0;

  
  // Código do Supervisor
  WbNodeRef robot_node = wb_supervisor_node_get_from_def("MY_ROBOT");
  WbFieldRef rotation = wb_supervisor_node_get_field(robot_node, "rotation");

  while (wb_robot_step(TIME_STEP) != -1) {
 
    // Leitura do valor dos sensores
    so0Value = wb_distance_sensor_get_value(so0); 
    so1Value = wb_distance_sensor_get_value(so1); 
    so2Value = wb_distance_sensor_get_value(so2); 
    so3Value = wb_distance_sensor_get_value(so3); 
    so4Value = wb_distance_sensor_get_value(so4); 
    so5Value = wb_distance_sensor_get_value(so5); 
    so6Value = wb_distance_sensor_get_value(so6);
    so7Value = wb_distance_sensor_get_value(so7);
    
    
    // Condição para realizar um giro de 90 graus
    if (1024 - so3Value <= minimumDistance || 
        1024 - so4Value <= minimumDistance) {
      // Se houver uma parede a uma distancia de 
      // 200 ou menos do sensor so3, adiciona 
      // velocidade negativa ao lado esquerdo
      // e positiva do lado direito
      
      wb_motor_set_velocity(frontLeftMotor, -3.0);
      wb_motor_set_velocity(backLeftMotor, -3.0);
      wb_motor_set_velocity(frontRightMotor, 3.0);
      wb_motor_set_velocity(backRightMotor, 3.0);
      
      // O robo permanece girando por 1200ms 
      delay(1200);
    }
    else {
      // Condicao para definir o maior valor dos sensores do lado direito
      maxRightSensorValue = so5Value > so6Value ? 
                            (so5Value > so7Value ? so5Value : so7Value) : 
                            (so6Value > so7Value ? so6Value : so7Value);
                            
      // Condicao para definir o maior valor dos sensores do lado esquerdo                     
      maxLeftSensorValue = so0Value > so1Value ?
                           (so0Value > so2Value ? so0Value : so2Value) :
                           (so1Value > so2Value ? so1Value : so2Value);                         
      
      // Aplicando a correcao aos valores dos sensores                      
      currentRightDistance = 1024 - maxRightSensorValue;
      currentLeftDistance = 1024 - maxLeftSensorValue;
      
      // Se a distancia para o lado esquerdo for inferior ao limiar, 
      // utiliza a nova formula de erro para manter a mesma distancia 
      // dos os dois lados do robo
      if (currentLeftDistance < 200) error = currentLeftDistance - currentRightDistance;
      // Caso contrario, utiliza o limiar de distancia minima para o lado direito
      else error = minimumDistance - currentRightDistance;
             
      // Incrementa o valor da integral (Controlador Integral)
      integral += error;
      // Calcula a diferenca entre o erro atual e o anterior (Controlador Derivativo)
      errorDifference = error - oldError;
      // Atualiza o erro anterior
      oldError = error;
      
      // Calcula a potencia a ser adicionada ao lado direito (Controlador PID)
      motorPower = (kp * error) + (ki * integral) + (kd * errorDifference);
         
      // Calcula a nova velocidade do lado direito
      rightSpeed = 3.0 + motorPower;
      
      // Condicoes para limitar as velocidades minima e maxima
      if (rightSpeed < 1.5) rightSpeed = 1.5;
      if (rightSpeed > 5.0) rightSpeed = 5.0;
             
      // Aplica as velocidades aos motores do robo
      wb_motor_set_velocity(frontLeftMotor, leftSpeed);
      wb_motor_set_velocity(backLeftMotor, leftSpeed);
      wb_motor_set_velocity(frontRightMotor, rightSpeed);
      wb_motor_set_velocity(backRightMotor, rightSpeed);
    
    }
    
    const double *rot_values = wb_supervisor_field_get_sf_rotation(rotation);
    
    double yAxis = rot_values[1];
    double yAngle = rot_values[3];   

    // Em algumas situações o webots inverte o eixo Y de -1 (configurado)
    // para 1 (valor padrão), portanto será realizada a validação para 
    // inverter os ângulos caso necessário
    if (yAxis > 0 && abs(yAxis - 1) < 0.5) yAngle = -yAngle;
    
    double theta = -yAngle + M_PI / 2;
    //double theta = -3 * yAngle + 3 * M_PI;
    printf("theta: %f\n", theta);
    
    // Obtendo a posição do robô
    const double *position = wb_supervisor_node_get_position(robot_node);
    double x = position[0];
    double z = position[2];
    
    // Convertendo os valores dos sensores para metros
    double sensorRawValues[] = { so0Value, so1Value, so2Value, so3Value,
                                 so4Value, so5Value, so6Value, so7Value };                             
    double sensorMetricValues[8];
    
    for(int sensorIndex = 0; sensorIndex < 8; sensorIndex++) {
      const double slope = -0.004947;
      const double intercept = 5.2;
      const double sensorRawValue = sensorRawValues[sensorIndex];
     
      sensorMetricValues[sensorIndex] = slope * sensorRawValue + intercept;;     
    }
    
    occupancyGridMapping(x, z, theta, sensorMetricValues);
    printMatrix(false);

    fflush(stdout);    
  };
  
  // Limpeza do ambiente do webots
  wb_robot_cleanup();

  return 0;
}
