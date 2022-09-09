# Multimeter (LCD version)

# Información general

El proyecto se basa en implementar un medidor que muestree señales analogicas en tiempo real para convertirlas en una señal digital, y posteriormente ser expuestas en una pantalla LCD, usando el microcontrolador ***STM32F103C8.*** 

## Implementación

La obtencion del voltaje analogo se realiza a través de un periferico ***ADC*** (parametros standar), el que adquiere las señales del potenciometro de 10[kΩ], guardanando los datos en una variable de informacion. Para poder leer la data, es necesaria ser convertida y guardada nuevamente. Se corrobora haciendo debugging en el código y, posteriormente, se lee el dato convertido en el breakpoint puesto de forma arbitraria. 

Se muestra en la tabala a continuacion el procedimiento descrito.

| Codigo en C, conversion y adquisicion de voltaje |
| --- |
| int main(void)
{
/* USER CODE BEGIN 1 /
uint16_t raw_data;
/ USER CODE END 1 /
/ USER CODE BEGIN 2 /
HAL_ADC_Start(&hadc1); //START ADC
/ USER CODE END 2 /
/ USER CODE BEGIN WHILE */
while (1)
{
//ADC
HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY); //CONVIERTE LA DATA LEIDA
raw_data = HAL_ADC_GetValue(&hadc1);//se lee el valor convertido
//END ADC |

Para poder transformar en voltaje la señal convertida en el ***ADC*** y obtener ambos voltajes (filtrado y no filtrado), se declaran variables tanto ***int*** como ***float***, describiendose valores constantes y la data. Esto para obtener la transformacion del voltaje analogo a digital, realizado con la ecuación descrita en el código de la tabla 2.

| Codigo transformacion voltaje analogo-digital no filtrado |
| --- |
| /* USER CODE BEGIN 2 /
float a;
float V1;
a=0.000805; //Constante para transformar el voltaje obtenido en el ADC
/ USER CODE END 2 /
while (1)
{
//Voltaje obtenido no filtrado
V1= raw_data*a; |

Se filtra la señal de V1, con un **M.A.F** (Mean average filter ó promedio filtrado), el que nos dará un voltaje promediado eliminando el ruido producido de la señal o señales que no son necesarias en nuestra lectura. El filtro se implementa con el promedio de los retardos de la señal V1, con un maximo de cuatro retardos. Una vez terminado, se obtiene el voltaje filtrado. Esto al igual que en la medición anterior, se establecen funciones ***int*** y ***float*** para su desarrollo, indicando las ganancias, la data de V1 y el número de retardos.

| MAF_Voltaje filtrado |
| --- |
| /* USER CODE BEGIN 2 /
float V1;
int A2=0,A3=0,A4=0;
int N=4;
float A1;
float V2;
/ USER CODE END 2 */
while (1)
{
//MEAN AVERAGE FILTER
A4=A3;
A3=A2;
A2=A1;
A1=V1;
V2 = (A1+A2+A3+A4)/N;
//End filter |

El muestreo de los voltajes filtrados y no filtrados expresados en la pantalla **LCD**, se ejecuta a partir del periferico ***I2C*** (con sus parametros en standar), haciendo uso de la libreria i2c-lcd.c, para programar en la pantalla. En este se genera el codigo para el mensaje "VOLTAJE [V]”, estableciendo su posicion. A continuación se establecen los codigos que convertiran los voltajes adquiridos guardados en nuevas variables de información para cada voltaje (que cambiaran cada 1 segundo dependiendo de la manipulación del potenciometro), en una señal de palabra que será descrita en la pantalla LCD, con su posición correspondiente.

La metodología anterior, se describe en el siguiente código:

| Pantalla LCD |
| --- |
| int main(void)
{
/* USER CODE BEGIN 1 /
uint16_t voltajepalabra[4];
uint16_t voltajepalabra2[4];
/ USER CODE END 1 /
while (1)
{
//LCD
lcd_clear(); //THIS FUNCTION CLEAR THE LCD AND
HAL_Delay(1000);
lcd_init ();	//INITIALIZE THE LCD (if the LCD is cleared, you need to initialize it again **)
lcd_put_cur(0,0); //posicion
HAL_Delay(10);
lcd_send_string ("VOLTAJE [V]");
//Voltaje no filtrado emitido en el LCD
gcvt(V1, 4, voltajepalabra); //convierte el voltaje obtenido en palabra para ser leida en el LCD
lcd_put_cur(1,2); //posicion
lcd_send_string (voltajepalabra);//palabra a emitir
//Voltaje filtrado emitido en el LCD
gcvt(V2, 4, voltajepalabra2);
lcd_put_cur(1,8);
lcd_send_string (voltajepalabra2);
HAL_Delay(1000);
/ USER CODE END WHILE */ |

### Resultados

Los voltajes muestreados en tiempo real se escrbiran en la pantalla LCD, como evidencia la siguiente figura:

![Resultado multimetro.jpeg](Multimeter%20(LCD%20version)%207c622f460e8e47da8e7365d5873ed11c/Resultado_multimetro.jpeg)

 

Donde la señal de la izquierda es el voltaje sin filtrar y el de la derecha es el voltaje filtrado.

### Autores

Daniel Parra.

Joaquín Torres.