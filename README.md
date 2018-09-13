# Controlador de placa de vías

Este firmware controla las placas de vias del proyecto de automatización de maquetas de tren.

## Funcionalidad

El controlador incluye las siguientes funciones básicas, disponibles a través de I2C:

* Control de la velocidad y sentido de cada tramo de vía utilizando PWM de baja frecuencia (configurable), ideal para este escenario.

* Medida de la intensidad pico consumida en cada tramo de vía, para permitir la detección de material rodante.

Y las siguientes funciones adicionales:

* Negociación de la dirección I2C con el programa de control, via wifi, evitando que sea necesario configurar direcciones con jumpers.

* Servidor HTTP mDNS para funciones de ayuda al desarrollo.

* Actualización de firmware OTA.

## Características Hardware

El ESP32 da soporte a:

  * 8 canales PWM
  * 8 canales de sentido
  * 1 bus I2C (maestro) de comunicación interno de la placa
  * 1 bus I2C (sólo esclavo) para comunicación con el maestro raspberry pi
  * 1 entrada analógica de configuración de dirección
  * Comunicación serie a través de USB (por defecto)

Para cada uno de los 8 canales, el hardware que controla este firmware consiste en:
  * Un puente H que soporta PWM y resistencia para medición de la corriente (1/2 L298N)
  * Inversores para generar ambos sentidos a partir de la señal de dirección (1/4 74LV14)
  * Circuitería de adaptación de señal para la medida de la intensidad
  * Conversor analógico-digital (1/4 de ADS1015 ó ADS1115) para la medida de intensidad
  * LED bicolor indicador de sentido y velocidad

