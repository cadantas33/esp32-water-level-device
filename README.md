# Medindo o nível de um reservatório com sensores de pressão

Este repositório possui o código fonte para um dispositivo de medição de nível de água através de sensores de pressão e uma placa ESP32.

## Utilização

Clone/baixe o repositório e abra a pasta no VSCode. É necessária a extensão ESP-IDF instalada juntamente à IDE.

## Registradores Modbus

| Item | Função | Pino |
| :----------: | :----------: | :----------: |
| Sonda | Holding Reg 01 | 32 |
| Boia 1 | Discrete In 01 | 18 |
| Boia 2 | Discrete In 02 | 19 |
| Boia 3 | Discrete In 03 | |
