# Medindo o nível de um reservatório com sensores de pressão

Medidor de nível para reservatório aberto, com ESP32, um par de boias e uma sonda hidrostática.

## Utilização

Baixe o repositório ou clone para o VSCode. É necessária a extensão da ESP-IDF para compilar o projeto & seus componentes.

## Pinos & funções

| Item | Função | Pino |
| :----------: | :----------: | :----------: |
| Sonda | Holding Reg 01 | 32 |
| Boia 1 | Discrete In 01 | 18 |
| Boia 2 | Discrete In 02 | 19 |
| Push button | Reboot | 21 |
| Disp OLED | SDA | 15 |
| Disp OLED | SCL | 2 |
