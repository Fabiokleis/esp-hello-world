# esp-hello-world
hello world no esp32. Esse repositorio tem um setup basico de projeto esp32 com display ili9341 e lvgl.


![IMG_20240808_202242412_HDR](https://github.com/user-attachments/assets/88572489-118f-44aa-8c44-ed636c5e8c2c)


## setup
instale o [nixos](https://nixos.org/download/) e rode na raiz desse repositorio:
```shell
nix-shell
```
isso deve instalar o `esp-idf-full`, com suporte para todas as esp.

## run
depois de instalar a `esp-idf`, ja pode ser gravado o codigo com o comando:
```shell
idf.py -p /dev/ttyUSB0 flash
```
## monitor
para ver o terminal serial do esp:
```shell
idf.py monitor
```
