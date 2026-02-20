# Настройка систем с Raspberry Pi CM5 и Waveshare CM5-NANO-A/B

## 1. Введение
Данная документация описывает настройку Raspberry Pi Compute Module 5 (CM5) на базе платы Waveshare CM5-NANO-B с поддержкой камеры, Docker и подключением полетного контроллера.

## 2. Установка операционной системы

### 2.1. Рекомендуемые версии ОС
Для полноценной работы с камерой рекомендуется использовать:

**Raspberry Pi OS (Legacy) Lite**  
- Основа: Debian Bookworm с обновлениями безопасности
- Без графического интерфейса (Lite версия)
- Размер: 422 MB (64-bit)
- Ядро: версии 6.12
- Дата релиза: 1 Oct 2025

> **Важно:** Ubuntu 24.04 Server не поддерживает камеру Raspberry Pi в полном объеме.

**Источники:**
- [Официальные образы Raspberry Pi OS](https://www.raspberrypi.com/software/operating-systems/)
- [Документация по Waveshare CM5-NANO-B](https://www.waveshare.com/wiki/CM5-NANO-B)

### 2.2. Особенности для разных ОС
- **Raspberry Pi OS (Legacy)**: Полная поддержка всех функций, включая камеру
- **Ubuntu 24.04 Server**: Ограниченная поддержка, отсутствует поддержка камеры

## 3. Основные модули системы

### 3.1. Настройка камеры [Raspberry Pi OS]
Для корректной работы камеры необходимо внести изменения в конфигурационный файл системы.

#### Шаг 1: Редактирование конфигурационного файла
```bash
sudo nano /boot/firmware/config.txt
```

#### Шаг 2: Настройка секции [cm5]
Добавьте или отредактируйте секцию `[cm5]` следующим образом:
```bash
[cm5]
dtoverlay=dwc2,dr_mode=host
dtoverlay=ov5647,cam0
start_x=1
gpu_mem=256
```

**Пояснение параметров:**
- `dtoverlay=dwc2,dr_mode=host` - включает USB OTG режим в режиме хоста
- `dtoverlay=ov5647,cam0` - загружает драйвер для камеры OV5647 на первом камере-порту
- `start_x=1` - включает поддержку камеры и GPU
- `gpu_mem=256` - выделяет 256MB памяти для GPU (минимально необходимый объем для работы камеры)

#### Шаг 3: Перезагрузка системы
```bash
sudo reboot
```

#### Шаг 4: Проверка работы камеры
```bash
rpicam-hello
```

**Что ожидать:** Откроется окно предварительного просмотра с изображением с камеры на 5 секунд. Для бесконечного просмотра используйте: `rpicam-hello --timeout 0`

**Дополнительная информация:**
- [Официальная документация по ПО камеры](https://www.raspberrypi.com/documentation/computers/camera_software.html)
- [Спецификации камеры Waveshare](https://www.waveshare.com/rpi-camera-g.htm)
- Для камеры OV5647 (V1) требуется именно этот overlay, для других моделей камеры параметры будут отличаться

### 3.2. Установка Docker
#### Подготовка системы
```bash
# Обновление списка пакетов и установка необходимых зависимостей
sudo apt update
sudo apt install ca-certificates curl
```

#### Шаг 1: Добавление GPG ключа Docker
```bash
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/debian/gpg -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc
```

**Пояснение:**
- `install -m 0755 -d /etc/apt/keyrings` - создает директорию с правами 755
- `curl -fsSL` - загружает файл без вывода прогресса, проверяя SSL сертификат
- `chmod a+r` - делает ключ доступным для чтения всеми пользователями

#### Шаг 2: Добавление репозитория Docker
```bash
sudo tee /etc/apt/sources.list.d/docker.sources <<EOF
Types: deb
URIs: https://download.docker.com/linux/debian  
Suites: $(. /etc/os-release && echo "$VERSION_CODENAME")
Components: stable
Signed-By: /etc/apt/keyrings/docker.asc
EOF
```

**Пояснение параметров:**
- `tee` - записывает вывод в файл и выводит на экран
- `$(. /etc/os-release && echo "$VERSION_CODENAME")` - автоматически определяет кодовое имя дистрибутива (bookworm для Raspberry Pi OS Legacy)
- `Signed-By` - указывает путь к GPG ключу для проверки подписи пакетов

#### Шаг 3: Обновление пакетов и установка Docker
```bash
sudo apt update
sudo apt install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
```

#### Шаг 4: Проверка установки
```bash
sudo systemctl status docker
```

**Что проверить:** Статус службы должен быть `active (running)`

#### Шаг 5: Настройка прав доступа
```bash
sudo usermod -aG docker $USER
sudo reboot
```

**Пояснение:**
- `usermod -aG docker $USER` - добавляет текущего пользователя в группу docker для запуска контейнеров без sudo
- Требуется перезагрузка для применения изменений в группах

**Документация:** [Официальная инструкция по установке Docker](https://docs.docker.com/engine/install/debian/)

### 3.3. Подключение полетного контроллера (Matek H743 Mini V3)
#### Физическое подключение
Для подключения полетного контроллера к CM5-NANO-B:
- Припаяйте провода к контактам RX4/TX4 на плате Matek
- Подключите:
  - **RX4** (Matek) → **GPIO14** (CM5-NANO-B) 
  - **TX4** (Matek) → **GPIO15** (CM5-NANO-B)
  - **GND** (Matek) → **GND** (CM5-NANO-B)

#### Шаг 1: Добавление пользователя в необходимые группы
```bash
sudo usermod -aG dialout $USER
sudo usermod -aG tty $USER  
sudo usermod -aG i2c $USER
sudo usermod -aG video $USER
sudo usermod -aG gpio $USER
```

**Пояснение групп:**
- `dialout` и `tty` - доступ к последовательным портам (UART)
- `i2c` - доступ к шине I2C для подключения дополнительных сенсоров
- `video` - доступ к видеокамере и видеодрайверам
- `gpio` - доступ к GPIO пинам для управления периферией

#### Шаг 2: Установка инструментов для работы с GPIO
```bash
sudo apt update
sudo apt install gpiod libgpiod-dev python3-libgpiod
```

#### Шаг 3: Проверка доступа к GPIO
```bash
sudo gpiodetect
sudo gpioinfo
```

**Что ожидать:** Команды покажут доступные GPIO чипы и их состояние

#### Шаг 4: Включение UART в системе
**Для Raspberry Pi OS (Legacy):**
```bash
# /boot/firmware/config.txt
enable_uart=1
```

**Для Ubuntu 24.04 Server:**
```bash
# /boot/firmware/config.txt
enable_uart=1
dtoverlay=uart4
dtoverlay=uart3
dtoverlay=uart2
```

**Пояснение параметров:**
- `enable_uart=1` - включает основной UART порт (GPIO14/15)
- `dtoverlay=uart4` - включает UART4 (используется для подключения полетного контроллера)
- `dtoverlay=uart3` и `dtoverlay=uart2` - включает дополнительные UART порты для расширения возможностей

**Важно:** После внесения изменений в config.txt требуется перезагрузка системы.
#### Шаг 5: Отключение консольного UART [Raspberry Pi OS]
Для работы по UART необходимо выставить UART для последовательной связи (не для консоли)
```bash
sudo raspi-config
# -> Interface Options -> Serial Port
# -> Would you like a login shell...? -> NO
# -> Would you like the serial port...? -> YES
```
## 4. Системное администрирование

### 4.1. Мониторинг температуры CPU
```bash
nano temp.sh
```

Содержимое скрипта:
```bash
#!/bin/bash
cpuTemp0=$(cat /sys/class/thermal/thermal_zone0/temp)
cpuTemp1=$(($cpuTemp0/1000))
cpuTemp2=$(($cpuTemp0/100))
cpuTempM=$(($cpuTemp2 % $cpuTemp1))

gpuTemp0=$(/opt/vc/bin/vcgencmd measure_temp)
gpuTemp0=${gpuTemp0//\'/º}
gpuTemp0=${gpuTemp0//temp=/}

echo CPU Temp: $cpuTemp1"."$cpuTempM"ºC"
```

Запуск скрипта:
```bash
bash temp.sh
```

**Примечание:** Для работы команды `vcgencmd` необходимо, чтобы камера была включена (`start_x=1`) и выделена достаточная память GPU (`gpu_mem=256`).

### 4.2. Сетевые настройки [Ubuntu]
Если сетевой интерфейс не поднимается автоматически:
```bash
sudo ip link set eth0 up
sudo dhclient -v eth0
```

**Пояснение:**
- `ip link set eth0 up` - принудительно включает сетевой интерфейс eth0
- `dhclient -v eth0` - запрашивает IP адрес по DHCP с выводом подробной информации

### 4.3. Управление системой
#### Безопасное выключение
```bash
sudo shutdown -h now
```

#### Перезагрузка системы
```bash
sudo reboot
```

**Рекомендация:** Всегда используйте команды shutdown/reboot вместо физического отключения питания, чтобы избежать повреждения файловой системы.

### 4.4. Восстановление файловой системы
При появлении сообщений о повреждении файловой системы при загрузке:

```bash
fsck -y /dev/sda1 ; reboot -f
```

**Пояснение параметров:**
- `fsck` - утилита проверки и восстановления файловой системы
- `-y` - автоматически отвечает "да" на все вопросы о восстановлении
- `/dev/sda1` - раздел диска, который необходимо проверить (замените на ваш реальный раздел)
- `reboot -f` - принудительная перезагрузка после завершения проверки

**Предупреждение:** Эта команда должна выполняться только в режиме восстановления (initramfs) или с отмонтированным корневым разделом. В нормальном режиме работы используйте `sudo fsck -y /dev/sda1` после отмонтирования раздела.

**Дополнительная информация:** [Подробное руководство по восстановлению файловой системы](https://ru.stackoverflow.com/questions/765130/%D0%9F%D1%80%D0%B8-%D0%B7%D0%B0%D0%BF%D1%83%D1%81%D0%BA%D0%B5-linux-%D0%BF%D0%B8%D1%88%D0%B5%D1%82-%D1%87%D1%82%D0%BE-%D1%82%D0%BE-%D0%BD%D0%B5%D0%BF%D0%BE%D0%BD%D1%8F%D1%82%D0%BD%D0%BE%D0%B5)

---
## 5. Настройка производительности [Raspberry Pi OS]
Для корректной работы микрокомпьютера, чтобы он не перегревался необходимо внести изменения в конфигурационный файл системы.

### Шаг 1: Редактирование конфигурационного файла
```bash
sudo nano /boot/firmware/config.txt
```

#### Шаг 2: Настройка секции [all]
Добавьте или отредактируйте секцию `[all]` следующим образом:
```bash
[all]
arm_freq=1500
arm_freq_min=600
temp_soft_limit=65000
```
### Шаг 3: Перезагрузка системы
```bash
sudo reboot