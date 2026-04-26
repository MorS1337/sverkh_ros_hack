# Ros2Drun

![Python](https://img.shields.io/badge/Python-3.10%2B-3776AB?logo=python&logoColor=white)
![ROS2](https://img.shields.io/badge/ROS-2-22314E?logo=ros&logoColor=white)
![OpenCV](https://img.shields.io/badge/OpenCV-4.x-5C3EE8?logo=opencv&logoColor=white)
![YOLOv8](https://img.shields.io/badge/YOLO-v8-111111)
![PX4](https://img.shields.io/badge/PX4-Offboard-orange)
![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)

## 1. Заголовок и краткое описание проекта

**Ros2Drun** — проект для автономного полета дрона в ROS 2 с несколькими стратегиями миссии (по ArUco-координатам, по точкам, по относительным перемещениям), съемкой кадров и пост-распознаванием объектов через YOLOv8.

Решение создано в рамках хакатон-интенсива **«Автономные дроны и бортовой ИИ» от Сверх** и демонстрирует интеграцию полетного контура PX4 Offboard и CV-контура для автоматизированного облета и анализа сцены.

---

## 2. Установка и запуск проекта

### 2.1. Предварительные требования

- Linux/macOS среда разработки
- Python 3.10+
- ROS 2 (рекомендуется Humble/Iron)
- Работающий полетный стек с сервисами:
  - `/navigate`
  - `/land`
  - `/get_telemetry`
- Подключенные зависимости ROS-проектов с интерфейсами:
  - `offboard_interfaces`
  - `sensor_msgs`
  - `std_srvs`
  - `cv_bridge`

Подробности по связке FCU/PX4/Offboard: в документах `docs/fcu.md` и `docs/offboard.md`.

### 2.2. Клонирование

```bash
git clone <URL_ВАШЕГО_РЕПОЗИТОРИЯ>
cd sverkh_ros_hack
```

### 2.3. Настройка окружения Python

```bash
python3 -m venv venv
source venv/bin/activate
pip install --upgrade pip
pip install ultralytics opencv-python
```

Примечание: `rclpy`, `cv_bridge` и ROS-сообщения/сервисы устанавливаются через вашу ROS 2 среду, а не через `pip`.

### 2.4. Подготовка каталогов для результатов

```bash
mkdir -p photos detections
```

### 2.5. Запуск проекта

Основной сценарий запуска:

```bash
python main.py
```

В `main.py` можно выбрать стратегию полета:
- `ArucoFramesDroneController` — полет по точкам в `aruco_map`
- `PointsFlightDroneController` — полет по заданным точкам
- `LuckyFlightDroneController` — полет по относительным смещениям

### 2.6. Запуск через контейнеры (Docker)

В текущем репозитории Docker-конфигурация отсутствует. Если нужен контейнерный запуск, можно использовать инструкции подготовки платформы и окружения из `docs/cm5_rpi_tutorial.md` и `docs/cm5_radxa_tutorial.md` как базу для собственного `Dockerfile`/`docker-compose.yml`.

---

## 3. Основной функционал проекта

- Автономный взлет/посадка через ROS 2 сервисы Offboard.
- Несколько режимов миссии с переключаемой стратегией полета.
- Полет в разных системах координат (`body`, `aruco_map`).
- Потоковая телеметрия и контроль состояния дрона во время миссии.
- Съемка кадров с камеры во время/после миссии.
- Пост-обработка снимков и детекция целевых объектов (`orange`, `teddy bear`) через YOLOv8.
- Сохранение аннотированных кадров с bounding boxes в отдельный каталог.

---

## 4. Технологии и инструменты

- Язык: Python
- Робототехнический стек: ROS 2, PX4 Offboard
- CV/ML: OpenCV, Ultralytics YOLOv8 (NCNN-экспорт модели)
- Интеграция сенсоров: `cv_bridge`, `sensor_msgs`
- Железо/платформы (по документации): Raspberry Pi CM5, Radxa CM5, Matek H743 Mini V3

---

## 5. Команда проекта

- Буянов Петр
- Морев Семен
- Табаков Максим

Разделения по ролям не фиксировалось: вклад команды был совместным на этапах интеграции полетной логики, настройки окружения и компьютерного зрения.

---

## 6. Архитектура и структура проекта

### 6.1. Структура репозитория

```text
.
├── main.py                     # Точка входа: выбор стратегии миссии + запуск распознавания
├── drone_controller.py         # Базовый контроллер ROS2: navigate/land/telemetry/camera
├── aruco_frames_flight.py      # Полет по координатам/рамкам в aruco_map
├── points_flight.py            # Полет по заданному списку точек
├── lucky_flight.py             # Полет по относительным смещениям
├── body_drun.py                # Альтернативная body-миссия (snake) + автофото
├── recognize.py                # Детекция объектов на снимках (YOLOv8)
├── test.py                     # Экспорт YOLO в NCNN
├── yolov8n_ncnn_model/         # NCNN-модель для инференса
└── docs/                       # Техническая документация по платформе и offboard
```

### 6.2. Диаграмма архитектуры

```mermaid
flowchart LR
    A[Mission Script main.py] --> B[DroneController ROS2 Node]
    B --> C[/navigate]
    B --> D[/get_telemetry]
    B --> E[/land]
    B --> F[Camera Topic /aruco_det/debug_image]
    F --> G[photos/*.jpg]
    G --> H[recognize.py + YOLOv8 NCNN]
    H --> I[detections/found_*.jpg]
```

---

## 7. Демонстрация работы проекта

- Видео/демо: https://t.me/sverk_official/122
- Скриншоты интерфейса/результатов:
  - Можно добавить кадры из `photos/`
  - Можно добавить аннотированные результаты из `detections/`

---

## 8. Финальный текст-заключение

Ros2Drun интересен как практический пример end-to-end контура автономной миссии дрона: от команд управления полетом и телеметрии до компьютерного зрения и сохранения результатов детекции. Проект показывает, как быстро собрать прикладной прототип на ROS 2 + PX4 + YOLO для задач инспекции и поиска объектов.

Потенциальные улучшения:
- Добавить автоматический выбор следующей точки маршрута на основе результатов детекции.
- Перевести пост-распознавание в online-режим прямо во время полета.
- Вынести конфиг миссий и параметров в YAML/CLI.
- Добавить Docker-окружение и CI-проверки.
- Добавить тесты и логирование метрик качества детекции/миссии.

---

## 10. Лицензия

Проект распространяется под лицензией **MIT**.
