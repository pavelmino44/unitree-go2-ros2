## Актульные отличия в топиках

Соотношение топиков в тз и симуляторе

| Наше ТЗ               | Симулятор                | Статус                               |
| --------------------- | ------------------------ | ------------------------------------ |
| `/cmd_vel`            | `/cmd_vel`               | ✅ точно совпадает                   |
| `/point_cloud2`       | `/velodyne_points`       | ⚠️ другой topic name, тип правильный |
| `/scan`               | `/hokuyo_frame/scan`     | ⚠️ другой topic name, тип правильный |
| `/imu`                | `/imu/data`              | ⚠️ другой topic name, тип правильный |
| `/odom`               | `/odom`                  | ✅ совпадает                         |
| `/tf`                 | `/tf`                    | ✅ совпадает                         |
| `/tf_static`          | `/tf_static`             | ✅ совпадает                         |
| `/camera/image_raw`   | `/image_raw`             | ⚠️ другой topic name, тип правильный |
| `/camera/compressed`  | `/image_raw/compressed`  | ⚠️ другой topic name, тип правильный |
| `/camera/camera_info` | `/image_raw/compressed`  | ❌отсутствует                        |
| `/robot_state`        | —                        | ❌отсутствует (и не будет)           |
