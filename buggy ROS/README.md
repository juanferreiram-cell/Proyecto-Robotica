# Buggy con RPLIDAR A1 para ROS 2 Lyrical y Gazebo Jetty

Este paquete conserva las 8 mallas STL y las articulaciones originales exportadas con `onshape-to-robot`, y agrega un LiDAR de 360 grados sobre el chasis.

## Cambios realizados

- `robot` renombrado a `buggy` (antes contenía una ruta local de Windows).
- Las rutas de las mallas pasan a `package://buggy_description/meshes/*.stl`.
- Los nombres acentuados de dirección se normalizaron a `direccion_...` para facilitar la portabilidad. No se cambiaron las articulaciones originales.
- Nuevo `lidar_link`, unido mediante `lidar_joint` de tipo `fixed` a `chasis__1`.
- El RPLIDAR simulado publica `/scan` con 360 muestras por vuelta, 5.5 Hz y rango de 0.15 a 6 m. Estos son parámetros iniciales conservadores; algunas revisiones del A1 alcanzan 12 m.
- El sensor se ubica provisionalmente en `xyz="0.14 0 0.14"` respecto a `chasis__1`. Confirmar su ubicación después de ver el conjunto en Gazebo.

## Integrarlo a tu espacio de trabajo de ROS 2

1. Descomprimí el archivo ZIP en la carpeta `src` de tu workspace (`~/ros2_ws/src/`, por ejemplo), de forma que quede `src/buggy_description/`.
2. Ejecutá desde la raíz de tu workspace:

```bash
source /opt/ros/lyrical/setup.bash
colcon build --packages-select buggy_description
source install/setup.bash
export GZ_SIM_RESOURCE_PATH="$(ros2 pkg prefix buggy_description)/share:${GZ_SIM_RESOURCE_PATH}"
```

3. El mundo que uses debe cargar el sistema de sensores. Dentro de `<world>`, agregá lo siguiente si aún no existe:

```xml
<plugin filename="gz-sim-sensors-system"
        name="gz::sim::systems::Sensors">
  <render_engine>ogre2</render_engine>
</plugin>
```

Si agregás plugins explícitos en un mundo que antes utilizaba los predeterminados, asegurate también de que estén cargados `gz-sim-physics-system`, `gz-sim-scene-broadcaster-system` y `gz-sim-user-commands-system`.

4. Abrí el mundo en Gazebo (primera terminal):

```bash
gz sim /ruta/a/tu/laberinto.sdf
```

5. Importá el buggy (segunda terminal, con ROS configurado):

```bash
ros2 run ros_gz_sim create -file "$(ros2 pkg prefix buggy_description)/share/buggy_description/urdf/buggy_rplidar_a1.urdf" -name buggy -z 0.3
```

6. Verificá que Gazebo publique `/scan` con `gz topic -l`. Si está presente, enlazalo a ROS 2 (tercera terminal):

```bash
ros2 run ros_gz_bridge parameter_bridge '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan'
```

7. Comprobá el escaneo (cuarta terminal):

```bash
ros2 topic echo /scan --qos-reliability best_effort
```

## Pendientes antes de navegación autónoma

- Las masas e inercias de la exportación original son `1e-09`: son valores de relleno y deben sustituirse por propiedades físicas realistas para que la simulación del movimiento sea fiable.
- El enlace raíz de la exportación sigue siendo `manubrio__2`, no el chasis. Para odometría / Nav2 conviene revisar la estructura y definir de forma consistente `base_link`, `odom` y TF.
- El URDF conserva los acoplamientos `<mimic>` originales, pero no incluye controladores de tracción y dirección. El LiDAR funcionará al iniciar la simulación; para conducir el buggy hará falta agregar los controladores apropiados.
- Los archivos de malla son los de la exportación original. El cuerpo del LiDAR es un cilindro aproximado: **no es un CAD exacto** del sensor físico.

El archivo original `Buggy.rar` no fue modificado.
