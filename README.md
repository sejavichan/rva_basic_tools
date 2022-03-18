# rva_basic_tools
Basic tools being designed and developed in Robotica y Vision Artifical subject of University Pablo de Olavide

Comandos para ejecutar el código:
1. "roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch", comando para lanzar gazebo y visualizar el robot.
2. Dentro de la carpeta rva_basic_tools, ejecutar "roslaunch rva_basic_tools test_path_publisher.launch path_file:=path_2", se incluye el atributo path_file para lanzar el path_2.

Proceso de ejecución:
1. Se suscribe al path y recorrerá los puntos que conforman este camino.
2. Irá recibiendo las coordenadas X e Y de los sucesivos puntos con una tolerancia relativamente alta, en comparación con el punto destino.
3. La velocidad linear y angular irá variando según dónde se encuentre el siguiente punto del camino a seguir.
4. Si el punto se encuentra en un ángulo respecto al robot superior al qe se le pasa como parámetro, girará con vel linear 0.
5. De lo contrario, avanzará con velocidad linear y angular relativa a dónde se encuentre el siguiente punto.


Comentarios adicionales:
1. Hemos comprobado que podríamos subir la tolerancia del ángulo y conseguimos una ejecución más rápida aunque 
se desviará algo más del camino a seguir.
2. El hecho de suscribirse al path e ir obteniendo los puntos ha sido de lo más complejo que nos hemos encontrado.
3. Optimizar la velocidad linear y angular ha requerido de muchas pruebas de ensayo y error.
