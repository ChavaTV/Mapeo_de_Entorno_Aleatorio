Saludos,

Para este proyecto en particular, es esencial tener en cuenta los siguientes aspectos:

Los siguientes archivos son fundamentales para la conexión entre el simulador y el código:
- `sim.py`
- `simConst.py`
- `remoteApi.dll`

El archivo denominado "map" constituye una tabla que representa el último mapa generado en 
la ejecución del programa `Proyecto_ASTV.py`. Es importante recalcar que, si se itera utilizando
el mismo escenario ("obstaculos.ttt"), el mapa se sobrescribirá, y en cada iteración mejorará progresivamente.
Siempre y cuando la generación de cubos este comentada.

Cabe mencionar que existe la posibilidad de extender el tiempo de ejecución en el código `Proyecto_ASTV.py` 
para proporcionar un período más amplio para la búsqueda.

En la sección del código que se encarga de generar los obstáculos, es posible que se presente un error: dos o 
más cubos pueden generarse en el mismo punto de coordenadas, lo que puede resultar en comportamientos inadecuados 
como objetos que se desplazan de forma impredecible.

Si se disminuye el número de cubos generados aleatoriamente, por ejemplo, al mover aproximadamente 5 cubos, se 
obtienen mapas semialeatorios que ofrecen resultados aceptables.

Es importante destacar que al iniciar la simulación, esta se encuentra en pausa debido a que el robot ejecuta
 una rutina de evasión de obstáculos. Es necesario esperar hasta que los objetos se hayan desplazado por completo
 para que el mapa pseudoaleatorio pueda formarse. Una vez culminada esta etapa, se puede activar la simulación en 
el escenario de CoppeliaSim.

En la consola, se presentarán las variables de velocidad, las cuales cumplen el rol de guía para iniciar la 
simulación de manera adecuada.

Atentamente,
Salvador