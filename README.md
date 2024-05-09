# Proyecto_final_movil


NOTAS SOBRE SLAM

- En SLAM no hay mapa, se tiene que construir a medida que el robot se mueve. 
- Los landmarks existentes en el entorno se deben ir añadiendo al mapa del robot a medida que los observa. Se pueden guardar en el propio estado X_k del robot a la vez que guarda el estado estimado de los landkmarks que va detectando.
También se debe incrementar la matriz de covarianza del estado, agregando la covarianza asociada al landmark nuevo detectado.
- Para calcular las observaciones predichas y^k para el error de innovación se deben tener en cuenta solo las que estén dentro del rango perceptual del robot. Si el robot ha visto una landmark pero deja de verla no se podría calcular el error de innovación porque aunque esté almacenada en el estado, no podemos tener su medida.
- El array de observaciones reales y y el array de observaciones predichas tienen dimensiones distintas (OJO EN LA ASOCIACION). Por ejemplo, en el instante k = 0, el estado es la posición / pose del robot. Si en el instante detecta una landmark la tiene que añadir al vector de estado que pasaría a tener [pose del robot; posición del landmark] (y la matriz de covarianza como corresponda). Según se va moviendo el robot encontrará landmarks nuevas y dejará de ver alguna otra. Supongamos que el robot ha visto 50 landmarks y vuelve a la posición inicial donde solo detecta la primera...pues el vector de estado tendrá la pose del robot + 50 posiciones, mientras que la observaicón será solo la posición de una landmark.