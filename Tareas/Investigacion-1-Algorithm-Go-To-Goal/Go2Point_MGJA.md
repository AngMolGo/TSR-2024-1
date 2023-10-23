# Investigación 01: # Algoritmo "Go to Point" Simple

|Asignatura|Trabajo de investigación|
| ------:| ----------- |
| **TSR-2024-I** | Investigación 01: Algoritmo **"Go to Point"** Simple|

## Contenido

- [Objetivo](##objetivo)
- [Introducción](##introduccion)
- [Desarrollo](##desarrollo)
- [Autor](#autor)
- [Referencias](#referencias)

## Objetivo

Conocer el algoritmo de "Go-to-Point".

## Introducción

El algoritmo "Go to Point Simple" o "Go to Goal" es una técnica utilizada en robótica y control de movimiento para navegar a un robot desde su posición actual a un punto de destino específico en un entorno. Este algoritmo se utiliza comúnmente en robots móviles y vehículos autónomos para lograr la navegación.

Las consideraciones que tomaremos en cuenta para la descripción del robot móvil serán las siguientes:

- El modelo simplificado considera un robot móvil simétrico y uniforme con 2 ruedas a sus lados. (Fig. 1[^PID])
![Fig 1. Modelo de robot móvil estándar]([/Multimedia/Figura2.jpg](https://github.com/AngMolGo/TSR-2024-1/blob/AngMolGo-Investigacion-Go-To-Point/Tareas/Investigacion-1-Algorithm-Go-To-Goal/Multimedia/Figura1.jpg))

- El sistema de referencia referenciado al robot será considerado una transformada homogénea del sistema, con una rotación sobre el eje z de $\theta$ y con un desplazamiento de $x_{0}$ y $y_{0}$. El vector $\vec{v}$ se describe sobre el eje x. La matriz de rotación se muestra a continuación:

$$
{R_{z}(\theta)} \ = \begin{bmatrix}
cos(\theta) & -sen(\theta) & x_{0}\\
-sen(\theta) & cos(\theta) & y_{0}\\
0 & 0 & 1
\end{bmatrix}
$$
 
De acuerdo con lo anterior, el algoritmo Go-to-Point puede considerarse como un algoritmo de dos procedimientos: girar y avanzar.

## Desarrollo

### Algoritmo **_Go to Point simple o Go to Goal_**

>**Problema**: Un robot móvil del **tipo (2,0)** se encuentra en el punto **$P_{0}(x_{0}, y_{0})$** con una rotación de **$\theta$** radianes en su eje **Z** (fig.2).  
Se requiere llegar al punto **$P_{f}(x_{1}, y_{1})$** sin importar su orientación (**θ** radianes) de giro sobre su eje **Z** (fig.3).
![Fig 2. Imagen de espacio 2D donde se mueve el carrito, con el robot móvil en un punto inicial.](/Multimedia/Figura1.jpg)
![Fig 3. Imagen de espacio 2D donde se mueve el carrito, con el robot móvil en el punto final deseado.](/Multimedia/Figura2.jpg)

Para solucionar esto, primero debemos asegurarnos de que el vector $\vec{v}$, sea paralelo al vector $\vec{w}$ formado por los puntos $P_{0}$ y $P_{f}$, para asegurarnos que el robot avanzará derecho  hacia el punto indicado, lo que quiere decir que $\vec{v} \times \vec{w} = 0$. 
Sin embargo, una manera más fácil de realizar esto, es haciendo una comparación escalar entre el ángulo que describe la dirección del vector $\vec{w}$ y el ángulo de giro $\theta$ de la transformada del sistema del robot. Lo que buscamos, es que sean el mismo ángulo, por lo tanto, la relación que buscamos es:
$$\theta - \arctan(\frac{y_{1}-y_{0}}{x_{1}-x_{0}})=0$$

Cuando se consigue esta relación podemos empezar la siguiente etapa: avanzar hacia el objetivo. 

Para dirigirse al objetivo solo es necesario que el robot avance en la dirección y sentido dado por la relación anteriormente validada, se detendrá cuando el error de distancia entre el punto actual y el punto final sea menor a un error de exactitud prestablecido.

### Propuesta de código

Para la propuesta del código que implementa el algoritmo solicitado se supondrá que contamos con una función que nos devuelve la pose del robot en ese momento.
También se puede considerar que tenemos una función que implementa un control PID que nos ayuda a llegar a los valores deseados de posición y sentido.

``` python

from math import atan2,hypot

def Movil():
	def __init__(self):
		self.pose = (0,0,0) # Pose: (x,y,theta)
		self.point_f = (0,0)
		self._error_dir = 0.0175
		self._error_dist = 0.05 # 5cm
		
	def refresh_pose(self):
		### Actualiza la pose en el momento
		self.pose = (x0,y0,theta0)

	def is_dir_correct(self):
		### la función regresa True si la dirección del vector(w) es igual al ángulo de rotación
		##  con un error aceptable de 1° = 0.0175rad
		dir_correct = False
		diff = self.pose[2] - atan2(self.point_f[1] - self.pose[1], self.point_f[0] - self.pose[0])
		if abs(diff) < self._error_dir:
			dir_correct = True
		return dir_correct

	def is_in_position(self):
		### la función regresa True si el robot llegó al destio
		pos_correct = False
		### Hace lo mismo que self.is_dir_correct()	pero con la posición. Igualmente implementaría un control PID.
		if self.getDistance() < self._error_dist;
			pos_correct = True
		return dir_correct

	def gira(self):
		diff = self.pose[2] - atan2(self.point_f[1] - self.pose[1], self.point_f[0] - self.pose[0])
		if diff < 0:
			### Gira para un lado
		elif diff > 0:
			### Gira para el otro lado
		### Se implementa el control PID en esta parte
		
	def go_to_point(self,x,y):
		self.refresh_pose()
		self.point_f = (x,y)
		while(not self.is_dir_correct())
			self.refresh_pose()
			self.gira()
		### Para de girar
		while(not self.is_in_position()):
			self.refresh_pose()
			self.avanzar()
		### Se detiene
		print("El movil llegó al punto indicado")

	def getDistance(self):
		math.hypot(self.pose[1] - self.point_f[1],self.pose[0] - self.point_f[0] )

def main():
	x,y = (5,10) # Punto de ejemplo
	movil = Movil()
	movil.go_to_point(x,y)

if __name__ == "__main__":
	main()
```

Entre las ventajas que se le atribuyen a atan2 son:
- Proporciona un resultado en el rango completo de -π a π (o -180° a 180°), lo que permite calcular ángulos en todas las direcciones sin ambigüedad.
- Maneja correctamente la división por cero, ya que evita problemas cuando el denominador es cero.


**¿Cómo variaría el algoritmo si la orientación **"θ"** del robot al alcanzar el punto P1 fuera obligatoria?**

Si se desea obtener una orientación deseada al final de la trayectoria, se debería realizar el primer paso de alinearse con el ángulo deseado, solo modificando el valor del ángulo de entrada por el ángulo deseado al final.

**¿A que tribuye los errores de orientación y distancia que se producen durante el movimiento?**

Se pueden atribuir a la falta de exactitud de los sensores de posición tales como el ruido o por la respuesta natural del sensor. Por otro lado, también se puede atribuir a errores mecánicos en las llantas como que una llanta está ligeramente más grande que la otra, el eje de las llantas no esta debidamente alineado, etc.
Es por eso que, en el mundo real, el monitoreo constante del status de la posición del robot móvil es muy importante para la ejecución correcta del programa. Para apoyar esta parte, se suele implementar un control PID[^PID] para controlar la trayectoria correcta del robot móvil.

## Conclusiones

El algoritmo Go-to-point simple es el algoritmo más básico para el desarrollo de la autonomía en un robot móvil. 

## Autor

**Autor:** José Ángel Molina González [GitHub profile](https://github.com/AngMolGo)

## Referencias

 [^PID]:  S. Tasnim, "Navigation and Obstacle Avoidance Control of an Autonomous Differential Wheeled Robot Using PID Controller in Matlab Simulatio", Mech. & Aero. Eng. Dep., Carleton University, Ottawa. 2019.
