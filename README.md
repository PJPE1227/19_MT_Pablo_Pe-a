# Implementación y validación de algoritmos de inteligencia de enjambre para planificación de trayectorias y problemas de optimización

**Autor:** Pablo Peña  
**Universidad del Valle de Guatemala**  
**Departamento de Ingeniería Electrónica, Mecatrónica y Biomédica**  
**Diseño e Innovación en Ingeniería**

---

## 🧩 Descripción general

Este repositorio contiene el código desarrollado para el trabajo de graduación titulado:  
**“Implementación y validación de algoritmos de inteligencia de enjambre para planificación de trayectorias y problemas de optimización.”**

El proyecto explora el **Bat Algorithm** y el **Firefly Algorithm** como alternativas a métodos de optimización clásicos (como PSO y ACO), aplicados a problemas relevantes en **ingeniería mecatrónica**, incluyendo:

- Planificación de trayectorias 2D libres de colisiones.  
- Validación de trayectorias en el simulador **Webots**.  
- Aplicación de los algoritmos a otros problemas de optimización, como el posicionamiento óptimo de sensores.  

Los algoritmos fueron programados en **MATLAB** y sus resultados se validaron en **Webots** utilizando un robot diferencial controlado mediante un esquema PID con acercamiento exponencial.

---

## ⚙️ Requisitos

### MATLAB
- MATLAB R2022a o superior.  
- Toolboxes recomendadas:  
  - *Optimization Toolbox*  
  - *Robotics System Toolbox*  
  - *Signal Processing Toolbox* (opcional, para análisis adicional).  

### Webots
- Versión recomendada: **Webots R2023b** o posterior.  
- Sistema operativo probado: **Windows 10 / 11 (64-bit)**.  

---

## 🚀 Ejecución

### En MATLAB
1. Clonar este repositorio o descargarlo como `.zip`.  
2. Abrir MATLAB y establecer la carpeta `MATLAB` como *working directory*.  
3. Revizar prototypeShowcase.m para ejemplos de como usar las funciones o leer los comentarios en las funciones

### En Webots
1. Abrir el proyecto desde la carpeta `Webots/`.  
2. Cargar el mundo principal (`laboratorio12.wbt`).  
3. Ejecutar la simulación para observar al robot diferencial siguiendo la trayectoria generada en MATLAB. Ajustar el  
codigo pololu3pi_controller.m para las distintas variaciones.

---

## 📊 Resultados principales

- El **Firefly Algorithm** mostró mayor estabilidad y precisión entre ejecuciones.  
- El **Bat Algorithm** fue más eficiente en tiempo de ejecución, pero con variabilidad mayor.  
- Ambos generaron trayectorias viables y libres de colisiones en entornos simulados.  
- Las simulaciones en Webots confirmaron la aplicabilidad práctica de ambos métodos en robótica móvil.  

---

## ✉️ Contacto

**Autor:** Pablo Peña  
📧 [Correo institucional] (pen21210@uvg.edu.gt)
🔗 [Repositorio principal en GitHub](https://github.com/PJPE1227/19_MT_Pablo_Pe-a)

---
