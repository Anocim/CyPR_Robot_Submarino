Encontramos desarrollados diferentes controladores que se han llevado a cabo hasta conseguir los finales.

Para llegar a la creación de dichos controladores, primero se han llevado a cabo ciertas pruebas para el desarrollo de estos:
- angulos
- mapeo

Primeros controladores desarrollados:
 - controlador.py
 - controlador2.py
 - controlador_PD
 - controlador_PID

Después, se evolucionaron algunos controladores añadiéndo la dinámica del robot:
 - control_dinamica
 - control_din_PID
 - control_din_PID_odom

 Por último, los dos controladores más óptimos a probar son:
 - control_din_PID_odom_cambio
 - control_sdre

 Para cambiar referencias y posiciones, los archivos desarrollados son:
 - cambio_pos
 - cambio_ref
 - mover

Finalmente, se ha desarrollado también un controlador a ALTO nivel:
- control_punto_punto

Se intentaron desarrollar simuladores de corrientes submarinas con:
- corriente_submarinas

Para acabar, cómo no se pudieron añadir agentes externos que descontrolaran al robot, se añadió al control SDRE perturbaciones:
- control_sdre_perturbaciones