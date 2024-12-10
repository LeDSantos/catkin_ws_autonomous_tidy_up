# TESTES JÁ FEITOS

Em https://wiki.ros.org/Robots/TIAGo/Tutorials

- play_motion libery: funciona

- key_teleoperade: não funcionou
    - mundo world:=simple_office_with_people não tem chão, robô cai no infinito. Solução: não informa o world e usa o default vazio.
    - Erro para rosrun key_teleop key_teleop.py: 
    ```sh
    Traceback (most recent call last):
  File "/home/user/simulation_ws/src/teleop_tools/key_teleop/scripts/key_teleop.py", line 262, in <module>
    curses.wrapper(main)
  File "/usr/lib/python2.7/curses/wrapper.py", line 43, in wrapper
    return func(stdscr, *args, **kwds)
  File "/home/user/simulation_ws/src/teleop_tools/key_teleop/scripts/key_teleop.py", line 257, in main
    app = SimpleKeyTeleop(TextWindow(stdscr))
  File "/home/user/simulation_ws/src/teleop_tools/key_teleop/scripts/key_teleop.py", line 50, in __init__
    curses.curs_set(0)
    curses.error: curs_set() returned ERR
    ```

- Moving the base through velocity commands: funciona

- Joint Trajectory Controller: funciona

- Moving individual joints: funciona

- Head control: click to a point funciona

- Playing pre-defined upper body motions: funciona, é a play_motion

- Create a map with gmapping: funciona

- Localization and path planning: funciona, mas o braço enrosca nos objetos

- Planning in joint space: funciona

- Planning in cartesian space: funciona

- Planning in cartesian space with TRAC-IK: funciona

- Planning with Octomap demo: funciona com o mundo small_office

- Pick & Place demo: problema com o mapa, tudo cai assim que começa.

- E SÃO ESSES OS TUTORIAS Q ME INTERESSAM.

- ALGUNS DOS MUNDOS DO TIAGO_GAZEBO NÃO FUNCIONAM(tutorial_office). Pacote em /home/user/simulation_ws/src/tiago_simulation/tiago_gazebo e mundos em /home/user/simulation_ws/src/pal_gazebo_worlds/worlds/tutorial_office.world

    - home funciona, USAR ESSE PARA EXPERIMENTOS

- Fazer código básico para mover o robô até a mesa.

- Depois, pegar e soltar um bloco com aruco que está na mesa.

- O modelo de bloco aruco não funciona.

- Mapa arrumando no launch

- usar export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/user/simulation_ws/src/pal_gazebo_worlds/models

- /home/user/simulation_ws/src/pal_gazebo_worlds/models

- **PRÓXIMO PASSO: pegar bloco genérico**

    - Cubo aruco muito problematico
    - Robo com octomap funcionando, ele estende o braço, se aproxima e olha em volta no my_simple_demo
    - Tentando fazer moveit funcionar dentro do meu pacote FUNCIONOU
    - exemplo em /home/user/simulation_ws/src/tiago_tutorials/tiago_moveit_tutorial
    - CONSEGUI FAZER O BRAÇO IR PARA UM PONTO
    - Consigui fazer o robô pegar um objeto imaginário
    - PRÓXIMO PASSO: PAGAR O OBJETO Q TA SOBRE A MESA
    - ATUALMENTE NÃO ACHA UM CAMINHO, VERIFICAR GOALS (TALVEZ O GREP PRECISA SER COM O BASE_FRAME) 

- roslaunch tiago_gazebo tiago_gazebo.launch public_sim:=true robot:=steel world:=home gzpose:="-x 1.40 -y -2.79 -z -0.003 -R 0.0 -P 0.0 -Y 0.0" use_moveit_camera:=true

- Lançar simulação: roslaunch tiago_gazebo tiago_gazebo.launch public_sim:=true robot:=steel world:=empty



roslaunch tiago_2dnav_gazebo tiago_navigation.launch public_sim:=true robot:=steel world:=home_plus_cube lost:=false

roslaunch tiago_2dnav_gazebo tiago_navigation.launch public_sim:=true robot:=steel world:=small_office_plus_cube map_file:=small_office lost:=false

roslaunch tiago_2dnav_gazebo tiago_navigation.launch public_sim:=truerobot:=steel world:=small_office_plus_cube map_file:=small_office use_moveit_camera:=true lost:=false

/home/user/simulation_ws/src/pal_gazebo_worlds/worlds/small_office_plus_cube.world

# Para rodar meu código

**roslaunch tiago_2dnav_gazebo tiago_navigation.launch public_sim:=true robot:=steel world:=small_office_plus_cube map_file:=small_office use_moveit_camera:=true lost:=false**

Usa mundo small_office em 

**roslaunch my_simple_demo run_my_simple_demo.launch**


TUTORIAL Q INSPIROU PICK AND PLACE: http://docs.ros.org/en/kinetic/api/moveit_tutorials/html/doc/pick_place/pick_place_tutorial.html