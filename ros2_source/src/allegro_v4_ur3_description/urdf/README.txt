##########################################
############# 중 요 !!!! ##################
##########################################

Official github에서 다운 받은 URDF를 활용하여, 
allegro hand V4 + UR3 URDF 파일은 생성되지만, 
실제로 돌려보면 아래와 같은 에러가 발생하면서 구동이 불가함. 
아무래도 interia등 파라미터 값이 이상하게 들어간 것 같음..
이건 아직 미 해결 상태임


(base) tw@tw-pc:~$ rviz2
[INFO] [1754642832.266854240] [rviz2]: Stereo is NOT SUPPORTED
[INFO] [1754642832.266925201] [rviz2]: OpenGl version: 4.6 (GLSL 4.6)
[INFO] [1754642832.350570586] [rviz2]: Stereo is NOT SUPPORTED
[ERROR] [1754642844.786119340] [rviz2]: The link link_10 is has unrealistic inertia, so the equivalent inertia box will not be shown.

[ERROR] [1754642844.786675651] [rviz2]: The link link_11 is has unrealistic inertia, so the equivalent inertia box will not be shown.

[ERROR] [1754642844.790231173] [rviz2]: The link link_12 is has unrealistic inertia, so the equivalent inertia box will not be shown.

[ERROR] [1754642844.790813174] [rviz2]: The link link_13 is has unrealistic inertia, so the equivalent inertia box will not be shown.

[ERROR] [1754642844.791715275] [rviz2]: The link link_14 is has unrealistic inertia, so the equivalent inertia box will not be shown.

[ERROR] [1754642844.792516715] [rviz2]: The link link_15 is has unrealistic inertia, so the equivalent inertia box will not be shown.

[ERROR] [1754642844.793063316] [rviz2]: The link link_3 is has unrealistic inertia, so the equivalent inertia box will not be shown.

[ERROR] [1754642844.793141816] [rviz2]: The link link_5 is has unrealistic inertia, so the equivalent inertia box will not be shown.

[ERROR] [1754642844.793169676] [rviz2]: The link link_6 is has unrealistic inertia, so the equivalent inertia box will not be shown.

[ERROR] [1754642844.793196166] [rviz2]: The link link_7 is has unrealistic inertia, so the equivalent inertia box will not be shown.

[ERROR] [1754642844.793314456] [rviz2]: The link link_9 is has unrealistic inertia, so the equivalent inertia box will not be shown.

^C[INFO] [1754642867.888942902] [rclcpp]: signal_handler(signum=2)
(base) tw@tw-pc:~$ 

