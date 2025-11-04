In file included from /home/a/catkin/src/mission_master_pkg/lib/callback.cpp:6:
/home/a/catkin/src/mission_master_pkg/include/mission_master_pkg/mission_master_node.h:20:10: fatal error: mission_master_pkg/AvoidAction.h: No such file or directory
   20 | #include "mission_master_pkg/AvoidAction.h"
      |          ^~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
compilation terminated.
In file included from /home/a/catkin/src/mission_master_pkg/lib/mission_master_node.cpp:6:
/home/a/catkin/src/mission_master_pkg/include/mission_master_pkg/mission_master_node.h:20:10: fatal error: mission_master_pkg/AvoidAction.h: No such file or directory
   20 | #include "mission_master_pkg/AvoidAction.h"
      |          ^~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
compilation terminated.
make[2]: *** [mission_master_pkg/CMakeFiles/mission_master_lib.dir/build.make:63: mission_master_pkg/CMakeFiles/mission_master_lib.dir/lib/callback.cpp.o] Error 1
make[2]: *** Waiting for unfinished jobs....
make[2]: *** [mission_master_pkg/CMakeFiles/mission_master_lib.dir/build.make:76: mission_master_pkg/CMakeFiles/mission_master_lib.dir/lib/mission_master_node.cpp.o] Error 1
make[1]: *** [CMakeFiles/Makefile2:1418: mission_master_pkg/CMakeFiles/mission_master_lib.dir/all] Error 2
make: *** [Makefile:141: all] Error 2
Invoking "make -j2 -l2" failed
a@ubuntu:~/catkin$ 

