mulu=${PWD##*/}
echo $mulu
cd ../../
catkin_make -DCATKIN_WHITELIST_PACKAGES=$mulu  --force-cmake -G"Eclipse CDT4 - Unix Makefiles" -DCMAKE_BUILD_TYPE=Debug -DCMAKE_ECLIPSE_MAKE_ARGUMENTS=-j8

awk -f $(rospack find mk)/eclipse.awk build/.project > build/.project_with_env && mv build/.project_with_env build/.project
