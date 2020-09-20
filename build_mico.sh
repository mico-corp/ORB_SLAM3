echo "Building MICO node"

cd Examples/mico
mkdir build
cd build
cmake ..
make -j

cp libmico-orbslam3.so ~/.flow/plugins
