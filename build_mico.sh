echo "Building MICO node"

cd Examples/mico
mkdir build
cd build
cmake ..
make -j
