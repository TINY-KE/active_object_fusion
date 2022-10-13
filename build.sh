echo "Configuring and building Thirdparty/DBoW2 ..."

cd Thirdparty/DBoW2
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Debug #Release
make -j

cd ../../g2o

echo "Configuring and building Thirdparty/g2o ..."

mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Debug #Release
make -j

# cd ../../Line3Dpp

# echo "Configuring and building Thirdparty/Line3Dpp ..."

# mkdir build
# cd build
# cmake .. -DCMAKE_BUILD_TYPE=Release
# make -j


# echo "Configuring and building EAO_SLAM ..."
# cd ../../
# mkdir build
# cd build
# cmake .. -DCMAKE_BUILD_TYPE=Release
# make -j10

