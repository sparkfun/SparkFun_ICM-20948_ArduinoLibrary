#! /bin/bash
SCRIPT_DIR=$( cd "$(dirname "${BASH_SOURCE[0]}" )" ; pwd -P )
cd "$SCRIPT_DIR"
rm -r ../build
mkdir ../build
cd ../build
cmake ../
make
./icm20948_DMP_example
