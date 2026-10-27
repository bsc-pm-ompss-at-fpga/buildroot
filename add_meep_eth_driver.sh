#!/bin/bash

mkdir -p output/build/linux-5.10.7/drivers/meep-pcie-ethernet
rsync -avp meep-pcie-ethernet output/build/linux-5.10.7/drivers/
echo "obj-y += meep-pcie-ethernet/" >> output/build/linux-5.10.7/drivers/Makefile
