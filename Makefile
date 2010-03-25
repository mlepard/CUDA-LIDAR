################################################################################
#
# Build script for project
#
################################################################################

# Add source files here
EXECUTABLE	:= lidar
# CUDA source files (compiled with cudacc)
CUFILES		:= lidar_project.cu
# CUDA dependency files
CU_DEPS		:= return_to_point.cu \
                   simple_slope.cu

# C/C++ source files (compiled with gcc / c++)
CCFILES		:= dat_reader.c \
                   lidar_packet.c 

################################################################################
# Rules and targets
USEGLLIB        := 1
USECUFFT        := 1
USEGLUT         := 1


include ../../common/common.mk
