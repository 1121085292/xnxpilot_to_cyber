#include <gtest/gtest.h>
#include <CL/cl.h>

#include "clutil.h"

TEST(clutilTest, get_device_id_test){
    cl_device_id device_id = cl_get_device_id(CL_DEVICE_TYPE_DEFAULT);
    EXPECT_EQ(device_id, CL_SUCCESS);
}