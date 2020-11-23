 #include <gtest/gtest.h>

TEST(TestSuite, testCase1)
{
    EXPECT_EQ(5, 2+3);
}
 
TEST(TestSuite, testCase2)
{
}
 
// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}