#ifndef TEST_TOOLS_H
#define TEST_TOOLS_H

// Shared buffer defined in test_main.c
extern char last_error_msg[256];

#define ASSERT_TRUE(condition, message) \
    if (!(condition)) { \
        snprintf(last_error_msg, sizeof(last_error_msg), \
                 "Line %d: %s", __LINE__, message); \
        return false; \
    }


#endif // TEST_TOOLS_H