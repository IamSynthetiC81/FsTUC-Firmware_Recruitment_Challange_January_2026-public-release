// ADC unit tests for simulation timing and API behavior
#include "unity.h"
#include "ADC.h"

#include <string.h>
#include <time.h>


static ADC_Handle_t adc;
static ADC_Config_t config;

static void init_polling_config(void) {
	memset(&config, 0, sizeof(config));
	config.mode = ADC_MODE_POLLING;
	config.channelEnable = 0x07;  /* Enable all 3 channels */
}

void setUp(void) {
	memset(&adc, 0, sizeof(adc));
	memset(&config, 0, sizeof(config));
}

void tearDown(void) {
}

void test_ADC_Init_ValidConfig(void) {
	init_polling_config();

	TEST_ASSERT_TRUE(ADC_Init(&adc, &config));
	TEST_ASSERT_EQUAL_INT(ADC_MODE_POLLING, adc.config.mode);
	TEST_ASSERT_EQUAL_INT(0x07, adc.config.channelEnable);
}

void test_ADC_Init_InvalidConfig(void) {
	/* NULL config pointer should return false */
	TEST_ASSERT_FALSE(ADC_Init(&adc, NULL));

	/* NULL handle should return false */
	init_polling_config();
	TEST_ASSERT_FALSE(ADC_Init(NULL, &config));
}

void test_ADC_Start_Stop_StateTransitions(void) {
	init_polling_config();
	TEST_ASSERT_TRUE(ADC_Init(&adc, &config));

	/* Polling mode: ADC_Start and ADC_Stop only work in AUTOMATIC mode */
	/* For polling mode, samples are retrieved directly via ADC_GetSample */
	TEST_ASSERT_FALSE(ADC_Start(&adc));
	TEST_ASSERT_FALSE(ADC_Stop(&adc));
}

void test_ADC_GetSample_OnlyPollingMode(void) {
	init_polling_config();
	config.mode = ADC_MODE_AUTOMATIC;  /* Use AUTOMATIC mode */
	/* ADC_Init should reject AUTOMATIC mode as it's not supported */
	TEST_ASSERT_FALSE(ADC_Init(&adc, &config));

	/* When not initialized properly, GetSample should still fail */
	ADC_Sample_t sample;
	TEST_ASSERT_FALSE(ADC_GetSample(&adc, &sample));
}

void test_ADC_GetSample_PollingMode(void) {
	ADC_Sample_t sample;

	init_polling_config();
	TEST_ASSERT_TRUE(ADC_Init(&adc, &config));

	/* Test that we can get samples in polling mode */
	TEST_ASSERT_TRUE(ADC_GetSample(&adc, &sample));
	TEST_ASSERT_TRUE(sample.raw_value >= 0);
	TEST_ASSERT_TRUE(sample.raw_value <= 16383);  /* 14-bit max value */

	TEST_ASSERT_TRUE(ADC_GetSample(&adc, &sample));
	TEST_ASSERT_TRUE(sample.raw_value >= 0);
	TEST_ASSERT_TRUE(sample.raw_value <= 16383);
}

void test_ADC_GetSample_ChannelCycling(void) {
	ADC_Sample_t sample;

	init_polling_config();
	TEST_ASSERT_TRUE(ADC_Init(&adc, &config));

	/* After init with all channels enabled (0x07), currentChannel is set to 3 */
	TEST_ASSERT_EQUAL_INT(3, adc.status.currentChannel);

	/* Get a sample - should advance to next enabled channel */
	TEST_ASSERT_TRUE(ADC_GetSample(&adc, &sample));
	uint8_t ch_after_1 = adc.status.currentChannel;

	/* Get another sample */
	TEST_ASSERT_TRUE(ADC_GetSample(&adc, &sample));
	uint8_t ch_after_2 = adc.status.currentChannel;

	/* Verify samples were actually retrieved */
	TEST_ASSERT_TRUE(sample.raw_value >= 0);
	TEST_ASSERT_TRUE(sample.raw_value <= 16383);

	/* Channels should be different after each GetSample */
	TEST_ASSERT_NOT_EQUAL(ch_after_1, ch_after_2);
}

int main(void) {
	UNITY_BEGIN();

	RUN_TEST(test_ADC_Init_ValidConfig);
	RUN_TEST(test_ADC_Init_InvalidConfig);
	RUN_TEST(test_ADC_Start_Stop_StateTransitions);
	RUN_TEST(test_ADC_GetSample_OnlyPollingMode);
	RUN_TEST(test_ADC_GetSample_PollingMode);
	RUN_TEST(test_ADC_GetSample_ChannelCycling);

	return UNITY_END();
}
