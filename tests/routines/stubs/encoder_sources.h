/* A second backend proves selection without changing firmware/plotter logic. */
#define FOC_ENCODER_SOURCE_LIST(X) \
    X(0, "AS5047P timer", FOC_EncoderInitTimer, FOC_EncoderReadTimer, 1) \
    X(7, "Test absolute source", TestEncoderInit, TestEncoderRead, 1)
