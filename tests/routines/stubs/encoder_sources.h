/* A second backend proves selection without changing firmware/plotter logic. */
#define FOC_ENCODER_SOURCE_LIST(X) \
    X(0, "AS5047P timer", FOC_EncoderInitTimer, FOC_EncoderReadTimer, 1) \
    X(1, "Internal AS5047P SPI DMA", FOC_EncoderInitSPI, FOC_EncoderReadSPI, 1) \
    X(2, "External AS5047P SPI DMA", FOC_EncoderInitExternalSPI, FOC_EncoderReadSPI, 1) \
    X(3, "External AS5047P quadrature", FOC_EncoderInitExternalTimer, FOC_EncoderReadTimer, 1) \
    X(7, "Test absolute source", TestEncoderInit, TestEncoderRead, 1)
