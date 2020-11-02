#include "../../include/jpegCodec.h"
#include <assert.h> // for assert()

/* constructor */
JPEGCodec::JPEGCodec(const int &a_macroblock_size = 8) : m_calculate_thread(&JPEGCodec::thread_constructor, this, a_macroblock_size)
{
}

void JPEGCodec::thread_constructor(const int &a_macroblock_size)
{
    m_macroblock_size = a_macroblock_size;
    this->calculate_c_terms(a_macroblock_size);
    this->calculate_cosines(a_macroblock_size);
}

JPEGCodec::~JPEGCodec()
{
    this->delete_cosine_table(m_macroblock_size);
    this->delete_c_terms(m_macroblock_size);
}

void JPEGCodec::encode(unsigned char *a_imgData, char *a_file_name, char *a_dest, int a_width, int a_height, int a_channels)
{
    this->encode(a_imgData, a_file_name, a_dest, a_width, a_height, a_channels, 100);
}

void JPEGCodec::scale_quant_table(float *a_YTable, float *a_UVTable, float *fdtbl_Y_scaled, float *fdtbl_UV_scaled, size_t a_qt_table_size)
{
    for (int k = 0; k < a_qt_table_size; k++)
    {
        fdtbl_Y_scaled[k] = (a_YTable[JFIFConstants::RLE_ZIG_ZAG[k]]); // the 1 is there for the quantization part by the looks of it . Like the divsor part if that makes sense
        fdtbl_UV_scaled[k] = (a_UVTable[JFIFConstants::RLE_ZIG_ZAG[k]]);
    }
}

void JPEGCodec::encode(unsigned char *a_imgData, char *a_file_name, char *a_dest, int a_width, int a_height, int a_channels, int a_quality)
{
    float YTable[64], UVTable[64]; //quant tables with specific quality change
    // float fdtbl_Y_scaled[64], fdtbl_UV_scaled[64]; //more specifically, re-arranged to zig-zag

    /* if true, do not process any further */
    if (!this->is_valid_input(a_imgData, a_width, a_height, a_channels))
    {
        return;
    }

    /* open file and start adding headers */
    m_output_encoded_file.open(a_dest, std::ios::trunc | std::ios::out | std::ios::binary);

    /* Set SOI */
    TByte l_SOI[] = {JFIFMarkers::START_SECTION, JFIFMarkers::SOI_MARKER};
    this->write_byte_in_file<TByte>(l_SOI, sizeof(l_SOI) / sizeof(l_SOI[0]));

    this->set_JIFF_APP0_header();
    this->set_JIFF_COM_header();

    // /* adjust the quantiztion tables if needed. Basically, we are adding the actual qunatization table in the jpeg foramt. I guess this allows for decoding? */
    this->calculate_quant_tables(a_quality, YTable, UVTable);
    this->set_JIFF_DQT_header(a_quality, a_channels, YTable, UVTable);
    // std::thread scale_thread(&JPEGCodec::scale_quant_table, this, YTable, UVTable, fdtbl_Y_scaled, fdtbl_UV_scaled, 64);

    this->set_JIFF_SOF_header(a_width, a_height, a_channels);

    this->set_JIFF_DHT_header(a_channels); // huffman tables
    this->set_JIFF_SOS_header(a_channels);

    /* we will just leave the actual encoding here for now but it should be moved */

    // adjust quantization tables with AAN scaling factors to simplify DCT

    const int ENCODE_STEP_SIZE = 8;
    int g_step, b_step;
    this->determine_channel_step(a_channels, g_step, b_step);

    /* move through image in blocks. From top right to bottom left */
    /* move down after going through the block row */
    int bitBuf = 0, bitCnt = 0;         /* looks to be needed for encoding part */
    int DC_Y = 0, DC_CB = 0, DC_CR = 0; //for relative Encoding of DC coefficient
    static const unsigned short fillBits[] = {0x7F, 7};

    m_calculate_thread.join(); //need to cosines terms before we start macro-block encoding
    for (int y = 0; y < a_height; y += ENCODE_STEP_SIZE)
    {
        /* move through whole row */
        for (int x = 0; x < a_width; x += ENCODE_STEP_SIZE)
        {
            int arr_pos = 0;
            float Y[64], Cb[64], Cr[64];

            /* go through each pixel in the macroblock area */
            for (int row = y, r = 0; row < (y + ENCODE_STEP_SIZE); ++row, ++r)
            {
                // row >= height => use last input row and populate Y, Cb and Cr with the same values (so like a phantom row)
                const int LAST_ROW = a_height - 1;
                int row_pos = (row < a_height) ? row : LAST_ROW;

                /* as we encode as macroblocks, when moving to the next row we actually have to move one width down. This is because we are working
                    with the RGB data image format here */
                int current_pixel_row = (row_pos)*a_width * a_channels;

                for (int col = x, c = 0; col < (x + ENCODE_STEP_SIZE); col++, arr_pos++, c++)
                {
                    /* same logic for the rows */
                    const int LAST_COL = a_width - 1;
                    int col_pos = (col < a_width) ? col : LAST_COL;

                    int current_pixel = current_pixel_row + (col_pos * a_channels);

                    this->convert_rgb_to_ycbcr(a_imgData, current_pixel, g_step, b_step, Y, Cb, Cr, arr_pos);
                }
            }

            DC_Y = this->encode_macro_block(&bitBuf, &bitCnt, Y, 8, YTable, DC_Y, HuffmanTable::YDC_HT, HuffmanTable::YAC_HT);
            DC_CB = this->encode_macro_block(&bitBuf, &bitCnt, Cb, 8, UVTable, DC_CB, HuffmanTable::UVDC_HT, HuffmanTable::UVAC_HT);
            DC_CR = this->encode_macro_block(&bitBuf, &bitCnt, Cr, 8, UVTable, DC_CR, HuffmanTable::UVDC_HT, HuffmanTable::UVAC_HT);
        }
    }

    // Do the bit alignment of the EOI marker
    this->write_bits_in_file(&bitBuf, &bitCnt, fillBits); //fill in the last bits in the buffer. This is a OBOB

    // EOI
    TByte l_EOI[] = {JFIFMarkers::START_SECTION, JFIFMarkers::EOI_MARKER};
    this->write_byte_in_file<TByte>(l_EOI, sizeof(l_EOI) / sizeof(l_EOI[0]));
}

/* encode macroblock */
int JPEGCodec::encode_macro_block(int *bitBuf, int *bitCnt, float *CDU, int a_square_size, float *quantTbl, const int &DC, const unsigned short HTDC[256][2], const unsigned short HTAC[256][2])
{
    const unsigned short EOB[2] = {HTAC[0x00][0], HTAC[0x00][1]};
    const unsigned short M16zeroes[2] = {HTAC[0xF0][0], HTAC[0xF0][1]}; //zrl

    int dataOff, i, j, n, diff, end0pos, x, y;
    int DU[64];

    int du_stride = a_square_size;

    this->compute_DCT(CDU);
    // Quantize/descale/zigzag the coefficients
    //we zig zag to increase the compression ratio for the Run Length Encoding of AC coefficients
    for (int z = 0; z < a_square_size * a_square_size; z++)
    {
        float l_quantized_coefficient;
        l_quantized_coefficient = CDU[z] / quantTbl[JFIFConstants::RLE_ZIG_ZAG[z]]; /* the actual quantization is here */
        int l_rounded_quantized_coefficent =
            (l_quantized_coefficient < 0)
                ? l_quantized_coefficient - 0.5f
                : l_quantized_coefficient + 0.5f;
        DU[JFIFConstants::RLE_ZIG_ZAG[z]] = l_rounded_quantized_coefficent; /* zig zagging of coefficients */
        // DU[l_jpg_zigzag[j]] = (int)(v < 0 ? v - 0.5f : v + 0.5f);
    }

    /* entropy encoding is happening here */
    // Encode DC. DC encoding is relative to the difference of the previous DC coefficient; similar to Cipher Block Chaining
    diff = DU[0] - DC;
    if (diff == 0)
    {
        this->write_bits_in_file(bitBuf, bitCnt, HTDC[0]); // we denote the number of bits needed to encode the value as 0 by the looks of it.
    }
    else
    {
        unsigned short bits[2];
        this->calculate_bit_string_length(diff, bits);

        this->write_bits_in_file(bitBuf, bitCnt, HTDC[bits[1]]); // first encode huffman value
        this->write_bits_in_file(bitBuf, bitCnt, bits);          // then encode actual value
    }
    // Encode ACs. Must use Run Length Encoding to encode AC coefficents of the macroblock
    end0pos = 63;
    for (; (end0pos > 0) && (DU[end0pos] == 0); --end0pos) //check if after this is all 0's
    {
    }
    // end0pos = first element in reverse order !=0
    if (end0pos == 0)
    {
        this->write_bits_in_file(bitBuf, bitCnt, EOB); //early end
        return DU[0];
    }
    //else do run length endcoding
    for (i = 1; i <= end0pos; ++i)
    {
        int startpos = i;
        int nrzeroes = 0;
        unsigned short bits[2];
        for (; DU[i] == 0 && i <= end0pos; ++i) //count number of zero's
        {
            nrzeroes++;
        }
        if (nrzeroes >= 16) //looks to be an teration  of the zrl . zrl represents 16 subsequent zeros (which is the maximum allowed number of subsequent zeros).
        {
            int lng = nrzeroes >> 4;
            int nrmarker;
            for (nrmarker = 1; nrmarker <= lng; ++nrmarker)
            {
                this->write_bits_in_file(bitBuf, bitCnt, M16zeroes);
            }
            nrzeroes &= 15; //clear the number
            // nrzeroes = 0;
        }
        // stbiw__jpg_calcBits(DU[i], bits);
        this->calculate_bit_string_length(DU[i], bits);
        int huffman_code = (nrzeroes << 4) + bits[1];                 //huffman code is number of zeros append bits (so tens unit is number of zeros and units is the bits counted if that makes sense)
        this->write_bits_in_file(bitBuf, bitCnt, HTAC[huffman_code]); //puts in huffman code for AC first
        this->write_bits_in_file(bitBuf, bitCnt, bits);               //then writes to file the acutal value
    }
    if (end0pos != 63) //i.e. there are just a number of 0's at the end. if there is denote a eob. eob is a special notifier in jpeg
    {
        this->write_bits_in_file(bitBuf, bitCnt, EOB);
    }
    return DU[0];
}

bool JPEGCodec::is_valid_input(const unsigned char *a_imgData, const int &a_width, const int &a_height, const int &a_channels)
{
    bool l_valid_inputs = (!a_imgData || !a_width || !a_height || (a_channels < 1 || a_channels > 4)) ? false : true;

    /* if valid just return */
    if (l_valid_inputs)
    {
        return l_valid_inputs;
    }

    /* display the values that are invalid */
    std::string l_error_string = "Error with values:\n";
    if (!a_imgData)
    {
        l_error_string.append(" - No image data was given.\n");
    }
    if (a_width == 0)
    {
        l_error_string.append(" - A width must be provided.\n");
    }
    if (a_height == 0)
    {
        l_error_string.append(" - A height must be provided.\n");
    }
    if (a_channels < 1 || a_channels > 4)
    {
        l_error_string.append(" - Number of channels must be between 1-4.\n");
    }
    std::cout << l_error_string;
    return l_valid_inputs;
}

/* if the image is colour or monochrome, this makes us go through RGB data differently*/
void JPEGCodec::determine_channel_step(const int &a_channels, int &a_g_step, int &a_b_step)
{
    /* set g channel step */
    a_g_step = (a_channels == 3) ? 1 : 0;
    a_b_step = (a_channels == 3) ? 2 : 0;
}

/* need to convert rgb to ycbcr for dct to work. */
void JPEGCodec::convert_rgb_to_ycbcr(unsigned char *a_imgData, const int &a_current_pixel, const int &a_g_step, const int &a_b_step, float *a_Y, float *a_Cb, float *a_Cr, const int &ycbcr_pos)
{

    /* pixel values in RGB */
    unsigned char R = a_imgData[a_current_pixel];
    unsigned char G = a_imgData[a_current_pixel + a_g_step];
    unsigned char B = a_imgData[a_current_pixel + a_b_step];

    /* conversion */
    float Y = (0.299f * R) + (0.587f * G) + (0.114f * B);
    float CB = -(0.1687f * R) - (0.3313f * G) + (0.5f * B) + 128.0f;
    float CR = (0.5f * R) - (0.4187f * G) - (0.0813f * B) + 128.0f;

    /* level the values to 128 */
    Y -= 128.0f;
    CB -= 128.0f;
    CR -= 128.0f;

    /* assign back to values */
    a_Y[ycbcr_pos] = Y;
    a_Cb[ycbcr_pos] = CB;
    a_Cr[ycbcr_pos] = CR;
    // std::cout << "File Size: " << a_file_size << " i " << i << '\n';
}

void JPEGCodec::write_bits_in_file(int *bitBufP, int *bitCntP, const unsigned short *bs)
{
    const int VAL = 0, BIT_STR_LEN = 1, BYTE = 8;
    const int MAX_BUFFER = 29; //24 was oringal value;
    assert(MAX_BUFFER < 32 && MAX_BUFFER >= 24);
    const int SHIFT_AMT_FOR_MSB = MAX_BUFFER - BYTE;

    int bitBuf = *bitBufP, bitCnt = *bitCntP;
    bitCnt += bs[BIT_STR_LEN];                  /* actual number of bits needed to encode the value. Here we're keeping a total */
    bitBuf |= bs[VAL] << (MAX_BUFFER - bitCnt); //bitcnt makes this relative. He sets first then left shifts

    while (bitCnt >= BYTE)
    {
        unsigned char c = (bitBuf >> SHIFT_AMT_FOR_MSB) & 255; //he's taking the byte
        m_output_encoded_file.put(c);
        if (c == 255) //this looks to be a rule of the encoding if we get 0xff
        {
            m_output_encoded_file.put(0);
        }
        bitBuf <<= BYTE; //get rid of the byte as we just added to the file
        bitCnt -= BYTE;  // minus the count as we added it in the file
    }
    *bitBufP = bitBuf;
    *bitCntP = bitCnt;
}

void JPEGCodec::calculate_bit_string_length(int val, unsigned short bits[2])
{
    const int VAL = 0, BIT_STR_LEN = 1;

    int l_bit_str_len_counter = (val < 0) ? -val : val; // we want to determine bit string length so we flip negative values to positive for easeability
    val = val < 0 ? val - 1 : val;                      //represnting the negative values as two's complement and not as one's complement

    int bit_str_len = 1;
    while (l_bit_str_len_counter >>= 1)
    {
        ++bit_str_len;
    }
    bits[BIT_STR_LEN] = bit_str_len;

    /* need to do the below 3 lines as due to negative values */
    int bit_str_level = 1 << bits[BIT_STR_LEN]; //level 6 is 64 which is of bit string length 6
    int l_val_mask = bit_str_level - 1;         // e.g. 64 = 0b1000000. 64 -1 = 63 which is 0b111111 and allows us to get the first 6 LSB's if it's negative.
    int a_val = val & l_val_mask;

    bits[VAL] = a_val;
}
