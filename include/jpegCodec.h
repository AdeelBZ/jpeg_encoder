#ifndef JPEG_CODEC
#define JPEG_CODEC
#include "codec.h"
#include "huffmanValues.h"
#include <thread>

typedef char TByte; // Tells us the size of  a particular section. Typically a character is 1 byte so this is just a clearer way to refer to a byte

namespace JFIFConstants
{
    const int MIN_QUAL = 1;
    const int MID_QUAL = 50;
    const int MAX_QUAL = 100;

    const int CHAR_MIN = 1;
    const int CHAR_MAX = 255;

    const int USE_Y_QT = 0x00;
    const int USE_UV_QT = 0x01;

    /* zig zag is done for Run Length Encoding */
    static const unsigned char RLE_ZIG_ZAG[64] =
        {
            0, 1, 5, 6, 14, 15, 27, 28,
            2, 4, 7, 13, 16, 26, 29, 42,
            3, 8, 12, 17, 25, 30, 41, 43,
            9, 11, 18, 24, 31, 40, 44, 53,
            10, 19, 23, 32, 39, 45, 52, 54,
            20, 22, 33, 38, 46, 51, 55, 60,
            21, 34, 37, 47, 50, 56, 59, 61,
            35, 36, 48, 49, 57, 58, 62, 63};
} // namespace JFIFConstants

// static const unsigned char hard_coded_zig_zag[64] =
//     {
//         0, 1, 8, 16, 9, 2, 3, 10,
//         17, 24, 32, 25, 18, 11, 4, 5,
//         12, 19, 26, 33, 40, 48, 41, 34,
//         27, 20, 13, 6, 7, 14, 21, 28,
//         35, 42, 49, 56, 57, 50, 43, 36,
//         29, 22, 15, 23, 30, 37, 44, 51,
//         58, 59, 52, 45, 38, 31, 39, 46,
//         53, 60, 61, 54, 47, 55, 62, 63};

namespace JFIFMarkers
{
    const TByte START_SECTION = 0xFF;
    const TByte SOI_MARKER = 0xD8;
    const TByte APP0_MARKER = 0xE0;
    const TByte DQT_MARKER = 0XDB; // Quant table marker
    const TByte SOF_MARKER = 0xC0;
    const TByte DHT_MARKER = 0xC4; // Huffman Table marker
    const TByte SOS_MARKER = 0xDA;
    const TByte EOI_MARKER = 0xD9;

    const TByte luminance_DC_HT_ID = 0x00;   // highest 4 bits: 0 => DC, lowest 4 bits: 0 => Y (baseline)
    const TByte luminance_AC_HT_ID = 0x10;   // highest 4 bits: 1 => AC, lowest 4 bits: 0 => Y (baseline)
    const TByte chrominance_DC_HT_ID = 0x01; // highest 4 bits: 0 => DC, lowest 4 bits: 1 => CbCr (baseline)
    const TByte chrominance_AC_HT_ID = 0x11; // highest 4 bits: 1 => AC, lowest 4 bits: 1 => CbCr (baseline)

} // namespace JFIFMarkers

class JPEGCodec : public Codec
{

private:
    // std::ofstream m_output_encoded_file;
    double **m_cosines;     //pre-determined cosines that are to be used for DCT calculation
    double *m_cosine_terms; // cosine terms for 2D DCT calculation
    int m_macroblock_size = 8;

    std::thread m_calculate_thread;
public:
    JPEGCodec(const int &a_macroblock_size);
    ~JPEGCodec();

    void encode(unsigned char *a_imgData, char *a_file_name, char *a_dest, int a_width, int a_height, int a_channels);
    void encode(unsigned char *a_imgData, char *a_file_name, char *a_dest, int a_width, int a_height, int a_channels, int a_quality);

protected:
    void thread_constructor(const int &a_macroblock_size);
    void scale_quant_table(float *a_YTable, float *a_UVTable, float *fdtbl_Y_scaled, float *fdtbl_UV_scaled, size_t a_qt_table_size = 64);

    /* c terms is for cosine terms. Needed for a part calculation of 2D DCT */
    void calculate_c_terms(int a_square_size = 8);
    /* The cosines only need to be calculated once before DCT,  Note that this is in representation to 2D array, we can also use a 1D array */
    void calculate_cosines(int a_square_size = 8);

    void set_JIFF_APP0_header();
    void set_JIFF_COM_header();
    void calculate_quant_tables(const int &a_quality, float *a_YTable, float *a_UVTable, size_t a_qt_table_size = 64);
    void set_JIFF_DQT_header(const int &a_quality, const int &a_channels, float *a_YTable, float *a_UVTable, size_t a_qt_table_size = 64); //set Quantization tables
    void set_JIFF_SOF_header(int a_width, int a_height, int a_channels, bool downsample = false);
    void set_JIFF_DHT_header(const int &a_channels); //set huffman tables
    void set_JIFF_SOS_header(int a_channels);

    void compute_DCT(float *dct_pixel_input, int a_square_size = 8);
    // void compute_DCT(float *dct_pixel_input, int a_square_size);

    bool is_valid_input(const unsigned char *a_imgData, const int &a_width, const int &a_height, const int &a_channels);
    void determine_channel_step(const int &a_channels, int &a_g_step, int &a_b_step);
    void convert_rgb_to_ycbcr(unsigned char *a_imgData, const int &a_current_pixel, const int &a_g_step, const int &a_b_step, float *a_Y, float *a_Cb, float *a_Cr, const int &ycbcr_pos);
    int encode_macro_block(int *bitBuf, int *bitCnt, float *CDU, int a_square_size, float *quantTbl, const int &DC, const unsigned short HTDC[256][2], const unsigned short HTAC[256][2]);
    void write_bits_in_file(int *bitBufP, int *bitCntP, const unsigned short *bs);
    void calculate_bit_string_length(int val, unsigned short bits[2]);

    void delete_cosine_table(int a_square_size = 8)
    {
        for (int i = 0; i < a_square_size; i++)
        {
            delete[] m_cosines[i];
        }
        delete[] m_cosines;
    }

    void delete_c_terms(int a_square_size = 8)
    {
        delete[] m_cosine_terms;
    }

    void write_marker_byte_length_in_file(size_t length)
    {
        // add length. Stored as MSB In JEPG format. Looks like length is always stored as MSB instead
        m_output_encoded_file.put((length >> 8)); // 8 MSB of 16 bit string
        m_output_encoded_file.put(length & 0xFF); // 8 LSB of 16 bit string
    }

    void write_density_byte_length_in_file(size_t length)
    {
        write_marker_byte_length_in_file(length);
    }
};

#endif